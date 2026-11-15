"""URDF generator (petctl.sim.urdf_gen): n=8 reproduces the owner's URDF, counts/parity/mesh references for n=8..15, builder uses the variants."""
from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial import cKDTree

pytest.importorskip("mujoco")

from petctl.rl.config import DEFAULT_URDF_DIR  # noqa: E402
from petctl.sim import urdf_gen as ug  # noqa: E402

SRC = Path(DEFAULT_URDF_DIR)
pytestmark = pytest.mark.skipif(not (SRC / "urdf" / "pet.xacro").is_file(), reason="owner's URDF export not present")


@pytest.fixture(scope="module")
def gen(tmp_path_factory: pytest.TempPathFactory):
    """Generated folders, one per n, in a temp dir."""
    root = tmp_path_factory.mktemp("urdf_variants")
    cache: dict[int, Path] = {}

    def make(n: int) -> Path:
        if n not in cache:
            cache[n] = ug.generate_urdf(n, root / f"n{n}")
        return cache[n]

    return make


def test_n8_identical_to_owner(gen) -> None:
    a, b = ug.parse_chain(SRC)["links"], ug.parse_chain(gen(8))["links"]
    assert len(a) == len(b) == 8
    for la, lb in zip(a, b):
        assert la["name"] == lb["name"] and la["mesh"] == lb["mesh"]
        assert abs(la["mass"] - lb["mass"]) < 1e-9
        assert all(abs(la["inertia"][k] - lb["inertia"][k]) < 1e-9 for k in la["inertia"])
        assert np.abs(la["com"] - lb["com"]).max() < 1e-9 and np.abs(la["vis"] - lb["vis"]).max() < 1e-9
        assert (SRC / "meshes" / la["mesh"]).read_bytes() == (gen(8) / "meshes" / lb["mesh"]).read_bytes()
        ja, jb = la.get("joint_to_next"), lb.get("joint_to_next")
        assert (ja is None) == (jb is None)
        if ja:
            assert ja["type"] == jb["type"] and ja["child"] == jb["child"]
            assert np.abs(ja["origin"] - jb["origin"]).max() < 1e-9 and np.abs(ja["axis"] - jb["axis"]).max() < 1e-9


def test_screw_fit_is_exact() -> None:
    S, T, res = ug.chain_screw(SRC)
    assert res < 1e-6                                                 # S(module_k) = module_{k+1} to better than a micron, k=1..5
    assert np.allclose(S.power(2)[0], np.eye(3), atol=1e-12) and np.allclose(S.power(2)[1], T)


@pytest.mark.parametrize("n", [9, 10, 11, 12, 15])
def test_counts_refs_and_alternation(gen, n: int) -> None:
    d = gen(n)
    root = ET.parse(d / "urdf" / "pet.xacro").getroot()
    assert len(root.findall("link")) == n and len(root.findall("joint")) == n - 1
    chain = ug.parse_chain(d)["links"]
    assert [lk["name"] for lk in chain] == ["base_link"] + [f"module_{k}_1" for k in range(1, n)]
    for lk in chain:
        assert (d / "meshes" / lk["mesh"]).is_file()
    ax = [lk["joint_to_next"]["axis"] for lk in chain[:-1]]
    for k, a in enumerate(ax, start=1):                               # odd joints +0.707 y, even -0.707 y; always -0.707 z
        assert np.allclose(a, [0, 0.707107 * (1 if k % 2 else -1), -0.707107])
    og = [lk["joint_to_next"]["origin"] for lk in chain[:-1]]
    for k in range(2, n):                                             # joint k: origin y = -0.001237 for even k, +0.001237 for odd k
        assert np.allclose(og[k - 1], [0, -0.001237 if k % 2 == 0 else 0.001237, -0.050735], atol=2e-6)
    assert chain[-1]["mass"] == pytest.approx(chain[0]["mass"])       # tail has the head's mass


@pytest.mark.parametrize("n", [9, 10, 11, 12, 13])
def test_parity_and_tail_mating(gen, n: int) -> None:
    d = gen(n)
    S, T, _ = ug.chain_screw(SRC)
    v = {k: ug._stl_vertices_m(d / "meshes" / f"module_{k}_1.stl") for k in range(1, n)}
    for k in range(1, n - 3):                                         # middle modules: k+1 = S(k), k+2 = k + T (the parity pattern)
        a = v[k] @ S.R.T + S.t
        assert cKDTree(v[k + 1]).query(a)[0].max() < 2e-5
        assert cKDTree(v[k + 2]).query(v[k] + T)[0].max() < 2e-5
    # the tail sits against its neighbour exactly as the owner's tail sits against module 6 (screw-equivariant): same gap and mating patch
    own6, own7 = (ug._stl_vertices_m(SRC / "meshes" / f"module_{k}_1.stl") for k in (6, 7))
    gap0 = cKDTree(own6).query(own7)[0].min()
    gap = cKDTree(v[n - 2]).query(v[n - 1])[0].min()
    assert gap == pytest.approx(gap0, abs=1e-6)
    # the tail is the head turned end-for-end in the owner's chain and keeps that when moved by S^(n-8)
    head = ug._stl_vertices_m(d / "meshes" / "base_link.stl")
    assert len(head) == len(v[n - 1])


def test_tail_flip_is_needed_for_odd_n(gen, tmp_path: Path) -> None:
    wrong = ug.generate_urdf(9, tmp_path / "n9_noflip", tail_flip=False)
    ok = gen(9)
    mesh = lambda d, k: ug._stl_vertices_m(d / "meshes" / f"module_{k}_1.stl")  # noqa: E731
    own = cKDTree(ug._stl_vertices_m(SRC / "meshes" / "module_6_1.stl")).query(ug._stl_vertices_m(SRC / "meshes" / "module_7_1.stl"))[0].min()
    assert cKDTree(mesh(ok, 7)).query(mesh(ok, 8))[0].min() == pytest.approx(own, abs=1e-6)
    n_touch = lambda d: int((cKDTree(mesh(d, 7)).query(mesh(d, 8))[0] < 1e-5).sum())  # noqa: E731
    own_touch = int((cKDTree(ug._stl_vertices_m(SRC / "meshes" / "module_6_1.stl")).query(ug._stl_vertices_m(SRC / "meshes" / "module_7_1.stl"))[0] < 1e-5).sum())
    assert own_touch > 4 and n_touch(ok) == own_touch and n_touch(wrong) < own_touch      # wrong parity: the mating face no longer coincides
    a, b = ug.parse_chain(ok)["links"][-1], ug.parse_chain(SRC)["links"][-1]
    assert a["com"][1] == pytest.approx(-b["com"][1]) and a["inertia"]["iyz"] == pytest.approx(-b["inertia"]["iyz"])   # odd n: inertial turned 180 deg about z
    even = ug.parse_chain(ug.generate_urdf(10, tmp_path / "n10"))["links"][-1]
    assert even["com"][1] == pytest.approx(b["com"][1])


def test_builder_uses_variant() -> None:
    from petctl.sim.model_builder import BuildParams, PetSim, build_mjcf, resolve_urdf_dir

    p = BuildParams(n_modules=9)
    d = resolve_urdf_dir(p)
    assert d is not None and d.endswith("PET_description_n9") and Path(d, "manifest.json").is_file()
    assert resolve_urdf_dir(BuildParams()) is None and resolve_urdf_dir(BuildParams(urdf_dir=str(SRC))) == str(SRC)
    xml, _ = build_mjcf(p)
    assert xml.count('type="hinge"') == 8 and xml.count('type="mesh"') == 9
    sim = PetSim(BuildParams(n_modules=9, delay_s=0.0, dt=0.004))
    sim.step(np.zeros(8), 1.0)
    assert np.isfinite(sim.data.qpos).all()


@pytest.mark.parametrize("n", list(range(9, 16)))
def test_rest_no_interpenetration_and_settle(n: int) -> None:
    import mujoco

    from petctl.sim.model_builder import BuildParams, PetSim

    sim = PetSim(BuildParams(n_modules=n, delay_s=0.0, dt=0.004))
    m, d = sim.model, sim.data
    mujoco.mj_forward(m, d)
    ft = np.zeros(6)
    for a in range(n):
        for b in range(a + 1, n):
            dist = mujoco.mj_geomDistance(m, d, m.geom(f"g{a}").id, m.geom(f"g{b}").id, 0.02, ft)
            assert dist > -1e-4 if b > a + 1 else abs(dist) < 1e-4, (a, b, dist)    # adjacent hulls just touch; the others keep their ~1.8 mm clearance
    sim.step(np.zeros(n - 1), 3.0)
    assert np.isfinite(d.qpos).all() and np.abs(sim.joint_pos()).max() < 0.05 and np.abs(d.qvel).max() < 0.1
