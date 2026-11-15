"""Generate an n-module copy of the owner's PET URDF/xacro folder (see ``docs/plans/URDF_GENERATOR.md``).

The owner's 8-module Fusion export (``PET_description``) is the only input; nothing in it is modified. The chain is a screw
repeat: module ``k+1`` is module ``k`` moved by ``S`` = 180 deg about the chain axis + half a pair-translation ``T`` along it
(exact for the CAD meshes, verified in ``tests/test_urdf_gen.py``). So

* modules 1..6 are the owner's (verbatim),
* extra middle modules ``k >= 7`` are the owner's modules 5 / 6 (same parity) moved by ``S^(k-5)`` / ``S^(k-6)``,
* head is unchanged, the tail is the owner's tail moved by ``S^(n-8)``: a pure translation for even n and, for odd n,
  additionally rotated 180 deg about the chain axis (``tail_flip``), because the module it mates with has the other parity.

Meshes (STL, mm, CAD world frame at q = 0, exactly the frame ``petctl.sim.model_builder.load_urdf`` reads) are copied when
unchanged and written as transformed binary STLs otherwise. The URDF part (link frames, joint origins, axes, inertials) repeats the
owner's pattern: joint origins alternate ``(0, -/+0.001237, -0.050735)``, axes alternate ``(0, +/-0.707107, -0.707107)``, odd/even
link inertials are related by a 180 deg turn about the link z axis (``x, y -> -x, -y``).

CLI: ``python -m petctl.sim.urdf_gen --n 9 --out data/urdf_variants/PET_description_n9``.
"""

from __future__ import annotations

import hashlib
import json
import os
import re
import shutil
import tempfile
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path

import numpy as np

GENERATOR_VERSION = 2
N_REF = 8                                   # the owner's module count
MAX_N = 15
REPO_ROOT = Path(__file__).resolve().parents[2]
VARIANTS_DIR = REPO_ROOT / "data" / "urdf_variants"
DEFAULT_SOURCE = Path(os.environ.get("PET_URDF_DIR", str(REPO_ROOT.parents[1] / "body" / "URDF" / "PET_description")))

_STL_DT = np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")])
_J_DZ = "-0.050735"                          # joint origin z step of the owner's chain (j3 has -0.050734 in the original: kept verbatim for k<=7)
_J_DY = "0.001237"
_AX = "0.707107"


# ----------------------------------------------------------------------------------------------------------- STL / screw motion


def read_stl(path: Path) -> np.ndarray:
    """Return the triangles of a binary STL as a structured array (normals, vertices in file units, attribute)."""
    b = Path(path).read_bytes()
    n = int(np.frombuffer(b[80:84], "<u4")[0])
    return np.frombuffer(b[84:84 + 50 * n], dtype=_STL_DT).copy()


def write_stl(path: Path, tri: np.ndarray, header: bytes = b"petctl urdf_gen") -> None:
    """Write triangles (as returned by ``read_stl``) as a binary STL."""
    Path(path).write_bytes(header.ljust(80, b" ")[:80] + np.uint32(len(tri)).tobytes() + tri.tobytes())


def _stl_vertices_m(path: Path) -> np.ndarray:
    return read_stl(path)["v"].reshape(-1, 3).astype(float) * 1e-3


def _rot180(u: np.ndarray) -> np.ndarray:
    """Rotation by 180 deg about the unit vector ``u``."""
    return 2.0 * np.outer(u, u) - np.eye(3)


@dataclass(frozen=True)
class Screw:
    """The chain's advance-by-one-module motion ``x -> R x + t`` (metres, CAD world frame): 180 deg about the chain axis plus half a pair shift."""

    R: np.ndarray
    t: np.ndarray

    def power(self, k: int) -> tuple[np.ndarray, np.ndarray]:
        """``S^k`` as ``(R, t)``."""
        R, t = np.eye(3), np.zeros(3)
        for _ in range(k):
            R, t = self.R @ R, self.R @ t + self.t
        return R, t


def _centroid(v: np.ndarray) -> np.ndarray:
    return np.unique(v.round(7), axis=0).mean(axis=0)


def chain_screw(src: Path) -> tuple[Screw, np.ndarray, float]:
    """Fit ``S`` from the owner's meshes: ``module_6 = S(module_5)``. Returns ``(S, T, residual)``, ``T`` = pair translation (= S^2) and
    the worst vertex mismatch (m) of ``S(module_5)`` against ``module_6`` and of ``S(module_4)`` against ``module_5`` ... (should be ~1e-5)."""
    v = {k: _stl_vertices_m(src / "meshes" / f"module_{k}_1.stl") for k in range(1, N_REF)}
    c = {k: _centroid(x) for k, x in v.items()}
    pair = (c[5] - c[3]) + (c[6] - c[4])
    u = pair / np.linalg.norm(pair)
    R = _rot180(u)
    S = Screw(R, c[6] - R @ c[5])
    from scipy.spatial import cKDTree  # type: ignore[import-untyped]

    res = 0.0
    for k in (1, 2, 3, 4, 5):                                           # S(module_k) must be module_{k+1}, up to the slight non-periodicity of the CAD
        a = v[k] @ S.R.T + S.t
        res = max(res, float(cKDTree(v[k + 1]).query(a)[0].max()))
    return S, S.power(2)[1], res


# ------------------------------------------------------------------------------------------------------------------- URDF text


def _f(x: float) -> str:
    return repr(float(x))


def _links_from(src: Path) -> dict[str, ET.Element]:
    root = ET.parse(src / "urdf" / "pet.xacro").getroot()
    return {el.get("name"): el for el in root.findall("link")}


def _inertial(el: ET.Element) -> dict[str, dict[str, str]]:
    i = el.find("inertial")
    return {"origin": dict(i.find("origin").attrib), "mass": dict(i.find("mass").attrib), "inertia": dict(i.find("inertia").attrib)}


def _flip_inertial(d: dict[str, dict[str, str]]) -> dict[str, dict[str, str]]:
    """The same link turned 180 deg about its z axis (the odd <-> even relation of the owner's middle modules): x, y -> -x, -y."""
    out = {k: dict(v) for k, v in d.items()}
    x, y, z = (float(s) for s in d["origin"]["xyz"].split())
    out["origin"]["xyz"] = f"{_f(-x)} {_f(-y)} {_f(z)}"
    for key in ("ixz", "iyz"):
        out["inertia"][key] = _f(-float(d["inertia"][key]))
    return out


def _inertial_xml(d: dict[str, dict[str, str]]) -> str:
    o, m, i = d["origin"], d["mass"], d["inertia"]
    return (f'  <inertial>\n    <origin xyz="{o["xyz"]}" rpy="{o["rpy"]}"/>\n    <mass value="{m["value"]}"/>\n'
            f'    <inertia ixx="{i["ixx"]}" iyy="{i["iyy"]}" izz="{i["izz"]}" ixy="{i["ixy"]}" iyz="{i["iyz"]}" ixz="{i["ixz"]}"/>\n  </inertial>\n')


def _link_xml(name: str, mesh: str, inertial: dict[str, dict[str, str]], vis_xyz: str) -> str:
    geom = f'    <geometry>\n      <mesh filename="package://pet_description/meshes/{mesh}" scale="0.001 0.001 0.001"/>\n    </geometry>\n'
    return (f'<link name="{name}">\n{_inertial_xml(inertial)}  <visual>\n    <origin xyz="{vis_xyz}" rpy="0 0 0"/>\n{geom}    <material name="silver"/>\n  </visual>\n'
            f'  <collision>\n    <origin xyz="{vis_xyz}" rpy="0 0 0"/>\n{geom}  </collision>\n</link>\n')


def _joint_xml(k: int, parent: str, child: str, xyz: str) -> str:
    ay = _AX if k % 2 else "-" + _AX
    return (f'<joint name="Revolute {k}" type="continuous">\n  <origin xyz="{xyz}" rpy="0 0 0"/>\n  <parent link="{parent}"/>\n  <child link="{child}"/>\n'
            f'  <axis xyz="-0.0 {ay} -{_AX}"/>\n</joint>\n')


def _link_name(k: int) -> str:
    return "base_link" if k == 0 else f"module_{k}_1"


def _fmt(v: float) -> str:
    return f"{v:.6f}".rstrip("0").rstrip(".") if abs(v) > 0 else "0.0"


def _urdf_text(n: int, src: Path, tail_flip: bool) -> str:
    links = _links_from(src)
    src_joints = {(j.find("parent").get("link"), j.find("child").get("link")): j for j in ET.parse(src / "urdf" / "pet.xacro").getroot().findall("joint")}
    parts = ['<?xml version="1.0" ?>\n<robot name="pet" xmlns:xacro="http://www.ros.org/wiki/xacro">\n\n'
             '<xacro:include filename="$(find pet_description)/urdf/materials.xacro" />\n'
             '<xacro:include filename="$(find pet_description)/urdf/pet.trans" />\n'
             '<xacro:include filename="$(find pet_description)/urdf/pet.gazebo" />\n']
    frame = np.zeros(3)                                                  # world position of the link frame (cumulative joint origins)
    jxml: list[str] = []
    for k in range(n):
        name = _link_name(k)
        if k > 0:
            if k <= 7 and (_link_name(k - 1), name) in src_joints:       # the owner's joint (verbatim origin), whichever n
                xyz = src_joints[(_link_name(k - 1), name)].find("origin").get("xyz")
            elif k == 1:
                xyz = "0.0 -0.090805 -0.051445"
            else:
                xyz = f"0.0 {'-' if k % 2 == 0 else ''}{_J_DY} {_J_DZ}"
            frame = frame + np.array([float(s) for s in xyz.split()])
            jxml.append(_joint_xml(k, _link_name(k - 1), name, xyz))
        vis = "0.0 0.0 0.0" if k == 0 else f"0.0 {_fmt(-frame[1])} {_fmt(-frame[2])}"
        if k == 0:
            ine, mesh = _inertial(links["base_link"]), "base_link.stl"
        elif k == n - 1:                                                 # tail: the owner's module_7 (even n) or its 180-deg-about-chain twin (odd n)
            ine = _inertial(links["module_7_1"])
            ine = _flip_inertial(ine) if tail_flip else ine
            mesh = f"module_{k}_1.stl"
        else:
            srcname = f"module_{k}_1" if k <= 6 else f"module_{5 if k % 2 else 6}_1"
            ine, mesh = _inertial(links[srcname]), f"module_{k}_1.stl"
        parts.append(_link_xml(name, mesh, ine, vis))
        parts.append("\n")
    parts.append("".join(j + "\n" for j in jxml))
    parts.append("</robot>\n")
    return parts[0] + "\n" + "".join(parts[1:])


def _trans_text(n: int) -> str:
    out = ['<?xml version="1.0" ?>\n<robot name="pet" xmlns:xacro="http://www.ros.org/wiki/xacro" >\n']
    for k in range(1, n):
        out.append(f'\n<transmission name="Revolute {k}_tran">\n  <type>transmission_interface/SimpleTransmission</type>\n  <joint name="Revolute {k}">\n'
                   f'    <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>\n  </joint>\n  <actuator name="Revolute {k}_actr">\n'
                   f'    <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>\n    <mechanicalReduction>1</mechanicalReduction>\n'
                   f'  </actuator>\n</transmission>\n')
    out.append("\n</robot>\n")
    return "".join(out)


def _gazebo_text(n: int, src: Path) -> str:
    text = (src / "urdf" / "pet.gazebo").read_text()
    head = text[:text.index('<gazebo reference="base_link">')]
    blk = ('<gazebo reference="{name}">\n  <material>${{body_color}}</material>\n  <mu1>0.2</mu1>\n  <mu2>0.2</mu2>\n  <selfCollide>true</selfCollide>{g}\n</gazebo>\n\n')
    body = "".join(blk.format(name=_link_name(k), g="\n  <gravity>true</gravity>" if k == 0 else "") for k in range(n))
    return head + body + "</robot>\n"


def _controller_files(n: int, src: Path) -> tuple[str, str]:
    yaml = "pet_controller:\n  # Publish all joint states -----------------------------------\n  joint_state_controller:\n    type: joint_state_controller/JointStateController\n    publish_rate: 50\n\n  # Position Controllers --------------------------------------\n"
    for k in range(1, n):
        yaml += f"  Revolute {k}_position_controller:\n    type: effort_controllers/JointPositionController\n    joint: Revolute {k}\n    pid: {{p: 100.0, i: 0.01, d: 10.0}}\n"
    text = (src / "launch" / "controller.launch").read_text()
    args = " ".join(f"Revolute {k}_position_controller" for k in range(1, n)) + " joint_state_controller "
    text = re.sub(r'args="[^"]*"', f'args="{args}"', text, count=1)
    return yaml, text


# ---------------------------------------------------------------------------------------------------------------------- driver


def generate_urdf(n_modules: int, out_dir: str | Path, source_dir: str | Path | None = None, tail_flip: bool | None = None) -> Path:
    """Write a self-contained n-module copy of the owner's URDF/xacro folder to ``out_dir`` and return it.

    ``tail_flip``: rotate the tail 180 deg about the chain axis relative to the owner's 8-module tail. ``None`` (default) = exactly when the
    alternation requires it, i.e. for odd ``n`` (the module the tail mates with has the other parity). The source folder is only read.
    """
    n = int(n_modules)
    if not N_REF <= n <= MAX_N:
        raise ValueError(f"n_modules must be in {N_REF}..{MAX_N}, got {n}")
    src = Path(source_dir) if source_dir is not None else DEFAULT_SOURCE
    if not (src / "urdf" / "pet.xacro").is_file():
        raise FileNotFoundError(f"owner's URDF not found under {src}")
    flip = (n % 2 == 1) if tail_flip is None else bool(tail_flip)
    out = Path(out_dir)
    (out / "urdf").mkdir(parents=True, exist_ok=True)
    (out / "meshes").mkdir(exist_ok=True)
    (out / "launch").mkdir(exist_ok=True)
    for f in ("CMakeLists.txt", "LICENSE", "package.xml"):
        shutil.copyfile(src / f, out / f)
    for f in ("display.launch", "gazebo.launch", "urdf.rviz"):
        shutil.copyfile(src / "launch" / f, out / "launch" / f)
    shutil.copyfile(src / "urdf" / "materials.xacro", out / "urdf" / "materials.xacro")
    (out / "urdf" / "pet.xacro").write_text(_urdf_text(n, src, flip))
    (out / "urdf" / "pet.trans").write_text(_trans_text(n))
    (out / "urdf" / "pet.gazebo").write_text(_gazebo_text(n, src))
    yaml, launch = _controller_files(n, src)
    (out / "launch" / "controller.yaml").write_text(yaml)
    (out / "launch" / "controller.launch").write_text(launch)

    mesh_src = src / "meshes"
    S, _, _ = chain_screw(src)
    shutil.copyfile(mesh_src / "base_link.stl", out / "meshes" / "base_link.stl")
    for k in range(1, n):
        dst = out / "meshes" / f"module_{k}_1.stl"
        if k == n - 1:                                                   # tail: owner's module_7 moved by S^(n-8) = axial shift (n-8)/2 pairs [+ 180 deg about the chain axis if flipped]
            if n == N_REF and not flip:
                shutil.copyfile(mesh_src / "module_7_1.stl", dst)
            else:
                R, t = _tail_motion(S, n - N_REF, flip)
                _write_moved(dst, read_stl(mesh_src / "module_7_1.stl"), R, t)
        elif k <= 6:
            shutil.copyfile(mesh_src / f"module_{k}_1.stl", dst)
        else:
            base = 5 if k % 2 else 6
            R, t = S.power(k - base)
            _write_moved(dst, read_stl(mesh_src / f"module_{base}_1.stl"), R, t)
    manifest = {"generator": GENERATOR_VERSION, "n_modules": n, "tail_flip": flip, "source": str(src), "source_sha": _source_sha(src)}
    (out / "manifest.json").write_text(json.dumps(manifest, indent=1))
    return out


def _tail_motion(S: Screw, steps: int, flip: bool) -> tuple[np.ndarray, np.ndarray]:
    """Rigid motion ``(R, t)`` of the tail for ``steps`` extra modules: translate ``steps/2`` pair shifts along the chain, then (``flip``) turn 180 deg
    about the chain axis line. For ``flip == (steps odd)`` this equals ``S^steps``."""
    tau = S.power(2)[1] * (steps / 2.0)
    if not flip:
        return np.eye(3), tau
    T = S.power(2)[1]
    uh = T / np.linalg.norm(T)
    c = (S.t - (S.t @ uh) * uh) / 2.0                                    # a point on the screw axis: (I - R) c = t_perp, (I - R) = 2 (I - u u^T) on the perpendicular plane
    return S.R, tau + c - S.R @ c


def _write_moved(path: Path, tri: np.ndarray, R: np.ndarray, t_m: np.ndarray) -> None:
    out = tri.copy()
    v = tri["v"].reshape(-1, 3).astype(float) @ R.T + t_m * 1e3
    out["v"] = v.reshape(-1, 9).astype("<f4")
    out["n"] = (tri["n"].astype(float) @ R.T).astype("<f4")
    write_stl(path, out)


def _source_sha(src: Path) -> str:
    h = hashlib.sha256()
    for p in sorted((src / "meshes").glob("*.stl")) + [src / "urdf" / "pet.xacro"]:
        h.update(p.read_bytes())
    return h.hexdigest()[:16]


@lru_cache(maxsize=32)
def variant_dir(n_modules: int, source_dir: str | None = None) -> str:
    """Path of the cached generated URDF for ``n_modules`` (under ``data/urdf_variants/``), generating it on first use (atomic, safe for parallel workers)."""
    n = int(n_modules)
    src = Path(source_dir) if source_dir else DEFAULT_SOURCE
    target = VARIANTS_DIR / f"PET_description_n{n}"
    want = {"generator": GENERATOR_VERSION, "n_modules": n, "tail_flip": n % 2 == 1, "source_sha": _source_sha(src)}
    man = target / "manifest.json"
    if man.is_file():
        try:
            have = json.loads(man.read_text())
            if all(have.get(k) == v for k, v in want.items()):
                return str(target)
        except (OSError, ValueError):
            pass
    VARIANTS_DIR.mkdir(parents=True, exist_ok=True)
    tmp = Path(tempfile.mkdtemp(prefix=f".gen_n{n}_", dir=VARIANTS_DIR))
    try:
        generate_urdf(n, tmp, src)
        if target.exists():
            shutil.rmtree(target, ignore_errors=True)
        try:
            os.replace(tmp, target)
        except OSError:
            shutil.rmtree(tmp, ignore_errors=True)                       # a parallel worker won the race
    finally:
        if tmp.exists():
            shutil.rmtree(tmp, ignore_errors=True)
    return str(target)


def parse_chain(urdf_dir: str | Path) -> dict:
    """Parse ``urdf/pet.xacro`` of a PET_description folder into chain order: links[k] = dict(name, mass, inertia, com, vis, mesh), joints[k-1] = dict(origin, axis, type)."""
    root = ET.parse(Path(urdf_dir) / "urdf" / "pet.xacro").getroot()
    links = {el.get("name"): el for el in root.findall("link")}
    joints = {j.find("parent").get("link"): j for j in root.findall("joint")}
    order, name = [], "base_link"
    while True:
        el = links[name]
        ine = el.find("inertial")
        vis = el.find("visual")
        order.append({
            "name": name, "mass": float(ine.find("mass").get("value")),
            "inertia": {k: float(v) for k, v in ine.find("inertia").attrib.items()},
            "com": np.array([float(s) for s in ine.find("origin").get("xyz").split()]),
            "vis": np.array([float(s) for s in vis.find("origin").get("xyz").split()]),
            "mesh": vis.find("geometry/mesh").get("filename").rsplit("/", 1)[-1]})
        j = joints.get(name)
        if j is None:
            break
        order[-1]["joint_to_next"] = {"type": j.get("type"), "origin": np.array([float(s) for s in j.find("origin").get("xyz").split()]),
                                      "axis": np.array([float(s) for s in j.find("axis").get("xyz").split()]), "child": j.find("child").get("link")}
        name = j.find("child").get("link")
    return {"links": order}


def main(argv: list[str] | None = None) -> None:
    """CLI: ``python -m petctl.sim.urdf_gen --n 9 --out data/urdf_variants/PET_description_n9``."""
    import argparse

    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--n", type=int, required=True, help="number of modules (8..15)")
    ap.add_argument("--out", default=None, help="output folder (default data/urdf_variants/PET_description_n<N>)")
    ap.add_argument("--source", default=None, help="owner's PET_description folder (read only)")
    ap.add_argument("--tail-flip", choices=["auto", "yes", "no"], default="auto", help="rotate the tail 180 deg about the chain axis (auto: odd n)")
    a = ap.parse_args(argv)
    out = Path(a.out) if a.out else VARIANTS_DIR / f"PET_description_n{a.n}"
    flip = {"auto": None, "yes": True, "no": False}[a.tail_flip]
    res = generate_urdf(a.n, out, a.source, flip)
    print(f"wrote {res}  (open {res}/urdf/pet.xacro: `xacro pet.xacro > pet.urdf`, or load the folder in urdf viewers; package name pet_description)")


if __name__ == "__main__":
    main()
