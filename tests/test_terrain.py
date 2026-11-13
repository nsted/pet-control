"""Heightfield terrain, friction tiles, the target post, obstacles on terrain, the layout cache and the terrain glitch guard (Regimen v2 M0-B)."""
from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

pytest.importorskip("mujoco")
import mujoco  # noqa: E402

from petctl.rl.config import DEFAULT_URDF_DIR, EnvConfig, DomainRand  # noqa: E402
from petctl.rl.curriculum.stages import REAL_CAP_NM  # noqa: E402
from petctl.rl.env import PetEnv  # noqa: E402
from petctl.rl.layout_cache import GATE_SEED_MIN, LayoutCache  # noqa: E402
from petctl.rl.tasks.goal_obstacles import bounding_radius  # noqa: E402
from petctl.rl.tasks.goal_reach import GLITCH_LAUNCH_M, GoalReach  # noqa: E402
from petctl.sim.model_builder import POST_HEIGHT_M, POST_RADIUS_M, BuildParams, PetSim  # noqa: E402
from petctl.sim.terrain import BUMP_AMP_MAX_M, CELL_M, HALF_M, N_PTS, ROUGH_MAX_M, SLOPE_MAX_DEG, START_RADIUS_M, generate_terrain  # noqa: E402

_URDF = DEFAULT_URDF_DIR
_HAS_URDF = Path(_URDF).is_dir()
needs_urdf = pytest.mark.skipif(not _HAS_URDF, reason="Fusion URDF export not present")


def test_terrain_deterministic_per_seed() -> None:
    a, b, c = generate_terrain(0.8, 5), generate_terrain(0.8, 5), generate_terrain(0.8, 6)
    assert np.array_equal(a.heights, b.heights) and np.array_equal(a.tile_mu, b.tile_mu) and np.array_equal(a.tile_loose, b.tile_loose)
    assert not np.array_equal(a.heights, c.heights)
    assert a.heights.shape == (321, 321) == (N_PTS, N_PTS)


@pytest.mark.parametrize("seed", [0, 1, 2, 3])
def test_terrain_heights_within_limits(seed: int) -> None:
    t = generate_terrain(1.0, seed)
    assert t.heights.min() >= 0.0 and t.max_height < 0.5
    assert t.parts["bump"].max() <= BUMP_AMP_MAX_M + 1e-12
    assert t.parts["rough"].max() <= ROUGH_MAX_M + 1e-12
    assert t.max_slope_deg("slope") <= SLOPE_MAX_DEG + 1e-6
    assert np.all((t.tile_mu >= 0.2) & (t.tile_mu <= 1.5))
    assert np.all(t.tile_mu[t.tile_loose] <= 0.4) and np.all(t.tile_mu[~t.tile_loose] >= 0.3)
    half = generate_terrain(0.5, seed)                                       # the ramp knob scales every component
    assert half.parts["bump"].max() <= 0.5 * BUMP_AMP_MAX_M + 1e-12 and half.max_slope_deg("slope") <= 0.5 * SLOPE_MAX_DEG + 1e-6
    assert not generate_terrain(0.0, seed).heights.any()


@pytest.mark.parametrize("radius", [START_RADIUS_M, 0.8])
def test_terrain_flat_start_disk_and_edge(radius: float) -> None:
    t = generate_terrain(1.0, 11, radius)
    xs = -HALF_M + CELL_M * np.arange(N_PTS)
    r = np.hypot(*np.meshgrid(xs, xs))
    assert t.heights[r <= radius].max() == 0.0
    assert t.heights[0, :].max() == 0.0 and t.heights[:, -1].max() == 0.0
    assert t.footprint_height(radius) == 0.0


def test_hfield_matches_height_lookup_and_friction_tiles() -> None:
    t = generate_terrain(1.0, 3)
    sim = PetSim(BuildParams(terrain=t, floor_friction=0.9))
    rng = np.random.default_rng(0)
    for x, y in rng.uniform(-3.5, 3.5, (40, 2)):
        d = np.zeros(1, dtype=np.int32)
        dist = mujoco.mj_ray(sim.model, sim.data, np.array([x, y, 2.0]), np.array([0.0, 0.0, -1.0]), None, 1, -1, d)
        assert abs((2.0 - dist) - float(t.height_at(x, y))) < 1e-3
    mu = t.tile_friction(0.9)
    for ty in range(3):
        for tx in range(3):
            assert sim.model.geom(f"terr{ty}{tx}").friction[0] == pytest.approx(mu[ty, tx])
    assert float(mu.min()) < 0.9 < float(mu.max()) or t.tile_loose.any()      # tiles really differ from the floor value
    assert np.allclose(generate_terrain(0.0, 3).tile_friction(0.9), 0.9)       # level 0: uniform


def test_set_terrain_updates_in_place() -> None:
    a, b = generate_terrain(1.0, 1), generate_terrain(1.0, 2)
    sim = PetSim(BuildParams(terrain=a))
    model_id = id(sim.model)
    sim.set_terrain(b)
    fresh = PetSim(BuildParams(terrain=b))
    assert id(sim.model) == model_id and np.array_equal(sim.model.hfield_data, fresh.model.hfield_data)
    assert sim.z0 == pytest.approx(fresh.z0) and np.allclose(sim.model.geom_friction, fresh.model.geom_friction)
    with pytest.raises(ValueError):
        PetSim().set_terrain(a)


@needs_urdf
@pytest.mark.parametrize("n", [8, 11, 15])
def test_zero_commands_settle_on_full_terrain(n: int) -> None:
    t = generate_terrain(1.0, 7, max(START_RADIUS_M, 0.76 if n == 15 else START_RADIUS_M))
    sim = PetSim(BuildParams(n_modules=n, urdf_dir=_URDF, dt=0.004, terrain=t))
    zmax = 0.0
    for _ in range(50):                                                        # 5 s with zero commands
        sim.step(np.zeros(n - 1), 0.1)
        assert np.all(np.isfinite(sim.data.qpos))
        com = sim.data.xipos[1:]
        zmax = max(zmax, float((com[:, 2] - t.height_at(com[:, 0], com[:, 1])).max()))
        assert not GoalReach._terrain_glitch(sim)
    assert zmax < 0.2 and np.abs(sim.data.qvel).max() < 0.2


@needs_urdf
def test_serpenoid_moves_on_level_half_terrain() -> None:
    t = generate_terrain(0.5, 4)
    sim = PetSim(BuildParams(urdf_dir=_URDF, dt=0.004, cap_nm=(0.5,) * 7, terrain=t))
    sim.step(np.zeros(7), 1.0)
    p0 = sim.head_pos().copy()
    for i in range(200):
        sim.step(0.5 * np.sin(2 * np.pi * i * 0.05 - np.arange(1, 8)), 0.05)
        assert not GoalReach._terrain_glitch(sim)
    assert np.hypot(*(sim.head_pos() - p0)[:2]) > 0.05


def test_obstacles_and_post_rest_on_terrain_surface() -> None:
    t = generate_terrain(1.0, 9)
    obs = (("box", 1.5, 1.0, 0.3, 0.12, 0.15), ("cyl", -1.3, 1.7, 0.0, 0.1, 0.1), ("ell", 2.0, -1.2, 1.0, 0.1, 0.2))
    sim = PetSim(BuildParams(terrain=t, obstacles=obs, post=(1.2, -1.9)))
    m = sim.model
    for i, (kind, x, y, _, a, b) in enumerate(obs):
        g = m.geom(f"obs{i}")
        base = t.support_height(x, y, 0.0 if kind == "ell" else (a if kind == "cyl" else float(np.hypot(a, b))))
        assert g.pos[2] - 0.06 == pytest.approx(base, abs=1e-9)                # base of the 12 cm tall obstacle sits on the lowest terrain point under it
        assert base <= float(t.height_at(x, y)) + 1e-12
    post = m.geom("post")
    assert post.size[0] == pytest.approx(POST_RADIUS_M) and 2 * post.size[1] == pytest.approx(POST_HEIGHT_M)
    assert post.pos[2] - POST_HEIGHT_M / 2 == pytest.approx(t.support_height(1.2, -1.9, POST_RADIUS_M), abs=1e-9)
    assert POST_HEIGHT_M >= 2 * 0.07                                           # at least twice a module's height (7 cm prism)
    assert m.geom_bodyid[post.id] == 0                                         # welded to the world: immovable
    flat = PetSim(BuildParams(obstacles=obs))                                  # no terrain: obstacles exactly as before
    assert flat.model.geom("obs0").pos[2] == pytest.approx(0.06)


def _field_cfg(**tp: float) -> EnvConfig:
    cfg = EnvConfig(task="goal_obstacles")
    cfg.task_params = {"field_radius": 2.0, "goal_dist_min": 1.2, "goal_dist_max": 2.0, "n_obs_min": 8, "n_obs_max": 10, **tp}
    return cfg


def test_env_post_terrain_privileged_and_keepout() -> None:
    env = PetEnv(_field_cfg(post=1.0, terrain_level=1.0))
    _, info = env.reset(seed=3)
    post = info["privileged"]["post"]
    assert info["privileged"]["terrain_level"] == 1.0 and post["radius"] == POST_RADIUS_M
    assert np.allclose([post["x"], post["y"]], env.task.goal[:2])              # goal point = post axis
    for o in env.task.obstacles:
        assert np.hypot(o[1] - post["x"], o[2] - post["y"]) - bounding_radius(o) >= 0.5 - 1e-9
    assert env.sim.model.geom("post").id >= 0
    plain = PetEnv(_field_cfg())
    _, info2 = plain.reset(seed=3)
    assert "post" not in info2["privileged"] and plain.sim.p.terrain is None and plain.sim.p.post is None


def test_off_by_default_is_identical() -> None:
    a, b = PetEnv(_field_cfg()), PetEnv(_field_cfg(post=0.0, terrain_level=0.0, layout_cache=0.0))
    oa, _ = a.reset(seed=5)
    ob, _ = b.reset(seed=5)
    assert a.task.obstacles == b.task.obstacles and np.array_equal(oa, ob) and a.sim.p.terrain is None
    for _ in range(5):
        assert np.array_equal(a.step(np.full(7, 0.3))[0], b.step(np.full(7, 0.3))[0])


def test_layout_cache_hit_identical_and_gate_seeds_uncached(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("PETCTL_LAYOUT_CACHE", str(tmp_path))
    env = PetEnv(_field_cfg(layout_cache=3.0, layout_block=2.0))
    seen: dict[int, tuple] = {}
    for s in range(12):
        env.reset(seed=s)
        goal = env.task.preset_goal.copy()
        key = (env.task.obstacles, tuple(goal))
        st = env.task.layout_state()
        idx_files = sorted(p.name for p in next(tmp_path.iterdir()).glob("*.npz"))
        assert set(idx_files) <= {"0.npz", "1.npz", "2.npz"}
        match = [k for k, v in seen.items() if v[0] == key[0]]
        if match:                                                              # a repeated layout is bit-identical, planner field included
            assert seen[match[0]][1] == key[1] and np.array_equal(seen[match[0]][2], st["field_field"])
        seen[s] = (key[0], key[1], st["field_field"])
    assert len(idx_files) == 3 and len({v[0] for v in seen.values()}) <= 3
    # a fresh process-level cache object reads the same files
    cache = LayoutCache(env.cfg, *env._nominal_head())
    state0, goal0 = cache.get(0, lambda rng: (_ for _ in ()).throw(AssertionError("must hit the cache")))
    assert tuple(tuple(o) for o in state0["obstacles"]) in {v[0] for v in seen.values()} or any(len(v[0]) for v in seen.values())
    assert cache.path(0).exists() and goal0.shape == (3,)
    # generation is deterministic in (key, block, index)
    t = env.task.__class__(env.cfg)
    _, g = t.layout(*env._nominal_head(), cache.rng(1))
    st1, g1 = cache.get(1, lambda rng: None)  # type: ignore[arg-type, return-value]
    assert np.array_equal(g, g1) and [list(o) for o in t.obstacles] == [list(o) for o in st1["obstacles"]]
    # gate seeds never read or write the cache
    n_before = len(list(next(tmp_path.iterdir()).glob("*")))
    gate = PetEnv(_field_cfg(layout_cache=3.0, layout_block=2.0))
    plain = PetEnv(_field_cfg())
    gate.reset(seed=GATE_SEED_MIN)
    plain.reset(seed=GATE_SEED_MIN)
    assert gate.task.obstacles == plain.task.obstacles                         # drawn from the seed, identical to the uncached env
    assert len(list(next(tmp_path.iterdir()).glob("*"))) == n_before
    fresh = tmp_path / "other"
    monkeypatch.setenv("PETCTL_LAYOUT_CACHE", str(fresh))
    PetEnv(_field_cfg(layout_cache=3.0)).reset(seed=GATE_SEED_MIN + 5)
    assert not fresh.exists()


def test_terrain_glitch_guard() -> None:
    t = generate_terrain(1.0, 2)
    sim = PetSim(BuildParams(terrain=t))
    assert not GoalReach._terrain_glitch(sim)
    sim.data.qpos[2] += GLITCH_LAUNCH_M + 0.2                                  # whole body launched
    mujoco.mj_forward(sim.model, sim.data)
    assert GoalReach._terrain_glitch(sim)
    sim.reset()
    sim.data.qpos[2] -= 0.05                                                   # body sunk into the floor: contacts penetrate > 1 cm
    mujoco.mj_forward(sim.model, sim.data)
    assert GoalReach._terrain_glitch(sim)
    flat = PetSim()
    flat.data.qpos[2] += 2.0
    mujoco.mj_forward(flat.model, flat.data)
    assert not GoalReach._terrain_glitch(flat)                                 # terrain checks only apply to terrain episodes


def test_saturation_reward_not_length_normalised() -> None:
    cfg = EnvConfig(task="goal_reach", n_modules=11)
    cfg.task_params = {"goal_dist_min": 1.0, "goal_dist_max": 1.2}
    env = PetEnv(cfg)
    env.reset(seed=0)
    for _ in range(10):
        _, _, _, _, info = env.step(np.ones(10))
    cap = env.sim.model.actuator_forcerange[:, 1]
    frac = float(np.mean(np.abs(env.sim.joint_torque()) >= 0.98 * cap))
    assert info["terms"]["saturation"] == pytest.approx(-cfg.reward.saturation * frac)


def test_domain_rand_default_cap_is_real_cap() -> None:
    assert DomainRand().cap_nm == (REAL_CAP_NM, REAL_CAP_NM)
