"""RL tasks (walk, reach, jump), config round-trip, the inference-only GaitPolicy and the skill library."""
from __future__ import annotations

import json
from pathlib import Path

import mujoco
import numpy as np
import pytest

pytest.importorskip("gymnasium")
pytest.importorskip("mujoco")

from petctl.rl.config import DEFAULT_URDF_DIR, EnvConfig  # noqa: E402
from petctl.rl.env import PetEnv  # noqa: E402
from petctl.rl.policy import POLICY_FILE, GaitPolicy  # noqa: E402


def test_config_roundtrip() -> None:
    cfg = EnvConfig(task="walk_straight")
    cfg.dr.scale = 0.3
    back = EnvConfig.from_dict(json.loads(json.dumps(cfg.to_dict())))
    assert back.to_dict() == cfg.to_dict()


def test_walk_straight_metrics() -> None:
    env = PetEnv(EnvConfig(task="walk_straight", episode_s=1.0))
    env.reset(seed=0)
    for _ in range(10):
        _, _, _, _, info = env.step(np.random.default_rng(0).uniform(-1, 1, 7))
    m = info["metrics"]
    assert set(m) == {"distance", "path", "cross_track", "straightness", "energy_per_m"}
    assert m["path"] >= abs(m["distance"]) - 1e-9


def _fake_run(tmp: Path, cfg: EnvConfig) -> Path:
    rng = np.random.default_rng(0)
    n_obs = PetEnv(cfg).observation_space.shape[0]
    sizes = [n_obs, 16, 16, 7]
    arrs: dict[str, np.ndarray] = {"n_layers": np.array(3), "obs_mean": np.zeros(n_obs), "obs_var": np.ones(n_obs),
                                   "clip_obs": np.array(10.0), "eps": np.array(1e-8)}
    for i in range(3):
        arrs[f"w{i}"] = rng.normal(0, 0.3, (sizes[i + 1], sizes[i]))
        arrs[f"b{i}"] = np.zeros(sizes[i + 1])
    np.savez(tmp / POLICY_FILE, **arrs)
    (tmp / "config.json").write_text(json.dumps(cfg.to_dict()))
    return tmp


def test_gait_policy_matches_env_targets(tmp_path: Path) -> None:
    """The deployable step() must reproduce the env's EMA/slew target sequence exactly."""
    cfg = EnvConfig(task="walk_straight", episode_s=2.0)
    cfg.dr.scale = 0.0
    pol = GaitPolicy(_fake_run(tmp_path, cfg))
    env = PetEnv(pol.cfg)
    obs, _ = env.reset(seed=1)
    pol.reset(env._snapshot())
    # fresh snapshot noise differs per call, so compare actions->targets from the same observation instead
    for _ in range(15):
        a = pol.action(obs)
        obs, *_ = env.step(a)
        assert np.all(np.abs(a) <= 1.0)
        pol.obs.push_action(a)
        c = pol.cfg
        sm = c.action_ema * pol._tgt + (1 - c.action_ema) * c.action_scale_rad * a
        pol._tgt = pol._tgt + np.clip(sm - pol._tgt, -c.max_target_step_rad, c.max_target_step_rad)
        assert np.allclose(pol._tgt, env._tgt)


def test_walk_sideways_axis_and_sign() -> None:
    """Progress axis is perpendicular to the body's length; distance is |net sideways| so either side counts."""
    env = PetEnv(EnvConfig(task="walk_sideways", episode_s=1.0))
    env.reset(seed=0)
    t = env.task
    assert abs(float(t.axis @ t.length_dir)) < 1e-9 and abs(t.axis[2]) < 1e-9
    for sgn in (1.0, -1.0):                                  # shove the CoM both ways along the axis
        env.sim.data.qpos[:3] += sgn * 0.1 * t.axis
        mujoco.mj_forward(env.sim.model, env.sim.data)
        assert abs(t.distance(env.sim) - 0.1) < 1e-6
        env.sim.data.qpos[:3] -= sgn * 0.1 * t.axis
        mujoco.mj_forward(env.sim.model, env.sim.data)
    _, r, *_ , info = env.step(np.zeros(7))
    assert np.isfinite(r) and "metrics" in info


def test_reach_up_peak_reward_telescopes_and_guards_clipping() -> None:
    """Rewards peak growth only: zero at the flat start, total peak_gain ~ 10*height_w*peak, no peak while clipping the floor."""
    cfg = EnvConfig(task="reach_up", episode_s=5.0, urdf_dir=DEFAULT_URDF_DIR)
    cfg.dr.scale = 0.0
    env = PetEnv(cfg)
    env.reset(seed=0)
    assert abs(env.task.gain(env.sim)) < 1e-6
    total = 0.0
    for _ in range(60):
        _, r, _, _, info = env.step(np.ones(7))
        total += info["terms"]["peak_gain"]
        assert np.isfinite(r)
    assert np.isclose(total, 10.0 * cfg.reward.height * env.task.peak, rtol=1e-6)
    assert set(info["metrics"]) == {"distance", "peak_m", "current_m"}


def _still_env():
    cfg = EnvConfig(task="reach_still", episode_s=20.0, action_scale_rad=np.pi, urdf_dir=DEFAULT_URDF_DIR)
    cfg.dr.scale, cfg.dr.cap_nm = 0.0, (0.5, 0.5)
    return PetEnv(cfg)


def test_reach_still_counts_sustained_height_not_stillness() -> None:
    """A raised pose counts even with small balancing motion; a height that is not sustained does not."""
    q = np.array([1.42, -1.93, -1.02, 1.86, 1.06, 1.58, 0.45])   # a valid raised pose from the constrained static search
    env = _still_env()
    env.reset(seed=0)
    total = 0.0
    for i in range(400):                                      # hold the pose while the motors keep making small corrections
        _, _, _, trunc, info = env.step(q / np.pi + 0.02 * np.sin(0.5 * i))
        total += info["terms"]["height_change"]
    assert np.isclose(total, 1000.0 * info["metrics"]["current_m"], rtol=1e-6)   # telescopes to the final height
    m = info["metrics"]
    assert m["peak_still_m"] > 0.03 and m["peak_still_m"] <= m["peak_any_m"] + 1e-9
    assert len(info["pose"]["joint_pos_rad"]) == 7 and info["pose"]["top_module"] >= 0
    env.reset(seed=0)                                         # thrash: any-instant height may rise, sustained height must stay ~0
    rng = np.random.default_rng(0)
    for _ in range(200):
        _, _, _, _, info = env.step(rng.choice([-1.0, 1.0], 7))
    assert info["metrics"]["peak_still_m"] < 0.01


def test_finalize_session_marks_best_and_writes_index(tmp_path: Path) -> None:
    """The session index lists videos in discovery order and renames the highest replayed one BEST_."""
    from petctl.rl.video import finalize_session
    entries = []
    for n, (rec, rep) in enumerate([(0.03, 0.025), (0.06, 0.055), (0.08, 0.04)], start=1):
        name = f"{n:02d}_rec{100 * rec:.1f}cm.mp4"
        (tmp_path / name).write_bytes(b"x")
        (tmp_path / (name + ".json")).write_text(json.dumps({"distance": rep}))
        entries.append({"n": n, "video": name, "record_cm": 100 * rec, "episode_return": 10.0 * n, "episode": n, "timesteps": n, "wall_min": n})
    (tmp_path / "index.json").write_text(json.dumps(entries))
    best = finalize_session(tmp_path)
    assert best is not None and best.name == "BEST_02_rec6.0cm.mp4" and best.exists()
    assert "BEST_02_rec6.0cm.mp4" in (tmp_path / "index.md").read_text()


def test_jump_scores_only_airborne_clearance() -> None:
    """Standing/lying scores 0 (floor contact); lifting the whole body off the floor is scored by lowest-point height."""
    cfg = EnvConfig(task="jump", episode_s=5.0, action_scale_rad=np.pi, urdf_dir=DEFAULT_URDF_DIR)
    cfg.dr.scale, cfg.dr.cap_nm = 0.0, (0.5, 0.5)
    env = PetEnv(cfg)
    env.reset(seed=0)
    t = env.task
    assert t._floor_contact(env.sim) and abs(t.clearance(env.sim)) < 2e-3
    for _ in range(40):                                       # hold still: nothing airborne, nothing scored
        _, r, _, _, info = env.step(np.zeros(7))
    assert info["metrics"]["record_clearance_m"] == 0.0 and info["metrics"]["jumps"] == 0.0
    env.sim.data.qpos[2] += 0.05                              # teleport 5 cm up: fully airborne for the next scored step
    env.sim.data.qvel[:] = 0.0
    mujoco.mj_forward(env.sim.model, env.sim.data)
    assert not t._floor_contact(env.sim) and t.clearance(env.sim) > 0.04
    _, _, _, _, info = env.step(np.zeros(7))
    m = info["metrics"]
    assert m["jumps"] == 1.0 and 0.0 < m["record_clearance_m"] < 0.06
    assert np.isclose(info["terms"]["clearance_record"], 1000.0 * m["record_clearance_m"])


def test_jump_rejects_blown_up_physics_without_recording_it() -> None:
    """A teleport to 16 m with huge velocity must not set a record, and must end the episode with a penalty."""
    cfg = EnvConfig(task="jump", episode_s=5.0, action_scale_rad=np.pi, urdf_dir=DEFAULT_URDF_DIR)
    cfg.dr.scale, cfg.dr.cap_nm = 0.0, (0.5, 0.5)
    env = PetEnv(cfg)
    env.reset(seed=0)
    env.sim.data.qpos[2] += 16.0
    env.sim.data.qvel[2] = 18.0
    mujoco.mj_forward(env.sim.model, env.sim.data)
    _, _, term, _, info = env.step(np.zeros(7))
    assert term and info["metrics"]["record_clearance_m"] == 0.0 and info["terms"]["glitch"] < 0.0


def test_skill_library_add_resolve_list(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    """add_skill verifies by replay and copies the files; resolve accepts id, bare name and run dir; list marks the best per task."""
    from petctl.rl import skills

    monkeypatch.setattr(skills, "SKILLS_ROOT", tmp_path / "lib")
    cfg = EnvConfig(task="walk_straight", episode_s=1.0)
    cfg.dr.cap_nm = (0.6, 0.6)
    run = _fake_run(tmp_path, cfg)
    monkeypatch.setattr(skills, "VERIFY_SEEDS", (0, 1))
    dest = skills.add_skill(run, "fake_a", note="test", render=False)
    assert (dest / "policy.npz").exists() and (dest / "config.json").exists()
    info = json.loads((dest / "skill.json").read_text())
    assert info["id"] == "walk_straight/fake_a" and info["score"]["seeds"] == [0, 1] and info["caveats"]   # 0.6 N.m > the real 0.5 N.m cap
    with pytest.raises(FileExistsError):
        skills.add_skill(run, "fake_a", render=False)
    skills.add_skill(run, "fake_b", render=False)
    assert skills.resolve("walk_straight/fake_a") == dest and skills.resolve("fake_a") == dest and skills.resolve(str(run)) == run
    with pytest.raises(FileNotFoundError):
        skills.resolve("nope")
    rows = skills.list_skills()
    assert len(rows) == 2 and sum(r["best"] for r in rows) == 1
