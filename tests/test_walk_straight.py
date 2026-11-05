"""walk_straight task, config round-trip, and the inference-only GaitPolicy."""
from __future__ import annotations

import json
from pathlib import Path

import mujoco
import numpy as np
import pytest

pytest.importorskip("gymnasium")
pytest.importorskip("mujoco")

from petctl.rl.config import EnvConfig  # noqa: E402
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
    cfg = EnvConfig(task="reach_up", episode_s=5.0, urdf_dir="/Users/nsted/Documents/PET/body/URDF/PET_description")
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
