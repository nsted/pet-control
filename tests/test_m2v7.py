"""M2 v7: learned hold channel (env vs GaitPolicy, h semantics, h init), stages, driver go / no-go logic with synthetic episodes."""
from __future__ import annotations

import json
import os
from pathlib import Path

import numpy as np
import pytest

pytest.importorskip("mujoco")

from petctl.rl.config import DEFAULT_URDF_DIR, DELTA_LIMIT_RAD, HOLD_EMA_BASE, HOLD_EMA_RANGE, EnvConfig  # noqa: E402
from petctl.rl.curriculum import m2v7  # noqa: E402
from petctl.rl.curriculum.runner import build_config  # noqa: E402
from petctl.rl.curriculum.stages import FLAT4_PILOT, STAGES, get_stage, stage_variant  # noqa: E402
from petctl.rl.env import PetEnv  # noqa: E402
from petctl.rl.obs import ObsBuilder, hold_value  # noqa: E402
from petctl.rl.policy import POLICY_FILE, GaitPolicy  # noqa: E402

needs_urdf = pytest.mark.skipif(not os.path.isdir(DEFAULT_URDF_DIR), reason="Fusion URDF export not present")


def _fake_run(tmp: Path, cfg: EnvConfig) -> Path:
    rng = np.random.default_rng(0)
    n_obs = PetEnv(cfg).observation_space.shape[0]
    sizes = [n_obs, 16, 16, cfg.n_actions]
    arrs: dict[str, np.ndarray] = {"n_layers": np.array(3), "obs_mean": np.zeros(n_obs), "obs_var": np.ones(n_obs), "clip_obs": np.array(10.0), "eps": np.array(1e-8), "n_joints": np.array(cfg.n_joints)}
    for i in range(3):
        arrs[f"w{i}"] = rng.normal(0, 0.8, (sizes[i + 1], sizes[i]))
        arrs[f"b{i}"] = np.zeros(sizes[i + 1])
    np.savez(tmp / POLICY_FILE, **arrs)
    (tmp / "config.json").write_text(json.dumps(cfg.to_dict()))
    return tmp


def _cfg(mode: str, task: str = "goal_reach") -> EnvConfig:
    cfg = EnvConfig(task=task, n_modules=8, episode_s=4.0, action_mode=mode, action_scale_rad=1.0, action_ema=0.3)
    cfg.dr.scale = 0.0
    return cfg


# ---------------------------------------------------------------- hold mapping
def test_hold_mapping_and_action_space() -> None:
    """n_joints + 1 outputs; h = (a_h + 1) / 2; h = 0 reproduces the abs mapping (EMA 0.3), h = 1 freezes the target, h = 0.5 gives EMA 0.65; the history holds joint outputs only."""
    cfg, cfg_abs = _cfg("hold"), _cfg("abs")
    assert cfg.n_actions == 8 and cfg_abs.n_actions == 7 and hold_value(np.array([0.0, -1.0])) == 0.0 and hold_value(np.array([0.0, 1.0])) == 1.0
    env, env_abs = PetEnv(cfg), PetEnv(cfg_abs)
    assert env.action_space.shape == (8,) and env_abs.action_space.shape == (7,)
    assert env.observation_space.shape[0] == env_abs.observation_space.shape[0] + 7                    # appended-only target block
    env.reset(seed=1), env_abs.reset(seed=1)
    rng = np.random.default_rng(0)
    for _ in range(6):                                                                                  # h = 0 (a_h = -1): identical targets to abs
        a = rng.uniform(-1, 1, 7)
        env.step(np.append(a, -1.0)), env_abs.step(a)
        assert np.allclose(env._tgt, env_abs._tgt)
    before = env._tgt.copy()
    env.step(np.append(rng.uniform(-1, 1, 7), 1.0))                                                     # h = 1: frozen
    assert np.allclose(env._tgt, before)
    a = rng.uniform(-1, 1, 7)
    env.step(np.append(a, 0.0))                                                                         # h = 0.5
    ema = HOLD_EMA_BASE + HOLD_EMA_RANGE * 0.5
    want = before + np.clip(ema * before + (1 - ema) * a - before, -0.35, 0.35)
    assert ema == pytest.approx(0.65) and np.allclose(env._tgt, want)
    assert np.allclose(env.obs_builder._act[-1], a)                                                     # history = joint outputs


@needs_urdf
def test_hold_env_matches_gait_policy(tmp_path: Path) -> None:
    """GaitPolicy.step reproduces the env's hold targets on a recorded sequence (policy output has the extra component)."""
    cfg = _cfg("hold")
    nj = cfg.n_joints
    pol = GaitPolicy(_fake_run(tmp_path, cfg))
    env = PetEnv(pol.cfg)
    obs, _ = env.reset(seed=3)
    pol.reset(env._snapshot())
    rec: list[np.ndarray] = []
    orig = pol.action
    pol.action = lambda o: rec.append(orig(o)) or rec[-1]                                              # type: ignore[method-assign]
    hs = []
    for _ in range(30):
        tgt = pol.step(env._snapshot())
        assert rec[-1].shape == (nj + 1,)
        obs, *_ = env.step(rec[-1])
        hs.append(hold_value(rec[-1]))
        assert np.allclose(tgt, env._tgt) and np.allclose(obs[-nj:], env._tgt / DELTA_LIMIT_RAD, atol=1e-6)
    assert 0.0 <= min(hs) and max(hs) <= 1.0 and np.ptp(hs) > 0.1 and np.abs(env._tgt).max() > 0.3


def test_hold_metrics_logged() -> None:
    """Episode metrics carry h_mean / h_wrap_n / state_hold_s in hold mode only."""
    for mode, has in (("hold", True), ("abs", False)):
        env = PetEnv(_cfg(mode))
        env.reset(seed=0)
        a = np.zeros(env.action_space.shape[0])
        for _ in range(5):
            *_, info = env.step(a)
        assert ("h_mean" in info["metrics"]) == has
        if has:
            assert info["metrics"]["h_mean"] == pytest.approx(0.5) and info["metrics"]["h_wrap_n"] == 0.0 and "h_wrap_mean" not in info["metrics"]


@needs_urdf
def test_hold_metrics_in_hug_task() -> None:
    """The hug task reports wrap-tick h and the state-hold streak keys (state_hold_s counts ticks with run >= 3 and span >= 90)."""
    cfg = build_config(get_stage("m2_hug7"), {"n_modules": 8.0, "dr_scale": 0.0})
    env = PetEnv(cfg)
    env.reset(seed=0)
    assert env.action_space.shape == (8,)
    for _ in range(3):
        *_, info = env.step(np.zeros(8))
    m = info["metrics"]
    assert {"h_mean", "h_wrap_n", "state_hold_s"} <= set(m) and m["state_hold_s"] == 0.0
    env._hold_step(0.9)                                                                                 # a synthetic wrap tick through the bookkeeping
    assert env._h_sum > 0


def test_hold_init_near_half() -> None:
    """A fresh PPO on a hold env outputs a_h ~ 0 (h = 0.5, EMA 0.65); init_hold_output sets it exactly."""
    sb3 = pytest.importorskip("stable_baselines3")
    import torch
    from stable_baselines3.common.vec_env import DummyVecEnv

    from petctl.rl.train import init_hold_output

    cfg = _cfg("hold")
    venv = DummyVecEnv([lambda: PetEnv(cfg)])
    model = sb3.PPO("MlpPolicy", venv, policy_kwargs={"net_arch": [32, 32], "log_std_init": -1.0}, seed=0, n_steps=64, batch_size=64)
    with torch.no_grad():
        model.policy.action_net.bias[7] = 0.9                                                         # pretend something moved it
    init_hold_output(model, cfg.n_joints)
    obs = venv.reset()
    act, _ = model.predict(obs, deterministic=True)
    assert model.policy.action_net.bias.shape == (8,) and abs(float(act[0, 7])) < 1e-6 and hold_value(act[0]) == pytest.approx(0.5)
    sampled = np.array([model.predict(obs, deterministic=False)[0][0, 7] for _ in range(200)])         # exploration around h = 0.5
    assert abs(hold_value(np.array([sampled.mean()])) - 0.5) < 0.1


# ---------------------------------------------------------------- stages
def test_stage_definitions() -> None:
    """m2_flat4 / m2_hug7 as ordered; no new stage uses delta; action_mode is a parameter; old stages are untouched."""
    f, h = get_stage("m2_flat4"), get_stage("m2_hug7")
    assert (f.task, f.action_mode, f.episode_s, f.ent_coef, f.dr_scale) == ("goal_reach", "abs", 20.0, 0.005, 0.6)
    assert (f.task_params["goal_dist_min"], f.task_params["goal_dist_max"], f.task_params["bearing_deg"]) == (0.6, 1.5, 180.0)
    assert FLAT4_PILOT == {"bearing_deg": 45.0, "goal_dist_min": 0.6, "goal_dist_max": 1.2, "dr_scale": 0.3}
    assert stage_variant("m2_flat4", "hold").action_mode == "hold" and stage_variant("m2_flat4").action_mode == "abs"
    cfg = build_config(stage_variant("m2_flat4", "hold"), {"n_modules": 10.0})
    assert cfg.action_mode == "hold" and cfg.action_scale_rad == 1.0 and cfg.action_ema == 0.3 and cfg.n_actions == 10
    assert (h.task, h.action_mode, h.episode_s, h.ent_coef, h.log_std_init, h.dr_scale) == ("goal_hug", "hold", 20.0, 0.002, -1.2, 0.6)
    tp = h.task_params
    assert (tp["shape_phase"], tp["hug_gap_min"], tp["hug_gap_max"], tp["bearing_deg"], tp["start_pose_rad"], tp["hug_stall_s"], tp["hug_buffer"], tp["hold_weight"]) == (1.0, 0.10, 0.40, 90.0, 0.4, 8.0, 1.0, 3.0)
    assert "hug_start_mix" not in tp and h.gate.task_params["hug_buffer"] == 0.0
    assert build_config(h, {"hold_weight": 6.0, "dr_scale": 0.3}).task_params["hold_weight"] == 6.0
    assert [n for n, s in STAGES.items() if s.action_mode == "hold"] == ["m2_hug7"]
    assert [n for n, s in STAGES.items() if s.action_mode == "delta"] == ["m2_flat3", "m2_hug6"]       # the old v6 stages are untouched
    assert ObsBuilder(build_config(h, {})).blocks()[-1] == ("target", 7)


# ---------------------------------------------------------------- go / no-go (synthetic episodes)
def _flat_rows(n: int, p: float, ratio: float, t0: int = 0, dt: int = 400, seed: int = 0) -> list[dict]:
    rng = np.random.default_rng(seed)
    return [{"timesteps": t0 + dt * (i + 1), "success": float(rng.random() < p), "final_dist": ratio, "dist0": 1.0} for i in range(n)]


def test_flat_signals_and_go() -> None:
    """Rolling success + median ratio over the last 100 episodes by the check step; both must pass."""
    good = _flat_rows(5000, 0.3, 0.5)
    sig = m2v7.flat_signals(good, 1_500_000)
    assert sig["n"] == 100 and sig["success"] > 0.15 and sig["median_ratio"] == 0.5 and m2v7.flat_go(sig)
    assert not m2v7.flat_go(m2v7.flat_signals(_flat_rows(5000, 0.3, 0.95), 1_500_000))             # robot barely moves
    assert not m2v7.flat_go(m2v7.flat_signals(_flat_rows(5000, 0.05, 0.5), 1_500_000))             # success too low
    late = _flat_rows(2000, 0.0, 1.0) + _flat_rows(2000, 0.5, 0.4, t0=800_000)                       # takes off after the check
    assert not m2v7.flat_go(m2v7.flat_signals(late, 500_000)) and m2v7.flat_go(m2v7.flat_signals(late, 2_000_000))
    assert not m2v7.flat_go(m2v7.flat_signals([], 1_500_000))


def test_time_to_takeoff_and_p1_rule() -> None:
    """Takeoff = end of the first 100-episode window with success >= 0.10; P1 needs it within 1.5x of P0's."""
    rows = _flat_rows(300, 0.0, 1.0) + [{"timesteps": 200_000 + 1000 * i, "success": 1.0 if i % 5 == 0 else 0.0, "final_dist": 0.5, "dist0": 1.0} for i in range(400)]
    t = m2v7.time_to_takeoff(rows)
    assert t is not None and 200_000 < t < 200_000 + 1000 * 400 and m2v7.time_to_takeoff(_flat_rows(500, 0.0, 1.0)) is None and m2v7.time_to_takeoff(_flat_rows(50, 1.0, 1.0)) is None
    ok = {"n": 100.0, "success": 0.3, "median_ratio": 0.4}
    assert m2v7.p1_go(ok, 1_000_000, 1_400_000) and not m2v7.p1_go(ok, 1_000_000, 1_600_000)
    assert not m2v7.p1_go(ok, 1_000_000, None) and not m2v7.p1_go({**ok, "success": 0.05}, 1_000_000, 900_000)


def _hug_rows(n: int, reach: float, hold: float, h_wrap: float, seed: int = 0) -> list[dict]:
    rng = np.random.default_rng(seed)
    rows = []
    for i in range(n):
        r = rng.random() < reach
        k = rng.random() < hold
        rows.append({"timesteps": 1000 * i, "hug_modules": 4.0 if r else 1.0, "normal_span_deg": 120.0 if r else 10.0, "state_hold_s": 3.0 if (r and k) else 0.5,
                     "h_mean": h_wrap, "h_wrap_n": 20.0 if r else 0.0, **({"h_wrap_mean": h_wrap} if r else {})})
    return rows


def test_p2_decisions() -> None:
    """approach -> hold_unused (h not rising) -> holdability -> go, over the last 300 episodes only."""
    assert m2v7.p2_decision(m2v7.p2_signals(_hug_rows(600, 0.02, 0.9, 0.9))) == "approach"
    assert m2v7.p2_decision(m2v7.p2_signals(_hug_rows(600, 0.3, 0.9, 0.5))) == "hold_unused"
    assert m2v7.p2_decision(m2v7.p2_signals(_hug_rows(600, 0.3, 0.1, 0.9))) == "holdability"
    sig = m2v7.p2_signals(_hug_rows(600, 0.3, 0.6, 0.85))
    assert m2v7.p2_decision(sig) == "go" and sig["n"] == 300 and 0.2 < sig["frac_reach"] < 0.4 and 0.4 < sig["frac_hold"] < 0.8 and sig["h_wrap"] == pytest.approx(0.85)
    no_wrap = [{**r, "h_wrap_n": 0.0} for r in _hug_rows(300, 0.3, 0.9, 0.9)]
    for r in no_wrap:
        r.pop("h_wrap_mean", None)
    assert m2v7.p2_signals(no_wrap)["h_wrap"] is None and m2v7.p2_decision(m2v7.p2_signals(no_wrap)) == "hold_unused"
    old = _hug_rows(400, 0.0, 0.0, 0.0) + _hug_rows(300, 0.5, 0.9, 0.9)                              # only the last 300 count
    assert m2v7.p2_decision(m2v7.p2_signals(old)) == "go"
    assert m2v7.p2_signals([])["frac_reach"] == 0.0 and m2v7.p2_decision(m2v7.p2_signals([])) == "approach"


def test_config_diff(tmp_path: Path) -> None:
    """The config diff lists changed, added and removed keys of nested dicts."""
    a, b = {"x": 1, "dr": {"scale": 0.3, "k": 2}, "only_a": 5}, {"x": 1, "dr": {"scale": 0.6, "k": 2}, "only_b": 7}
    d = m2v7.config_diff(a, b, ("ours", "ref"))
    assert d == ["dr.scale: ours=0.3  ref=0.6", "only_a: ours=5  ref='<absent>'", "only_b: ours='<absent>'  ref=7"]
    (tmp_path / "a.json").write_text(json.dumps(a)), (tmp_path / "b.json").write_text(json.dumps(b))
    m2v7.write_config_diff(tmp_path / "a.json", tmp_path / "b.json", tmp_path / "diff.txt", ["note"])
    assert "dr.scale" in (tmp_path / "diff.txt").read_text() and "note" in (tmp_path / "diff.txt").read_text()
