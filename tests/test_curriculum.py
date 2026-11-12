"""Curriculum machinery: goal_reach task, run watcher verdicts, frozen gate, knob validation, warm start."""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pytest

pytest.importorskip("gymnasium")
pytest.importorskip("mujoco")

from petctl.rl.config import EnvConfig  # noqa: E402
from petctl.rl.curriculum.gate import run_gate  # noqa: E402
from petctl.rl.curriculum.runner import build_config, validate_overrides  # noqa: E402
from petctl.rl.curriculum.stages import STAGES, Gate, Stage  # noqa: E402
from petctl.rl.curriculum.watch import analyze  # noqa: E402
from petctl.rl.env import PetEnv  # noqa: E402
from petctl.rl.policy import POLICY_FILE  # noqa: E402


def _episodes(run: Path, successes: list[float], glitch: float = 0.0, final: bool = False) -> None:
    run.mkdir(exist_ok=True)
    with open(run / "episodes.jsonl", "w") as f:
        for i, s in enumerate(successes):
            f.write(json.dumps({"timesteps": i * 300, "return": 1.0, "distance": 0.1 * s, "success": s, "glitch": glitch}) + "\n")
    (run / "status.json").write_text(json.dumps({"timesteps": len(successes) * 300, "episodes": len(successes), "final": final}))


def _series(rates: list[float], window: int = 20) -> list[float]:
    out: list[float] = []
    for r in rates:
        k = int(round(r * window))
        out += [1.0] * k + [0.0] * (window - k)
    return out


def test_goal_reach_obs_and_metrics() -> None:
    env = PetEnv(EnvConfig(task="goal_reach", episode_s=1.0, task_params={"bearing_deg": 90.0}))
    obs, _ = env.reset(seed=3)
    assert obs.shape == env.observation_space.shape
    u, d = obs[-4:-1], obs[-1]
    assert abs(np.linalg.norm(u) - 1.0) < 1e-5 and 0.2 < d < 0.4        # unit goal direction, default 0.25-0.35 m
    for _ in range(5):
        _, _, _, _, info = env.step(np.zeros(7))
    m = info["metrics"]
    assert {"distance", "success", "glitch", "time_to_goal_s", "dist0"} <= set(m) and m["success"] == 0.0
    assert abs(m["distance"]) < 0.01                                      # zero action barely moves the robot


def test_goal_reach_success_terminates() -> None:
    env = PetEnv(EnvConfig(task="goal_reach", task_params={"goal_dist_min": 0.01, "goal_dist_max": 0.02}))   # inside the 5 cm radius at once
    env.reset(seed=0)
    _, r, term, _, info = env.step(np.zeros(7))
    assert term and info["metrics"]["success"] == 1.0 and r > 5.0
    m = info["metrics"]
    assert -1e-3 <= m["distance"] <= m["dist0"] + 1e-6                      # progress is counted once, never more than the goal distance


def test_watch_verdicts(tmp_path: Path) -> None:
    assert analyze(tmp_path / "none", 20).state == "warming_up"
    _episodes(tmp_path / "a", _series([0.1, 0.2, 0.3, 0.45]))
    assert analyze(tmp_path / "a", 20).state == "improving"
    assert analyze(tmp_path / "a", 20, pass_score=0.4).state == "passing"
    _episodes(tmp_path / "b", _series([0.3, 0.3, 0.3, 0.3, 0.3, 0.3]))
    assert analyze(tmp_path / "b", 20).state == "plateau"
    _episodes(tmp_path / "c", _series([0.2, 0.5, 0.6, 0.1]))
    assert analyze(tmp_path / "c", 20).state == "collapse"
    _episodes(tmp_path / "d", _series([0.2, 0.3, 0.4]), glitch=1.0)
    assert analyze(tmp_path / "d", 20).state == "glitch"
    _episodes(tmp_path / "e", _series([0.2, 0.3, 0.4]), final=True)
    assert analyze(tmp_path / "e", 20).state == "finished"


def test_override_validation() -> None:
    st = STAGES["s1_goal_ahead"]
    validate_overrides(st, {"ent_coef": 0.01, "goal_dist_max": 1.8})
    with pytest.raises(ValueError):
        validate_overrides(st, {"ent_coef": 0.5})               # out of range
    with pytest.raises(ValueError):
        validate_overrides(st, {"action_scale": 1.0})           # not a declared knob
    cfg = build_config(st, {"goal_dist_max": 1.8, "dr_scale": 0.5})
    assert cfg.task_params["goal_dist_max"] == 1.8 and cfg.dr.scale == 0.5 and cfg.dr.cap_nm == (0.5, 0.5) and cfg.episode_s == 30.0 and cfg.action_scale_rad == 1.0 and cfg.action_ema == 0.3


def _fake_policy_run(tmp: Path, cfg: EnvConfig, scale: float = 0.3) -> Path:
    rng = np.random.default_rng(0)
    n = PetEnv(cfg).observation_space.shape[0]
    sizes = [n, 16, 7]
    arrs: dict[str, np.ndarray] = {"n_layers": np.array(2), "obs_mean": np.zeros(n), "obs_var": np.ones(n), "clip_obs": np.array(10.0), "eps": np.array(1e-8)}
    for i in range(2):
        arrs[f"w{i}"], arrs[f"b{i}"] = rng.normal(0, scale, (sizes[i + 1], sizes[i])), np.zeros(sizes[i + 1])
    np.savez(tmp / POLICY_FILE, **arrs)
    (tmp / "config.json").write_text(json.dumps(cfg.to_dict()))
    return tmp


def test_gate_pass_and_fail(tmp_path: Path) -> None:
    run = _fake_policy_run(tmp_path, EnvConfig(task="goal_reach", episode_s=1.0))
    easy = Stage("t_easy", "goal_reach", "goal already reached", {}, Gate({"goal_dist_min": 0.01, "goal_dist_max": 0.02}, episodes=3, min_success=0.9))
    hard = Stage("t_hard", "goal_reach", "unreachable in 1 s", {}, Gate({"goal_dist_min": 0.5, "goal_dist_max": 0.6}, episodes=3, min_success=0.5))
    res = run_gate(run, easy)
    assert res["passed"] and res["robust"]["success_rate"] == 1.0 and (run / "gate_t_easy.json").exists()
    bad = run_gate(run, hard)
    assert not bad["passed"] and bad["fail_reasons"]


def test_warm_start_pads_new_observation_columns(tmp_path: Path) -> None:
    pytest.importorskip("stable_baselines3")
    from stable_baselines3 import PPO
    from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize

    from petctl.rl.train import make_env_fn, warm_start

    def make(task: str) -> tuple[PPO, VecNormalize]:
        v = VecNormalize(DummyVecEnv([make_env_fn(EnvConfig(task=task, episode_s=1.0), 0, 0)]))
        return PPO("MlpPolicy", v, n_steps=64, batch_size=64, policy_kwargs={"net_arch": [16, 16]}), v

    src, sv = make("walk_straight")
    src.save(str(tmp_path / "model"))
    sv.save(str(tmp_path / "vecnormalize.pkl"))
    dst, dv = make("goal_reach")
    counts = warm_start(dst, dv, tmp_path)
    assert counts["adapted"] == 2 and counts["skipped"] == 0
    old_w = src.policy.state_dict()["mlp_extractor.policy_net.0.weight"].numpy()
    new_w = dst.policy.state_dict()["mlp_extractor.policy_net.0.weight"].numpy()
    assert np.allclose(new_w[:, :old_w.shape[1]], old_w) and np.all(new_w[:, old_w.shape[1]:] == 0)


def test_training_stops_early_when_gate_passes(tmp_path: Path) -> None:
    pytest.importorskip("stable_baselines3")
    from petctl.rl.train import train

    easy = Stage("t_stop", "goal_reach", "goal inside the success radius at reset", {"goal_dist_min": 0.01, "goal_dist_max": 0.02},
                 Gate({"goal_dist_min": 0.01, "goal_dist_max": 0.02}, episodes=3, min_success=0.8))
    cfg = EnvConfig(task="goal_reach", episode_s=1.0, task_params=dict(easy.task_params))
    out = train(cfg, 200_000, 1, tmp_path / "run", video=False, stop_when_gate_passes=easy)
    status = json.loads((out / "status.json").read_text())
    gate = json.loads((out / "gate_t_stop.json").read_text())
    assert gate["passed"] and status["final"] and status["timesteps"] < 20_000


def test_goal_reach_standing_still_earns_no_progress_reward() -> None:
    """The progress window must start on the CoM (not the head), else standing still is penalised by a constant offset."""
    env = PetEnv(EnvConfig(task="goal_reach"))
    env.reset(seed=0)
    prog = [env.step(np.zeros(7))[4]["terms"]["progress"] for _ in range(10)]
    assert max(abs(x) for x in prog) < 0.5


def test_multi_goal_sequence_and_unlimited_mode() -> None:
    near = {"goal_dist_min": 0.01, "goal_dist_max": 0.02}
    env = PetEnv(EnvConfig(task="goal_reach", task_params={**near, "n_goals": 2}))
    env.reset(seed=0)
    _, _, term, _, info = env.step(np.zeros(7))
    assert not term and info["metrics"]["goals_reached"] == 1.0 and info["metrics"]["success"] == 0.0      # next goal spawned, episode continues
    for _ in range(3):
        _, _, term, _, info = env.step(np.zeros(7))
        if term:
            break
    assert term and info["metrics"]["goals_reached"] == 2.0 and info["metrics"]["success"] == 1.0
    env = PetEnv(EnvConfig(task="goal_reach", task_params={**near, "n_goals": 0}))                     # unlimited: never ends on arrival
    env.reset(seed=0)
    for _ in range(6):
        _, _, term, _, info = env.step(np.zeros(7))
    assert not term and info["metrics"]["goals_reached"] >= 1.0


def test_mid_route_redirect_and_set_goal() -> None:
    env = PetEnv(EnvConfig(task="goal_reach", task_params={"goal_dist_min": 0.8, "goal_dist_max": 0.9, "bearing_deg": 0.0, "redirect_prob": 1.0,
                                                        "redirect_min_s": 0.2, "redirect_max_s": 0.2, "bearing_deg": 180.0}))
    env.reset(seed=1)
    g0 = env.task.goal.copy()
    for _ in range(6):
        env.step(np.zeros(7))
    assert np.linalg.norm(env.task.goal - g0) > 0.01                       # the goal moved mid-route
    u_before = env.step(np.zeros(7))[0][-4:-1].copy()
    env.task.set_goal(env.sim, np.array([-1.0, 0.5]))                      # user click
    assert np.allclose(env.task.goal[:2], [-1.0, 0.5])
    u_after = env.step(np.zeros(7))[0][-4:-1]
    assert np.linalg.norm(u_after - u_before) > 0.05                       # the observed goal direction changed


def test_random_start_pose_never_starts_toppled() -> None:
    cfg = EnvConfig(task="goal_reach", task_params={"start_pose_rad": 1.0})
    cfg.dr.cap_nm = (0.5, 0.5)                                              # as in the curriculum stages
    env = PetEnv(cfg)
    starts: list[np.ndarray] = []
    for seed in range(12):
        env.reset(seed=seed)
        assert env._up_dot() >= env.cfg.flip_up_dot
        assert np.allclose(env.sim.joint_pos(), env._tgt)                  # the first action blends from the settled pose, no snap
        starts.append(env.sim.joint_pos().copy())
    assert np.std(np.array(starts), axis=0).min() > 0.05                    # the start pose really varies between episodes


def test_success_is_judged_at_the_head_not_the_centre_of_mass() -> None:
    env = PetEnv(EnvConfig(task="goal_reach", episode_s=2.0, task_params={"goal_dist_min": 0.5, "goal_dist_max": 0.5}))
    env.reset(seed=0)
    t = env.task
    com, head = t._com(env.sim), t._head(env.sim)
    assert np.linalg.norm((com - head)[:2]) > 0.08                         # the head really is well away from the CoM
    t.set_goal(env.sim, com[:2])                                           # goal exactly at the CoM: the head is >5 cm away -> not reached
    _, _, term, _, info = env.step(np.zeros(7))
    assert not term and info["metrics"]["success"] == 0.0
    t.set_goal(env.sim, t._head(env.sim)[:2])                              # goal exactly at the head -> reached
    _, _, term, _, info = env.step(np.zeros(7))
    assert term and info["metrics"]["success"] == 1.0


def test_goal_sequence_layout_lookahead_and_order() -> None:
    near = {"goal_dist_min": 0.01, "goal_dist_max": 0.02, "bearing_deg": 180.0, "seq_bearing_deg": 180.0}
    env = PetEnv(EnvConfig(task="goal_sequence", episode_s=3.0, task_params={**near, "n_goals": 3}))
    obs, _ = env.reset(seed=0)
    t = env.task
    assert len(t.targets) == 3 and obs.shape == env.observation_space.shape
    legs = [np.linalg.norm((b - a)[:2]) for a, b in zip([t._head(env.sim)] + t.targets, t.targets)]
    assert all(0.0099 <= d <= 0.0201 for d in legs[1:])                       # later targets are placed relative to the previous target
    assert np.any(obs[-4:] != 0.0) and len(t.markers(env.sim)) == 3          # a next target is visible and all targets are drawn
    reached = 0.0
    for _ in range(12):
        obs, _, term, _, info = env.step(np.zeros(7))
        reached = info["metrics"]["goals_reached"]
        if term:
            break
    assert term and reached == 3.0 and info["metrics"]["success"] == 1.0     # visited t1, t2, t3 in order
    assert np.all(obs[-4:] == 0.0)                                            # no lookahead after the last target


def test_goal_sequence_layout_is_random_per_pass_and_fixed_within_one() -> None:
    env = PetEnv(EnvConfig(task="goal_sequence", episode_s=2.0, task_params={"goal_dist_min": 0.4, "goal_dist_max": 0.7, "n_goals": 4}))
    env.reset(seed=1)
    a = [x.copy() for x in env.task.targets]
    env.step(np.zeros(7)); env.step(np.zeros(7))
    assert all(np.allclose(x, y) for x, y in zip(a, env.task.targets))        # the layout does not move while the robot works through it
    env.reset(seed=2)
    assert not np.allclose(a[0], env.task.targets[0])                         # a new random layout next pass


def test_automatic_retargeting_moves_the_target_repeatedly() -> None:
    cfg = EnvConfig(task="goal_sequence", episode_s=20.0, task_params={"goal_dist_min": 0.8, "goal_dist_max": 0.9, "bearing_deg": 180.0, "n_goals": 2,
                                                                       "redirect_every_s": 1.0, "redirect_min_gap_s": 0.5})
    env = PetEnv(cfg)
    obs, _ = env.reset(seed=3)
    t = env.task
    other = t.targets[1].copy()
    seen = [t.goal.copy()]
    for _ in range(200):                                                     # 10 s with a robot that stands still: the target is never reached
        obs, _, term, _, info = env.step(np.zeros(7))
        if not np.allclose(t.goal, seen[-1]):
            seen.append(t.goal.copy())
        if term:
            break
    assert info["metrics"]["redirects"] >= 2.0 and len(seen) >= 3             # the target moved several times, at random times
    assert np.allclose(t.targets[1], other)                                  # only the current target moves; the later one stays put
    assert info["metrics"]["goals_reached"] == 0.0 and np.isfinite(obs).all()
    times_ok = info["metrics"]["redirects"] <= 10.0 / 0.5                     # respects the minimum gap
    assert times_ok


def test_goal_wall_layout_and_touch_obs() -> None:
    """The wall lies across the line to the goal; the contact channels are appended to the observation and zeroed by touch_off."""
    from petctl.rl.config import EnvConfig
    from petctl.rl.env import PetEnv

    tp = {"goal_dist_min": 1.5, "goal_dist_max": 1.5, "wall_len_min": 0.5, "wall_len_max": 0.5, "wall_offset": 0.0}
    cfg = EnvConfig(task="goal_wall", task_params=dict(tp))
    env = PetEnv(cfg)
    obs, _ = env.reset(seed=3)
    (wx, wy, _, half), = env.sim.p.walls
    head = env.task._head(env.sim)
    assert 0.4 < np.hypot(wx - head[0], wy - head[1]) < 1.0 and half == 0.5          # between the head and the goal (1.5 m away)
    assert env.task.goal[0] > wx                                                      # goal is beyond the wall
    base = EnvConfig(task="goal_reach", task_params=dict(tp))
    assert obs.shape[0] == PetEnv(base).observation_space.shape[0] + 28
    for _ in range(40):
        obs, *_ = env.step(np.random.default_rng(0).uniform(-1, 1, 7))
    assert np.abs(obs[-28:-14]).max() > 0.0
    cfg_off = EnvConfig(task="goal_wall", task_params={**tp, "touch_off": 1.0})
    e2 = PetEnv(cfg_off)
    o2, _ = e2.reset(seed=3)
    for _ in range(40):
        o2, *_ = e2.step(np.random.default_rng(0).uniform(-1, 1, 7))
    assert np.all(o2[-28:] == 0.0)
