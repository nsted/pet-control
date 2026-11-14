"""M2 v6: delta actions (env vs GaitPolicy, obs append-only), stages, hug start buffer (fill / FIFO / restore / start mix), stall stop, driver go / no-go logic and budget rule."""
from __future__ import annotations

import importlib.util
import json
import os
import sys
import time
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

pytest.importorskip("mujoco")

from petctl.rl.config import DEFAULT_URDF_DIR, DELTA_LIMIT_RAD, DELTA_STEP_RAD, EnvConfig  # noqa: E402
from petctl.rl.curriculum import m2v6  # noqa: E402
from petctl.rl.curriculum.runner import build_config  # noqa: E402
from petctl.rl.curriculum.stages import STAGES, get_stage  # noqa: E402
from petctl.rl.env import PetEnv  # noqa: E402
from petctl.rl.hug_buffer import BUFFER_ENV, MIN_STATES, HugStartBuffer, make_entry  # noqa: E402
from petctl.rl.obs import ObsBuilder  # noqa: E402
from petctl.rl.policy import POLICY_FILE, GaitPolicy  # noqa: E402
from petctl.rl.target_shapes import HELD_OUT_NAMES  # noqa: E402

needs_urdf = pytest.mark.skipif(not os.path.isdir(DEFAULT_URDF_DIR), reason="Fusion URDF export not present")
ROOT = Path(__file__).resolve().parents[1]


def _fake_run(tmp: Path, cfg: EnvConfig) -> Path:
    rng = np.random.default_rng(0)
    n_obs, nj = PetEnv(cfg).observation_space.shape[0], cfg.n_joints
    sizes = [n_obs, 16, 16, nj]
    arrs: dict[str, np.ndarray] = {"n_layers": np.array(3), "obs_mean": np.zeros(n_obs), "obs_var": np.ones(n_obs), "clip_obs": np.array(10.0), "eps": np.array(1e-8), "n_joints": np.array(nj)}
    for i in range(3):
        arrs[f"w{i}"] = rng.normal(0, 0.8, (sizes[i + 1], sizes[i]))
        arrs[f"b{i}"] = np.zeros(sizes[i + 1])
    np.savez(tmp / POLICY_FILE, **arrs)
    (tmp / "config.json").write_text(json.dumps(cfg.to_dict()))
    return tmp


# ---------------------------------------------------------------- delta actions
def test_delta_step_clip_and_slew() -> None:
    """+1 actions raise the target by 0.25 rad per tick up to the +-1.2 rad clip; no EMA; abs mode unchanged."""
    cfg = EnvConfig(task="goal_reach", n_modules=8, episode_s=5.0, action_mode="delta")
    cfg.dr.scale = 0.0
    env = PetEnv(cfg)
    env.reset(seed=0)
    seq = []
    for _ in range(7):
        env.step(np.ones(7))
        seq.append(float(env._tgt[0]))
    assert np.allclose(seq[:4], [0.25, 0.5, 0.75, 1.0]) and np.allclose(seq[4:], DELTA_LIMIT_RAD)
    env.step(-np.ones(7))
    assert np.isclose(env._tgt[0], DELTA_LIMIT_RAD - DELTA_STEP_RAD)
    with pytest.raises(ValueError):
        EnvConfig(action_mode="nope")


@needs_urdf
def test_delta_env_matches_gait_policy_and_obs_is_append_only(tmp_path: Path) -> None:
    """GaitPolicy.step reproduces the env's delta targets on a recorded sequence; the target block is the last obs entry; delta obs = abs obs + the appended target vector."""
    cfg = EnvConfig(task="goal_reach", n_modules=8, episode_s=4.0, action_mode="delta")
    cfg.dr.scale = 0.0
    nj = cfg.n_joints
    cfg_abs = EnvConfig.from_dict({**cfg.to_dict(), "action_mode": "abs"})
    assert ObsBuilder(cfg).size == ObsBuilder(cfg_abs).size + nj and ObsBuilder(cfg).blocks()[-1] == ("target", nj) and ObsBuilder(cfg_abs).blocks() == ObsBuilder(cfg).blocks()[:-1]
    pol = GaitPolicy(_fake_run(tmp_path, cfg))
    env, env_abs = PetEnv(pol.cfg), PetEnv(cfg_abs)
    obs, _ = env.reset(seed=3)
    obs_abs, _ = env_abs.reset(seed=3)
    assert np.allclose(obs[:-nj], obs_abs, atol=1e-6) and np.allclose(obs[-nj:], 0.0)
    pol.reset(env._snapshot())
    rec: list[np.ndarray] = []
    orig = pol.action
    pol.action = lambda o: rec.append(orig(o)) or rec[-1]                      # type: ignore[method-assign]
    for k in range(25):
        tgt = pol.step(env._snapshot())
        obs, *_ = env.step(rec[-1])
        assert np.allclose(tgt, env._tgt) and np.allclose(obs[-nj:], env._tgt / DELTA_LIMIT_RAD, atol=1e-6)
    assert np.abs(env._tgt).max() > 0.3                                         # the sequence actually moved the targets


def test_stage_definitions() -> None:
    """m2_flat3 / m2_hug6 as ordered; every older stage stays absolute; r11 -> r9 in the held-out set."""
    f, h = get_stage("m2_flat3"), get_stage("m2_hug6")
    assert (f.task, f.action_mode, f.episode_s, f.ent_coef, f.dr_scale) == ("goal_reach", "delta", 20.0, 0.005, 0.6)
    assert (f.task_params["goal_dist_min"], f.task_params["goal_dist_max"]) == (0.6, 1.5) and f.gate.episode_s == 20.0
    assert (h.task, h.action_mode, h.episode_s, h.ent_coef, h.log_std_init, h.dr_scale) == ("goal_hug", "delta", 20.0, 0.002, -1.2, 0.6)
    tp = h.task_params
    assert (tp["hug_gap_min"], tp["hug_gap_max"], tp["bearing_deg"], tp["start_pose_rad"], tp["hug_stall_s"], tp["hug_buffer"]) == (0.10, 0.40, 90.0, 0.4, 8.0, 1.0)
    assert "hug_start_mix" not in tp and "hug_mix_v3" not in tp                  # no scripted coil / half starts
    assert h.gate.task_params["hug_buffer"] == 0.0 and h.gate.task_params["target_heldout"] == 1.0 and h.gate.episode_s == 20.0
    assert [n for n, s in STAGES.items() if s.action_mode == "delta"] == ["m2_flat3", "m2_hug6"]
    assert get_stage("m2_hug5").task_params["hug_start_mix"] == 1.0              # old stages keep the scripted starts
    assert "cyl_r9" in HELD_OUT_NAMES and "cyl_r11" not in HELD_OUT_NAMES
    cfg = build_config(h, {"n_modules": 10.0, "dr_scale": 0.3})
    assert cfg.action_mode == "delta" and cfg.dr.scale == 0.3 and build_config(get_stage("m2_hug5"), {}).action_mode == "abs"


# ---------------------------------------------------------------- start buffer
def _entry(i: int = 0) -> dict:
    return make_entry(np.full(9, 0.1 * i), np.full(9, 0.2), np.array([0, 0, 0.04, 1, 0, 0, 0.0]), (1.0, 2.0), ("cyl", 0.05, 0.0, 0.0, 0.3), 5, 120.0 + i)


def test_buffer_fill_fifo_and_sampler(tmp_path: Path) -> None:
    """Entries append, the oldest drop beyond the capacity; the sampler is silent below 20 states and then draws ~30 % of the starts."""
    b = HugStartBuffer(tmp_path / "b.json", capacity=25)
    assert len(b) == 0 and b.sample(np.random.default_rng(0)) is None
    for i in range(19):
        b.add(_entry(i))
    rng = np.random.default_rng(1)
    assert len(b) == 19 and all(b.sample(rng) is None for _ in range(200))      # < MIN_STATES: never
    for i in range(19, 40):
        b.add(_entry(i))
    es = b.load()
    assert len(es) == 25 and es[0]["span"] == 120.0 + 15 and es[-1]["span"] == 120.0 + 39      # FIFO
    hits = sum(b.sample(rng) is not None for _ in range(2000))
    assert 0.25 < hits / 2000 < 0.35 and MIN_STATES == 20
    assert sorted(p.name for p in tmp_path.iterdir() if "tmp" in p.name) == []        # atomic writes leave no temp files


@needs_urdf
def test_buffer_store_and_restore_reproduces_run_and_span(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    """A state stored during an episode (scripted coil used here only to reach a hug) is restored by a buffer start with the same run / span, shape and targets; knob off = no buffer."""
    monkeypatch.setenv(BUFFER_ENV, str(tmp_path / "b.json"))
    cfg = build_config(get_stage("m2_hug6"), {"n_modules": 10.0})
    cfg.task_params.update({"hug_start_mix": 1.0, "hug_force_mode": 1.0, "hug_buffer_min_run": 2.0, "hug_buffer_min_span": 30.0, "hug_buffer_min": 1.0, "hug_buffer_p": 0.0, "shape_phase": 1.0})
    cfg.dr.scale = 0.0
    env, nj = PetEnv(cfg), 9
    stored = []
    for seed in range(4):
        env.reset(seed=seed)
        for _ in range(20):
            env.step(np.zeros(nj))
        if env.task._stored:
            stored.append((env.task._hug.last.hug_modules, env.task._hug.last.span_deg))
    assert len(stored) >= 2 and len(HugStartBuffer.from_env()) == len(stored)
    cfg2 = build_config(get_stage("m2_hug6"), {"n_modules": 10.0})
    cfg2.task_params.update({"hug_buffer_min": 1.0, "hug_buffer_p": 1.0, "hug_buffer_perturb": 0.0, "hug_buffer_min_run": 99.0})
    cfg2.dr.scale = 0.0
    env2 = PetEnv(cfg2)
    for s in range(3):
        env2.reset(seed=s)
        t, b = env2.task, env2.task._bstart
        assert b is not None and env2.target_shape.kind == b["shape"][0] and np.allclose(env2.sim.p.post, b["post"])
        assert np.allclose(env2._tgt, b["tgt"])
        for _ in range(8):
            env2.step(np.zeros(nj))
        assert t._hug.last.hug_modules == b["run"] and abs(t._hug.last.span_deg - b["span"]) < 5.0
    monkeypatch.delenv(BUFFER_ENV)
    env3 = PetEnv(cfg2)
    env3.reset(seed=0)
    assert env3.task._bstart is None and env3.task._buffer is None


@needs_urdf
def test_straight_start_gap_bearing_and_stall_stop() -> None:
    """Straight starts put the head 0.10-0.40 m from the object surface; a policy that never approaches is stopped after 8 s with the time penalty charged once."""
    cfg = build_config(get_stage("m2_hug6"), {"n_modules": 10.0})
    cfg.dr.scale = 0.0
    env = PetEnv(cfg)
    for seed in range(6):
        env.reset(seed=seed)
        sh = env.target_shape
        from petctl.sim.shapes import shape_of
        gap = float(shape_of(sh).dist_xy(env.sim.head_pos()[None, :2], np.array(env.sim.p.post))[0])
        assert 0.05 < gap < 0.50, gap                                              # surface gap (settling after the random pose moves the head a little)
        assert abs(env.task._hug.last.span_deg if env.task._hug.last else 0.0) < 1.0
    env.reset(seed=0)
    n, end = 0, None
    while True:
        _, r, te, tr, info = env.step(np.zeros(9))
        n += 1
        if te or tr:
            end = info
            break
    assert te and n < 8.5 * cfg.control_hz + 5 and n > 7.5 * cfg.control_hz and info["metrics"]["stalled"] == 1.0 and info["metrics"]["term_hug_stall"] < 0


# ---------------------------------------------------------------- driver logic
def test_budget_rule_and_go_no_go() -> None:
    """steps = 0.75 x budget x sps capped; n_envs from the benchmark; every go / no-go with synthetic numbers."""
    assert m2v6.plan_steps(1000.0, 600.0) == 450_000 and m2v6.plan_steps(5000.0, 2700.0) == m2v6.MAX_STEPS
    assert m2v6.projected_wall_s(m2v6.plan_steps(1000.0, 600.0), 1000.0) <= 600.0 * 0.75 + 1e-6
    assert m2v6.best_n_envs({"a": {"6": 4.0, "8": 5.0, "10": 5.5}, "b": {"6": 5.0, "8": 6.0, "10": 4.5}}) == 8
    assert m2v6.takeoff_ok([0.0] * 190 + [1.0] * 20)[0] and not m2v6.takeoff_ok([0.0] * 200)[0] and not m2v6.takeoff_ok([])[0]
    rows = [{"hug_modules": 4.0, "normal_span_deg": 100.0}] * 6 + [{"hug_modules": 5.0, "normal_span_deg": 50.0}, {"hug_modules": 2.0, "normal_span_deg": 200.0}] * 7
    fr = m2v6.hug_pilot_fraction(rows)
    assert abs(fr - 6 / 20) < 1e-9 and m2v6.hug_pilot_go(fr) and not m2v6.hug_pilot_go(m2v6.hug_pilot_fraction([{"hug_modules": 1.0}] * 100)) and m2v6.hug_pilot_go(0.05)
    assert m2v6.screen_go("flat_main", {"success": 0.55}) and not m2v6.screen_go("flat_main", {"success": 0.5})
    assert m2v6.screen_go("hug_run1", {"hold_duty_mean": 0.21}) and not m2v6.screen_go("hug_run1", {"hold_duty_mean": 0.2})
    assert m2v6.screen_go("hug_run2", {"success": 0.6}) and not m2v6.screen_go("hug_run2", {"success": 0.5})
    seeds = m2v6.screen_seeds(30, False)
    assert len({(s - 10_000) // 20 for s in seeds}) == 5 and m2v6.screen_seeds(30, True)[-1] == 10_029


def test_budget_callback_targets_and_stops() -> None:
    """After the 2-minute measurement the target follows the rule; training stops at the target, or at the wall budget."""
    cb = m2v6.BudgetCallback(600.0, 600_000, measure_s=120.0)
    cb.model = SimpleNamespace(lr_schedule=lambda f: f * 3e-4)
    cb._t0 = time.time() - 130
    cb.num_timesteps = 130_000
    assert cb._on_step() is True and abs(cb.target - 450_000) < 2000 and cb.model.lr_schedule(0.5) == pytest.approx((1 - 0.5 * 600_000 / cb.target) * 3e-4, rel=1e-3)
    assert cb.model.lr_schedule(1.0) == pytest.approx(3e-4)                      # progress 0 stays at the full rate
    cb.num_timesteps = cb.target
    assert cb._on_step() is False and cb.info["stopped_by"] == "target"
    cb2 = m2v6.BudgetCallback(100.0, 10 ** 6, measure_s=1e9)
    cb2._t0, cb2.num_timesteps = time.time() - 101, 10
    assert cb2._on_step() is False and cb2.info["stopped_by"] == "wall"


def test_wall_takeoff_callback() -> None:
    """The takeoff rule is applied once at the check time from the rolling success."""
    cb = m2v6.WallTakeoffCallback(check_s=0.0)
    cb.num_timesteps, cb._t0 = 1000, time.time() - 1
    cb.locals = {"dones": np.ones(5, bool), "infos": [{"metrics": {"success": 0.0}}] * 5}
    assert cb._on_step() is False and cb.failed and cb.rate == 0.0
    cb2 = m2v6.WallTakeoffCallback(check_s=0.0)
    cb2.num_timesteps, cb2._t0 = 1000, time.time() - 1
    cb2.locals = {"dones": np.ones(5, bool), "infos": [{"metrics": {"success": 1.0}}] * 5}
    assert cb2._on_step() is True and not cb2.failed


def _driver():
    spec = importlib.util.spec_from_file_location("m2v6_driver", ROOT / "scripts" / "rl" / "m2v6_driver.py")
    sys.path.insert(0, str(ROOT / "scripts" / "rl"))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_driver_markers_and_resume(tmp_path: Path) -> None:
    """A failed go / no-go writes STOP_<reason>; recorded go lines make a phase 'done'; a STOP marker blocks a restart."""
    d = _driver()
    assert d.done_line(tmp_path, "flat_pilot") is None
    d.record(tmp_path, {"phase": "flat_pilot", "seed": 0, "go": False})
    assert d.done_line(tmp_path, "flat_pilot") is None
    d.record(tmp_path, {"phase": "flat_pilot", "seed": 1, "go": True})
    assert d.done_line(tmp_path, "flat_pilot")["seed"] == 1
    assert d.stop(tmp_path, "hug_pilot", "x") == 0 and (tmp_path / "STOP_hug_pilot").exists()
    assert not d.finished(tmp_path)
    assert m2v6.screen_go.__doc__ and sorted(p.name for p in tmp_path.glob("STOP_*")) == ["STOP_hug_pilot"]
