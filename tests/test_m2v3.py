"""M2 v3 hug: stage definition, dense per-tick coverage, post-force / bus-peak penalties, start-mix phases, phase switch + stop rule, driver order, 20k-step smoke run."""
from __future__ import annotations

import dataclasses
import importlib.util
import json
import os
import sys
from pathlib import Path

import numpy as np
import pytest

pytest.importorskip("mujoco")

from petctl.rl.capacitive import PostRead  # noqa: E402
from petctl.rl.config import DEFAULT_URDF_DIR  # noqa: E402
from petctl.rl.curriculum.runner import build_config  # noqa: E402
from petctl.rl.curriculum.stages import STAGES, get_stage  # noqa: E402
from petctl.rl.env import PetEnv  # noqa: E402
from petctl.rl.hug_start import HugV3PhaseCallback, v3_phase_decision  # noqa: E402

needs_urdf = pytest.mark.skipif(not os.path.isdir(DEFAULT_URDF_DIR), reason="Fusion URDF export not present")


def test_stage_definition() -> None:
    """m2_hug3 carries exactly the ordered parameters; earlier stages are untouched; only m2_hug3 sets log_std_init."""
    s = get_stage("m2_hug3")
    tp = s.task_params
    assert (tp["coverage_weight"], tp["coverage_ratchet"], tp["hold_weight"], tp["hug_bonus"], tp["post_force_penalty"], tp["bus_peak_penalty"]) == (1.0, 5.0, 3.0, 300.0, 0.02, 2.0)
    assert (tp["approach_weight"], tp["time_penalty"], tp["progress_weight"], tp["joint_acc_weight"], tp["action_rate_weight"], tp["tilt_weight"], tp["yaw_weight"]) == (200.0, 0.2, 0.0, 0.0, 0.02, 0.0, 0.0)
    assert (tp["squeeze_weight"], tp["speed_gain"], tp["bus_penalty"], tp["hug_v3"], tp["hug_mix_v3"]) == (0.0, 1.0, 1.0, 1.0, 1.0)
    assert (s.task, s.episode_s, s.steps, s.ent_coef, s.log_std_init, s.dr_scale) == ("goal_hug", 30.0, 3_000_000, 0.0, -1.6, 0.6)
    assert s.gate == get_stage("m2_hug2").gate
    assert [n for n, st in STAGES.items() if st.log_std_init is not None] == ["m2_hug3"]
    h2 = get_stage("m2_hug2").task_params
    assert h2["coverage_weight"] == 30.0 and "hug_v3" not in h2 and "bus_peak_penalty" not in h2 and get_stage("m2_hug2").ent_coef == 0.005


def test_phase_decision_and_stop_rule() -> None:
    """Early switch at rolling mean > 0.5 (full window); at 1M steps: stop if < 0.3, else switch; nothing once switched."""
    assert v3_phase_decision([0.6] * 200, 100_000, False) == "switch"
    assert v3_phase_decision([0.6] * 50, 100_000, False) == "none"                      # window not full: no early switch
    assert v3_phase_decision([0.4] * 200, 999_999, False) == "none"
    assert v3_phase_decision([0.4] * 200, 1_000_000, False) == "switch"
    assert v3_phase_decision([0.29] * 200, 1_000_000, False) == "stop"
    assert v3_phase_decision([], 1_000_000, False) == "stop"
    assert v3_phase_decision([0.0] * 200, 2_000_000, True) == "none"


def test_phase_callback_switch_then_no_stop(tmp_path: Path) -> None:
    """The callback writes the flag on an early switch and then never stops; without a switch it stops at phase1_steps."""
    class _M:
        num_timesteps = 5000

    def run(duty: float, steps: int, phase1: int) -> HugV3PhaseCallback:
        cb = HugV3PhaseCallback(tmp_path / f"f{duty}{steps}", phase1_steps=phase1, window=10)
        cb.model, cb.num_timesteps = _M(), steps
        cb.locals = {"dones": np.ones(10, bool), "infos": [{"metrics": {"hold_duty": duty}}] * 10}
        cb._on_step()
        return cb
    sw = run(0.8, 5000, 100_000)
    assert sw.switched_at == 5000 and (tmp_path / "f0.85000").exists() and not sw.stopped
    st = run(0.1, 100_000, 100_000)
    assert st.stopped and st.checked_at == 100_000 and st.switched_at is None


@needs_urdf
def test_dense_coverage_penalties_and_hold_terms(monkeypatch: pytest.MonkeyPatch) -> None:
    """Coil start: per-tick coverage = 1 x run/H EVERY tick (not once), ratchet paid once per new peak module, hold flat 3 on hug ticks, penalties as specified."""
    cfg = build_config(get_stage("m2_hug3"), {"n_modules": 8.0})
    cfg.task_params.update({"hug_force_mode": 1.0, "hug_perturb_scale": 0.0})
    env = PetEnv(cfg)
    env.reset(seed=3)
    z = np.zeros(7, np.float32)
    rows = []
    for _ in range(40):
        _, _, _, _, info = env.step(z)
        rows.append(info["terms"])
    H = env.task._hug.h
    run = env.task._hug.last.hug_modules
    assert run >= 1
    cov = [t["coverage"] for t in rows]
    assert cov[-1] == pytest.approx(1.0 * run / H) and cov[-2] == pytest.approx(cov[-1])          # paid every tick
    assert sum(1 for t in rows if t["coverage_ratchet"] > 0) <= 8 and sum(t["coverage_ratchet"] for t in rows) == pytest.approx(5.0 * env.task._hug_peak)
    hug_rows = [t for t in rows if t["hold"] != 0.0]
    if hug_rows:
        assert all(t["hold"] == pytest.approx(3.0) for t in hug_rows)
    # post-force penalty: 0.02 per N of TOTAL force above 5 N
    probe = env.task._hug_probe
    base = probe.read()
    for total, expect in ((4.0, 0.0), (25.0, -0.02 * 20.0)):
        f = np.zeros_like(base.force)
        f[0, 0] = total
        monkeypatch.setattr(probe, "read", lambda f=f: PostRead(base.touch, f, base.any_contact, base.penetration, base.face_dist))
        _, terms = env.task.hug_step(env.sim)
        assert terms["post_force"] == pytest.approx(expect)
    # bus-peak penalty: 2 x (demand - 10 A) per tick, on the DEMAND (before the clamp); nothing below 10 A
    real = env.bus.end_tick
    for peak, expect in ((9.0, 0.0), (13.5, -2.0 * 3.5)):
        monkeypatch.setattr(env.bus, "end_tick", lambda p=peak: {**real(), "peak": p})
        _, _, _, _, info = env.step(z)
        assert info["terms"]["bus_peak"] == pytest.approx(expect)


@needs_urdf
def test_start_mix_phases() -> None:
    """Phase 1: coil/half only (no straight), perturbation +-0.10 rad; phase 2: ~40 % straight, +-0.20 rad."""
    cfg = build_config(get_stage("m2_hug3"), {"n_modules": 8.0})
    env = PetEnv(cfg)
    modes = []
    for s in range(60):
        env.reset(seed=s)
        modes.append(env.task._start_mode)
    assert "straight" not in modes and modes.count("coil") > modes.count("half") > 0
    cfg2 = build_config(get_stage("m2_hug3"), {"n_modules": 8.0})
    cfg2.task_params["hug_mix_switched"] = 1.0
    env2 = PetEnv(cfg2)
    m2 = []
    for s in range(60):
        env2.reset(seed=s)
        m2.append(env2.task._start_mode)
    assert 15 <= m2.count("straight") <= 35
    # perturbation scale reaches arrange_coil
    import petctl.rl.tasks.goal_hug as gh
    seen: list[float] = []
    orig = gh.arrange_coil
    gh.arrange_coil = lambda sim, rng, mode, pert=1.0: (seen.append(pert), orig(sim, rng, mode, pert))[1]
    try:
        cfg.task_params["hug_force_mode"] = 1.0
        PetEnv(cfg).reset(seed=1)
        cfg2.task_params["hug_force_mode"] = 1.0
        PetEnv(cfg2).reset(seed=1)
    finally:
        gh.arrange_coil = orig
    assert seen == [0.5, 1.0]


def _driver():
    spec = importlib.util.spec_from_file_location("m2v3_driver", Path(__file__).resolve().parents[1] / "scripts" / "rl" / "m2v3_driver.py")
    mod = importlib.util.module_from_spec(spec)
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts" / "rl"))
    spec.loader.exec_module(mod)
    return mod


def test_driver_order(tmp_path: Path) -> None:
    """(a) L=10 seed0 no warm start, (b) L=8 seed0 warm start, (c) seed1 on the held length, then stop; nothing held -> None."""
    d = _driver()

    def add(L: int, seed: int, success: float, lo: float, duty: float) -> None:
        with open(tmp_path / "summary.jsonl", "a") as f:
            f.write(json.dumps({"L": L, "task": "m2_hug3", "seed": seed, "success": success, "wilson_lo": lo, "hold_duty_median": duty}) + "\n")
    assert d.next_job(tmp_path) == {"L": 10, "seed": 0, "init": None}
    add(10, 0, 0.0, 0.0, 0.1)
    assert d.next_job(tmp_path) == {"L": 8, "seed": 0, "init": d.PRIOR_FLAT8}
    add(8, 0, 0.0, 0.0, 0.2)
    assert d.next_job(tmp_path) is None                                                # neither held
    add(8, 0, 0.4, 0.3, 0.9)                                                            # (a later duplicate line is fine: L=8 now held)
    assert d.next_job(tmp_path) == {"L": 8, "seed": 1, "init": d.PRIOR_FLAT8}
    add(8, 1, 0.3, 0.2, 0.8)
    assert d.next_job(tmp_path) is None
    assert not d.held({"wilson_lo": 0.2, "hold_duty_median": 0.4}) and not d.held({"wilson_lo": 0.0, "hold_duty_median": 0.9})


@needs_urdf
def test_smoke_20k_steps(tmp_path: Path) -> None:
    """A 20k-step m2_hug3 training run (low log-std applied, ent 0, phases + stop rule wired) finishes and writes its artefacts."""
    from petctl.rl.train import train
    os.environ.update({"PETCTL_GAMMA": "0.997", "PETCTL_NSTEPS": "256", "PETCTL_GAE": "0.97"})
    st = dataclasses.replace(get_stage("m2_hug3"))
    cfg = build_config(st, {"n_modules": 8.0})
    flag = tmp_path / "flag"
    os.environ["PETCTL_HUG_MIX_FLAG"] = str(flag)
    try:
        cb = HugV3PhaseCallback(flag, phase1_steps=10**9)
        train(cfg, 20_000, 2, tmp_path / "run", 0, "ppo", None, st.ent_coef, None, video=False, asymmetric=True, extra_callbacks=[cb], log_std_init=st.log_std_init)
    finally:
        os.environ.pop("PETCTL_HUG_MIX_FLAG", None)
    import stable_baselines3 as sb3
    m = sb3.PPO.load(str(tmp_path / "run" / "model"), device="cpu")
    assert (tmp_path / "run" / "policy.npz").exists() and not cb.stopped
    assert abs(float(m.policy.log_std.mean()) - (-1.6)) < 0.2
