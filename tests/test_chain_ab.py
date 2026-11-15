"""A1-A2-B1-B2 chain: stage definitions, lateral metric / bearing knob, gate specs, chain logic on synthetic summaries, DR ramp."""

from __future__ import annotations

import numpy as np
import pytest

from petctl.rl.curriculum import chain_ab as ca
from petctl.rl.curriculum.runner import build_config
from petctl.rl.curriculum.stages import CHAIN_STAGES, get_stage
from petctl.rl.env import PetEnv


def test_stage_definitions() -> None:
    """Frozen thresholds, hold mode, shared observation block, terrain only in B, gates at n=100 / dr 0.6."""
    assert CHAIN_STAGES == ("flat_nav", "flat_omni", "terrain_easy", "terrain_full")
    thr = {"flat_nav": 0.80, "flat_omni": 0.70, "terrain_easy": 0.75, "terrain_full": 0.70}
    for n in CHAIN_STAGES:
        st = get_stage(n)
        assert st.action_mode == "hold" and st.task == "goal_reach" and st.n_envs == 10 and st.dr_scale == 0.3
        assert st.gate.min_success == thr[n] and st.gate.episodes == 100 and st.gate.dr_scale == 0.6
        tp = st.task_params
        assert tp["delta_frame"] == tp["delta_dr"] == tp["reply_dr"] == tp["touch_long"] == tp["capacitive_obs"] == tp["bus_model"] == tp["bus_penalty"] == 1.0
        assert (tp["success_bonus"], tp["speed_gain"], tp["time_penalty"], tp["start_pose_rad"]) == (150.0, 3.0, 0.5, 0.6)
        assert "terrain_level_min" not in st.gate.task_params and "bus_penalty" not in st.gate.task_params
    assert (get_stage("flat_nav").task_params["goal_dist_min"], get_stage("flat_nav").task_params["goal_dist_max"]) == (0.8, 2.0)
    assert (get_stage("flat_omni").task_params["goal_dist_min"], get_stage("flat_omni").task_params["goal_dist_max"]) == (0.3, 0.8)
    assert "terrain_level" not in get_stage("flat_omni").task_params
    b1, b2 = get_stage("terrain_easy"), get_stage("terrain_full")
    assert (b1.task_params["terrain_level_min"], b1.task_params["terrain_level"], b1.gate.task_params["terrain_level"]) == (0.3, 0.6, 0.6)
    assert (b2.task_params["terrain_level_min"], b2.task_params["terrain_level"], b2.gate.task_params["terrain_level"]) == (0.6, 1.0, 1.0)
    assert (b2.task_params["goal_dist_min"], b2.task_params["goal_dist_max"]) == (1.5, 2.5)


def test_observation_layout_matches_flat_hold_p1() -> None:
    """The chain stages' observation size equals that of the L=8 flat hold stage (the P1 warm start: 139 entries) at every stage, terrain included."""
    from petctl.rl.curriculum.stages import stage_variant

    ref = PetEnv(build_config(stage_variant("m2_flat4", "hold"), {"n_modules": 8.0})).obs_builder.size
    assert ref == 139
    for n in CHAIN_STAGES:
        assert PetEnv(build_config(get_stage(n), {"n_modules": 8.0})).obs_builder.size == ref


def test_gate_specs() -> None:
    """B2 alone has the paired imu_off ablation (drop CI lower bound >= 0.15); all gates n=100 at dr 0.6."""
    for n in CHAIN_STAGES:
        sp = ca.gate_spec(n)
        assert sp.n == 100 and sp.dr_scale == 0.6 and sp.bus_model
        assert [a.name for a in sp.ablations] == (["imu_off"] if n == "terrain_full" else [])
    a = ca.gate_spec("terrain_full").ablations[0]
    assert a.overrides == {"imu_off": 1.0} and a.min_drop == 0.15
    with pytest.raises(KeyError):
        ca.gate_spec("m2_flat4")


def test_lateral_summary() -> None:
    """Median of lateral_peak_m against 0.4 m."""
    rows = [{"metrics": {"lateral_peak_m": v, "success": 0.0}} for v in (0.1, 0.5, 0.6)]
    s = ca.lateral_summary(rows)
    assert s["median_m"] == 0.5 and s["passed"] and s["n"] == 3
    assert not ca.lateral_summary([{"metrics": {"lateral_peak_m": v}} for v in (0.1, 0.2, 0.9)])["passed"]
    assert not ca.lateral_summary([])["passed"]


def test_second_measurement_folds_into_gate() -> None:
    """A failed lateral measurement fails an otherwise passing A2 gate; other stages are untouched."""
    res = {"fail_reasons": [], "passed": True, "status": "PASS", "void": False}
    out = ca.apply_second_measurement(dict(res), "flat_omni", {"passed": False, "median_m": 0.2, "min_m": 0.4})
    assert not out["passed"] and out["status"] == "FAIL" and "lateral" in out["fail_reasons"][0]
    assert ca.apply_second_measurement(dict(res), "flat_nav", {"passed": False, "median_m": 0.2, "min_m": 0.4})["passed"]
    ok = ca.apply_second_measurement(dict(res), "flat_omni", {"passed": True, "median_m": 0.5, "min_m": 0.4})
    assert ok["passed"] and ok["status"] == "PASS"


def test_lateral_env_metric_and_bearing_knob() -> None:
    """Goals at exactly +-90 deg (both signs over seeds) and a lateral metric that starts at 0 and stays non-negative."""
    cfg = build_config(get_stage("flat_omni"), {"n_modules": 8.0})
    cfg.task_params.update(ca.LATERAL_PARAMS)
    cfg.dr.scale = 0.0
    env = PetEnv(cfg)
    signs = set()
    for seed in range(6):
        env.reset(seed=seed)
        b = float(np.rad2deg(env.task.bearing0))
        assert abs(abs(b) - 90.0) < 1e-6
        signs.add(np.sign(b))
        assert env.task.metrics(env.sim)["lateral_peak_m"] == 0.0
    assert signs == {-1.0, 1.0}
    for _ in range(40):
        _, _, te, tr, info = env.step(np.zeros(env.action_space.shape, np.float32))
        if te or tr:
            break
    assert info["metrics"]["lateral_peak_m"] >= 0.0


def _gate(stage: str, attempt: int, passed: bool) -> dict:
    return {"phase": "gate", "stage": stage, "attempt": attempt, "passed": passed}


def _train(stage: str, attempt: int) -> dict:
    return {"phase": "train", "stage": stage, "attempt": attempt}


def test_chain_logic() -> None:
    """Pass -> next stage; fail -> one identical continuation; fail twice -> stop; full pass -> done; a trained run without a gate line is gated."""
    assert ca.next_action([]) == {"action": "train", "stage": "flat_nav", "attempt": 1}
    ls = [_train("flat_nav", 1)]
    assert ca.next_action(ls) == {"action": "gate", "stage": "flat_nav", "attempt": 1}
    ls += [_gate("flat_nav", 1, True)]
    assert ca.next_action(ls) == {"action": "train", "stage": "flat_omni", "attempt": 1}
    ls += [_train("flat_omni", 1), _gate("flat_omni", 1, False)]
    assert ca.next_action(ls) == {"action": "train", "stage": "flat_omni", "attempt": 2}
    ls += [_train("flat_omni", 2)]
    assert ca.next_action(ls) == {"action": "gate", "stage": "flat_omni", "attempt": 2}
    stopped = ls + [_gate("flat_omni", 2, False)]
    assert ca.next_action(stopped) == {"action": "stop", "stage": "flat_omni"}
    ok = ls + [_gate("flat_omni", 2, True)]
    assert ca.passing_attempt(ok, "flat_omni") == 2 and ca.passing_attempt(ok, "flat_nav") == 1
    assert ca.next_action(ok) == {"action": "train", "stage": "terrain_easy", "attempt": 1}
    full = ok + [_train("terrain_easy", 1), _gate("terrain_easy", 1, True), _train("terrain_full", 1), _gate("terrain_full", 1, True)]
    assert ca.next_action(full) == {"action": "done"}


def test_dr_ramp() -> None:
    """0.3 until the step target is known, then linear to 0.6 over the first half of the target."""
    assert ca.dr_at(500_000, None) == 0.3
    assert ca.dr_at(0, 1_000_000) == pytest.approx(0.3)
    assert ca.dr_at(250_000, 1_000_000) == pytest.approx(0.45)
    assert ca.dr_at(900_000, 1_000_000) == pytest.approx(0.6)


def test_env_set_dr_scale() -> None:
    """The ramp broadcast reaches the env's randomisation scale."""
    env = PetEnv(build_config(get_stage("flat_nav"), {"n_modules": 8.0}))
    env.set_dr_scale(0.55)
    assert env.cfg.dr.scale == 0.55
