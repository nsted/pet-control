"""Regimen v2 long hug run: length-dependent held-out set, mixed shape phase, stage hug_long_01, constant learning rate, in-run checks and screen rules."""
from __future__ import annotations

import numpy as np
import pytest

from petctl.rl.curriculum import hug_long as hl
from petctl.rl.curriculum.m2v5 import gate2_spec_hug5
from petctl.rl.curriculum.stages import STAGES, get_stage
from petctl.rl.target_shapes import (GATE_SEED0, HELD_OUT_L8, HELD_OUT_L10, HELD_OUT_NAMES, held_out_names, held_out_set, heldout_group, heldout_shape, near_heldout,
                                     sample_training_shape)
from petctl.rl.train import lr_schedule


def test_length_dependent_heldout_sets() -> None:
    assert held_out_names(8) == ("cyl_r3", "cyl_r7", "square_8x8", "rect_5x14", "rounded_8x8_r2")
    assert held_out_names(10) == HELD_OUT_NAMES == ("cyl_r3", "cyl_r9", "square_8x8", "rect_5x14", "rounded_12x12_r2")
    assert held_out_set(10) is HELD_OUT_L10 and held_out_set(8) is HELD_OUT_L8 and held_out_set(12) is HELD_OUT_L10
    assert heldout_group(GATE_SEED0 + 25, 8) == "cyl_r7" and heldout_group(GATE_SEED0 + 25) == "cyl_r9" and heldout_group(GATE_SEED0 + 95, 8) == "rounded_8x8_r2"
    assert heldout_shape(1, 10_025, 8).a == pytest.approx(0.07) and heldout_shape(1, 10_025).a == pytest.approx(0.09)
    r8 = heldout_shape(4, 10_090, 8)
    assert (r8.kind, r8.a, r8.b, r8.rc) == ("rbox", 0.04, 0.04, 0.02)
    assert [gate2_spec_hug5(8).group_of(GATE_SEED0 + 20 * i) for i in range(5)] == list(held_out_names(8))
    assert gate2_spec_hug5().group_of(GATE_SEED0 + 20) == "cyl_r9" and gate2_spec_hug5(8).n == 100 and gate2_spec_hug5(8).ablations[0].name == "capacitive_off"


def test_training_excludes_the_lengths_margins() -> None:
    rng = np.random.default_rng(0)
    shapes = [sample_training_shape(rng, 2, 8) for _ in range(3000)]
    assert not any(near_heldout(s, 8) for s in shapes)
    assert any(abs(s.a - 0.09) < 0.005 for s in shapes if s.kind == "cyl")           # r 9 is trainable at L = 8 ...
    rng = np.random.default_rng(0)
    shapes10 = [sample_training_shape(rng, 2, 10) for _ in range(3000)]
    assert not any(near_heldout(s, 10) for s in shapes10)
    assert any(abs(s.a - 0.07) < 0.005 for s in shapes10 if s.kind == "cyl")        # ... and r 7 at L = 10 (it is only held out at L = 8)
    assert near_heldout(heldout_shape(1, 1, 8), 8) and not near_heldout(heldout_shape(1, 1, 8), 10)


def test_fractional_shape_phase_mixes_the_ranges() -> None:
    rng = np.random.default_rng(1)
    shapes = [sample_training_shape(rng, 1.5, 8) for _ in range(4000)]
    cyl = [s.a for s in shapes if s.kind == "cyl"]
    outside = np.mean([not (0.04 <= a <= 0.08) for a in cyl])                      # phase-1 cylinders are always r 4-8: only the full-range half can fall outside
    assert 0.2 < outside < 0.4
    assert any(a > 0.08 for a in cyl) and not any(abs(a - 0.07) < 0.01 or abs(a - 0.03) < 0.01 for a in cyl) and max(cyl) <= 0.12 and min(cyl) >= 0.02
    inside = [sample_training_shape(np.random.default_rng(i), 1.0, 8) for i in range(300)]
    assert all(0.04 <= s.a <= 0.08 for s in inside if s.kind == "cyl")      # (the r7 margin leaves r 4-6 of that range at L = 8)


def test_stage_hug_long_01_as_ordered() -> None:
    st = get_stage("hug_long_01")
    tp = st.task_params
    assert (st.task, st.action_mode, st.episode_s, st.ent_coef, st.dr_scale, st.log_std_init, st.n_envs) == ("goal_hug", "hold", 20.0, 0.002, 0.3, -1.2, 10)
    assert (tp["approach_weight"], tp["coverage_weight"], tp["hold_weight"], tp["hug_bonus"], tp["post_force_penalty"], tp["current_weight"], tp["time_penalty"], tp["action_rate_weight"],
            tp["joint_acc_weight"], tp["tilt_weight"], tp["yaw_weight"], tp["hug_h"], tp["hug_span_deg"]) == (200.0, 1.0, 10.0, 300.0, 0.02, 0.0, 0.2, 0.02, 0.0, 0.0, 0.0, 4.0, 180.0)
    assert (tp["shape_phase"], tp["hug_gap_min"], tp["hug_gap_max"], tp["bearing_deg"], tp["start_pose_rad"], tp["hug_stall_s"], tp["hug_buffer"], tp["target_family"]) == (1.5, 0.10, 0.40, 90.0, 0.4, 8.0, 1.0, 1.0)
    assert tp["bus_peak_penalty"] == 2.0 and tp["bus_penalty"] == 1.0 and "hug_buffer_p" not in tp                  # 30 % buffer starts = the default
    assert st.gate.dr_scale == 0.6 and st.gate.episode_s == 20.0 and st.gate.task_params["target_heldout"] == 1.0 and st.gate.task_params["hug_buffer"] == 0.0
    assert get_stage("m2_hug7").task_params["hold_weight"] == 3.0 and get_stage("m2_hug7").task_params["shape_phase"] == 1.0           # older stages untouched
    assert [n for n, s in STAGES.items() if s.action_mode == "hold"] == ["m2_hug7", "hug_long_01", "hug_curr_01"]


def test_constant_learning_rate(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.delenv("PETCTL_LR_CONST", raising=False)
    monkeypatch.setenv("PETCTL_LR", "1.5e-4")
    lin = lr_schedule()
    assert lin(1.0) == pytest.approx(1.5e-4) and lin(0.25) == pytest.approx(0.375e-4) and lin(0.0) == 0.0          # default: linear decay, unchanged
    assert lr_schedule(True)(0.1) == pytest.approx(1.5e-4)
    monkeypatch.setenv("PETCTL_LR_CONST", "1")
    assert lin(0.1) == pytest.approx(1.5e-4) and lr_schedule()(0.0) == pytest.approx(1.5e-4)


def _row(reach: bool, hold_s: float = 0.0, h: float = 0.8, n: float = 50.0, succ: float = 0.0) -> dict:
    return {"hug_modules": 4.0 if reach else 1.0, "normal_span_deg": 120.0 if reach else 10.0, "state_hold_s": hold_s, "h_wrap_mean": h, "h_wrap_n": n, "success": succ}


def test_check1_rule() -> None:
    good = [_row(True, 2.5, 0.8) if i % 2 else _row(False) for i in range(300)]
    ok, sig = hl.check1(good)
    assert ok and sig["frac_hold"] == 1.0 and sig["h_wrap"] == pytest.approx(0.8)
    assert not hl.check1([_row(True, 2.5, 0.7)] * 300)[0]                                                            # h over wrap ticks below 0.75
    mostly_short = [_row(True, 2.5, 0.9)] * 19 + [_row(True, 0.5, 0.9)] * 81                                         # frac_hold 0.19
    assert not hl.check1(mostly_short)[0]
    assert hl.check1([_row(True, 2.5, 0.9)] * 20 + [_row(True, 0.5, 0.9)] * 80)[0]                                   # frac_hold 0.20
    assert not hl.check1([_row(False)] * 300)[0] and not hl.check1([])[0]


def test_check2_rule_and_screen_rules() -> None:
    ok, rate, n = hl.check2([_row(True, succ=1.0 if i % 10 == 0 else 0.0) for i in range(500)])
    assert ok and rate == pytest.approx(0.10) and n == 200
    assert not hl.check2([_row(True, succ=1.0 if i % 11 == 0 else 0.0) for i in range(500)])[0] and not hl.check2([])[0]
    assert (hl.screen_decision(0.55), hl.screen_decision(0.5499), hl.screen_decision(0.30), hl.screen_decision(0.2999)) == ("final", "continue", "continue", "stop")
    seeds = hl.balanced_screen_seeds(30)
    assert len(seeds) == 30 and sorted({(s - GATE_SEED0) // 20 for s in seeds}) == [0, 1, 2, 3, 4] and all(sum((s - GATE_SEED0) // 20 == i for s in seeds) == 6 for i in range(5))
    rows = [{"seed": s, "metrics": {"success": 1.0 if (s - GATE_SEED0) // 20 == 1 else 0.0}} for s in seeds]
    assert hl.per_shape_success(rows, 8) == {"cyl_r3": 0.0, "cyl_r7": 1.0, "square_8x8": 0.0, "rect_5x14": 0.0, "rounded_8x8_r2": 0.0}


def test_long_run_checks_callback(tmp_path) -> None:
    import json
    (tmp_path / "episodes.jsonl").write_text("\n".join(json.dumps(_row(True, 2.5, 0.9)) for _ in range(50)) + '\n{"hug_mod')           # last line half written
    cb = hl.LongRunChecks(tmp_path, 1.0, 2.0)
    assert cb.run_check1(steps=123) and cb.results["check1"]["timesteps"] == 123 and cb.reason is None
    assert not cb.run_check2(at_end=True, steps=456) and cb.results["check2"]["at_end"] and cb.results["check2"]["rate"] == 0.0
