"""Targeted tests of the hug_anneal pilot: lr schedule, log-std reset, hold-break classification, stage definition, GO rule."""

from __future__ import annotations

import numpy as np

from petctl.rl import hold_diag as hdg
from petctl.rl.curriculum import hug_range as hr
from petctl.rl.curriculum.stages import STAGES, get_stage
from petctl.rl.train import lr_schedule


def test_lr_schedule_final(monkeypatch) -> None:  # type: ignore[no-untyped-def]
    monkeypatch.setenv("PETCTL_LR", "1.5e-4")
    monkeypatch.delenv("PETCTL_LR_CONST", raising=False)
    monkeypatch.setenv("PETCTL_LR_FINAL", "2e-5")
    s = lr_schedule()
    assert abs(s(1.0) - 1.5e-4) < 1e-12 and abs(s(0.0) - 2e-5) < 1e-12 and abs(s(0.5) - 8.5e-5) < 1e-12
    monkeypatch.delenv("PETCTL_LR_FINAL")
    assert abs(lr_schedule()(0.5) - 0.75e-4) < 1e-12              # default unchanged: lr0 x f
    monkeypatch.setenv("PETCTL_LR_CONST", "1")
    monkeypatch.setenv("PETCTL_LR_FINAL", "2e-5")
    assert abs(lr_schedule()(0.1) - 1.5e-4) < 1e-12               # constant wins


def test_stage_definition() -> None:
    a, b = get_stage("hug_anneal"), get_stage("hug_scale15")
    assert a.log_std_init == -2.0 and a.ent_coef == 0.0 and a.action_scale == 1.5 and a.action_obs_scale == 1.5
    assert a.task_params["hold_diag"] == 1.0 and "hold_diag" not in b.task_params and b.ent_coef == 0.002
    assert {k: v for k, v in a.task_params.items() if k != "hold_diag"} == b.task_params and a.gate == b.gate
    assert STAGES["hug_scale15"].log_std_init == -1.2          # older stage untouched


def test_log_std_reset_passed_without_keep() -> None:
    # train_phase passes stage.log_std_init (-2.0) unless keep_log_std; train() re-applies it after the warm start (learned afterwards, never frozen)
    import inspect

    from petctl.rl.train import train
    src = inspect.getsource(train)
    assert "model.policy.log_std.data.fill_" in src and "requires_grad" not in src.split("log_std.data.fill_")[1][:200]


def _run(tr: hdg.HoldBreakTracker, ticks: list[dict]) -> None:
    ticks = [{}] * 20 + ticks                                      # idle ticks flush the 0.5 s look-back window
    for t in ticks:
        d = dict(hug_tick=False, just_held=False, held=False, run=0, run_start=0, h=4, span_deg=0.0, span_min=180.0, at_cap=np.zeros(7, bool), bus_a=0.0, n_joints=7)
        d.update(t)
        tr.update(**d)


def test_hold_break_causes() -> None:
    good = dict(hug_tick=True, run=5, run_start=2, span_deg=190.0)
    tr = hdg.HoldBreakTracker(30.0)
    _run(tr, [good] * 10 + [dict(run=3, span_deg=190.0)])                                   # run fell below H
    assert (tr.started, tr.broken, tr.completed) == (1, 1, 0) and tr.causes["run_below_h"] == 1 and tr.causes["span_below"] == 0
    _run(tr, [good] * 10 + [dict(run=5, run_start=2, span_deg=100.0)])                       # span fell
    assert tr.causes["span_below"] == 1 and tr.causes["none"] == 0
    cap = np.zeros(7, bool)
    cap[3] = True                                                                           # joint 3 = module 4: in run (modules 2..6 -> joints idx 2..5)
    _run(tr, [good] * 5 + [dict(good, at_cap=cap)] + [good] * 3 + [dict(run=5, span_deg=100.0)])
    assert tr.causes["torque_cap"] == 1
    cap2 = np.zeros(7, bool)
    cap2[0] = True                                                                          # joint 0 is outside the run: ignored
    _run(tr, [dict(good, at_cap=cap2)] * 3 + [dict(run=5, span_deg=100.0)])
    assert tr.causes["torque_cap"] == 1
    _run(tr, [dict(good, bus_a=4.5)] + [good] * 3 + [dict(run=5, span_deg=100.0)])
    assert tr.causes["bus_over"] == 1
    old = [dict(good, at_cap=cap)] + [good] * 20 + [dict(run=5, span_deg=100.0)]            # cap event older than 0.5 s (15 ticks): not counted
    _run(tr, old)
    assert tr.causes["torque_cap"] == 1 and tr.broken == 6


def test_hold_completion_and_summary() -> None:
    tr = hdg.HoldBreakTracker(30.0)
    good = dict(hug_tick=True, run=5, run_start=2, span_deg=190.0)
    _run(tr, [good] * 5 + [dict(good, just_held=True, held=True)] + [dict(held=True)] * 3 + [dict(good, held=True)])       # completed; later ticks ignored
    assert (tr.started, tr.broken, tr.completed) == (1, 0, 1)
    m = tr.metrics()
    s = hdg.summarise([m, {"hb_started": 2.0, "hb_broken": 2.0, "hb_completed": 0.0, "hb_cause_run_below_h": 1.0, "hb_cause_none": 1.0}, {"success": 1.0}])
    assert s["holds_started"] == 3 and s["holds_broken"] == 2 and s["holds_completed"] == 1 and s["break_fractions"]["run_below_h"] == 0.5


def test_anneal_go_rule() -> None:
    sh = {"cyl_r3": 0.0, "cyl_r7": 0.0, "square_8x8": 0.0, "rect_5x14": 0.0, "rounded_8x8_r2": 0.0}
    assert not hr.anneal_go(0.2, {**sh, "cyl_r3": 0.5})[0]
    assert hr.anneal_go(0.05, {**sh, "cyl_r7": 1 / 12})[0] and hr.anneal_go(0.25, sh)[0]
