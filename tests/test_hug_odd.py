"""Odd-length (L = 9) study: decision rules with synthetic data, generalized gate spec / held-out sets, driver wiring, and regression that the L = 8 driver / stage behaviour is unchanged."""
from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import numpy as np
import pytest

pytest.importorskip("mujoco")

from petctl.rl.curriculum import hug_odd as ho  # noqa: E402
from petctl.rl.curriculum.m2v5 import gate2_spec_hug5  # noqa: E402
from petctl.rl.curriculum.stages import get_stage  # noqa: E402
from petctl.rl.target_shapes import GATE_SEED0, PER_SHAPE, held_out_names, held_out_set, near_heldout, sample_training_shape, V2_SAMPLER  # noqa: E402

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts" / "rl"


def _load(name: str):
    sys.path.insert(0, str(SCRIPTS))
    spec = importlib.util.spec_from_file_location(name, SCRIPTS / f"{name}.py")
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


def _rows(success: list[float], step: int = 1000) -> list[dict]:
    return [{"success": s, "timesteps": (i + 1) * step, "final_dist": 0.1, "dist0": 1.0} for i, s in enumerate(success)]


def test_flat_pilot_go_rule() -> None:
    ok, sig = ho.flat_pilot_go(_rows([0.0] * 100 + [1.0] * 15 + [0.0] * 85), 200_000)
    assert ok and sig["success"] == pytest.approx(0.15)
    ok, sig = ho.flat_pilot_go(_rows([0.0] * 100 + [1.0] * 14 + [0.0] * 86), 200_000)
    assert not ok and sig["success"] == pytest.approx(0.14)
    assert not ho.flat_pilot_go([], 1000)[0]
    assert ho.flat_pilot_go(_rows([1.0] * 300), 100_000)[1]["n"] == 100.0          # only the last 100 episodes finished by the step count count


def test_budgets_follow_the_orders() -> None:
    assert (ho.FLAT_BUDGET_MIN, ho.FLAT_CAP) == (25.0, 6_000_000) and (ho.RETRY_BUDGET_MIN, ho.RETRY_CAP) == (45.0, 12_000_000) and (ho.HUG_BUDGET_MIN, ho.HUG_CAP) == (45.0, 4_000_000)


def test_hour_decisions() -> None:
    names = held_out_names(9)
    z = {n: 0.0 for n in names}
    assert ho.hour_decision(1, 0.55, z, 9) == "final" and ho.hour_decision(3, 0.60, z, 9) == "final"
    assert ho.hour_decision(1, 0.0, z, 9) == "next"                                                    # hour 2 always follows hour 1
    assert ho.hour_decision(2, 0.10, z, 9) == "stop"
    assert ho.hour_decision(2, 0.15, z, 9) == "next"
    assert ho.hour_decision(2, 0.0333, {**z, "cyl_r8": 2 / 6}, 9) == "next"                           # curved shape at 2/6
    assert ho.hour_decision(2, 0.0333, {**z, "rounded_8x8_r2": 2 / 6}, 9) == "next"
    assert ho.hour_decision(2, 0.0333, {**z, "cyl_r8": 1 / 6}, 9) == "stop"
    assert ho.hour_decision(2, 0.0333, {**z, "square_8x8": 3 / 6, "rect_5x14": 3 / 6}, 9) == "stop"   # boxes do not count as curved
    assert ho.hour_decision(3, 0.30, z, 9) == "stop_after3"
    assert ho.curved_shape_names(9) == ("cyl_r3", "cyl_r8", "rounded_8x8_r2")


def test_generalised_gate_spec_and_sets() -> None:
    assert held_out_names(9) == ("cyl_r3", "cyl_r8", "square_8x8", "rect_5x14", "rounded_8x8_r2") and held_out_set(9)[1][2] == (0.08,)
    spec = gate2_spec_hug5(9)
    assert spec.n == 100 and spec.dr_scale == 0.6 and spec.min_group_success == 0.5 and [a.name for a in spec.ablations] == ["capacitive_off"]
    assert [spec.group_of(GATE_SEED0 + PER_SHAPE * i) for i in range(5)] == list(held_out_names(9))
    assert [gate2_spec_hug5(8).group_of(GATE_SEED0 + PER_SHAPE * i) for i in range(5)] == list(held_out_names(8))
    assert gate2_spec_hug5().group_of(GATE_SEED0 + PER_SHAPE) == "cyl_r9"                              # default (L = 10) unchanged


def test_training_exclusion_follows_the_length() -> None:
    rng = np.random.default_rng(1)
    shapes = [sample_training_shape(rng, 1.5, 9, **V2_SAMPLER) for _ in range(4000)]
    assert not any(near_heldout(s, 9, 0.005) for s in shapes)
    radii = np.array([s.a for s in shapes if s.kind == "cyl"])
    assert ((radii > 0.075) & (radii < 0.085)).sum() == 0                                              # +-0.5 cm around the L = 9 gate cylinder r8
    assert ((radii > 0.065) & (radii < 0.075)).sum() > 0                                               # r7 (the L = 8 gate radius) is trained at L = 9


def test_body_note() -> None:
    assert ho.body_note(8) == "" and "generated URDF" in ho.body_note(9) and "unconfirmed" in ho.body_note(9)


def test_odd_driver_wiring() -> None:
    d = _load("hug_curr_odd_driver")
    assert d.STAGE == "hug_curr_02"
    import inspect
    assert "--length" in inspect.getsource(d.main) and "hold-ramp" in inspect.getsource(d.main)


def test_l8_driver_and_stage_unchanged() -> None:
    h = _load("hug_curr2_driver")
    assert (h.STAGE, h.L, h.BUDGET_MIN, h.CAP_STEPS) == ("hug_curr_02", 8, 45.0, 4_000_000)
    assert str(h.INIT) == "data/rl/hug_curr/hug_curr_01" and str(h.INIT_BUFFER) == "data/rl/hug_curr/hug_buffer.json"
    import inspect
    src = inspect.getsource(h.main)
    assert 'default=0.0, help="hold_ramp task knob' in src and "gate2_spec_hug5(L)" in src and 'ap.add_argument("--root", type=Path, default=Path("data/rl/hug_curr2"))' in src
    p = get_stage("hug_curr_02").task_params
    assert p["hug_curriculum"] == 2.0 and p["shape_mix"] == 1.0 and p["hold_ramp"] == 0.0 and p["hold_weight"] == 10.0 and p["shape_phase"] == 1.5 and p["hug_stall_s"] == 8.0
    assert get_stage("hug_curr_02").action_mode == "hold" and get_stage("hug_curr_02").ent_coef == 0.002
    assert held_out_names(8) == ("cyl_r3", "cyl_r7", "square_8x8", "rect_5x14", "rounded_8x8_r2")
