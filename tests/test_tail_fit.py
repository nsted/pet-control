"""Closed-loop simulation-based identification: recover J, B, C, K, kp, kd and delay from simulated telemetry."""

from __future__ import annotations

import math

import numpy as np
import pytest

from petctl.tail_fit import ClosedLoopFit, Params, Trial, fit_closed_loop, format_closed_loop, simulate

TRUE = Params(J=6.0e-4, B=0.004, C=0.004, K=0.0, kp=0.30, kd=0.020, delay=0.030)


def _smooth_step_cmd(amp: float, T: float, rate: float = 3.0) -> tuple[np.ndarray, np.ndarray]:
    """Command sent every 10 ms: a rate-limited step (the real command path keeps |rate| well under the 6 rad/s ramp)."""
    t = np.arange(0.0, T, 0.01)
    cmd = np.clip((t - 0.2) * rate, 0.0, abs(amp)) * math.copysign(1.0, amp)
    return t, cmd


def _chirp_cmd(amp: float, f0: float, f1: float, T: float) -> tuple[np.ndarray, np.ndarray]:
    t = np.arange(0.0, T, 0.01)
    return t, amp * np.sin(2 * math.pi * (f0 * t + (f1 - f0) * t * t / (2.0 * T))) * np.clip(t / 0.5, 0, 1)


def _make_trial(truth: Params, cmd_t: np.ndarray, cmd: np.ndarray, T: float, rng: np.random.Generator, label: str = "") -> Trial:
    """Synthesize telemetry: 50 Hz with arrival jitter, 0.0004 rad position quantization, noisy torque."""
    ts = np.sort(np.arange(0.0, T, 0.02) + np.abs(rng.normal(0, 0.003, int(T / 0.02))))
    ts = ts[ts < T - 0.01]
    probe = Trial(ts, np.zeros_like(ts), np.zeros_like(ts), cmd_t, cmd)
    th, ta = simulate(truth, probe)
    return Trial(ts, np.round(th / 0.0004) * 0.0004, ta + rng.normal(0, 0.0005, len(ta)), cmd_t, cmd, label)


def _trials(truth: Params, seed: int = 0) -> list[Trial]:
    rng = np.random.default_rng(seed)
    out = []
    for amp in (0.3, -0.3, 0.4, -0.4):
        ct, c = _smooth_step_cmd(amp, 2.5)
        out.append(_make_trial(truth, ct, c, 2.5, rng, f"step{amp}"))
    for f0, f1 in ((0.2, 1.0), (0.4, 1.4)):
        ct, c = _chirp_cmd(0.15, f0, f1, 5.0)
        out.append(_make_trial(truth, ct, c, 5.0, rng, f"chirp{f0}-{f1}"))
    return out


def test_the_simulator_reproduces_a_first_order_check() -> None:
    """Sanity: with huge damping and no friction, the step response settles at the target without overshoot."""
    p = Params(J=6e-4, B=0.0, C=0.0, K=0.0, kp=0.3, kd=0.05, delay=0.0)
    ct, c = _smooth_step_cmd(0.3, 3.0)
    tr = Trial(np.arange(0, 3.0, 0.02), np.zeros(150), np.zeros(150), ct, c)
    th, ta = simulate(p, tr)
    assert th[-1] == pytest.approx(0.3, abs=0.005)
    assert th.max() < 0.3 * 1.15
    assert abs(ta[-1]) < 0.005


def test_recovers_the_plant_the_controller_and_the_delay() -> None:
    fit = fit_closed_loop(_trials(TRUE), Params(J=1.0e-3, B=0.01, C=0.002, K=0.0, kp=0.25, kd=0.03, delay=0.01),
                          max_iter=450, polish_iter=300)
    assert isinstance(fit, ClosedLoopFit)
    p = fit.params
    assert p.kp == pytest.approx(TRUE.kp, rel=0.15)
    assert p.kd == pytest.approx(TRUE.kd, rel=0.25)
    assert p.J == pytest.approx(TRUE.J, rel=0.25)
    assert p.C == pytest.approx(TRUE.C, abs=0.0015)
    assert p.delay == pytest.approx(TRUE.delay, abs=0.015)
    assert fit.theta_rmse < 0.01
    assert "inertia" in format_closed_loop(fit, 0.3, 0.02)


def test_measures_the_gain_units_when_the_motor_applies_a_different_gain() -> None:
    """The controller applies 2x the commanded kp: the fit must report the effective gain, not the nominal one."""
    truth = Params(J=6.0e-4, B=0.004, C=0.004, K=0.0, kp=0.60, kd=0.020, delay=0.030)      # 'commanded' 0.30
    fit = fit_closed_loop(_trials(truth, seed=2), Params(J=8.0e-4, B=0.006, C=0.003, K=0.0, kp=0.3, kd=0.02, delay=0.02),
                          max_iter=450, polish_iter=300)
    assert fit is not None
    # From a start at half the true gain the fit lands within ~25% of it: enough to tell 'same as commanded' (ratio 1) from 'twice'.
    assert fit.params.kp == pytest.approx(0.60, rel=0.3)
    assert 1.6 < fit.params.kp / 0.30 < 2.7


def test_empty_or_short_trials_give_none() -> None:
    assert fit_closed_loop([], TRUE) is None
    short = Trial(np.arange(5.0), np.zeros(5), np.zeros(5), np.array([0.0]), np.array([0.0]))
    assert fit_closed_loop([short], TRUE) is None
