"""Hanging-test analysis: gravity stiffness, friction hysteresis, and the static current fit."""

from __future__ import annotations

import numpy as np
import pytest

from petctl.hang_analysis import (
    HoldPoint,
    fit_current_coefficient,
    fit_gravity_friction,
    modules_below,
    predicted_stiffness,
    predicted_stiffness_point_masses,
)


def _sweep(k: float, c: float, b0: float = 0.0, noise: float = 0.001, seed: int = 0) -> list[HoldPoint]:
    """Up/down passes on both sides, as the script produces them."""
    rng = np.random.default_rng(seed)
    pts = []
    for sign in (1.0, -1.0):
        grid = [0.02 * i for i in range(1, 6)]
        for th in grid:                                    # moving away from zero
            d = sign
            pts.append(HoldPoint(sign * th, k * sign * th + c * d + b0 + rng.normal(0, noise), d))
        for th in reversed(grid[:-1] + [0.0]):             # coming back
            d = -sign
            pts.append(HoldPoint(sign * th, k * sign * th + c * d + b0 + rng.normal(0, noise), d))
    return pts


def test_stiffness_and_friction_are_recovered_from_both_hysteresis_branches() -> None:
    fit = fit_gravity_friction(_sweep(k=1.2, c=0.03, b0=0.004))
    assert fit is not None
    assert fit.stiffness_nm_per_rad == pytest.approx(1.2, rel=0.03)
    assert fit.coulomb_nm == pytest.approx(0.03, abs=0.003)
    assert fit.offset_nm == pytest.approx(0.004, abs=0.003)
    assert fit.resid_rms_nm < 0.003


def test_a_frictionless_light_joint_reads_zero_coulomb() -> None:
    fit = fit_gravity_friction(_sweep(k=0.05, c=0.0, noise=0.0005))
    assert fit is not None and fit.coulomb_nm == pytest.approx(0.0, abs=0.001)


def test_under_determined_data_returns_none() -> None:
    assert fit_gravity_friction([HoldPoint(0.1, 0.1, 1.0)]) is None
    same_direction = [HoldPoint(0.02 * i, 0.5 * 0.02 * i, 1.0) for i in range(1, 8)]
    assert fit_gravity_friction(same_direction) is None          # friction not separable from offset


def test_current_coefficient_uses_only_static_points() -> None:
    a_true = 20.0
    pts = [HoldPoint(0.1, t, 1.0, delta_bus_a=a_true * t * t, abs_vel_rad_s=0.0) for t in (0.04, 0.08, 0.12)]
    pts.append(HoldPoint(0.3, 0.1, 1.0, delta_bus_a=5.0, abs_vel_rad_s=0.4))     # moving: excluded
    pts.append(HoldPoint(0.0, 0.005, 1.0, delta_bus_a=0.3))                      # tiny torque: excluded
    res = fit_current_coefficient(pts)
    assert res is not None
    assert res[0] == pytest.approx(a_true, rel=1e-6)


def test_current_coefficient_none_without_data() -> None:
    assert fit_current_coefficient([HoldPoint(0.0, 0.0, 1.0)]) is None


def test_modules_below_for_a_robot_held_by_the_head_and_by_the_tail() -> None:
    assert [modules_below(k, held_module=0) for k in range(1, 8)] == [7, 6, 5, 4, 3, 2, 1]
    assert [modules_below(k, held_module=7) for k in range(1, 8)] == [1, 2, 3, 4, 5, 6, 7]
    assert modules_below(1, held_module=1) == 1 and modules_below(2, held_module=1) == 6   # held by module 1


def test_predicted_stiffness_scales_with_the_square_of_the_chain() -> None:
    k7 = predicted_stiffness(7, 2.2, 0.051)
    k1 = predicted_stiffness(1, 2.2, 0.051)
    assert k7 / k1 == pytest.approx(49.0)
    assert k7 == pytest.approx(9.81 * 0.5 * 0.275 * 0.051 * 49 / 2, rel=1e-3)      # sin^2(45 deg) = 0.5


def test_point_masses_at_motor_axes_give_n_choose_two_levers() -> None:
    masses = [0.275] * 8
    g_eff = 9.81 * 0.5                                  # sin^2(45 deg)
    for n, expect_levers in ((7, 21), (6, 15), (5, 10), (4, 6), (3, 3), (2, 1), (1, 0)):
        k = 8 - n
        got = predicted_stiffness_point_masses(k, 0, masses, 0.1)
        assert got == pytest.approx(g_eff * 0.275 * 0.1 * expect_levers, abs=1e-9)


def test_tail_joint_held_by_the_head_has_no_gravity_lever_and_head_mass_counts_when_held_by_the_tail() -> None:
    masses = [0.5, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.3]
    assert predicted_stiffness_point_masses(7, 0, masses, 0.1) == 0.0
    # held by the tail: servo 1 swings the head (0.5 kg) a pitch away from its axis
    assert predicted_stiffness_point_masses(1, 7, masses, 0.1) == pytest.approx(9.81 * 0.5 * 0.5 * 0.1)


def test_axis_factor_is_the_horizontal_pendulum_at_ninety_degrees_and_half_at_forty_five() -> None:
    from petctl.hang_analysis import axis_factor

    assert axis_factor(90.0) == pytest.approx(1.0)
    assert axis_factor(45.0) == pytest.approx(0.5)
    assert axis_factor(135.0) == pytest.approx(0.5)                # the alternate joints
    assert axis_factor(0.0) == pytest.approx(0.0, abs=1e-12)       # axis along the chain: no restoring torque
