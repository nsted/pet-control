"""Coupled gravity model of the hanging robot."""

from __future__ import annotations

import numpy as np
import pytest

from petctl.gravity_model import G, equilibrium, grad, positions

M = np.full(7, 0.275)


def test_isolated_joint_stiffness_matches_the_closed_form() -> None:
    """Hessian diagonal = g sin^2(45) m pitch n(n-1)/2 (pitch 5.9 cm), within the CAD's small deviations."""
    h = 1e-4
    for k in range(1, 8):
        e = np.zeros(7)
        e[k - 1] = h
        diag = (grad(e, M)[k - 1] - grad(-e, M)[k - 1]) / (2 * h)
        n = 8 - k
        simple = G * 0.5 * 0.275 * 0.059 * n * (n - 1) / 2
        assert diag == pytest.approx(simple, abs=0.03)


def test_equilibrium_balances_motor_and_gravity_torque() -> None:
    cmd = np.array([0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0])
    phi, tau = equilibrium(cmd, M, 0.8)
    assert np.allclose(tau, grad(phi, M), atol=1e-8)                 # statics: motor torque = dU/dphi
    assert np.allclose(0.8 * (cmd - phi), tau)


def test_zero_command_gives_only_small_static_torques() -> None:
    """The CAD chain drifts ~2 cm sideways over its length, so the straight pose carries a few mN·m of static torque
    (the robot showed 1-14 mN·m at the start of each hanging run)."""
    phi, tau = equilibrium(np.zeros(7), M, 0.8)
    assert np.max(np.abs(phi)) < 0.03
    assert np.max(np.abs(tau)) < 0.025


def test_coupling_makes_the_measured_stiffness_decrease_toward_the_tail() -> None:
    """With neighbours held by kp 0.8 springs the effective stiffness seen at a joint falls monotonically."""
    ks = []
    for k in range(1, 8):
        c = np.zeros(7); c[k - 1] = 0.05
        ph, t = equilibrium(c, M, 0.8)
        pm, tm = equilibrium(-c, M, 0.8)
        ks.append((t[k - 1] - tm[k - 1]) / (ph[k - 1] - pm[k - 1]))
    assert all(a > b for a, b in zip(ks, ks[1:]))
    assert ks[-1] == pytest.approx(0.0, abs=1e-6)         # the tail joint has no gravity lever
    assert 0.8 < ks[0] < 0.95                             # servo 1 ~0.86 N·m/rad


def test_module_positions_start_on_their_axes() -> None:
    r = positions(np.zeros(7))
    assert set(r) == set(range(1, 8))
    assert np.linalg.norm(r[7] - r[1]) == pytest.approx(0.36, abs=0.03)       # six pitches of ~5.9 cm
