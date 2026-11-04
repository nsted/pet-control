"""The MuJoCo model's static equilibrium must agree with the analytic coupled gravity model."""
from __future__ import annotations

import numpy as np
import pytest

pytest.importorskip("mujoco")

from petctl.gravity_model import equilibrium  # noqa: E402
from petctl.sim.mujoco_model import MODULE_MASS_KG, SimParams, Simulator  # noqa: E402


def test_static_equilibrium_matches_gravity_model() -> None:
    cmd = np.zeros(7)
    cmd[3] = 0.3
    phi, tau = equilibrium(cmd, np.full(7, MODULE_MASS_KG), 0.8)
    sim = Simulator(SimParams(coulomb=(0.0,) * 7))
    sim.reset(np.zeros(7))
    sim.step_to(cmd, np.full(7, 5.0), 6.0)
    assert np.allclose(sim.data.qpos, phi, atol=0.01)
    assert np.allclose(sim.last_tau, tau, atol=0.01)
