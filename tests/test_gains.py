"""Dynamic joint gains: PetSim.set_gains changes stiffness / damping on the fly, the torque cap still binds, no jumps or NaN."""

from __future__ import annotations

import numpy as np

from petctl.sim.model_builder import BuildParams, PetSim


def deflection(kp: float, push_nm: float = 0.2) -> float:
    """Steady joint deflection (rad) of joint 1 of a 10-module body when a constant torque is applied to it (stiffer = smaller)."""
    sim = PetSim(BuildParams(n_modules=10, dt=0.004))
    sim.set_gains(kp=kp, kd=0.05)
    sim.reset()
    sim.set_cap(np.full(9, 5.0))
    sim.data.qfrc_applied[sim.model.jnt_dofadr[1]] = push_nm
    sim.step(np.zeros(9), 2.0)
    return float(abs(sim.joint_pos()[0]))


def test_stiffer_gain_deflects_less() -> None:
    soft, stiff = deflection(0.4), deflection(1.6)
    assert stiff < 0.5 * soft, (soft, stiff)


def test_gains_roundtrip_and_midrun_change_is_smooth() -> None:
    sim = PetSim(BuildParams(n_modules=11, dt=0.004))
    kp, kd = sim.gains()
    assert np.allclose(kp, sim.p.kp) and np.allclose(kd, sim.p.kd)
    sim.step(np.full(10, 0.3), 0.5)
    q0 = sim.joint_pos().copy()
    sim.set_gains(kp=np.linspace(0.2, 2.0, 10), kd=0.08)
    sim.step(np.full(10, 0.3), 0.5)
    assert np.all(np.isfinite(sim.data.qpos)) and np.abs(sim.joint_pos() - q0).max() < 0.5
    assert np.allclose(sim.gains()[0], np.linspace(0.2, 2.0, 10))


def test_torque_cap_still_binds_at_high_gain() -> None:
    sim = PetSim(BuildParams(n_modules=10, dt=0.004))
    sim.set_gains(kp=50.0, kd=0.5)
    sim.step(np.full(9, 3.0), 0.5)
    assert np.abs(sim.joint_torque()).max() <= sim.model.actuator_forcerange[:, 1].max() + 1e-9
