"""GIM3505-8 bus-current model (owner measurements 2026-11-26): hold / move coefficients, idle, voltage scaling; legacy model unchanged."""

from __future__ import annotations

import numpy as np
import pytest

from petctl.config import GIM3505_HOLD_A_PER_NM2, GIM3505_IDLE_PER_MOTOR_A, GIM3505_MOVE_A_PER_NM2, REGIMEN_BUS_PEAK_A
from petctl.sim.bus import IDLE_ELECTRONICS_A, BusClamp
from petctl.sim.model_builder import BuildParams, PetSim

N = 10


def sim_with_bus(**kw: float) -> tuple[PetSim, BusClamp]:
    sim = PetSim(BuildParams(n_modules=N, dt=0.004))
    sim.set_gains(kp=50.0, kd=0.0)                                   # saturate every joint at the cap for a large target error
    sim.set_cap(np.full(N - 1, 0.5))
    idle = IDLE_ELECTRONICS_A + GIM3505_IDLE_PER_MOTOR_A * (N - 1)
    bus = BusClamp(sim.model, GIM3505_HOLD_A_PER_NM2, 1e9, clamp=False, move_coeff=GIM3505_MOVE_A_PER_NM2, move_omega0=1.0, idle_a=idle, **kw)
    sim.bus = bus
    return sim, bus


def test_hold_and_move_current_at_cap_match_owner_fit() -> None:
    """Every joint saturated at the 0.5 N.m cap: still joints draw 2.4 * 0.25 = 0.6 A each (owner hold fit), joints at >= 1 rad/s 4.6 * 0.25 = 1.15 A (rotate fit)."""
    sim, bus = sim_with_bus()
    sim.reset()
    d, m = sim.data, sim.model
    d.ctrl[:] = np.where(np.arange(N - 1) % 2 == 0, 3.0, -3.0)                # far from q = 0: kp * error >> cap
    d.qvel[:] = 0.0
    tau = bus.demand(m, d, m.actuator_forcerange[:, 1])
    assert np.allclose(np.abs(tau), 0.5)
    assert bus._weighted_ss(tau, d) / (N - 1) == pytest.approx(GIM3505_HOLD_A_PER_NM2 * 0.25, rel=1e-6)
    dofs = bus._dsel
    d.qvel[dofs] = 2.0
    assert bus._weighted_ss(tau, d) / (N - 1) == pytest.approx(GIM3505_MOVE_A_PER_NM2 * 0.25, rel=1e-6)
    d.qvel[dofs] = 0.5                                                          # half the blend speed: halfway between the coefficients
    assert bus._weighted_ss(tau, d) / (N - 1) == pytest.approx(0.25 * (GIM3505_HOLD_A_PER_NM2 + GIM3505_MOVE_A_PER_NM2) / 2, rel=1e-6)


def test_voltage_scaling_and_idle() -> None:
    sim, bus = sim_with_bus()
    assert bus.idle == pytest.approx(IDLE_ELECTRONICS_A + 0.02 * (N - 1))
    assert bus.move_coeff / bus.coeff == pytest.approx(GIM3505_MOVE_A_PER_NM2 / GIM3505_HOLD_A_PER_NM2)


def test_legacy_model_is_default_and_unchanged() -> None:
    sim = PetSim(BuildParams(n_modules=8, dt=0.004))
    bus = BusClamp(sim.model, 13.3, REGIMEN_BUS_PEAK_A)
    assert bus.move_coeff is None and bus.idle == pytest.approx(0.25 + 0.05 * 7)


def test_env_switch_selects_motor_model() -> None:
    from petctl.rl.config import EnvConfig
    from petctl.rl.env import PetEnv

    for motor, expect_move in ((0.0, False), (1.0, True)):
        cfg = EnvConfig(task="goal_reach", n_modules=N, episode_s=0.5, action_mode="hold", task_params={"goal_dist_min": 1.0, "goal_dist_max": 2.0, "bus_model": 1.0, "bus_motor": motor})
        env = PetEnv(cfg)
        env.reset(seed=1)
        assert (env.bus.move_coeff is not None) == expect_move
        env.step(np.zeros(cfg.n_actions, np.float32))
    # 14.7 V supply lowers the coefficients by 13 / 14.7
    cfg = EnvConfig(task="goal_reach", n_modules=N, episode_s=0.5, action_mode="hold", task_params={"goal_dist_min": 1.0, "goal_dist_max": 2.0, "bus_model": 1.0, "bus_motor": 1.0, "bus_voltage": 14.7})
    env = PetEnv(cfg)
    env.reset(seed=1)
    assert env.bus.coeff == pytest.approx(GIM3505_HOLD_A_PER_NM2 * 13.0 / 14.7)
