"""STAGE_1_GATES 1.4: motor heat model, T_coil observation and torque derating (all thermal parameters unvalidated until T13)."""
from __future__ import annotations

import numpy as np
import pytest

pytest.importorskip("gymnasium")
pytest.importorskip("mujoco")

from petctl.rl.config import EnvConfig  # noqa: E402
from petctl.rl.env import PetEnv  # noqa: E402
from petctl.rl.thermal import R_TH_K_PER_W, T_AMB_C, T_KNOB_C, T_LIMIT_C, ThermalModel, derate, heat_power_w  # noqa: E402

NJ = 10


def test_steady_state_matches_analytic_within_one_percent() -> None:
    m = ThermalModel(3, 0.05, tau_th_s=100.0)
    tq = np.array([0.1, 0.3, 0.5])
    for _ in range(int(10 * 100.0 / 0.05)):                  # 10 time constants
        m.step(tq)
    want = T_AMB_C + R_TH_K_PER_W * heat_power_w(tq)
    assert np.allclose((m.temp - T_AMB_C), (want - T_AMB_C), rtol=0.01)


def test_step_response_reaches_63_percent_at_tau() -> None:
    tau = 200.0
    m = ThermalModel(1, 0.05, tau_th_s=tau)
    tq = np.array([0.5])
    full = R_TH_K_PER_W * float(heat_power_w(tq)[0])
    for _ in range(int(tau / 0.05)):
        m.step(tq)
    assert (m.temp[0] - T_AMB_C) / full == pytest.approx(1 - np.exp(-1), rel=0.02)


def test_derating_is_monotone_one_to_zero() -> None:
    t = np.linspace(20, 120, 401)
    d = derate(t)
    assert np.all(np.diff(d) <= 1e-12) and d[0] == 1.0 and d[-1] == 0.0
    assert derate(T_KNOB_C) == 1.0 and derate(T_LIMIT_C) == 0.0 and 0 < derate(0.5 * (T_KNOB_C + T_LIMIT_C)) < 1


def test_reading_is_whole_degrees_and_start_is_randomised() -> None:
    rng = np.random.default_rng(0)
    starts = [ThermalModel.sample(NJ, 0.05, rng, 1.0).temp for _ in range(20)]
    assert np.ptp(np.array(starts)) > 20.0
    m = ThermalModel(NJ, 0.05, temp=np.linspace(30.04, 71.6, NJ))
    assert np.allclose(m.reading(), np.round(m.reading()))
    z = ThermalModel.sample(NJ, 0.05, rng, 0.0)
    assert np.allclose(z.temp, T_AMB_C)                       # scale 0: ambient start, mid tau


def _env(**kw: float) -> PetEnv:
    tp = {"mit_frame": 1.0, "thermal": 1.0, "action_unbounded": 1.0, **kw}
    return PetEnv(EnvConfig(n_modules=11, episode_s=6.0, action_mode="delta", task_params=tp))


def test_env_observes_t_coil_and_obs_size_grows_by_n_joints() -> None:
    base = PetEnv(EnvConfig(n_modules=11, action_mode="delta", task_params={"mit_frame": 1.0, "action_unbounded": 1.0}))
    env = _env()
    assert env.observation_space.shape[0] == base.observation_space.shape[0] + NJ
    o, _ = env.reset(seed=0)
    block = o[-NJ:] * 75.0 + T_AMB_C
    assert np.allclose(block, np.round(block), atol=1e-3)                     # whole degrees
    assert env.thermal is not None and np.allclose(block, env.thermal.reading(), atol=1e-3)


def test_hot_motor_loses_torque_cap_in_the_env() -> None:
    env = _env()
    env.reset(seed=0)
    env.thermal.temp[:] = 95.0
    env.step(np.zeros(NJ))
    cap = env.sim.model.actuator_forcerange[:, 1]
    assert np.all(cap < env._cap_nom * 0.5) and np.all(cap > 0)
    env.thermal.temp[:] = 120.0
    env.step(np.zeros(NJ))
    assert np.allclose(env.sim.model.actuator_forcerange[:, 1], 0.0)


def test_acceleration_heats_within_an_episode_and_default_does_not() -> None:
    hot, cool = _env(thermal_accel=200.0), _env()
    for env in (hot, cool):
        env.cfg.dr.scale = 0.0                                                # ambient start, tau 134 s (log-mid)
        env.reset(seed=0)
        for _ in range(100):
            env.step(np.ones(NJ) * 2.0)
    assert hot.thermal.temp.max() > cool.thermal.temp.max() + 5.0
    assert cool.thermal.temp.max() < T_AMB_C + 5.0


def test_knob_off_leaves_default_obs_and_rng_untouched() -> None:
    a = PetEnv(EnvConfig(n_modules=11, action_mode="delta", task_params={"action_unbounded": 1.0}))
    b = PetEnv(EnvConfig(n_modules=11, action_mode="delta", task_params={"action_unbounded": 1.0, "thermal": 0.0}))
    oa, _ = a.reset(seed=4)
    ob, _ = b.reset(seed=4)
    assert np.array_equal(oa, ob) and a.thermal is None
