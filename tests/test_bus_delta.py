"""Bus-current model/clamp/metrics, head-frame goal delta + delta DR, and reply-frame DR (Regimen v2 M0-D)."""
from __future__ import annotations

from collections import deque

import numpy as np
import pytest

pytest.importorskip("mujoco")
import mujoco  # noqa: E402

from petctl.config import REGIMEN_BUS_MEAN_A, REGIMEN_BUS_PEAK_A  # noqa: E402
from petctl.rl.config import BUS_A_PER_NM2, EnvConfig  # noqa: E402
from petctl.rl.env import REPLY_TORQUE_NOISE_NM, PetEnv  # noqa: E402
from petctl.sim.bus import BusClamp, bus_current_a, idle_current_a  # noqa: E402
from petctl.sim.model_builder import BuildParams, PetSim  # noqa: E402

CAP = 0.5


def _sim(n: int = 8) -> PetSim:
    sim = PetSim(BuildParams(n_modules=n, dt=0.004, delay_s=0.0, cap_nm=(CAP,) * (n - 1)))
    sim.step(np.zeros(n - 1), 1.0)
    return sim


def _target(n_sat: int, nj: int = 7) -> np.ndarray:
    t = np.zeros(nj)
    t[:n_sat] = 3.0                                              # far beyond reach: these joints saturate at the cap
    return t


# ---------------------------------------------------------------- current model / clamp
def test_idle_current_model() -> None:
    assert idle_current_a(7) == pytest.approx(0.6) and idle_current_a(14) == pytest.approx(0.95)
    assert bus_current_a(np.full(7, 0.5), BUS_A_PER_NM2) == pytest.approx(0.6 + 13.3 * 7 * 0.25)


def test_current_matches_formula_on_saturated_joints() -> None:
    """One saturated joint: analytic formula within 2%. 1-7 saturated: reported current equals the formula on the measured torques, and each saturated joint sits at the cap."""
    sim = _sim()
    sim.bus = BusClamp(sim.model, BUS_A_PER_NM2, 1e9, clamp=False)
    sim.step(_target(1), 0.05)
    tick = sim.bus.end_tick()
    assert tick["mean"] == pytest.approx(idle_current_a(7) + BUS_A_PER_NM2 * CAP ** 2, rel=0.02)
    assert tick["peak"] >= tick["mean"] and tick["over"] == 0.0
    for n_sat in (2, 3, 7):
        sim = _sim()
        sim.bus = BusClamp(sim.model, BUS_A_PER_NM2, 1e9, clamp=False)
        sim.step(_target(n_sat), 0.004)                                  # a single physics substep
        tau = sim.joint_torque()
        assert np.allclose(np.abs(tau[:n_sat]), CAP)
        assert sim.bus.end_tick()["mean"] == pytest.approx(0.25 + 0.05 * 7 + 13.3 * float(np.sum(tau ** 2)), rel=1e-9)
        assert sim.bus.idle == pytest.approx(0.6)


def test_demand_equals_actuator_force_unclamped() -> None:
    sim = _sim()
    bus = BusClamp(sim.model, BUS_A_PER_NM2, 1e9, clamp=False)
    for k in range(5):
        sim.data.ctrl[:] = _target(3) * (k + 1) / 5
        d = bus.demand(sim.model, sim.data, sim.model.actuator_forcerange[:, 1])
        mujoco.mj_step(sim.model, sim.data)
        assert np.allclose(d, sim.data.actuator_force, atol=1e-9)


def test_clamp_scales_over_peak_and_leaves_under_unchanged() -> None:
    a, b = _sim(), _sim()
    a.bus = BusClamp(a.model, BUS_A_PER_NM2, REGIMEN_BUS_PEAK_A, clamp=True)
    # 4 saturated joints demand 0.6 + 13.3 * 4 * 0.25 = 13.9 A > 10 A -> clamped to 10 A
    a.step(_target(4), 0.05)
    t = a.bus.end_tick()
    assert t["peak"] == pytest.approx(13.9, rel=0.03) and t["over"] == 1.0
    assert t["applied_peak"] <= REGIMEN_BUS_PEAK_A * 1.01
    assert t["applied_peak"] == pytest.approx(REGIMEN_BUS_PEAK_A, rel=0.01)
    assert np.all(a.model.actuator_forcerange[:, 1] == CAP) and np.all(a.model.actuator_forcerange[:, 0] == -CAP)     # restored after the step
    # 2 saturated joints: 0.6 + 6.65 = 7.25 A < 10 A -> identical trajectory with and without the clamp
    c = _sim()
    c.bus = BusClamp(c.model, BUS_A_PER_NM2, REGIMEN_BUS_PEAK_A, clamp=True)
    c.step(_target(2), 0.2)
    b.step(_target(2), 0.2)
    assert c.bus.end_tick()["over"] == 0.0
    assert np.array_equal(c.data.qpos, b.data.qpos) and np.array_equal(c.data.actuator_force, b.data.actuator_force)


def test_bus_off_by_default_and_metrics_present_when_on() -> None:
    env = PetEnv(EnvConfig(task="goal_reach", episode_s=2.0))
    env.reset(seed=0)
    assert env.bus is None and env.sim.bus is None
    _, _, _, _, info = env.step(np.zeros(7))
    assert "bus_mean_a" not in info["metrics"] and "bus" not in info["terms"]
    env = PetEnv(EnvConfig(task="goal_reach", episode_s=2.0, task_params={"bus_model": 1.0}))
    env.reset(seed=0)
    for _ in range(5):
        _, _, _, _, info = env.step(np.ones(7))
    m = info["metrics"]
    for k in ("bus_mean_a", "bus_peak_a", "bus_peak_applied_a", "frac_ticks_over_10a"):
        assert k in m
    assert m["bus_mean_a"] >= idle_current_a(7) - 1e-9 and m["bus_peak_a"] >= m["bus_mean_a"] and 0.0 <= m["frac_ticks_over_10a"] <= 1.0
    assert "bus" not in info["terms"]                                  # no penalty unless bus_penalty > 0


def test_bus_penalty_on_two_second_running_mean() -> None:
    env = PetEnv(EnvConfig(task="goal_reach", episode_s=5.0, task_params={"bus_penalty": 2.0}))
    env.reset(seed=0)
    assert env._bus_win.maxlen == 40
    env._bus_win = deque([6.0] * 40, maxlen=40)
    _, _, _, _, info = env.step(np.zeros(7))
    expect = -2.0 * max(0.0, float(np.mean(env._bus_win)) - REGIMEN_BUS_MEAN_A)
    assert info["terms"]["bus"] == pytest.approx(expect) and expect < -3.0
    env2 = PetEnv(EnvConfig(task="goal_reach", episode_s=5.0, task_params={"bus_penalty": 2.0}))
    env2.reset(seed=0)
    _, _, _, _, info = env2.step(np.zeros(7))
    assert info["terms"]["bus"] == 0.0                                 # idle ~0.6 A: under the 4 A budget


# ---------------------------------------------------------------- head-frame delta
def _quat(yaw: float, roll: float) -> np.ndarray:
    qz = np.array([np.cos(yaw / 2), 0, 0, np.sin(yaw / 2)])
    qx = np.array([np.cos(roll / 2), np.sin(roll / 2), 0, 0])
    w1, x1, y1, z1 = qz
    w2, x2, y2, z2 = qx
    return np.array([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2, w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2])


def _head_env(**tp: float) -> PetEnv:
    env = PetEnv(EnvConfig(task="goal_reach", episode_s=5.0, task_params={"delta_frame": 1.0, "goal_dist_min": 0.5, "goal_dist_max": 0.5, "bearing_deg": 60.0, **tp}))
    env.cfg.dr.scale = 0.0
    return env


def test_delta_frame_default_unchanged_and_dim_adapts() -> None:
    old = PetEnv(EnvConfig(task="goal_reach"))
    new = _head_env()
    assert old.obs_builder.extra_dim == 4 and new.obs_builder.extra_dim == 5
    assert new.observation_space.shape[0] == old.observation_space.shape[0] + 1
    seq = PetEnv(EnvConfig(task="goal_sequence", task_params={"delta_frame": 1.0}))
    assert seq.obs_builder.extra_dim == 8                              # the knob does not alter the sequence task
    obs, _ = new.reset(seed=3)
    assert obs.shape == new.observation_space.shape


@pytest.mark.parametrize("yaw,roll", [(0.0, 0.0), (np.pi / 2, 0.0), (1.0, np.pi / 2), (-2.0, np.pi)])
def test_head_frame_delta_follows_head_rotation(yaw: float, roll: float) -> None:
    env = _head_env()
    env.reset(seed=1)
    sim, task = env.sim, env.task
    base = task.extra_obs(sim).copy()
    assert np.linalg.norm(base[:3]) == pytest.approx(1.0, abs=1e-5)
    sim.data.qpos[3:7] = _quat(yaw, roll)                               # rotate the whole robot about its origin (free joint), recompute kinematics
    mujoco.mj_forward(sim.model, sim.data)
    got = task.extra_obs(sim)
    R = sim.data.site_xmat[sim.model.site("head").id].reshape(3, 3)
    v = task.goal - sim.head_pos()
    v[2] = 0.0
    d = np.linalg.norm(v)
    assert np.allclose(got[:3], R.T @ v / d, atol=1e-5)
    assert np.allclose(R @ got[:3], v / d, atol=1e-5)                    # back in the world frame it points at the goal
    assert got[3] == pytest.approx(min(d, 2.0), abs=1e-5) and got[4] == pytest.approx(min(d, 0.3) / 0.3, abs=1e-5)
    if yaw != 0.0 or roll != 0.0:
        assert not np.allclose(got[:3], base[:3], atol=1e-2) or abs(yaw) < 1e-9


def test_near_field_channel_resolves_close_goal() -> None:
    env = _head_env()
    env.reset(seed=1)
    task, sim = env.task, env.sim
    h = sim.head_pos()
    for dist, far, near in [(0.05, 0.05, 0.05 / 0.3), (0.2, 0.2, 0.2 / 0.3), (0.6, 0.6, 1.0), (3.0, 2.0, 1.0)]:
        task.goal = np.array([h[0] + dist, h[1], h[2]])
        o = task.extra_obs(sim)
        assert o[3] == pytest.approx(far, abs=1e-5) and o[4] == pytest.approx(near, abs=1e-5)


def test_delta_dr_statistics_and_scale_zero() -> None:
    env = _head_env()
    env.reset(seed=1)
    t = env.task
    biases, sigmas, delays = [], [], []
    for s in range(400):
        env.cfg.dr.scale = 1.0
        t._init_delta_dr(s)
        biases.append(np.linalg.norm(t._d_bias))
        sigmas.append(t._d_sigma)
        delays.append(t._d_delay)
    assert 0.0 <= min(biases) and max(biases) <= 0.05 and np.mean(biases) == pytest.approx(0.025, abs=0.004)
    assert 0.01 <= min(sigmas) and max(sigmas) <= 0.02 and np.mean(sigmas) == pytest.approx(0.015, abs=0.001)
    assert min(delays) == 0 and max(delays) <= 3 and np.mean(delays) == pytest.approx(1.5, abs=0.2)      # U(0, 0.15 s) at 20 Hz = U(0, 3) ticks
    env.cfg.dr.scale = 0.0
    t._init_delta_dr(5)
    assert np.allclose(t._d_bias, 0.0) and t._d_sigma == 0.0 and t._d_delay == 0
    clean = t.extra_obs(env.sim).copy()
    env.cfg.dr.scale = 1.0
    t._init_delta_dr(7)
    t._d_delay = 0
    t._d_hist = deque(maxlen=1)
    t._d_sigma, t._d_bias = 0.015, np.array([0.03, 0.0, 0.0])
    h = env.sim.head_pos()
    t.goal = np.array([h[0] + 1.0, h[1], h[2]])
    R = env.sim.data.site_xmat[env.sim.model.site("head").id].reshape(3, 3)
    vs = []
    for _ in range(4000):
        o = t.extra_obs(env.sim)
        vs.append(R @ o[:3] * o[3])                                      # reconstruct the (noisy) world delta (far channel = d / 1 m)
    vs = np.array(vs)
    assert vs[:, 0].mean() == pytest.approx(1.03, abs=0.002) and vs[:, 1].mean() == pytest.approx(0.0, abs=0.002)
    assert vs[:, 0].std() == pytest.approx(0.015, rel=0.1) and vs[:, 1].std() == pytest.approx(0.015, rel=0.1)
    assert clean.shape == (5,)


def test_delta_delay_and_fixed_bias_knob() -> None:
    env = _head_env(delta_bias_m=0.04)
    env.reset(seed=2)
    t = env.task
    assert np.linalg.norm(t._d_bias) == pytest.approx(0.04)             # ablation knob: exact bias magnitude, independent of dr.scale
    env.cfg.dr.scale = 1.0
    t._init_delta_dr(11)
    t._d_delay, t._d_sigma, t._d_bias = 3, 0.0, np.zeros(3)
    t._d_hist = deque(maxlen=4)
    h = env.sim.head_pos()
    seen = []
    for k in range(8):
        t.goal = np.array([h[0] + 0.1 * (k + 1), h[1], h[2]])
        seen.append(t.extra_obs(env.sim)[3])
    assert np.allclose(seen[3:], [0.1 * (k + 1) for k in range(5)], atol=1e-5)          # output lags the truth by 3 ticks
    assert np.allclose(seen[:3], 0.1, atol=1e-5)


# ---------------------------------------------------------------- reply-frame DR
def _reply_env(scale: float, **tp: float) -> PetEnv:
    env = PetEnv(EnvConfig(task="goal_reach", episode_s=5.0, task_params={"reply_dr": 1.0, **tp}))
    env.cfg.dr.scale = scale
    return env


def test_reply_dr_off_by_default_and_zero_at_scale_zero() -> None:
    env = PetEnv(EnvConfig(task="goal_reach"))
    env.reset(seed=0)
    assert env._reply_buf is None and env._reply_mult == 1.0
    env = _reply_env(0.0)
    env.reset(seed=0)
    assert env._reply_mult == 1.0 and env._reply_drop == 0.0 and env._reply_buf.maxlen == 2      # 1 tick of delay at scale 0


def test_reply_dr_ranges() -> None:
    mults, delays = [], set()
    env = _reply_env(1.0)
    for s in range(60):
        env.reset(seed=s)
        mults.append(env._reply_mult)
        delays.add(env._reply_buf.maxlen - 1)
        assert env._reply_drop == pytest.approx(0.05)
    assert 0.5 <= min(mults) and max(mults) <= 2.0 and min(mults) < 0.75 and max(mults) > 1.4
    assert delays == {1, 2}


def test_reply_dr_noise_delay_drop_statistics(monkeypatch: pytest.MonkeyPatch) -> None:
    env = _reply_env(1.0)
    env.reset(seed=4)
    mult, delay = env._reply_mult, env._reply_buf.maxlen - 1
    k = {"i": 0}

    def fake_torque() -> np.ndarray:
        k["i"] += 1
        return np.full(7, float(k["i"]))                                  # delivered value identifies which tick's frame arrived

    monkeypatch.setattr(env.sim, "joint_torque", fake_torque)
    n = 6000
    idx = []
    noise = []
    for _ in range(n):
        trq = env._snapshot().torque
        idx.append(np.round(trq[0]))
        noise.append(trq - np.round(trq))
    idx = np.array(idx)
    dropped = idx[1:] == idx[:-1]
    assert dropped.mean() == pytest.approx(0.05, abs=0.012)
    lag = np.arange(2, n + 1)[~dropped] - idx[1:][~dropped]               # frames arrive `delay` ticks late (call i delivers frame i - delay)
    assert np.all(lag == delay)
    assert np.std(np.concatenate(noise)) == pytest.approx(REPLY_TORQUE_NOISE_NM * mult, rel=0.1)


def test_reply_dr_does_not_change_default_stream() -> None:
    a = PetEnv(EnvConfig(task="goal_reach", episode_s=2.0))
    b = PetEnv(EnvConfig(task="goal_reach", episode_s=2.0))
    oa, _ = a.reset(seed=9)
    ob, _ = b.reset(seed=9)
    for _ in range(5):
        oa, *_ = a.step(np.ones(7) * 0.3)
        ob, *_ = b.step(np.ones(7) * 0.3)
    assert np.array_equal(oa, ob)
