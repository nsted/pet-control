"""STAGE_1_GATES 1.2 / 1.3: the GIM3505-8 MIT reply frame (task knob ``mit_frame``) at n = 11: quantised direct velocity, +-2 N.m torque field, one delay for the frame."""
from __future__ import annotations

import numpy as np
import pytest

pytest.importorskip("gymnasium")
pytest.importorskip("mujoco")

from petctl.rl.config import MIT_TORQUE_Q_NM, MIT_TORQUE_RANGE_NM, MIT_VEL_Q_RAD_S, MIT_VEL_RANGE_RAD_S, EnvConfig  # noqa: E402
from petctl.rl.env import PetEnv  # noqa: E402

NJ = 10


def _env(reply_dr: bool, **extra: float) -> PetEnv:
    tp = {"mit_frame": 1.0, "action_unbounded": 1.0, **extra}
    if reply_dr:
        tp["reply_dr"] = 1.0
    env = PetEnv(EnvConfig(n_modules=11, episode_s=12.0, action_mode="delta", task_params=tp))
    env.cfg.dr.scale = 0.0                                   # noise, delay jitter and drops off (reply delay stays 1 tick when reply_dr is on)
    return env


def _rollout(env: PetEnv, steps: int) -> tuple[np.ndarray, np.ndarray]:
    """Returns (obs velocity block, true qvel at the same tick) arrays of shape (steps, NJ)."""
    rng = np.random.default_rng(0)
    env.reset(seed=1)
    o_v, t_v = [], []
    a = rng.uniform(-1, 1, NJ)
    for k in range(steps):
        if k % 10 == 0:
            a = rng.uniform(-1, 1, NJ)
        o, *_ = env.step(a)
        o_v.append(o[6 + NJ:6 + 2 * NJ].copy())
        t_v.append(env.sim.joint_vel().copy())
    return np.array(o_v), np.array(t_v)


def test_constants_match_the_gate() -> None:
    assert MIT_VEL_Q_RAD_S == pytest.approx(60 / 4096) and MIT_TORQUE_Q_NM == pytest.approx(4 / 4096) and MIT_TORQUE_RANGE_NM == 2.0 and MIT_VEL_RANGE_RAD_S == 30.0


def test_velocity_matches_truth_without_delay_or_noise() -> None:
    o_v, t_v = _rollout(_env(False), 200)
    assert np.sqrt(np.mean((o_v - t_v) ** 2)) <= 0.05
    assert np.abs(o_v).max() > 0.05                          # the rollout actually moves
    q = o_v / MIT_VEL_Q_RAD_S
    assert np.allclose(q, np.round(q), atol=1e-4)            # values sit on the 60/4096 grid


def test_velocity_is_delayed_like_the_frame() -> None:
    env = _env(True)
    assert env._reply_buf is None or True
    o_v, t_v = _rollout(env, 200)
    assert env._reply_buf is not None and env._reply_buf.maxlen == 2          # 1 tick
    assert np.sqrt(np.mean((o_v[1:] - t_v[:-1]) ** 2)) <= MIT_VEL_Q_RAD_S      # obs = truth from one tick earlier, to within the quantisation step
    assert np.sqrt(np.mean((o_v[1:] - t_v[1:]) ** 2)) > 3 * np.sqrt(np.mean((o_v[1:] - t_v[:-1]) ** 2))


def test_torque_field_saturates_and_is_on_its_grid() -> None:
    env = _env(False)
    env.reset(seed=0)
    for _ in range(20):
        o, *_ = env.step(np.ones(NJ) * 5.0)
    snap = env._snapshot()
    assert np.abs(snap.torque).max() <= MIT_TORQUE_RANGE_NM + 1e-9
    q = snap.torque / MIT_TORQUE_Q_NM
    assert np.allclose(q, np.round(q), atol=1e-6)
