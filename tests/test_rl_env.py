"""RL template: observation builder, Gymnasium API compliance, determinism, torque-cap respect."""
from __future__ import annotations

import numpy as np
import pytest

pytest.importorskip("gymnasium")
pytest.importorskip("mujoco")

from gymnasium.utils.env_checker import check_env  # noqa: E402

from petctl.rl.config import EnvConfig  # noqa: E402
from petctl.rl.env import PetEnv  # noqa: E402
from petctl.rl.obs import ObsBuilder, Snapshot, gravity_in_body  # noqa: E402


def test_gravity_direction_for_identity_quaternion() -> None:
    assert np.allclose(gravity_in_body(np.array([1.0, 0, 0, 0])), [0, 0, -1])
    # 90 deg roll about x: body y points up in world -> gravity along -y_body... check magnitude and orthogonality to rotation axis
    g = gravity_in_body(np.array([np.cos(np.pi / 4), np.sin(np.pi / 4), 0, 0]))
    assert np.isclose(np.linalg.norm(g), 1.0) and abs(g[0]) < 1e-9


def test_obs_builder_shape_and_velocity_window() -> None:
    cfg = EnvConfig()
    ob = ObsBuilder(cfg)
    snap = Snapshot(np.zeros(7), np.array([1.0, 0, 0, 0]), np.zeros(3), np.full(7, 0.164))
    ob.reset(snap)
    o0 = ob.build(snap)
    assert o0.shape == (ob.size,)
    snap2 = Snapshot(np.full(7, 0.1), snap.quat, snap.gyro, snap.cap_nm)
    ob.push_action(np.zeros(7))
    o1 = ob.build(snap2)
    assert o1[9 + 7] > 0                                   # velocity estimate positive after moving up


def test_env_passes_gymnasium_checker() -> None:
    check_env(PetEnv(EnvConfig(episode_s=1.0)), skip_render_check=True)


def test_env_is_deterministic_for_a_seed() -> None:
    def rollout() -> np.ndarray:
        env = PetEnv(EnvConfig(episode_s=1.0))
        o, _ = env.reset(seed=3)
        acts = np.random.default_rng(0).uniform(-1, 1, (10, 7))
        for a in acts:
            o, *_ = env.step(a)
        return o
    assert np.allclose(rollout(), rollout())


def test_applied_torque_never_exceeds_cap() -> None:
    env = PetEnv(EnvConfig(episode_s=2.0))
    env.reset(seed=1)
    for _ in range(20):
        env.step(np.ones(7))
        cap = env.sim.model.actuator_forcerange[:, 1]
        assert np.all(np.abs(env.sim.joint_torque()) <= cap + 1e-9)
