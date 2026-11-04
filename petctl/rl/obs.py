"""Observation builder shared by the training env and the deployed policy so the two cannot drift apart.

Inputs are exactly what the real robot provides: tail-IMU quaternion + gyro, the 7 joint angles (from MIT feedback), and the previous
actions. Absolute yaw is NOT used (the BNO085 heading drifts); the gravity direction in the IMU frame replaces the quaternion.
Velocity comes from a window of positions because MIT velocity feedback clips at +-0.5 rad/s.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass

import numpy as np

from petctl.rl.config import N_JOINTS, EnvConfig


@dataclass
class Snapshot:
    """One sample of what the policy may see. ``quat`` is (w, x, y, z), IMU frame -> world (world z up)."""

    joint_pos: np.ndarray            # (7,) rad
    quat: np.ndarray                 # (4,)
    gyro: np.ndarray                 # (3,) rad/s
    cap_nm: np.ndarray               # (7,) current per-joint torque cap, N.m


def gravity_in_body(quat: np.ndarray) -> np.ndarray:
    """Unit gravity direction expressed in the IMU frame from a (w,x,y,z) quaternion (body->world)."""
    w, x, y, z = quat / max(np.linalg.norm(quat), 1e-9)
    # third row of the rotation matrix R(body->world) gives world-z in body coordinates; gravity is -z_world
    zb = np.array([2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)])
    return -zb


class ObsBuilder:
    """Stateful: keeps the joint-position window and action history. Call ``reset`` at the start of an episode."""

    def __init__(self, cfg: EnvConfig) -> None:
        """Create a builder for ``cfg`` (history length, velocity window, control rate)."""
        self.cfg = cfg
        self.dt = 1.0 / cfg.control_hz
        self.size = 3 + 3 + N_JOINTS + N_JOINTS + N_JOINTS + cfg.history * N_JOINTS
        self._pos: deque[np.ndarray] = deque(maxlen=cfg.pos_window + 1)
        self._act: deque[np.ndarray] = deque(maxlen=cfg.history)

    def reset(self, first: Snapshot) -> None:
        """Clear the histories using the first snapshot."""
        self._pos.clear()
        self._act.clear()
        for _ in range(self.cfg.pos_window + 1):
            self._pos.append(first.joint_pos.copy())
        for _ in range(self.cfg.history):
            self._act.append(np.zeros(N_JOINTS))

    def push_action(self, action: np.ndarray) -> None:
        """Record the action just applied."""
        self._act.append(np.asarray(action, float).copy())

    def build(self, snap: Snapshot) -> np.ndarray:
        """Return the observation vector for ``snap`` (call after ``push_action`` for the previous step)."""
        self._pos.append(snap.joint_pos.copy())
        w = len(self._pos) - 1
        vel = (self._pos[-1] - self._pos[0]) / (w * self.dt)
        return np.concatenate([gravity_in_body(snap.quat), snap.gyro, snap.joint_pos, np.clip(vel, -10, 10), snap.cap_nm / 0.164,
                               np.concatenate(list(self._act))]).astype(np.float32)
