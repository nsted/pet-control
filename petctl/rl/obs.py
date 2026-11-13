"""Observation builder shared by the training env and the deployed policy so the two cannot drift apart.

Inputs are exactly what the real robot provides: tail-IMU quaternion + gyro, the joint angles (7 on the real robot) (from MIT feedback), and the previous
actions. Absolute yaw is NOT used (the BNO085 heading drifts); the gravity direction in the IMU frame replaces the quaternion.
Velocity comes from a window of positions because MIT velocity feedback clips at +-0.5 rad/s.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field

import numpy as np

from petctl.rl.config import EnvConfig


TOUCH_MEMORY_S = (0.5, 3.0)       # time constants (s) of the running averages of the contact signals
TOUCH_LONG_S = (10.0, 30.0)         # extra slow averages, appended after the touch block when the task sets touch_long
TOUCH_ERR_SCALE = 3.0            # rad -> observation units for the commanded-minus-measured position error


@dataclass
class Snapshot:
    """One sample of what the policy may see. ``quat`` is (w, x, y, z), IMU frame -> world (world z up)."""

    joint_pos: np.ndarray            # (n_joints,) rad
    quat: np.ndarray                 # (4,)
    gyro: np.ndarray                 # (3,) rad/s
    cap_nm: np.ndarray               # (n_joints,) current per-joint torque cap, N.m
    extra: np.ndarray = field(default_factory=lambda: np.zeros(0, dtype=np.float32))   # task-specific entries (goal direction ...), supplied by the caller
    torque: np.ndarray | None = None  # (n_joints,) N.m, from the MIT reply frame (used by tasks with touch_obs); zeros when omitted
    target: np.ndarray | None = None  # (n_joints,) rad, the position target last commanded; zeros when omitted

    def __post_init__(self) -> None:
        """Fill omitted reply-frame fields with zeros of the joint count."""
        if self.torque is None:
            self.torque = np.zeros(len(self.joint_pos))
        if self.target is None:
            self.target = np.zeros(len(self.joint_pos))


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
        from petctl.rl.tasks import TASKS            # local import: tasks import the sim, which obs consumers may not need

        self.extra_dim = TASKS[cfg.task].extra_obs_dim
        self.touch = TASKS[cfg.task].touch_obs
        self.touch_off = float(cfg.task_params.get("touch_off", 0.0)) > 0.0          # ablation: zero the contact channels
        self.touch_long = self.touch and float(cfg.task_params.get("touch_long", 0.0)) > 0.0
        self.n_joints = nj = cfg.n_joints
        self.size = 3 + 3 + nj + nj + nj + cfg.history * nj + self.extra_dim + (8 * nj if self.touch else 0) + (4 * nj if self.touch_long else 0)
        self._touch_prev = np.zeros(2 * self.n_joints)
        self._touch_ema = np.zeros((2, 2 * self.n_joints))
        self._touch_slow = np.zeros((2, 2 * self.n_joints))
        self._pos: deque[np.ndarray] = deque(maxlen=cfg.pos_window + 1)
        self._act: deque[np.ndarray] = deque(maxlen=cfg.history)

    def reset(self, first: Snapshot) -> None:
        """Clear the histories using the first snapshot."""
        self._touch_prev = np.zeros(2 * self.n_joints)
        self._touch_ema = np.zeros((2, 2 * self.n_joints))
        self._touch_slow = np.zeros((2, 2 * self.n_joints))
        self._pos.clear()
        self._act.clear()
        for _ in range(self.cfg.pos_window + 1):
            self._pos.append(first.joint_pos.copy())
        for _ in range(self.cfg.history):
            self._act.append(np.zeros(self.n_joints))

    def push_action(self, action: np.ndarray) -> None:
        """Record the action just applied."""
        self._act.append(np.asarray(action, float).copy())

    def build(self, snap: Snapshot) -> np.ndarray:
        """Return the observation vector for ``snap`` (call after ``push_action`` for the previous step)."""
        self._pos.append(snap.joint_pos.copy())
        w = len(self._pos) - 1
        vel = (self._pos[-1] - self._pos[0]) / (w * self.dt)
        parts = [gravity_in_body(snap.quat), snap.gyro, snap.joint_pos, np.clip(vel, -10, 10), snap.cap_nm / 0.164,
                 np.concatenate(list(self._act)), np.asarray(snap.extra, float).reshape(-1)[:self.extra_dim]]
        if self.touch:                                       # contact is inferred from the MIT reply: torque and tracking error, this tick and the previous one
            cur = np.concatenate([np.clip(snap.torque / np.maximum(snap.cap_nm, 1e-6), -1.5, 1.5), np.clip((snap.target - snap.joint_pos) * TOUCH_ERR_SCALE, -3.0, 3.0)])
            cur = np.zeros_like(cur) if self.touch_off else cur
            for k, tau in enumerate(TOUCH_MEMORY_S):                 # running averages over ~0.5 s and ~3 s: memory of where and how hard the body was blocked
                self._touch_ema[k] += (self.dt / tau) * (cur - self._touch_ema[k])
            parts.append(np.concatenate([cur, self._touch_prev, self._touch_ema[0], self._touch_ema[1]]))
            self._touch_prev = cur
            if self.touch_long:                                  # 10 s / 30 s memory: what was already tried a while ago
                for k, tau in enumerate(TOUCH_LONG_S):
                    self._touch_slow[k] += (self.dt / tau) * (cur - self._touch_slow[k])
                parts.append(np.concatenate(list(self._touch_slow)))
        return np.concatenate(parts).astype(np.float32)
