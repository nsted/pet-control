"""Observation builder shared by the training env and the deployed policy so the two cannot drift apart.

Inputs are exactly what the real robot provides: tail-IMU quaternion + gyro, the joint angles (7 on the real robot) (from MIT feedback), and the previous
actions. Absolute yaw is NOT used (the BNO085 heading drifts); the gravity direction in the IMU frame replaces the quaternion.
Velocity comes from a window of positions (legacy GL40 II: its MIT velocity feedback clipped at +-0.5 rad/s) unless task knob ``mit_frame`` is set: then it is the GIM3505-8 velocity field (+-30 rad/s, 12 bit).
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field

import numpy as np

from petctl.rl.config import MIT_VEL_RANGE_RAD_S, DELTA_LIMIT_RAD, DELTA_STEP_RAD, HOLD_EMA_BASE, HOLD_EMA_RANGE, EnvConfig


TOUCH_MEMORY_S = (0.5, 3.0)       # time constants (s) of the running averages of the contact signals
TOUCH_LONG_S = (10.0, 30.0)         # extra slow averages, appended after the touch block when the task sets touch_long
CAP_EMA_S = 0.5                  # contact-memory time constant (s) of the capacitive touch EMA
TOUCH_ERR_SCALE = 3.0            # rad -> observation units for the commanded-minus-measured position error


def next_target(cfg: EnvConfig, tgt: np.ndarray, a: np.ndarray) -> np.ndarray:
    """The joint-angle target after one control tick with action ``a`` (already clipped to [-1, 1]); shared by ``PetEnv.step`` and ``GaitPolicy.step`` so they cannot drift.

    ``abs``: EMA of ``action_scale_rad * a``, slew-limited. ``delta``: ``tgt + DELTA_STEP_RAD * a`` clipped to +-``DELTA_LIMIT_RAD``, slew-limited, no EMA.
    ``hold``: ``a`` has ``n_joints + 1`` entries; ``h = (a[-1] + 1) / 2`` sets the EMA coefficient ``0.3 + 0.7 h`` of the ``abs`` mapping (h = 0: EMA 0.3, h = 1: frozen), slew-limited.
    """
    if cfg.action_mode == "hold":
        nj = len(tgt)
        ema = HOLD_EMA_BASE + HOLD_EMA_RANGE * hold_value(a)
        smoothed = ema * tgt + (1 - ema) * (cfg.action_scale_rad * np.asarray(a)[:nj])
    elif cfg.action_mode == "delta":
        smoothed = np.clip(tgt + DELTA_STEP_RAD * a, -DELTA_LIMIT_RAD, DELTA_LIMIT_RAD)
    else:
        smoothed = cfg.action_ema * tgt + (1 - cfg.action_ema) * (cfg.action_scale_rad * a)
    return tgt + np.clip(smoothed - tgt, -cfg.max_target_step_rad, cfg.max_target_step_rad)


def unbounded(cfg: EnvConfig) -> bool:
    """True when the joint action has NO angle range (task param ``action_unbounded``): the policy's joint outputs are raw angle targets in rad (``action_scale_rad`` x action, scale 1.0 = rad),
    any size and sign, multi-turn; observed angles are wrapped to [-pi, pi). Only the hold channel (not an angle) stays in [-1, 1]. Motor speed is still bounded by the protective slew limit."""
    return float(cfg.task_params.get("action_unbounded", 0.0)) > 0.0


def wrap_angle(x: np.ndarray | float) -> np.ndarray:
    """Wrap angles (rad) to [-pi, pi): a joint turned 2 pi looks identical to the policy (and is, geometrically)."""
    return (np.asarray(x, float) + np.pi) % (2.0 * np.pi) - np.pi


TURN_CLIP = 3.0                  # turn_obs: turn counts are clipped to +-3 (3 full turns = 6 pi rad = ~19 rad)


def turn_count(x: np.ndarray | float) -> np.ndarray:
    """Whole turns of an unwrapped angle: ``round((x - wrap_angle(x)) / 2 pi)``, clipped to +-``TURN_CLIP``; with ``wrap_angle(x)`` it identifies ``x`` uniquely (within the clip)."""
    x = np.asarray(x, float)
    return np.clip(np.rint((x - wrap_angle(x)) / (2.0 * np.pi)), -TURN_CLIP, TURN_CLIP)


def clip_action(cfg: EnvConfig, a: np.ndarray) -> np.ndarray:
    """The action the environment applies: clipped to [-1, 1], except the joint entries when ``unbounded(cfg)`` (the hold channel, last entry in ``hold`` mode, is still clipped)."""
    a = np.asarray(a, float)
    if not unbounded(cfg):
        return np.clip(a, -1.0, 1.0)
    out = a.copy()
    if cfg.action_mode == "hold":
        out[-1] = np.clip(out[-1], -1.0, 1.0)
    return out


def hold_value(a: np.ndarray) -> float:
    """The hold channel ``h`` in [0, 1] of a ``hold``-mode action (last entry, mapped from [-1, 1])."""
    return float((float(np.clip(np.asarray(a, float)[-1], -1.0, 1.0)) + 1.0) / 2.0)


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
    capacitive: np.ndarray | None = None  # (6 * n_modules,) post-only face sensors: touch (n,3) then pressure (n,3), faces (left, right, middle); zeros when omitted
    joint_vel: np.ndarray | None = None   # (n_joints,) rad/s, the MIT frame's velocity field (task knob ``mit_frame``); None = derive the velocity from a position window

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

        self.extra_dim = TASKS[cfg.task].extra_dim(cfg.task_params)
        self.touch = TASKS[cfg.task].touch_obs
        self.touch_off = float(cfg.task_params.get("touch_off", 0.0)) > 0.0          # ablation: zero the contact channels
        self.touch_long = self.touch and float(cfg.task_params.get("touch_long", 0.0)) > 0.0
        self.n_joints = nj = cfg.n_joints
        self.capacitive = float(cfg.task_params.get("capacitive_obs", 0.0)) > 0.0         # post-only face sensors; block appended after everything else (9 per module)
        self.capacitive_off = float(cfg.task_params.get("capacitive_off", 0.0)) > 0.0      # ablation: zero the capacitive block
        self.imu_off = float(cfg.task_params.get("imu_off", 0.0)) > 0.0                    # ablation: zero the IMU channels (gravity direction + gyro)
        self.mit_frame = float(cfg.task_params.get("mit_frame", 0.0)) > 0.0                # GIM3505 reply frame: the velocity block is the quantised MIT velocity field (+-30 rad/s), not a position-window estimate
        self.n_modules = cfg.n_modules
        self.unbounded = unbounded(cfg)                                                    # no angle range: observed joint angles and targets are wrapped to [-pi, pi)
        self.delta = cfg.action_mode in ("delta", "hold")                                  # the current target vector (/ DELTA_LIMIT_RAD) is appended last (delta and hold modes)
        self.turn_obs = self.unbounded and float(cfg.task_params.get("turn_obs", 0.0)) > 0.0         # multi-turn state: whole turns of each joint angle (and of the commanded target in delta/hold modes), appended last
        self.size = (3 + 3 + nj + nj + nj + cfg.history * nj + self.extra_dim + (8 * nj if self.touch else 0) + (4 * nj if self.touch_long else 0)
                     + (9 * cfg.n_modules if self.capacitive else 0) + (nj if self.delta else 0) + (self.n_turn_entries() if self.turn_obs else 0))
        self._cap_ema = np.zeros(3 * cfg.n_modules)
        self._touch_prev = np.zeros(2 * self.n_joints)
        self._touch_ema = np.zeros((2, 2 * self.n_joints))
        self._touch_slow = np.zeros((2, 2 * self.n_joints))
        self._pos: deque[np.ndarray] = deque(maxlen=cfg.pos_window + 1)
        self._act: deque[np.ndarray] = deque(maxlen=cfg.history)

    def n_turn_entries(self) -> int:
        """Size of the turn-count block: one entry per joint angle, plus one per commanded target in ``delta`` / ``hold`` modes."""
        return self.n_joints * (2 if self.delta else 1)

    def blocks(self) -> list[tuple[str, int]]:
        """Ordered ``(name, size)`` blocks of the observation vector (they sum to ``size``); used to map warm-start columns between tasks whose layouts differ."""
        nj = self.n_joints
        out = [("base", 3 + 3 + nj + nj + nj + self.cfg.history * nj + self.extra_dim)]
        if self.touch:
            out.append(("touch", 8 * nj))
        if self.touch_long:
            out.append(("touch_long", 4 * nj))
        if self.capacitive:
            out.append(("capacitive", 9 * self.cfg.n_modules))
        if self.delta:
            out.append(("target", nj))
        if self.turn_obs:
            out.append(("turns", self.n_turn_entries()))
        return out

    def reset(self, first: Snapshot) -> None:
        """Clear the histories using the first snapshot."""
        self._touch_prev = np.zeros(2 * self.n_joints)
        self._touch_ema = np.zeros((2, 2 * self.n_joints))
        self._touch_slow = np.zeros((2, 2 * self.n_joints))
        self._cap_ema = np.zeros(3 * self.n_modules)
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
        if self.mit_frame and snap.joint_vel is not None:
            vel = np.clip(np.asarray(snap.joint_vel, float), -MIT_VEL_RANGE_RAD_S, MIT_VEL_RANGE_RAD_S)
        grav, gyro = (np.zeros(3), np.zeros(3)) if self.imu_off else (gravity_in_body(snap.quat), snap.gyro)
        parts = [grav, gyro, wrap_angle(snap.joint_pos) if self.unbounded else snap.joint_pos, vel if self.mit_frame else np.clip(vel, -10, 10), snap.cap_nm / 0.164,
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
        if self.capacitive:                                  # [touch (n,3), pressure (n,3), 0.5 s EMA of touch (n,3)], faces (left, right, middle)
            nf = 3 * self.n_modules
            raw = np.zeros(2 * nf) if snap.capacitive is None else np.asarray(snap.capacitive, float).reshape(-1)[:2 * nf]
            raw = np.zeros(2 * nf) if self.capacitive_off else raw
            self._cap_ema += (self.dt / CAP_EMA_S) * (raw[:nf] - self._cap_ema)
            parts.append(np.concatenate([raw, self._cap_ema]))
        if self.delta:                                       # the brain's own commanded targets (deployable), appended last
            parts.append((wrap_angle(snap.target) if self.unbounded else np.asarray(snap.target, float)) / DELTA_LIMIT_RAD)
        if self.turn_obs:                                    # whole turns (angle - wrapped angle) / 2 pi, clipped to +-TURN_CLIP: the part the wrapped angles above cannot show
            parts.append(turn_count(snap.joint_pos))
            if self.delta:
                parts.append(turn_count(snap.target))
        return np.concatenate(parts).astype(np.float32)
