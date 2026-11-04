"""Gymnasium environment for PET on the MuJoCo ground model. The action mapping and observation match deployment (see obs.py)."""

from __future__ import annotations

from typing import Any

import gymnasium as gym
import numpy as np
from gymnasium import spaces

from petctl.rl.config import N_JOINTS, EnvConfig
from petctl.rl.domain_rand import sample_params
from petctl.rl.obs import ObsBuilder, Snapshot
from petctl.rl.tasks import TASKS
from petctl.sim.model_builder import BuildParams, PetSim

SETTLE_S = 0.75


def _qmul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                     w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2])


class PetEnv(gym.Env):  # type: ignore[type-arg]
    """Action: 7 values in [-1, 1] -> joint-angle targets (rad) about the zero pose, EMA-filtered and slew-limited like the deployed path."""

    metadata = {"render_modes": []}

    def __init__(self, cfg: EnvConfig | None = None, base: BuildParams | None = None) -> None:
        """Create the env; ``base`` overrides nominal plant parameters (e.g. weighed masses)."""
        self.cfg = cfg or EnvConfig()
        if base is None:
            base = BuildParams(urdf_dir=self.cfg.urdf_dir, dt=self.cfg.sim_dt)
        self.base = base
        self.dt = 1.0 / self.cfg.control_hz
        self.obs_builder = ObsBuilder(self.cfg)
        self.observation_space = spaces.Box(-np.inf, np.inf, (self.obs_builder.size,), np.float32)
        self.action_space = spaces.Box(-1.0, 1.0, (N_JOINTS,), np.float32)
        self.max_steps = int(round(self.cfg.episode_s * self.cfg.control_hz))
        self.sim: PetSim | None = None

    # ------------------------------------------------------------------ helpers
    def _snapshot(self) -> Snapshot:
        assert self.sim is not None
        d, rng = self.sim.data, self._rng
        dr = self.cfg.dr
        pos = self.sim.joint_pos() + self._joint_off + rng.normal(0, dr.pos_noise_rad * dr.scale, N_JOINTS)
        quat = _qmul(self._tilt, d.sensor("quat").data.copy())
        gyro = d.sensor("gyro").data + rng.normal(0, dr.gyro_noise * dr.scale, 3)
        return Snapshot(pos, quat, gyro, self.sim.model.actuator_forcerange[:, 1].copy())

    def _up_dot(self) -> float:
        assert self.sim is not None
        return float(self.sim.data.site_xmat[self.sim.model.site("imu").id].reshape(3, 3)[2, 2])

    # ------------------------------------------------------------------ gym API
    def reset(self, *, seed: int | None = None, options: dict[str, Any] | None = None) -> tuple[np.ndarray, dict[str, Any]]:
        """Randomise the plant, settle the robot on the floor, and return the first observation."""
        super().reset(seed=seed)
        self._rng = self.np_random
        params, self.privileged = sample_params(self.cfg.dr, self._rng, self.base)
        self.sim = PetSim(params)
        dr = self.cfg.dr
        self._joint_off = self._rng.uniform(-dr.joint_offset_rad, dr.joint_offset_rad, N_JOINTS) * dr.scale
        ax = self._rng.normal(size=3)
        ang = self._rng.uniform(0, dr.imu_tilt_rad * dr.scale)
        self._tilt = np.concatenate([[np.cos(ang / 2)], np.sin(ang / 2) * ax / np.linalg.norm(ax)])
        self.sim.step(np.zeros(N_JOINTS), SETTLE_S)
        self.task = TASKS[self.cfg.task](self.cfg)
        self.task.reset(self.sim, self._rng)
        self._tgt = np.zeros(N_JOINTS)
        self._prev_action = np.zeros(N_JOINTS)
        self._t = 0
        snap = self._snapshot()
        self.obs_builder.reset(snap)
        return self.obs_builder.build(snap), {"privileged": self.privileged}

    def step(self, action: np.ndarray) -> tuple[np.ndarray, float, bool, bool, dict[str, Any]]:
        """Apply one policy step (1/control_hz seconds of physics)."""
        assert self.sim is not None
        a = np.clip(np.asarray(action, float), -1.0, 1.0)
        c = self.cfg
        raw = c.action_scale_rad * a
        smoothed = c.action_ema * self._tgt + (1 - c.action_ema) * raw
        self._tgt = self._tgt + np.clip(smoothed - self._tgt, -c.max_target_step_rad, c.max_target_step_rad)
        self.sim.step(self._tgt, self.dt)
        if not np.all(np.isfinite(self.sim.data.qpos)):
            raise FloatingPointError("simulation diverged (NaN)")
        self.obs_builder.push_action(a)
        reward, terms = self.task.reward(self.sim, a, self._prev_action)
        self._prev_action = a
        self._t += 1
        terminated = self._up_dot() < c.flip_up_dot or self.task.terminated(self.sim)
        truncated = self._t >= self.max_steps
        info = {"terms": terms, "distance": getattr(self.task, "distance", lambda s: 0.0)(self.sim)}
        if hasattr(self.task, "metrics"):
            info["metrics"] = self.task.metrics(self.sim)
        return self.obs_builder.build(self._snapshot()), reward, terminated, truncated, info
