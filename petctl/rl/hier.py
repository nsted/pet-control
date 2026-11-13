"""Hierarchical navigation: a slow selector steers a frozen goal-conditioned low-level policy by feeding it a rotated goal bearing.

The low-level policy (e.g. S8) sees the goal direction in the IMU frame; the selector, every ``hold`` control ticks (1 s at 20 Hz),
chooses a bearing offset and the wrapper rotates the goal direction about world-z before the low-level policy sees it. The selector
sees the true goal, gravity, the touch channels (incl. 10 s / 30 s averages) and its own recent choices - never the obstacles.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import gymnasium as gym
import numpy as np
from gymnasium import spaces

from petctl.rl.config import N_JOINTS, EnvConfig
from petctl.rl.env import PetEnv
from petctl.rl.policy import GaitPolicy

OFFSETS_DEG = (0.0, 45.0, -45.0, 90.0, -90.0, 135.0, -135.0, 180.0)
N_RECENT = 10


class SelectorEnv(gym.Env):
    """Gym env whose action is an index into ``OFFSETS_DEG``; one step = ``hold`` control ticks of the frozen low-level policy."""

    metadata = {"render_modes": []}

    def __init__(self, low_run: str | Path, task_params: dict[str, float] | None = None, dr_scale: float | None = None, hold: int = 20,
                 force_offset: int | None = None, env_run: str | Path | None = None) -> None:
        """``low_run``: run dir of the frozen low-level policy (its config gives the env); ``force_offset``: always pick that offset (baseline)."""
        low_run = Path(low_run)
        self.low = GaitPolicy(low_run)
        cfg = EnvConfig.from_dict(json.loads((Path(env_run or low_run) / "config.json").read_text()))        # env_run: take the task/config from another run (e.g. S8) when the low-level policy is from a different stage
        tp = dict(task_params or {})
        base = {} if tp.pop("_replace", None) else cfg.task_params            # _replace: use exactly these task params (e.g. a barrier layout without the low run's field_radius)
        cfg.task_params = {**base, **tp, "touch_long": 1.0}
        if dr_scale is not None:
            cfg.dr.scale = dr_scale
        self.env = PetEnv(cfg)
        self.hold, self.force = hold, force_offset
        self.n_low = self.low._mean.shape[0]
        self.e0 = 3 + 3 + 3 * N_JOINTS + cfg.history * N_JOINTS            # start of the task-specific (goal) entries
        self.action_space = spaces.Discrete(len(OFFSETS_DEG))
        n = 4 + 3 + (self.env.obs_builder.size - self.e0 - 4) + N_RECENT + 1
        self.observation_space = spaces.Box(-np.inf, np.inf, (n,), np.float32)
        self._recent = np.zeros(N_RECENT)
        self._obs = np.zeros(self.env.obs_builder.size, np.float32)
        self._t = 0

    def _low_obs(self, theta: float) -> np.ndarray:
        o = self._obs[:self.n_low].copy()
        r = self.env.sim.data.site_xmat[self.env.sim.model.site("imu").id].reshape(3, 3)
        u = r @ o[self.e0:self.e0 + 3]
        c, s = np.cos(theta), np.sin(theta)
        u = np.array([c * u[0] - s * u[1], s * u[0] + c * u[1], u[2]])
        o[self.e0:self.e0 + 3] = r.T @ u
        return o

    def _sel_obs(self) -> np.ndarray:
        o = self._obs
        return np.concatenate([o[self.e0:self.e0 + 4], o[:3], o[self.e0 + 4:], self._recent, [self._t * self.hold / self.env.max_steps]]).astype(np.float32)

    def reset(self, *, seed: int | None = None, options: dict[str, Any] | None = None) -> tuple[np.ndarray, dict[str, Any]]:
        """Reset the underlying env and the selector history."""
        self._obs, info = self.env.reset(seed=seed, options=options)
        self._recent = np.zeros(N_RECENT)
        self._t = 0
        return self._sel_obs(), info

    def step(self, action: int) -> tuple[np.ndarray, float, bool, bool, dict[str, Any]]:
        """Hold the chosen bearing offset for ``hold`` ticks; returns the summed reward."""
        a = self.force if self.force is not None else int(action)
        theta = np.deg2rad(OFFSETS_DEG[a])
        self._recent = np.roll(self._recent, 1)
        self._recent[0] = OFFSETS_DEG[a] / 180.0
        total, term, trunc, info = 0.0, False, False, {}
        for _ in range(self.hold):
            self._obs, r, term, trunc, info = self.env.step(self.low.action(self._low_obs(theta)))
            total += r
            if term or trunc:
                break
        self._t += 1
        info["offset_idx"] = a
        return self._sel_obs(), total, term, trunc, info
