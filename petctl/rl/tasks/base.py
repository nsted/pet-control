"""Task interface: a task owns the reward and any task-specific termination; the env owns physics, observations and actions."""

from __future__ import annotations

from abc import ABC, abstractmethod

import numpy as np

from petctl.rl.config import EnvConfig
from petctl.sim.model_builder import PetSim


class Task(ABC):
    """One experiment. ``reward`` returns the total and a dict of separately logged terms."""

    name = "task"
    score_label = "m"             # unit/meaning of metrics()["distance"], shown in the skill library
    terminate_on_flip = True      # end the episode when the tail IMU is tipped past EnvConfig.flip_up_dot (off for reaching poses)
    touch_obs = False             # append motor-reply contact signals (measured torque, commanded-minus-measured position; this tick and the last) to the observation
    extra_obs_dim = 0             # task-specific observation entries appended after the proprioception (e.g. goal direction)

    def __init__(self, cfg: EnvConfig) -> None:
        """Store the env config (reward weights, control rate)."""
        self.cfg = cfg

    def param(self, key: str, default: float) -> float:
        """A task knob from ``EnvConfig.task_params`` (falls back to ``default``)."""
        return float(self.cfg.task_params.get(key, default))

    def layout(self, head: np.ndarray, heading: np.ndarray, rng: np.random.Generator) -> tuple[tuple[tuple[float, float, float, float], ...], np.ndarray | None]:
        """Static world geometry placed before the sim is built, given the nominal head position and heading: ``(walls, goal_xy or None)``."""
        return (), None

    def markers(self, sim: PetSim) -> list[tuple[np.ndarray, float, tuple[float, float, float, float]]]:
        """Visual-only markers (world position, radius, rgba) drawn in videos/viewer, e.g. the goal. Never seen by the policy."""
        return []

    def extra_obs(self, sim: PetSim) -> np.ndarray:
        """Task-specific observation entries (length ``extra_obs_dim``); must only use what the real robot could be told."""
        return np.zeros(self.extra_obs_dim, dtype=np.float32)

    @abstractmethod
    def reset(self, sim: PetSim, rng: np.random.Generator) -> None:
        """Called after the sim is reset; cache the initial heading/position."""

    @abstractmethod
    def reward(self, sim: PetSim, action: np.ndarray, prev_action: np.ndarray) -> tuple[float, dict[str, float]]:
        """Reward for the step just simulated."""

    def terminated(self, sim: PetSim) -> bool:
        """Task-specific termination (the env already handles flips)."""
        return False
