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

    def __init__(self, cfg: EnvConfig) -> None:
        """Store the env config (reward weights, control rate)."""
        self.cfg = cfg

    @abstractmethod
    def reset(self, sim: PetSim, rng: np.random.Generator) -> None:
        """Called after the sim is reset; cache the initial heading/position."""

    @abstractmethod
    def reward(self, sim: PetSim, action: np.ndarray, prev_action: np.ndarray) -> tuple[float, dict[str, float]]:
        """Reward for the step just simulated."""

    def terminated(self, sim: PetSim) -> bool:
        """Task-specific termination (the env already handles flips)."""
        return False
