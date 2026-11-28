"""Clamp of the PPO policy ``log_std`` to ``LOG_STD_RANGE`` (RL restart, owner-accepted 2026-11-28): the lower bound -3 lets exploration noise shrink; the upper bound is the init value -1, so std can never grow
(ent_coef stays 0). The clamp runs when each rollout starts, after every PPO update and at the end of training."""

from __future__ import annotations

from typing import Any

import torch
from stable_baselines3.common.callbacks import BaseCallback

from petctl.rl.config import LOG_STD_RANGE


def clamp_log_std(policy: Any, bounds: tuple[float, float] = LOG_STD_RANGE) -> None:
    """In-place clamp of ``policy.log_std`` (no-op for policies without one)."""
    ls = getattr(policy, "log_std", None)
    if ls is not None:
        with torch.no_grad():
            ls.clamp_(bounds[0], bounds[1])


class LogStdClamp(BaseCallback):
    """SB3 callback applying ``clamp_log_std`` at rollout start and training end."""

    def __init__(self, bounds: tuple[float, float] = LOG_STD_RANGE) -> None:
        """``bounds`` = (lower, upper) log-std."""
        super().__init__()
        self.bounds = bounds

    def _on_rollout_start(self) -> None:
        clamp_log_std(self.model.policy, self.bounds)

    def _on_step(self) -> bool:
        return True

    def _on_training_end(self) -> None:
        clamp_log_std(self.model.policy, self.bounds)
