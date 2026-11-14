"""Goal-hug task (Regimen v2): cross the obstacle field to the post, then coil round it and hug it (plan sec 5.3).

``GoalObstacles`` with the target post always on (``post`` defaults to 1) plus the ground-truth hug metric and reward of ``petctl.rl.hug``. The navigation
progress reward keeps paying until the first post contact, after which the hug terms take over; the episode ends successfully when the hug has been held
(``hug_hold_s``, default 3 s). ``success`` in the metrics is the hug success. Knobs (besides those of GoalObstacles/GoalReach): ``hug_hold_s``, ``hug_h``
(contiguous-module threshold; default max(5, ceil(0.4 n))), ``approach_weight``, ``coverage_weight``, ``squeeze_weight``, ``hold_weight``, ``hug_bonus``,
``speed_gain``. Observation knobs ``capacitive_obs`` / ``capacitive_off`` apply as for any task (see obs.py).
"""

from __future__ import annotations

import numpy as np

from petctl.rl.hug import HugMixin
from petctl.rl.tasks.goal_obstacles import GoalObstacles
from petctl.rl.tasks.goal_reach import GLITCH_PENALTY, GLITCH_PENETRATION_M
from petctl.sim.model_builder import PetSim


class GoalHug(HugMixin, GoalObstacles):
    """GoalObstacles + post hug reward; terminates on a held hug."""

    name = "goal_hug"
    score_label = "m towards goal"
    DEFAULTS = {"post": 1.0}

    def param(self, key: str, default: float) -> float:
        """Task knob with this task's own defaults (``post`` is on)."""
        return float(self.cfg.task_params.get(key, self.DEFAULTS.get(key, default)))

    def reset(self, sim: PetSim, rng: np.random.Generator) -> None:
        """As GoalObstacles; starts the hug tracker."""
        super().reset(sim, rng)
        self.hug_reset(sim)

    def reward(self, sim: PetSim, action: np.ndarray, prev_action: np.ndarray) -> tuple[float, dict[str, float]]:
        """GoalObstacles reward (progress stops paying at first post contact) plus the hug terms and the post-penetration glitch guard."""
        total, terms = super().reward(sim, action, prev_action)
        hug, hterms = self.hug_step(sim)
        if not np.isnan(self._hug.t_contact) and terms.get("progress", 0.0) != 0.0:
            total -= terms["progress"]
            terms["progress"] = 0.0
        terms.update(hterms)
        probe = self._hug_probe
        if probe is not None and probe.read().penetration > GLITCH_PENETRATION_M and not self.glitch:
            self.glitch = True
            terms["glitch"] = -GLITCH_PENALTY
            total += terms["glitch"]
        return total + hug, terms

    def terminated(self, sim: PetSim) -> bool:
        """End on a held hug, or as GoalObstacles."""
        return self.hug_success or super().terminated(sim)

    def metrics(self, sim: PetSim) -> dict[str, float]:
        """GoalObstacles metrics with ``success`` = hug held, plus the hug diagnostics."""
        m = super().metrics(sim)
        m["goal_reached"] = m["success"]
        h = self.hug_metrics()
        return {**m, **h, "success": h["hug_success"]}
