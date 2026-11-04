"""Walk-straight task: efficient translation of the whole body along one fixed direction (the heading at reset)."""

from __future__ import annotations

from collections import deque

import numpy as np

from petctl.rl.tasks.locomote import Locomote
from petctl.sim.model_builder import PetSim


class WalkStraight(Locomote):
    """Locomote reward measured at the body centre of mass, plus a cross-track drift penalty and path/energy bookkeeping.

    ``metrics`` reports net distance, path length, straightness (net/path), cross-track offset and energy per metre so
    efficiency and directness can be evaluated, not just distance.
    """

    name = "walk_straight"

    @staticmethod
    def _com(sim: PetSim) -> np.ndarray:
        return sim.data.subtree_com[1].copy()

    def tracked_pos(self, sim: PetSim) -> np.ndarray:
        """Reward the centre of mass, not the head."""
        return self._com(sim)

    def reset(self, sim: PetSim, rng: np.random.Generator) -> None:
        """As Locomote, but anchored on the centre of mass."""
        super().reset(sim, rng)
        self.p0 = self._com(sim)
        n = max(1, int(round(self.cfg.progress_window_s * self.cfg.control_hz)))
        self._hist = deque([self.p0.copy()] * (n + 1), maxlen=n + 1)
        self._last = self.p0.copy()
        self.path_m = 0.0
        self.energy = 0.0                                   # sum of tau^2 * dt over the episode, (N.m)^2 s

    def distance(self, sim: PetSim) -> float:
        """Net displacement of the centre of mass along the reset heading (m)."""
        return float((self._com(sim) - self.p0) @ self.heading)

    def reward(self, sim: PetSim, action: np.ndarray, prev_action: np.ndarray) -> tuple[float, dict[str, float]]:
        """Locomote terms evaluated on the CoM plus ``drift``; also accumulates path length and energy."""
        com = self._com(sim)
        w = self.cfg.reward
        dt = 1.0 / self.cfg.control_hz
        self.path_m += float(np.linalg.norm((com - self._last)[:2]))
        self._last = com
        self.energy += float(np.sum(sim.joint_torque() ** 2)) * dt
        total, terms = super().reward(sim, action, prev_action)
        cross = float((com - self.p0) @ self.side)
        terms["drift"] = -w.drift * min(abs(cross), 1.0)
        return total + terms["drift"], terms

    def metrics(self, sim: PetSim) -> dict[str, float]:
        """Episode-level efficiency and directness numbers."""
        d = self.distance(sim)
        cross = float((self._com(sim) - self.p0) @ self.side)
        return {"distance": d, "path": self.path_m, "cross_track": cross,
                "straightness": d / self.path_m if self.path_m > 1e-6 else 0.0,
                "energy_per_m": self.energy / d if d > 1e-3 else float("inf")}
