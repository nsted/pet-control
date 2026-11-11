"""Walk-sideways task: translate the body as far as possible perpendicular to its length, to whichever side works best."""

from __future__ import annotations

import numpy as np

from petctl.rl.tasks.walk_straight import WalkStraight
from petctl.sim.model_builder import PetSim


class WalkSideways(WalkStraight):
    """Progress is measured along the body's lateral axis, in the direction the body is currently moving (either side).

    Reward uses the windowed speed along that axis (so the sign is free, but reversing direction costs the progress made).
    The inherited ``lateral`` and ``drift`` terms now penalise motion along the body's length, so the gait stays a pure
    side-step. ``distance`` is the net displacement |sideways| in metres and ``cross_track`` the offset along the body.
    """

    name = "walk_sideways"
    score_label = "m sideways"

    def reset(self, sim: PetSim, rng: np.random.Generator) -> None:
        """Take the lateral axis as the progress axis and the body's length as the off-axis direction."""
        super().reset(sim, rng)
        self.axis = self.side.copy()                        # unit vector across the body (floor plane), the body's left
        self.length_dir = self.heading.copy()               # along the body (towards the head)
        self.side = self.length_dir                         # off-axis for the inherited lateral/drift penalties
        self._d_prev = 0.0

    def distance(self, sim: PetSim) -> float:
        """Net sideways displacement |(com - start) . axis| (m)."""
        return abs(float((self._com(sim) - self.p0) @ self.axis))

    def reward(self, sim: PetSim, action: np.ndarray, prev_action: np.ndarray) -> tuple[float, dict[str, float]]:
        """walk_straight terms, but ``progress`` = growth of net |sideways| displacement from the start.

        The increments telescope to the final net distance, so rocking from side to side earns nothing (a windowed-speed
        reward was hacked that way: high reward, ~4 cm of net travel), while either direction is still rewarded.
        """
        total, terms = super().reward(sim, action, prev_action)
        w = self.cfg.reward
        d_now = self.distance(sim)
        gain = (d_now - self._d_prev) * self.cfg.control_hz                # m/s, same units as the walk_straight progress
        self._d_prev = d_now
        new = w.progress * float(np.clip(gain, -0.5, 0.5))
        terms["progress"], total = new, total - terms["progress"] + new if "progress" in terms else total
        return total, terms
