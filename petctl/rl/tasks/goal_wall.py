"""Goal behind a wall: the head must reach a target on the far side of a static wall that the policy is NOT told about (curriculum stage S6).

Each pass: the target is placed ``goal_dist_min``-``goal_dist_max`` m from the head at a random bearing (+-``bearing_deg`` about the heading); a wall is placed
across the straight line to it, ``wall_frac_min``-``wall_frac_max`` of the way along, perpendicular to the line, with half-length
``wall_len_min``-``wall_len_max`` m and a random sideways offset (+-``wall_offset``) so the way round is shorter on one side. The observation is
the goal direction/distance as before (the target is visible through the wall) plus the motor-reply contact channels (``touch_obs``: measured
torque and commanded-minus-measured position, this tick and the last). Nothing else reveals the wall, so the policy has to find the way round by
how the body feels the contact. ``contact_penalty`` (per tick touching the wall) and ``flip_penalty`` (once, when the body flips) shape the reward. ``touch_off=1`` zeroes the contact channels (ablation). All other knobs are those of ``GoalReach``.
"""

from __future__ import annotations

import numpy as np

from petctl.rl.tasks.goal_reach import GoalReach
from petctl.sim.model_builder import PetSim


class GoalWall(GoalReach):
    """GoalReach with a wall between the robot and the target and contact-channel observations."""

    name = "goal_wall"
    score_label = "m towards goal"
    touch_obs = True

    preset_goal: np.ndarray | None = None

    def layout(self, head: np.ndarray, heading: np.ndarray, rng: np.random.Generator) -> tuple[tuple[tuple[float, float, float, float], ...], np.ndarray | None]:
        """Sample the target and the wall across the line to it (world frame, from the nominal head position and heading)."""
        d = float(rng.uniform(self.param("goal_dist_min", 1.2), self.param("goal_dist_max", 1.8)))
        b = float(np.deg2rad(rng.uniform(-1.0, 1.0) * self.param("bearing_deg", 0.0)))
        yaw = float(np.arctan2(heading[1], heading[0])) + b
        u = np.array([np.cos(yaw), np.sin(yaw)])
        perp = np.array([-u[1], u[0]])
        frac = float(rng.uniform(self.param("wall_frac_min", 0.4), self.param("wall_frac_max", 0.55)))
        off = float(rng.uniform(-1.0, 1.0) * self.param("wall_offset", 0.2))
        half = float(rng.uniform(self.param("wall_len_min", 0.3), self.param("wall_len_max", 0.5)))
        centre = head[:2] + frac * d * u + off * perp
        goal = np.array([*(head[:2] + d * u), head[2]])
        return ((float(centre[0]), float(centre[1]), yaw + np.pi / 2, half),), goal

    def reset(self, sim: PetSim, rng: np.random.Generator) -> None:
        """As GoalReach; also counts wall-contact ticks."""
        super().reset(sim, rng)
        self.contact_ticks = 0
        self._wall_ids = {sim.model.geom(f"wall{i}").id for i in range(len(sim.p.walls))}

    def _spawn_goal(self, origin: np.ndarray, first: bool = False, sim: PetSim | None = None) -> None:
        """The first goal is the one laid out with the wall; later goals (n_goals > 1) are sampled as in GoalReach."""
        if first and self.preset_goal is not None:
            self.goal = self.preset_goal.copy()
            self.goal[2] = self._z
            self.dist0 = self._dist_start = float(np.linalg.norm((self.goal - origin)[:2]))
            self.bearing0 = 0.0
            return
        super()._spawn_goal(origin, first, sim)

    def reward(self, sim: PetSim, action: np.ndarray, prev_action: np.ndarray) -> tuple[float, dict[str, float]]:
        """GoalReach reward; counts ticks in contact with the wall."""
        total, terms = super().reward(sim, action, prev_action)
        touching = any(c.geom1 in self._wall_ids or c.geom2 in self._wall_ids for c in sim.data.contact[:sim.data.ncon])
        self.contact_ticks += int(touching)
        terms["wall_contact"] = -self.param("contact_penalty", 0.0) if touching else 0.0       # discourages pushing into the wall
        up = float(sim.data.site_xmat[sim.model.site("imu").id].reshape(3, 3)[2, 2])
        terms["flip"] = -self.param("flip_penalty", 0.0) if up < self.cfg.flip_up_dot else 0.0  # the env ends the episode on this tick; make it cost something
        return total + terms["wall_contact"] + terms["flip"], terms

    def metrics(self, sim: PetSim) -> dict[str, float]:
        """GoalReach metrics plus the fraction of the episode spent touching the wall."""
        return {**super().metrics(sim), "contact_frac": self.contact_ticks / max(self._t, 1)}
