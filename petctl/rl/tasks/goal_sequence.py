"""Goal-sequence task: visit several targets in order, t1 -> t2 -> ... -> tN, with a fresh random layout every pass (curriculum stage S4+).

The whole layout is placed at reset (not when each target is reached): t1 is sampled from the head at a random bearing about the initial heading
(``bearing_deg``) and distance (``goal_dist_min``/``goal_dist_max``); every later target is placed relative to the previous target at a random
turn (``seq_bearing_deg`` about the previous leg's direction) and distance in the same range. The policy observes the current target and the next
one (direction in the IMU frame + distance each; zeros after the last), so it can set up each approach for the leg that follows. Success = the head
reaches every target in order. ``n_goals`` is the number of targets (default 4). All other knobs are those of ``GoalReach``.
"""

from __future__ import annotations

import numpy as np

from petctl.rl.tasks.goal_reach import DIST_OBS_SCALE_M, GoalReach
from petctl.sim.model_builder import PetSim

RGBA_CURRENT = (0.95, 0.1, 0.1, 0.9)
RGBA_NEXT = (1.0, 0.6, 0.1, 0.8)
RGBA_DONE = (0.1, 0.8, 0.2, 0.6)


class GoalSequence(GoalReach):
    """GoalReach with a pre-placed ordered list of targets and a next-target lookahead in the observation."""

    name = "goal_sequence"
    score_label = "m towards targets"
    extra_obs_dim = 8            # current target (dir 3 + dist) and next target (dir 3 + dist, zeros when there is none)

    def param(self, key: str, default: float) -> float:
        """Task knob with this task's own defaults (4 targets)."""
        return super().param(key, 4.0 if key == "n_goals" else default)

    def reset(self, sim: PetSim, rng: np.random.Generator) -> None:
        """Place the whole layout, then start on the first target."""
        self.targets: list[np.ndarray] = []
        super().reset(sim, rng)

    def _spawn_goal(self, origin: np.ndarray, first: bool = False, sim: PetSim | None = None) -> None:
        """First call: sample all targets. Every call: make target number ``goals_done`` the current goal."""
        if first:
            lo, hi = self.param("goal_dist_min", 0.4), self.param("goal_dist_max", 0.7)
            turn = np.deg2rad(self.param("seq_bearing_deg", 180.0))
            self.bearing0 = float(np.deg2rad(self._rng.uniform(-1.0, 1.0) * self.param("bearing_deg", 180.0)))
            yaw = float(np.arctan2(self.heading[1], self.heading[0])) + self.bearing0
            prev = origin.copy()
            self.targets = []
            for k in range(max(1, self.n_goals)):
                if k > 0:
                    yaw += float(self._rng.uniform(-1.0, 1.0) * turn)
                prev = prev + float(self._rng.uniform(lo, hi)) * np.array([np.cos(yaw), np.sin(yaw), 0.0])
                prev[2] = self._z
                self.targets.append(prev.copy())
            self.dist0 = float(np.linalg.norm((self.targets[0] - origin)[:2]))
        self.goal = self.targets[min(self.goals_done, len(self.targets) - 1)].copy()
        self._dist_start = float(np.linalg.norm((self.goal - origin)[:2]))

    def _relocate_goal(self, origin: np.ndarray, sim: PetSim | None) -> None:
        """An automatic target move replaces the CURRENT target only; later targets stay where they were placed."""
        super()._relocate_goal(origin, sim)
        self.targets[min(self.goals_done, len(self.targets) - 1)] = self.goal.copy()

    def _obs_for(self, sim: PetSim, target: np.ndarray) -> np.ndarray:
        v = target - self._head(sim)
        v[2] = 0.0
        d = float(np.linalg.norm(v))
        r = sim.data.site_xmat[sim.model.site("imu").id].reshape(3, 3)
        return np.concatenate([r.T @ (v / max(d, 1e-9)), [min(d / DIST_OBS_SCALE_M, 2.0)]])

    def extra_obs(self, sim: PetSim) -> np.ndarray:
        """Current target and the one after it."""
        cur = min(self.goals_done, len(self.targets) - 1)
        nxt = self._obs_for(sim, self.targets[cur + 1]) if cur + 1 < len(self.targets) else np.zeros(4)
        return np.concatenate([self._obs_for(sim, self.targets[cur]), nxt]).astype(np.float32)

    def markers(self, sim: PetSim) -> list[tuple[np.ndarray, float, tuple[float, float, float, float]]]:
        """Every target: reached ones green, the current one red, upcoming ones orange and a little smaller."""
        out = []
        for k, t in enumerate(self.targets):
            r = self.radius if k <= self.goals_done else 0.8 * self.radius
            colour = RGBA_DONE if k < self.goals_done or self.reached else RGBA_CURRENT if k == self.goals_done else RGBA_NEXT
            out.append((np.array([t[0], t[1], r]), r, colour))
        return out

    def metrics(self, sim: PetSim) -> dict[str, float]:
        """GoalReach metrics plus the number of targets."""
        return {**super().metrics(sim), "n_targets": float(len(self.targets))}
