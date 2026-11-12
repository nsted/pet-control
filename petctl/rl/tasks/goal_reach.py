"""Goal-reach task: bring the robot's HEAD to a goal point on the floor (curriculum stages S1-S4).

Success, the goal direction/distance the policy observes, and goal sampling all use the head position; progress shaping uses the centre of mass
(a windowed head displacement can be gamed by swinging the head). The goal is sampled at reset relative to the initial heading and head position. The policy is told the goal direction (unit vector in the
tail-IMU frame, consistent with the gravity entry) and the distance. On the real robot this requires an external localiser (the IMU yaw drifts),
so deployment will need a goal-bearing source; the sim supplies ground truth.

Knobs (``EnvConfig.task_params``): ``goal_dist_min``/``goal_dist_max`` (m), ``bearing_deg`` (goal bearing drawn uniformly from +-this about the
current head heading), ``goal_radius`` (m), ``success_bonus``, and for course changes ``n_goals`` (goals reached in sequence, each new goal sampled
from where the robot is when the previous is reached; 0 = unlimited, the episode never ends on arrival, used for interactive click-to-navigate),
``redirect_every_s`` (automatic target moves: while a goal is still unreached it jumps to a new random position, with random timing, on average every
this many seconds and never closer than ``redirect_min_gap_s`` (default 5 s) apart; 0 = off) or the older single-shot ``redirect_prob`` (chance an episode
moves the current goal once, at a random time in [``redirect_min_s``, ``redirect_max_s``]). ``start_pose_rad`` (random initial joint angles) is applied by the env.
"""

from __future__ import annotations

from collections import deque

import numpy as np

from petctl.rl.tasks.locomote import Locomote
from petctl.sim.model_builder import PetSim

GLITCH_COM_SPEED = 3.0       # m/s; no real PET motion is anywhere near this, so it is a physics blow-up
GLITCH_PENALTY = 300.0
DIST_OBS_SCALE_M = 1.0


class GoalReach(Locomote):
    """Reward = progress towards the goal (windowed, as Locomote), a success bonus that shrinks with elapsed time, minus the Locomote costs.

    ``metrics()["distance"]`` is the reduction in distance-to-goal (m); ``success`` is 1.0 once the head is within ``goal_radius``.
    """

    name = "goal_reach"
    score_label = "m towards goal"
    extra_obs_dim = 4            # unit goal direction in the IMU frame (3) + distance / 1 m clipped to [0, 2]

    @staticmethod
    def _com(sim: PetSim) -> np.ndarray:
        return sim.data.subtree_com[1].copy()

    @staticmethod
    def _head(sim: PetSim) -> np.ndarray:
        return sim.head_pos().copy()

    def tracked_pos(self, sim: PetSim) -> np.ndarray:
        """Progress shaping follows the centre of mass (robust to head swinging); success is judged at the head."""
        return self._com(sim)

    def reset(self, sim: PetSim, rng: np.random.Generator) -> None:
        """Sample the goal relative to the initial heading and head position."""
        super().reset(sim, rng)
        self.p0 = self._com(sim)
        n = max(1, int(round(self.cfg.progress_window_s * self.cfg.control_hz)))
        self._hist = deque([self.p0.copy()] * (n + 1), maxlen=n + 1)      # Locomote anchored the window on the head; this task tracks the CoM
        lo, hi = self.param("goal_dist_min", 0.25), self.param("goal_dist_max", 0.35)
        self.radius = self.param("goal_radius", 0.05)
        self.bonus = self.param("success_bonus", 10.0)
        self._rng = rng
        self.n_goals = int(self.param("n_goals", 1))
        self.goals_done = 0
        self._banked = 0.0                                  # distance already covered towards goals that were reached or replaced
        self.redirects = 0
        self._schedule_redirect(0.0, first=True)
        self._z = float(self.p0[2])
        self._spawn_goal(self._head(sim), first=True)
        self._last = self.p0.copy()
        self._t = 0
        self.path_m = 0.0
        self.energy = 0.0
        self.reached = False
        self.glitch = False
        self.t_goal = float("nan")
        self._point_at_goal(self._head(sim))

    def _spawn_goal(self, origin: np.ndarray, first: bool = False, sim: PetSim | None = None) -> None:
        """Sample a goal from ``origin`` (the head position) at a random distance and bearing about the current head heading (initial heading for the first)."""
        lo, hi = self.param("goal_dist_min", 0.25), self.param("goal_dist_max", 0.35)
        bearing = float(np.deg2rad(self._rng.uniform(-1.0, 1.0) * self.param("bearing_deg", 0.0)))
        if first:
            self.bearing0, ref = bearing, self.heading
            self.dist0 = float(self._rng.uniform(lo, hi))
            dist = self.dist0
        else:
            self._relocate_goal(origin, sim)
            return
        yaw = float(np.arctan2(ref[1], ref[0])) + bearing
        self.goal = origin + dist * np.array([np.cos(yaw), np.sin(yaw), 0.0])
        self.goal[2] = self._z
        self._dist_start = dist

    def _relocate_goal(self, origin: np.ndarray, sim: PetSim | None) -> None:
        """Put the current goal at a new random position relative to ``origin`` (the head): random distance, bearing about the current head heading."""
        assert sim is not None
        lo, hi = self.param("goal_dist_min", 0.25), self.param("goal_dist_max", 0.35)
        bearing = float(np.deg2rad(self._rng.uniform(-1.0, 1.0) * self.param("bearing_deg", 0.0)))
        fwd = sim.data.site_xmat[sim.model.site("head").id].reshape(3, 3)[:, 0].copy()
        yaw = float(np.arctan2(fwd[1], fwd[0])) + bearing
        dist = float(self._rng.uniform(lo, hi))
        self.goal = origin + dist * np.array([np.cos(yaw), np.sin(yaw), 0.0])
        self.goal[2] = self._z
        self._dist_start = dist

    def _schedule_redirect(self, now_s: float, first: bool = False) -> None:
        """Draw the time of the next automatic goal move (``inf`` = none)."""
        every = self.param("redirect_every_s", 0.0)
        if every > 0.0:
            gap = self.param("redirect_min_gap_s", 5.0)
            self._next_redirect_s = now_s + gap + float(self._rng.exponential(max(every - gap, 0.5)))
        elif first and self._rng.random() < self.param("redirect_prob", 0.0):
            self._next_redirect_s = float(self._rng.uniform(self.param("redirect_min_s", 3.0), self.param("redirect_max_s", 12.0)))
        else:
            self._next_redirect_s = float("inf")

    def set_goal(self, sim: PetSim, xy: np.ndarray) -> None:
        """Move the goal to world position ``xy`` (m) now, e.g. from a user click; progress bookkeeping restarts from the current position."""
        head = self._head(sim)
        self._banked += self._dist_start - self._to_goal(head)[1]
        self.goal = np.array([float(xy[0]), float(xy[1]), self._z])
        self._dist_start = self._to_goal(head)[1]
        self._restart_window(self._com(sim))
        self.reached = False

    def _restart_window(self, com: np.ndarray) -> None:
        n = max(1, int(round(self.cfg.progress_window_s * self.cfg.control_hz)))
        self._hist = deque([com.copy()] * (n + 1), maxlen=n + 1)      # no progress spike from the goal jump

    def _to_goal(self, com: np.ndarray) -> tuple[np.ndarray, float]:
        v = self.goal - com
        v[2] = 0.0
        d = float(np.linalg.norm(v))
        return v / max(d, 1e-9), d

    def _point_at_goal(self, com: np.ndarray) -> float:
        """Make the Locomote progress/lateral terms measure motion towards (and across) the current goal direction."""
        u, d = self._to_goal(com)
        self.heading = u
        self.side = np.array([-u[1], u[0], 0.0])
        return d

    def extra_obs(self, sim: PetSim) -> np.ndarray:
        """Goal direction in the IMU frame and the (clipped) distance."""
        u, d = self._to_goal(self._head(sim))
        r = sim.data.site_xmat[sim.model.site("imu").id].reshape(3, 3)
        return np.concatenate([r.T @ u, [min(d / DIST_OBS_SCALE_M, 2.0)]]).astype(np.float32)

    def markers(self, sim: PetSim) -> list[tuple[np.ndarray, float, tuple[float, float, float, float]]]:
        """A red ball of the success radius on the floor at the goal (turns green once reached)."""
        colour = (0.1, 0.8, 0.2, 0.9) if self.reached else (0.95, 0.1, 0.1, 0.9)
        return [(np.array([self.goal[0], self.goal[1], self.radius]), self.radius, colour)]

    def distance(self, sim: PetSim) -> float:
        """Distance covered towards goals so far (m): completed/replaced legs plus the reduction on the current goal."""
        return self._banked + self._dist_start - self._to_goal(self._head(sim))[1]

    def reward(self, sim: PetSim, action: np.ndarray, prev_action: np.ndarray) -> tuple[float, dict[str, float]]:
        """Locomote terms towards the goal plus ``goal`` bonus on arrival and ``glitch`` penalty on a physics blow-up."""
        dt = 1.0 / self.cfg.control_hz
        com = self._com(sim)
        self._t += 1
        step = float(np.linalg.norm((com - self._last)[:2]))
        self.path_m += step
        self.energy += float(np.sum(sim.joint_torque() ** 2)) * dt
        self._last = com
        head = self._head(sim)
        d = self._point_at_goal(head)
        total, terms = super().reward(sim, action, prev_action)
        terms["goal"] = terms["glitch"] = 0.0
        if step / dt > GLITCH_COM_SPEED:
            self.glitch = True
            terms["glitch"] = -GLITCH_PENALTY
        else:
            if self._t * dt >= self._next_redirect_s and d > self.radius:                  # automatic target move
                self._banked += self._dist_start - d
                self._relocate_goal(head, sim)
                self.redirects += 1
                self._restart_window(com)
                d = self._point_at_goal(head)
                self._schedule_redirect(self._t * dt)
            if d <= self.radius:
                self.goals_done += 1
                terms["goal"] = self.bonus * (1.0 + (1.0 - self._t / max(1, int(round(self.cfg.episode_s * self.cfg.control_hz)))))
                self._banked += self._dist_start - d
                self._dist_start = d                                                    # the leg's progress is banked; do not count it twice
                if self.n_goals == 0 or self.goals_done < self.n_goals:
                    self._spawn_goal(head, sim=sim)                                      # next goal from where the head is
                    self._restart_window(com)
                else:
                    self.reached = True
                    self.t_goal = self._t * dt
        return total + terms["goal"] + terms["glitch"], terms

    def terminated(self, sim: PetSim) -> bool:
        """End on arrival or on a physics blow-up."""
        return self.reached or self.glitch

    def metrics(self, sim: PetSim) -> dict[str, float]:
        """Episode-level numbers used by the curriculum watcher and gate."""
        d = self.distance(sim)
        return {"distance": d, "success": float(self.reached), "final_dist": self._to_goal(self._head(sim))[1], "dist0": self.dist0, "goals_reached": float(self.goals_done), "redirects": float(self.redirects),
                "bearing0_deg": float(np.rad2deg(self.bearing0)), "time_to_goal_s": self.t_goal, "path": self.path_m,
                "energy_per_m": self.energy / d if d > 1e-3 else float("inf"), "glitch": float(self.glitch)}
