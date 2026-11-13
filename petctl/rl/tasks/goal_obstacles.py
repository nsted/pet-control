"""Goal beyond a random scatter of obstacles of different shapes that the policy is NOT told about (curriculum stage S8).

Each pass: the target is placed ``goal_dist_min``-``goal_dist_max`` m from the head (bearing +-``bearing_deg`` about the heading); ``n_obs_min``-``n_obs_max``
obstacles (boxes, cylinders and ellipsoids of random size and orientation, 12 cm tall) are scattered at random positions in the area between the start and
the target (``spread`` m either side of the line). The start area and the target are kept clear and every layout is checked to have a way through.

The policy only sees the straight-line goal direction/distance and the motor-reply contact channels (``touch_obs``, with memory), so it has to feel its way
round. The reward is shaped with the shortest path to the target round the obstacles (a grid distance field, training-only privileged information, like
``GoalWall``): progress is measured along the gradient of that field, and an episode ends when the best path distance has not improved by ``stall_gain_m``
for ``stall_s`` seconds. Other knobs: ``contact_penalty``, ``allow_flip``, ``touch_off`` and those of ``GoalReach``.
"""

from __future__ import annotations

import heapq

import numpy as np

from petctl.rl.tasks.goal_reach import GoalReach
from petctl.sim.model_builder import PetSim

GRID_RES_M = 0.05
PLAN_MARGIN_M = 0.13          # clearance kept round obstacles when planning (about half the body's swing width)
START_KEEP_OUT = (-0.2, 0.62)  # start keep-out circle: centre offset along the heading from the head (m), radius (m)
GOAL_KEEP_OUT_M = 0.28
LOOKAHEAD_M = 0.25
EXPLORE_CELL_M = 0.10


def inside(kind: str, x: np.ndarray, y: np.ndarray, ob: tuple, margin: float) -> np.ndarray:
    """Boolean mask: points (x, y) within ``margin`` of the obstacle ``ob`` = (kind, cx, cy, yaw, a, b)."""
    _, cx, cy, yaw, a, b = ob
    c, s = np.cos(yaw), np.sin(yaw)
    lx, ly = (x - cx) * c + (y - cy) * s, -(x - cx) * s + (y - cy) * c
    if kind == "cyl":
        return lx ** 2 + ly ** 2 <= (a + margin) ** 2
    if kind == "ell":
        return (lx / (a + margin)) ** 2 + (ly / (b + margin)) ** 2 <= 1.0
    return (np.abs(lx) <= a + margin) & (np.abs(ly) <= b + margin)


def bounding_radius(ob: tuple) -> float:
    """Radius of a circle that contains the obstacle."""
    kind, _, _, _, a, b = ob
    return a if kind == "cyl" else float(np.hypot(a, b)) if kind == "box" else max(a, b)


class DistanceField:
    """Shortest-path distance (m) to ``goal`` over a 2-D grid with the inflated obstacles blocked; blocked cells take the nearest free cell's value plus the gap."""

    def __init__(self, obstacles: tuple, goal: np.ndarray, lo: np.ndarray, hi: np.ndarray) -> None:
        """Build the field over the box ``lo``..``hi`` (world xy)."""
        self.lo = lo.copy()
        self.nx, self.ny = (np.ceil((hi - lo) / GRID_RES_M).astype(int) + 1)
        xs = lo[0] + GRID_RES_M * np.arange(self.nx)
        ys = lo[1] + GRID_RES_M * np.arange(self.ny)
        X, Y = np.meshgrid(xs, ys, indexing="ij")
        blocked = np.zeros((self.nx, self.ny), bool)
        for ob in obstacles:
            blocked |= inside(ob[0], X, Y, ob, PLAN_MARGIN_M)
        gi, gj = self._ij(goal)
        blocked[max(gi - 1, 0):gi + 2, max(gj - 1, 0):gj + 2] = False         # the target cell is always reachable
        dist = np.full((self.nx, self.ny), np.inf)
        dist[gi, gj] = 0.0
        heap = [(0.0, gi, gj)]
        steps = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0), (1, 1, 2 ** 0.5), (1, -1, 2 ** 0.5), (-1, 1, 2 ** 0.5), (-1, -1, 2 ** 0.5)]
        while heap:
            d, i, j = heapq.heappop(heap)
            if d > dist[i, j]:
                continue
            for di, dj, w in steps:
                a, b = i + di, j + dj
                if 0 <= a < self.nx and 0 <= b < self.ny and not blocked[a, b]:
                    nd = d + w * GRID_RES_M
                    if nd < dist[a, b]:
                        dist[a, b] = nd
                        heapq.heappush(heap, (nd, a, b))
        from scipy import ndimage
        free = np.isfinite(dist)
        gap, idx = ndimage.distance_transform_edt(~free, return_indices=True)
        self.field = dist[idx[0], idx[1]] + gap * GRID_RES_M                     # fill blocked / unreachable cells
        self.reachable = free
        self.gx, self.gy = np.gradient(self.field, GRID_RES_M)

    def _ij(self, p: np.ndarray) -> tuple[int, int]:
        return (int(np.clip(round((p[0] - self.lo[0]) / GRID_RES_M), 0, self.nx - 1)), int(np.clip(round((p[1] - self.lo[1]) / GRID_RES_M), 0, self.ny - 1)))

    def value(self, p: np.ndarray) -> float:
        """Path distance (m) at world point ``p``."""
        return float(self.field[self._ij(p)])

    def is_free(self, p: np.ndarray) -> bool:
        """True if ``p`` is on a reachable free cell."""
        return bool(self.reachable[self._ij(p)])

    def direction(self, p: np.ndarray) -> np.ndarray:
        """Unit vector of steepest descent (towards the goal along the shortest path) at ``p``, or zeros on a plateau."""
        i, j = self._ij(p)
        g = np.array([self.gx[i, j], self.gy[i, j]])
        n = float(np.linalg.norm(g))
        return -g / n if n > 1e-9 else np.zeros(2)


class GoalObstacles(GoalReach):
    """GoalReach with a random obstacle scatter between the robot and the target, contact-channel observations, path-shaped progress and a stall stop."""

    name = "goal_obstacles"
    score_label = "m towards goal"
    touch_obs = True

    preset_goal: np.ndarray | None = None
    obstacles: tuple = ()

    def layout(self, head: np.ndarray, heading: np.ndarray, rng: np.random.Generator) -> tuple[tuple, np.ndarray | None]:
        """Sample the target and the obstacles; retry until a way through exists."""
        if self.param("field_radius", 0.0) > 0.0:
            return self._layout_field(head, heading, rng)
        d = float(rng.uniform(self.param("goal_dist_min", 2.0), self.param("goal_dist_max", 2.8)))
        b = float(np.deg2rad(rng.uniform(-1.0, 1.0) * self.param("bearing_deg", 0.0)))
        yaw = float(np.arctan2(heading[1], heading[0])) + b
        u = np.array([np.cos(yaw), np.sin(yaw)])
        perp = np.array([-u[1], u[0]])
        goal = head[:2] + d * u
        n = int(rng.integers(int(self.param("n_obs_min", 4)), int(self.param("n_obs_max", 8)) + 1))
        spread = self.param("spread", 1.0)
        keep_c = head[:2] + START_KEEP_OUT[0] * np.array([np.cos(yaw), np.sin(yaw)])
        lo, hi = np.minimum(head[:2], goal) - 1.6, np.maximum(head[:2], goal) + 1.6
        for _ in range(30):
            obs: list[tuple] = []
            n_block = int(self.param("n_block", 1))               # obstacles placed ON the straight line to the target, so the direct path is always blocked
            hw = self.param("barrier_halfwidth", 0.0)
            if hw > 0.0:                                          # a barrier ACROSS the route, wider than the gait's sideways sweep, with one gap at a random place:
                s_bar = float(rng.uniform(0.8, max(0.9, d - 0.7)))      # the robot cannot arc round it and has to feel along it to find the gap
                gap_c = float(rng.uniform(-hw, hw))
                gap_w = self.param("barrier_gap", 0.45)
                for t in np.arange(-hw, hw + 1e-6, 0.26):
                    if abs(t - gap_c) < gap_w / 2 + 0.1:
                        continue
                    kind = ("box", "cyl", "ell")[int(rng.integers(0, 3))]
                    a = float(rng.uniform(0.10, 0.16))
                    bb = float(rng.uniform(0.10, 0.18)) if kind != "cyl" else a
                    c = head[:2] + (s_bar + float(rng.uniform(-0.05, 0.05))) * u + (t + float(rng.uniform(-0.03, 0.03))) * perp
                    obs.append((kind, float(c[0]), float(c[1]), float(rng.uniform(0, np.pi)), a, bb))
                n_block = 0
            n_target = len(obs) + (n if hw <= 0.0 else int(self.param("n_extra", 0)))
            for i in range(n_target * 6):
                if len(obs) == n_target:
                    break
                kind = ("box", "cyl", "ell")[int(rng.integers(0, 3))]
                blocker = len(obs) < n_block and hw <= 0.0
                a = float(rng.uniform(0.12, 0.22)) if blocker else float(rng.uniform(0.05, 0.18))
                bb = float(rng.uniform(0.10, 0.28)) if kind != "cyl" else a
                s_along = float(rng.uniform(0.7, d - 0.5)) if blocker else float(rng.uniform(0.5, d - 0.3))
                t = float(rng.uniform(-0.08, 0.08)) if blocker else float(rng.uniform(-spread, spread))
                c = head[:2] + s_along * u + t * perp
                ob = (kind, float(c[0]), float(c[1]), float(rng.uniform(0, np.pi)), a, bb)
                r = bounding_radius(ob)
                if np.linalg.norm(c - keep_c) < START_KEEP_OUT[1] + r or np.linalg.norm(c - goal) < GOAL_KEEP_OUT_M + r:
                    continue
                if any(np.linalg.norm(c - np.array(o[1:3])) < r + bounding_radius(o) + 0.04 for o in obs):
                    continue
                obs.append(ob)
            field = DistanceField(tuple(obs), goal, lo, hi)
            direct_blocked = any(not field.is_free(head[:2] + f * (goal - head[:2])) for f in np.linspace(0.1, 0.9, 33))
            if direct_blocked and field.is_free(head[:2]) and np.isfinite(field.value(head[:2])) and field.value(head[:2]) < 3.0 * d:
                break
        else:
            obs, field = [], DistanceField((), goal, lo, hi)
        self.obstacles, self._field = tuple(obs), field
        return (), np.array([*goal, head[2]])

    def _layout_field(self, head: np.ndarray, heading: np.ndarray, rng: np.random.Generator) -> tuple[tuple, np.ndarray | None]:
        """Robot surrounded by an obstacle field (``field_radius`` m) with the target somewhere amongst it, at any bearing.

        Obstacles keep at least ``min_gap`` m (surface to surface) from each other so the robot can always squeeze through; the start area and the target
        stay clear, and a layout is only accepted if a route to the target exists (and is not more than ``max_detour`` times the straight distance).
        """
        R = self.param("field_radius", 2.0)
        n = int(rng.integers(int(self.param("n_obs_min", 10)), int(self.param("n_obs_max", 16)) + 1))
        gap = self.param("min_gap", 0.40)
        centre = head[:2] + START_KEEP_OUT[0] * heading[:2] / max(np.linalg.norm(heading[:2]), 1e-9)
        lo, hi = centre - R - 0.5, centre + R + 0.5
        for attempt in range(90):
            d = float(rng.uniform(self.param("goal_dist_min", 1.2), self.param("goal_dist_max", 2.0)))
            ang = float(rng.uniform(0.0, 2.0 * np.pi))
            goal = head[:2] + d * np.array([np.cos(ang), np.sin(ang)])
            obs: list[tuple] = []
            for it in range(n * 14):
                if len(obs) == n:
                    break
                kind = ("box", "cyl", "ell")[int(rng.integers(0, 3))]
                a = float(rng.uniform(0.05, 0.20))
                bb = float(rng.uniform(0.05, 0.25)) if kind != "cyl" else a
                if it < int(self.param("n_corridor", 0)) * 6:          # first the obstacles in the band between robot and target, so there are several layers to get through
                    ud = (goal - head[:2]) / d
                    c = head[:2] + float(rng.uniform(0.45, d - 0.35)) * ud + float(rng.uniform(-0.40, 0.40)) * np.array([-ud[1], ud[0]])
                else:
                    rr = R * float(np.sqrt(rng.uniform(0.0, 1.0)))
                    th = float(rng.uniform(0.0, 2.0 * np.pi))
                    c = centre + rr * np.array([np.cos(th), np.sin(th)])
                ob = (kind, float(c[0]), float(c[1]), float(rng.uniform(0, np.pi)), a, bb)
                r = bounding_radius(ob)
                if np.linalg.norm(c - centre) < START_KEEP_OUT[1] + r or np.linalg.norm(c - goal) < GOAL_KEEP_OUT_M + r:
                    continue
                if any(np.linalg.norm(c - np.array(o[1:3])) < r + bounding_radius(o) + gap for o in obs):
                    continue
                obs.append(ob)
            field = DistanceField(tuple(obs), goal, lo, hi)
            v = field.value(head[:2])
            seg = goal - head[:2]
            L2 = float(seg @ seg)
            on_line = sum(1 for o in obs if abs(float(np.cross(seg, np.array(o[1:3]) - head[:2]))) / np.sqrt(L2) < self.param("line_band", 0.40)
                          and 0.0 < float((np.array(o[1:3]) - head[:2]) @ seg) / L2 < 1.0)        # obstacles sitting between the robot and the target = "layers"
            if on_line < int(self.param("min_on_line", 0)) * (1 if attempt < 60 else 0):                 # relax the layer requirement before ever falling back to an empty floor
                continue
            if len(obs) >= n // 2 and field.is_free(head[:2]) and np.isfinite(v) and v < self.param("max_detour", 2.5) * d:
                self.obstacles, self._field = tuple(obs), field
                return (), np.array([*goal, head[2]])
        raise RuntimeError("obstacle layout: no valid layout found in 90 attempts; relax field_radius/min_gap/max_detour instead of training on an empty floor")

    def reset(self, sim: PetSim, rng: np.random.Generator) -> None:
        """As GoalReach; also counts obstacle-contact ticks and starts the progress-stall clock."""
        super().reset(sim, rng)
        self.contact_ticks = 0
        self._obs_ids = {sim.model.geom(f"obs{i}").id for i in range(len(self.obstacles))}
        self._best_path = self._field.value(self._head(sim)[:2])
        self._last_gain_t = 0
        self._visited: set[tuple[int, int]] = set()
        self._explore_total = 0.0

    def _spawn_goal(self, origin: np.ndarray, first: bool = False, sim: PetSim | None = None) -> None:
        """The first goal is the one laid out with the obstacles."""
        if first and self.preset_goal is not None:
            self.goal = self.preset_goal.copy()
            self.goal[2] = self._z
            self.dist0 = self._dist_start = float(np.linalg.norm((self.goal - origin)[:2]))
            self.bearing0 = 0.0
            return
        super()._spawn_goal(origin, first, sim)

    def _point_at_goal(self, com: np.ndarray) -> float:
        """Shaping direction = steepest descent of the path-distance field (the way round the obstacles); returns the straight distance to the goal."""
        d2 = self._field.direction(com[:2])
        if not np.any(d2):
            u, _ = self._to_goal(com)
            d2 = u[:2]
        self.heading = np.array([d2[0], d2[1], 0.0])
        self.side = np.array([-d2[1], d2[0], 0.0])
        return self._to_goal(com)[1]

    def reward(self, sim: PetSim, action: np.ndarray, prev_action: np.ndarray) -> tuple[float, dict[str, float]]:
        """GoalReach reward; contact penalty per tick touching an obstacle; tracks the best path distance for the stall test."""
        total, terms = super().reward(sim, action, prev_action)
        touching = any(c.geom1 in self._obs_ids or c.geom2 in self._obs_ids for c in sim.data.contact[:sim.data.ncon])
        self.contact_ticks += int(touching)
        terms["obstacle_contact"] = -self.param("contact_penalty", 0.0) if touching else 0.0
        hp = self._head(sim)[:2]
        cell = (int(np.floor(hp[0] / EXPLORE_CELL_M)), int(np.floor(hp[1] / EXPLORE_CELL_M)))
        new_cell = cell not in self._visited                    # search bonus: reward the head for reaching ground it has not been on, so it sweeps along an obstacle instead of pushing at it
        self._visited.add(cell)
        terms["explore"] = self.param("explore_weight", 0.0) if new_cell and self._explore_total < self.param("explore_cap", 3.0) else 0.0       # capped per episode so wandering cannot outearn arriving
        self._explore_total += terms["explore"]
        total += terms["explore"]
        pd = self._field.value(hp)
        if pd < self._best_path - self.param("stall_gain_m", 0.10):
            self._best_path, self._last_gain_t = pd, self._t
        stall = self.param("stall_s", 0.0)
        terms["stall"] = 0.0
        if stall > 0.0 and (self._t - self._last_gain_t) / self.cfg.control_hz > stall:         # giving up must not beat wandering: charge the time penalty for the rest of the episode
            remaining_s = max(0.0, self.cfg.episode_s - self._t / self.cfg.control_hz)
            terms["stall"] = -self.param("time_penalty", 0.0) * remaining_s
        return total + terms["obstacle_contact"] + terms["stall"], terms

    def terminated(self, sim: PetSim) -> bool:
        """End on arrival, glitch, or when the best path distance has not improved by ``stall_gain_m`` for ``stall_s`` seconds (0 = never)."""
        stall = self.param("stall_s", 0.0)
        stalled = stall > 0.0 and (self._t - self._last_gain_t) / self.cfg.control_hz > stall
        return super().terminated(sim) or stalled

    def metrics(self, sim: PetSim) -> dict[str, float]:
        """GoalReach metrics plus the number of obstacles and the fraction of the episode spent touching one."""
        return {**super().metrics(sim), "n_obstacles": float(len(self.obstacles)), "contact_frac": self.contact_ticks / max(self._t, 1)}
