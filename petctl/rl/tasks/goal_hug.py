"""Goal-hug task (Regimen v2): cross the obstacle field to the post, then coil round it and hug it (plan sec 5.3).

``GoalObstacles`` with the target post always on (``post`` defaults to 1) plus the ground-truth hug metric and reward of ``petctl.rl.hug``. The navigation
progress reward keeps paying until the first post contact, after which the hug terms take over; the episode ends successfully when the hug has been held
(``hug_hold_s``, default 3 s window, 90 % duty). ``target_family`` > 0: the post is replaced by a random cylinder / box / rounded box (``petctl.rl.target_shapes``; ``shape_phase``
1|2, ``target_heldout`` 1 = the frozen held-out gate set by seed) and ``hug_span_deg`` (180) switches on hug metric v2 (normal span). ``success`` in the metrics is the hug success. Knobs (besides those of GoalObstacles/GoalReach): ``hug_hold_s``, ``hug_h``
(contiguous-module threshold; default max(5, ceil(0.4 n))), ``approach_weight``, ``coverage_weight``, ``squeeze_weight`` (default 0: no squeeze condition), ``hold_weight``, ``hug_bonus``,
``speed_gain``. Start distribution knob ``hug_start_mix`` (> 0: per episode a start mode is drawn: coil round the post / half coil / straight, see ``petctl.rl.hug_start``; mix 30/30/40
until the switch flag exists, then 10/20/70; ``hug_mix_switched`` > 0 forces the later mix; ``hug_force_mode`` 1/2 forces coil/half). Observation knobs ``capacitive_obs`` / ``capacitive_off`` apply as for any task (see obs.py).
"""

from __future__ import annotations

import numpy as np

from petctl.rl.hug import HugMixin
from petctl.rl.hug_start import MIX_V3_EARLY, MIX_V3_LATE, MODES, PERTURB_V3_EARLY, PERTURB_V3_LATE, arrange_coil, mix_probs, mix_switched, sample_start_mode
from petctl.rl.tasks.goal_obstacles import DistanceField, GoalObstacles
from petctl.rl.tasks.goal_reach import GLITCH_PENALTY, GLITCH_PENETRATION_M
from petctl.sim.model_builder import PetSim
from petctl.sim.shapes import shape_of


class GoalHug(HugMixin, GoalObstacles):
    """GoalObstacles + post hug reward; terminates on a held hug."""

    name = "goal_hug"
    score_label = "m towards goal"
    DEFAULTS = {"post": 1.0}
    _start_mode = "straight"
    _start_placed = False
    _mode_params: dict[str, float] = {}
    _v3_switched = False
    target_shape = None             # set by the env before ``layout`` when ``target_family`` > 0 (petctl.rl.target_shapes)

    def param(self, key: str, default: float) -> float:
        """Task knob with this task's own defaults (``post`` is on); the start mode of this episode overrides the layout knobs."""
        if key in self._mode_params:
            return self._mode_params[key]
        return float(self.cfg.task_params.get(key, self.DEFAULTS.get(key, default)))

    def layout(self, head: np.ndarray, heading: np.ndarray, rng: np.random.Generator) -> tuple[tuple, np.ndarray | None]:
        """With ``hug_start_mix``: draw this episode's start mode; ``straight`` places the post 5-20 cm (surface gap) from the head at +-60 deg, the coil modes a placeholder 0.3 m ahead
        (``arrange_start`` moves it). Without the knob nothing is drawn (random stream unchanged)."""
        self._start_mode, self._mode_params, self._start_placed = "straight", {}, False
        if self.param("hug_start_mix", 0.0) > 0.0:
            switched = self.param("hug_mix_switched", 0.0) > 0.0 or mix_switched()
            self._v3_switched = switched
            if self.param("hug_mix_v3", 0.0) > 0.0:                                          # hug v3: coil-heavy phase 1 (70/30/0), phase 2 30/30/40
                self._start_mode = sample_start_mode(rng, *(MIX_V3_LATE if switched else MIX_V3_EARLY))
            else:
                self._start_mode = sample_start_mode(rng, *mix_probs(switched))
            if self.param("hug_force_mode", 0.0) > 0.0:                                       # tests / scripted-policy checks: 1 = coil, 2 = half, 0 = draw
                self._start_mode = MODES[int(self.param("hug_force_mode", 0.0))]
            if self._start_mode == "straight":
                self._mode_params = {"goal_dist_min": shape_of(self.target_shape).bound_radius + 0.05, "goal_dist_max": shape_of(self.target_shape).bound_radius + 0.20, "bearing_deg": 60.0}
            else:
                self._mode_params = {"goal_dist_min": 0.3, "goal_dist_max": 0.3, "bearing_deg": 0.0}
        return super().layout(head, heading, rng)

    def arrange_start(self, sim: PetSim, rng: np.random.Generator) -> np.ndarray | None:
        """Coil / half-coil start modes: pose the body round the post (see ``hug_start.arrange_coil``), move the post to the coil centre and rebuild the target field. None = ordinary start."""
        if self._start_mode == "straight":
            return None
        scale = self.param("hug_perturb_scale", 1.0)
        if self.param("hug_mix_v3", 0.0) > 0.0:
            scale *= PERTURB_V3_LATE if self._v3_switched else PERTURB_V3_EARLY
        goal = arrange_coil(sim, rng, self._start_mode, scale)
        if goal is None:
            return None
        self._start_placed = True
        self._field = DistanceField((), goal[:2], goal[:2] - 1.6, goal[:2] + 1.6)
        return goal

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
        mode = self._start_mode if self._start_placed or self._start_mode == "straight" else "straight"
        return {**m, **h, "success": h["hug_success"], "start_mode": float(MODES.index(mode)), "start_failed": float(self._start_mode != "straight" and not self._start_placed)}
