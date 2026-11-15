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

import mujoco  # type: ignore[import-untyped]
import numpy as np

from petctl.rl.hug import HugMixin
from petctl.rl.hug_buffer import MIN_RUN, MIN_SPAN_DEG, MIN_STATES, P_BUFFER, PERTURB_RAD, HugStartBuffer, make_entry
from petctl.rl.hug_start import MIX_V3_EARLY, MIX_V3_LATE, MODES, PERTURB_V3_EARLY, PERTURB_V3_LATE, arrange_coil, mix_probs, mix_switched, sample_start_mode
from petctl.rl.tasks.goal_obstacles import DistanceField, GoalObstacles
from petctl.rl.hug_search import START_PEN_M
from petctl.rl.tasks.goal_reach import GLITCH_PENALTY, GLITCH_PENETRATION_M
from petctl.sim.model_builder import PetSim
from petctl.sim.shapes import shape_of

SHAPE_KINDS = ("cyl", "box", "rbox")      # ``shape_kind`` metric index (curriculum runs): cylinders, boxes, rounded boxes


class GoalHug(HugMixin, GoalObstacles):
    """GoalObstacles + post hug reward; terminates on a held hug."""

    name = "goal_hug"
    score_label = "m towards goal"
    DEFAULTS = {"post": 1.0}
    _start_mode = "straight"
    _start_placed = False
    _mode_params: dict[str, float] = {}
    _v3_switched = False
    _bstart: dict | None = None     # buffer start of this episode (``pick_buffer_start``), restored by ``restore_buffer_start``
    _buffer: HugStartBuffer | None = None
    start_buffer: HugStartBuffer | None = None   # the env's in-memory buffer (set by the env before ``pick_buffer_start``)
    _stored = False
    _stalled = False
    hug_family_levels: dict = {}    # per-family ladder indices of the success curriculum (``hug_curriculum`` 2; set by the env before ``reset``)
    hug_level = 3                   # training success curriculum level of this episode (task knob ``hug_curriculum``; set by the env before ``reset``)
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
        if self.param("hug_gap_min", 0.0) > 0.0:                                              # v6 straight start: head hug_gap_min..hug_gap_max (m) from the object SURFACE at +-bearing_deg
            r = shape_of(self.target_shape).bound_radius
            self._mode_params = {"goal_dist_min": r + self.param("hug_gap_min", 0.0), "goal_dist_max": r + self.param("hug_gap_max", 0.4)}
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

    # ------------------------------------------------------------ self-bootstrapping start buffer (m2_hug6, petctl.rl.hug_buffer)
    def pick_buffer_start(self, rng: np.random.Generator) -> dict | None:
        """Called by the env before the target shape is drawn: with the ``hug_buffer`` knob on (and a buffer named by ``$PETCTL_HUG_BUFFER``) opens the buffer and, once it holds enough
        states, returns a stored state for this episode with probability ``hug_buffer_p`` (default 0.3); else None. Draws from the env rng only when the knob is on."""
        self._bstart, self._buffer, self._stored, self._stalled = None, None, False, False
        if self.param("hug_buffer", 0.0) <= 0.0 or self.param("target_family", 0.0) <= 0.0:
            return None
        self._buffer = self.start_buffer
        if self._buffer is None:
            return None
        r = np.random.default_rng(int(rng.integers(2 ** 31 - 1)))
        self._bstart = self._buffer.sample(r, self.param("hug_buffer_p", P_BUFFER), int(self.param("hug_buffer_min", MIN_STATES)))
        return self._bstart

    def restore_buffer_start(self, sim: PetSim, rng: np.random.Generator) -> np.ndarray:
        """Put the sim into the stored state (the env built it with the stored object at the stored place): body pose, joints and targets (each joint perturbed by up to +-0.1 rad, halved
        until the object is not penetrated), 0.25 s settle holding the targets. Rebuilds the target field round the object. Returns the joint targets (the env's initial target)."""
        from petctl.rl.capacitive import PostProbe

        b = self._bstart
        assert b is not None
        q, tgt, n = np.asarray(b["q"], float), np.asarray(b["tgt"], float), len(b["q"])
        post = (float(b["post"][0]), float(b["post"][1]))
        pert = rng.uniform(-1.0, 1.0, n)
        for scale in (1.0, 0.5, 0.25, 0.0):
            d = PERTURB_RAD * scale * self.param("hug_buffer_perturb", 1.0) * pert
            sim.reset(q + d)
            sim.data.qpos[:7] = b["qpos7"]
            sim.data.qvel[:] = 0.0
            sim._last_target = (tgt + d).copy()
            mujoco.mj_forward(sim.model, sim.data)
            probe = PostProbe.of(sim)
            if probe is None or probe.read().penetration <= START_PEN_M:
                break
        sim.step(tgt + d, 0.25)
        self._field = DistanceField((), np.array(post), np.array(post) - 1.6, np.array(post) + 1.6)
        self._bstart_run, self._bstart_span = int(b["run"]), float(b["span"])
        return np.asarray(tgt + d, float)

    def maybe_store(self, sim: PetSim, tgt: np.ndarray) -> bool:
        """Called by the env after each tick: once per episode, store the current state in the buffer when the hug run >= 3 and the normal span >= 90 deg (episodes that started from the buffer
        only when they beat the stored state's run or its span by 20 deg). Returns True if stored."""
        if self._buffer is None or self._stored or self._hug.last is None:
            return False
        st = self._hug.last
        if st.hug_modules < self.param("hug_buffer_min_run", MIN_RUN) or st.span_deg < self.param("hug_buffer_min_span", MIN_SPAN_DEG):
            return False
        if self._bstart is not None and not (st.hug_modules > self._bstart_run or st.span_deg >= self._bstart_span + 20.0):
            return False
        sh = self.target_shape
        self._buffer.add(make_entry(sim.joint_pos(), tgt, sim.data.qpos[:7], sim.p.post, (sh.kind, sh.a, sh.b, sh.rc, sh.yaw), st.hug_modules, st.span_deg))
        self._stored = True
        return True

    def reset(self, sim: PetSim, rng: np.random.Generator) -> None:
        """As GoalObstacles; starts the hug tracker."""
        super().reset(sim, rng)
        self.hug_reset(sim)
        self._stalled = False
        self._stall_best_dist, self._stall_best_span, self._stall_t = self._hug_prev_dist, 0.0, 0

    def reward(self, sim: PetSim, action: np.ndarray, prev_action: np.ndarray) -> tuple[float, dict[str, float]]:
        """GoalObstacles reward (progress stops paying at first post contact) plus the hug terms and the post-penetration glitch guard."""
        total, terms = super().reward(sim, action, prev_action)
        hug, hterms = self.hug_step(sim)
        if not np.isnan(self._hug.t_contact) and terms.get("progress", 0.0) != 0.0:
            total -= terms["progress"]
            terms["progress"] = 0.0
        terms.update(hterms)
        stall_s = self.param("hug_stall_s", 0.0)
        if stall_s > 0.0 and self._hug.last is not None and not self._stalled:               # no approach (mean surface distance -1 cm) and no span gain (+5 deg) for stall_s seconds: end the episode
            dist, span = self._hug_prev_dist, self._hug.last.span_deg
            if dist < self._stall_best_dist - 0.01 or span > self._stall_best_span + 5.0:
                self._stall_best_dist, self._stall_best_span, self._stall_t = min(dist, self._stall_best_dist), max(span, self._stall_best_span), self._t
            elif (self._t - self._stall_t) / self.cfg.control_hz > stall_s:
                self._stalled = True
                remaining_s = max(0.0, self.cfg.episode_s - self._t / self.cfg.control_hz)
                terms["hug_stall"] = -self.param("time_penalty", 0.0) * remaining_s            # giving up must not beat wandering
                total += terms["hug_stall"]
        probe = self._hug_probe
        if probe is not None and probe.read().penetration > GLITCH_PENETRATION_M and not self.glitch:
            self.glitch = True
            terms["glitch"] = -GLITCH_PENALTY
            total += terms["glitch"]
        return total + hug, terms

    def terminated(self, sim: PetSim) -> bool:
        """End on a held hug, or as GoalObstacles."""
        return self.hug_success or self._stalled or super().terminated(sim)

    def metrics(self, sim: PetSim) -> dict[str, float]:
        """GoalObstacles metrics with ``success`` = hug held, plus the hug diagnostics."""
        m = super().metrics(sim)
        m["goal_reached"] = m["success"]
        h = self.hug_metrics()
        mode = self._start_mode if self._start_placed or self._start_mode == "straight" else "straight"
        out = {**m, **h, "success": h["hug_success"], "start_mode": float(MODES.index(mode)), "start_failed": float(self._start_mode != "straight" and not self._start_placed)}
        if self.param("hug_buffer", 0.0) > 0.0 or self.param("hug_stall_s", 0.0) > 0.0:             # v6 diagnostics (absent from the older stages' metrics)
            out.update({"buffer_start": float(self._bstart is not None), "buffer_stored": float(self._stored), "stalled": float(self._stalled)})
        if self._hug_levels:                                                                          # success curriculum diagnostics (absent unless ``hug_curriculum``)
            out["shape_kind"] = float(SHAPE_KINDS.index(self.target_shape.kind)) if self.target_shape is not None else -1.0
            sh = self.target_shape
            out.update({"shape_a": float(sh.a) if sh is not None else -1.0, "shape_b": float(sh.b) if sh is not None else -1.0, "shape_rc": float(sh.rc) if sh is not None else -1.0,
                        "contact": float(not np.isnan(self._hug.t_contact))})
        return out
