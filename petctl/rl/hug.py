"""Ground-truth hug metric and hug reward (Regimen v2 plan, sec 5.3, proximity definition after M1). Privileged: reads the post probe, never the sensor model.

Hug metric v2 (task knob ``hug_span_deg`` = 180, shape-agnostic): a wrap tick additionally needs the unwrapped angular span (about the object's vertical axis) of the outward
surface normals at the nearest surface points of the run's hugging faces to be >= 180 deg (flat side: 0; one corner: 90; half-way round anything: 180); ``hug_modules >= H``
(``hug_h``, 4 for v2) as before. Held = 3 s window with >= 90 % wrap ticks. Encirclement, normal force and the old wrap angle stay diagnostics.

A module "hugs" when any of its faces is within ``D_PROX_M`` (15 mm) of the post surface in at least ``DEBOUNCE_K`` of the last ``DEBOUNCE_N`` ticks.
``hug_modules`` is the longest chain-contiguous run of hugging modules; a tick is a hug tick when ``hug_modules >= H = max(5, ceil(0.4 n))``; the hug is
*held* when, in the sliding ``hold_s`` (default 3 s) window, at least ``DUTY`` (90 %) of the ticks are hug ticks (dynamic holds with brief dropouts count).
No squeeze/pressure condition. Encirclement (post axis inside the polygon of the hugging modules' centres), normal force and wrap angle are diagnostics only.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass

import numpy as np

from petctl.rl.capacitive import PostProbe, PostRead
from petctl.sim.model_builder import PetSim
from petctl.sim.shapes import shape_of, span_deg

DEBOUNCE_N, DEBOUNCE_K = 5, 3
HOLD_S = 3.0
DUTY = 0.9                       # fraction of the ticks of the hold window that must be hug ticks
SQUEEZE_CAP_N = 1.0              # per-face normal force at which the (optional, default weight 0) squeeze term saturates
PEN_VALID_M = 0.005              # post penetration beyond this is charged by the validity term
POST_FORCE_FREE_N = 5.0          # v3: total post normal force charged only above this
DEFAULT_WEIGHTS = {"approach_weight": 5.0, "coverage_weight": 2.0, "squeeze_weight": 0.0, "hold_weight": 1.0, "hug_bonus": 100.0, "speed_gain": 1.0}


def hug_threshold(n_modules: int) -> int:
    """Modules that must be in a contiguous run for a hug tick: ``max(5, ceil(0.4 n))``."""
    return max(5, math.ceil(0.4 * n_modules))


def encircles(centres_xy: np.ndarray, post_xy: np.ndarray) -> bool:
    """True if ``post_xy`` lies inside the polygon through ``centres_xy`` (chain order, closed): ray-casting point-in-polygon test (diagnostic)."""
    if len(centres_xy) < 3:
        return False
    x, y = float(post_xy[0]), float(post_xy[1])
    inside = False
    for (x1, y1), (x2, y2) in zip(centres_xy, np.roll(centres_xy, -1, axis=0)):
        if (y1 > y) != (y2 > y) and x < (x2 - x1) * (y - y1) / (y2 - y1) + x1:
            inside = not inside
    return inside


def longest_run(mask: np.ndarray) -> tuple[int, int]:
    """Length and start index of the longest run of True in ``mask`` ((0, 0) if none)."""
    best, best_i, cur, start = 0, 0, 0, 0
    for i, v in enumerate(mask):
        if v:
            if cur == 0:
                start = i
            cur += 1
            if cur > best:
                best, best_i = cur, start
        else:
            cur = 0
    return best, best_i


@dataclass
class HugState:
    """Result of one tick of the metric."""

    module_hug: np.ndarray       # (n,) bool, debounced
    hug_modules: int             # longest chain-contiguous run
    run_start: int
    hug_tick: bool
    hold_ticks: int              # consecutive hug ticks so far
    duty: float                  # fraction of hug ticks in the hold window (0 until the window is full)
    held: bool                   # hold completed (sticky)
    just_held: bool              # hold completed on this very tick
    force_n: float               # total normal force on the post from the hugging modules' faces
    face_force: np.ndarray       # (n, 3) per-face normal force
    wrap_deg: float              # diagnostic floor-plane angle swept by the run's module centres about the post axis
    coverage: float              # hugging modules / n_modules (debounced, any run)
    encircled: bool = False      # diagnostic: post axis inside the polygon of the run's module centres
    span_deg: float = 0.0        # hug v2: unwrapped angular span of the outward object normals at the nearest surface points of the run's hugging faces


class HugTracker:
    """Per-episode state of the hug metric. Call ``update`` once per control tick."""

    def __init__(self, n_modules: int, control_hz: float, hold_s: float = HOLD_S, h_min: int | None = None, span_min_deg: float | None = None) -> None:
        """``h_min`` overrides the default threshold (tests). ``span_min_deg`` (hug metric v2, shape-agnostic): a hug tick additionally needs the normal span of the run >= this (None = v1)."""
        self.span_min = span_min_deg
        self.n = n_modules
        self.h = int(h_min) if h_min is not None else hug_threshold(n_modules)
        self.hold_need = max(1, int(round(hold_s * control_hz)))       # ticks in the sliding hold window
        self.dt = 1.0 / control_hz
        self.reset()

    def reset(self) -> None:
        """Clear the debounce history and counters."""
        self._hist: deque[np.ndarray] = deque(maxlen=DEBOUNCE_N)
        self.hold_ticks = 0
        self._win: deque[bool] = deque(maxlen=self.hold_need)
        self.duty = 0.0
        self.held = False
        self.t = 0
        self.t_contact = float("nan")      # first tick any module touched the post (any contact, s)
        self.t_hug = float("nan")          # tick the hold completed (s)
        self.t_hug_start = float("nan")    # start of the final continuous hug (s)
        self.peak_modules = 0
        self.peak_wrap_deg = 0.0
        self.peak_span_deg = 0.0
        self._ang = np.full(self.n, np.nan)       # last known outward-normal angle per module (a debounced hugging module keeps it through single-tick dropouts)
        self.peak_force_n = 0.0
        self.peak_duty = 0.0
        self.ever_encircled = False
        self.last: HugState | None = None

    def push_hug_tick(self, hug_tick: bool, now: float) -> bool:
        """Advance the 3 s duty window with one hug tick; returns True on the tick the hold is first satisfied."""
        self._win.append(bool(hug_tick))
        self.hold_ticks = self.hold_ticks + 1 if hug_tick else 0
        self.duty = float(np.mean(self._win)) if len(self._win) == self.hold_need else 0.0
        self.peak_duty = max(self.peak_duty, self.duty)
        if len(self._win) == self.hold_need and self.duty >= DUTY - 1e-9 and not self.held:
            self.held, self.t_hug, self.t_hug_start = True, now, now - (self.hold_need - 1) * self.dt
            return True
        return False

    def update(self, read: PostRead, centres_xy: np.ndarray, post_xy: np.ndarray) -> HugState:
        """Advance one tick with the ground-truth contacts ``read``, the module centres (n, 2) and the post axis xy."""
        self.t += 1
        now = self.t * self.dt
        face_contact = read.touch.any(axis=1)
        if bool(read.any_contact.any()) and math.isnan(self.t_contact):
            self.t_contact = now
        self._hist.append(face_contact)
        module_hug = np.sum(np.array(self._hist), axis=0) >= DEBOUNCE_K
        run, start = longest_run(module_hug)
        in_run = np.zeros(self.n, bool)
        in_run[start:start + run] = run > 0
        span = 0.0
        if self.span_min is not None:
            if read.normal_ang is not None:
                cur = np.asarray(read.normal_ang, float)
                self._ang = np.where(np.isfinite(cur), cur, self._ang)
                self._ang[~module_hug] = np.nan
            vals = self._ang[in_run]
            span = span_deg(vals[np.isfinite(vals)])
        hug_tick = run >= self.h and (self.span_min is None or span >= self.span_min - 1e-6)
        just = self.push_hug_tick(hug_tick, now)
        wrap = 0.0
        if run >= 2:
            rel = centres_xy[in_run] - post_xy
            ang = np.unwrap(np.arctan2(rel[:, 1], rel[:, 0]))
            wrap = float(np.rad2deg(abs(ang[-1] - ang[0])))
        enc = bool(run >= 3 and encircles(centres_xy[in_run], post_xy))
        self.ever_encircled |= enc
        force = float(read.force[module_hug].sum())
        self.peak_modules = max(self.peak_modules, run)
        self.peak_wrap_deg = max(self.peak_wrap_deg, wrap)
        self.peak_span_deg = max(self.peak_span_deg, span)
        self.peak_force_n = max(self.peak_force_n, force)
        self.last = HugState(module_hug, run, start, hug_tick, self.hold_ticks, self.duty, self.held, just, force, read.force.copy(), wrap, float(module_hug.sum()) / self.n, enc, span)
        return self.last


class HugMixin:
    """Hug metric + reward for a goal task whose episode has a post (needs ``self.param``, ``self.cfg``; use with ``GoalReach`` descendants).

    Reward terms (weights are task params, defaults in ``DEFAULT_WEIGHTS``): ``approach`` (telescoping reduction of the mean module-to-post surface distance),
    ``coverage`` (ratchet on the running peak of the longest contiguous run), ``squeeze`` (optional, default weight 0: no squeeze condition), ``hold``
    (per hug tick, growing with the consecutive count), terminal ``hug_bonus`` x (1 + speed_gain x remaining-time fraction), ``hug_validity`` (post
    penetration beyond 5 mm). Nothing here reads the sensor model or an angle.
    """

    def hug_active(self, sim: PetSim) -> bool:
        """True if the episode has a post."""
        return sim.p.post is not None

    def hug_reset(self, sim: PetSim) -> None:
        """Start the tracker for this episode (call from ``reset``)."""
        self._hug_probe = PostProbe.of(sim)
        self._hug = HugTracker(int(sim.p.n_modules), self.cfg.control_hz, self.param("hug_hold_s", HOLD_S), None if self.param("hug_h", 0.0) <= 0 else int(self.param("hug_h", 0.0)),
                                None if self.param("hug_span_deg", 0.0) <= 0 else self.param("hug_span_deg", 0.0))  # type: ignore[attr-defined]
        self._hug_peak = 0
        self._hug_prev_dist = self._hug_mean_dist(sim) if self._hug_probe is not None else 0.0
        self.hug_success = False

    def _hug_mean_dist(self, sim: PetSim) -> float:
        d = shape_of(sim.p.post_shape).dist_xy(sim.data.xipos[1:self._hug.n + 1, :2], np.array(sim.p.post))
        return float(np.mean(d))

    def hug_step(self, sim: PetSim) -> tuple[float, dict[str, float]]:
        """Update the metric for the tick just simulated; returns (reward, logged terms)."""
        terms = {"approach": 0.0, "coverage": 0.0, "squeeze": 0.0, "hold": 0.0, "hug_bonus": 0.0, "hug_validity": 0.0}
        if self._hug_probe is None:
            return 0.0, terms
        P = self.param
        st = self._hug.update(self._hug_probe.read(), sim.data.xipos[1:self._hug.n + 1, :2], np.array(sim.p.post))
        dist = self._hug_mean_dist(sim)
        terms["approach"] = P("approach_weight", DEFAULT_WEIGHTS["approach_weight"]) * (self._hug_prev_dist - dist)
        self._hug_prev_dist = dist
        v3 = P("hug_v3", 0.0) > 0.0
        if v3:                                                  # v3: dense per-tick coverage (pays for STAYING), small discovery ratchet on the running peak
            if P("hug_v5", 0.0) > 0.0:                          # v5 (shape-agnostic): per tick x min(run/H, 1) x min(normal span / 180 deg, 1)
                terms["coverage"] = P("coverage_weight", DEFAULT_WEIGHTS["coverage_weight"]) * min(st.hug_modules / self._hug.h, 1.0) * min(st.span_deg / 180.0, 1.0)
            else:
                terms["coverage"] = P("coverage_weight", DEFAULT_WEIGHTS["coverage_weight"]) * st.hug_modules / self._hug.h
            terms["coverage_ratchet"] = P("coverage_ratchet", 0.0) * max(0, st.hug_modules - self._hug_peak)
            self._hug_peak = max(self._hug_peak, st.hug_modules)
            terms["post_force"] = -P("post_force_penalty", 0.0) * max(0.0, float(self._hug_probe.read().force.sum()) - POST_FORCE_FREE_N)
        elif st.hug_modules > self._hug_peak:
            terms["coverage"] = P("coverage_weight", DEFAULT_WEIGHTS["coverage_weight"]) * (st.hug_modules - self._hug_peak)
            self._hug_peak = st.hug_modules
        if st.hug_tick:
            sq_w = P("squeeze_weight", DEFAULT_WEIGHTS["squeeze_weight"])
            if sq_w != 0.0:
                on = self._hug_probe.read().touch & st.module_hug[:, None]
                f = np.minimum(st.face_force[on], SQUEEZE_CAP_N) / SQUEEZE_CAP_N
                terms["squeeze"] = sq_w * float(f.mean()) if f.size else 0.0
            terms["hold"] = P("hold_weight", DEFAULT_WEIGHTS["hold_weight"]) * (1.0 if v3 else 1.0 + st.hold_ticks / self._hug.hold_need)
        if st.just_held:
            self.hug_success = True
            max_steps = max(1, int(round(self.cfg.episode_s * self.cfg.control_hz)))
            terms["hug_bonus"] = P("hug_bonus", DEFAULT_WEIGHTS["hug_bonus"]) * (1.0 + P("speed_gain", DEFAULT_WEIGHTS["speed_gain"]) * (1.0 - self._hug.t / max_steps))
        pen = self._hug_probe.read().penetration
        terms["hug_validity"] = -self.cfg.reward.validity * max(0.0, pen - PEN_VALID_M)
        return float(sum(terms.values())), terms

    def hug_metrics(self) -> dict[str, float]:
        """Episode-level hug numbers (ground truth) for the gate / eval."""
        h, st = self._hug, self._hug.last
        return {"hug_success": float(self.hug_success), "hug_modules": float(h.peak_modules), "hug_modules_now": float(0 if st is None else st.hug_modules),
                "hug_threshold": float(h.h), "hold_duty": h.peak_duty, "encircled": float(h.ever_encircled), "wrap_deg": h.peak_wrap_deg, "normal_span_deg": h.peak_span_deg, "post_force_n": h.peak_force_n, "coverage_frac": float(0 if st is None else st.coverage),
                "t_contact_s": h.t_contact, "t_hug_s": h.t_hug, "t_hug_start_s": h.t_hug_start}
