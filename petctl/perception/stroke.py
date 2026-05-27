"""
Stroke and hold detectors — classify moving vs. static contact along the robot body.

StrokeDetector: tracks a hand moving along the body (centroid velocity above threshold).
HoldDetector:   detects sustained static contact on ≥2 modules or a wide single blob
                (centroid velocity below threshold).  Mutually exclusive with stroke
                by construction — both use the same VELOCITY_THRESHOLD.

No FSR data is used; capacitive pad readings are sufficient.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field

from petctl.types import RobotState

PAD_THRESHOLD: float = 0.35     # minimum per-pad value to contribute to centroid
TOUCH_THRESHOLD: float = 0.08   # minimum touch_total for module-level blob grouping
VELOCITY_THRESHOLD: float = 0.8  # body-units/second — below this is not a stroke
MIN_STROKE_TRAVEL: float = 0.35  # centroid must travel this far (body-units) over the velocity window
MIN_CONFIDENCE: float = 0.25    # R² floor — below this the centroid trajectory is too noisy to be a real stroke
WINDOW_FRAMES: int = 15          # rolling window depth (~0.75 s at 20 Hz)
MIN_WINDOW_FRAMES: int = 5       # minimum frames before fitting
TOUCH_GAP_GRACE_S: float = 0.25  # don't clear velocity window during brief inter-module gaps

# ---------------------------------------------------------------------------
# Pad adjacency tables
# ---------------------------------------------------------------------------

# Side faces (right / left): 4 pads — top_head(0), top_mid(1), top_rear(2), bottom_solo(3).
# Pads 0-1-2 form a linear chain along the body axis; pad 3 is physically isolated.
_SIDE_ADJACENCY: tuple[tuple[int, int], ...] = ((0, 1), (1, 2))

# Middle face: 6 pads arranged as a 2-wide × 3-deep grid (head→tail).
#   top pair: 0,1  (sub ≈ 0.23)
#   mid pair: 2,3  (sub ≈ 0.62)
#   bot pair: 4,5  (sub ≈ 1.00)
_MIDDLE_ADJACENCY: tuple[tuple[int, int], ...] = (
    (0, 1), (2, 3), (4, 5),   # within each side-by-side pair
    (0, 2), (1, 3),            # left/right columns, top→mid
    (2, 4), (3, 5),            # left/right columns, mid→bot
)

# Cross-module boundary pad indices (tail of module N ↔ head of module N+1).
_CROSS_SIDE_TAIL: int = 2          # top_rear
_CROSS_SIDE_HEAD: int = 0          # top_head
_CROSS_MID_TAIL: tuple[int, int] = (4, 5)   # bot pair
_CROSS_MID_HEAD: tuple[int, int] = (0, 1)   # top pair

# Sub-position of each pad along the body axis within its module (0 = head-end, 1 = tail-end).
# Derived from _PAD_CENTERS z-coordinates in rerun_viz.py:
#   z range: -1.75 (head) to -6.37 (tail), span = 4.62 cm
# Order: right pads 0-3, left pads 0-3, middle pads 0-5
# Canonical pad order: top_head(0), top_mid(1), top_rear(2), bottom_solo(3) for side faces;
#                      top×2(0-1), mid×2(2-3), bot×2(4-5) for middle face.
_PAD_BODY_SUB: tuple[float, ...] = (
    0.14, 0.43, 0.72, 0.0,          # right: top_head, top_mid, top_rear, bottom_solo
    0.14, 0.43, 0.72, 0.0,          # left:  top_head, top_mid, top_rear, bottom_solo
    0.23, 0.23, 0.62, 0.62, 1.0, 1.0,  # middle: top×2, mid×2, bot×2
)


@dataclass
class TouchBlob:
    """A contiguous run of active modules at one instant."""
    centroid: float      # weighted average module index (0.0 = head, 7.0 = tail)
    modules: list[int]   # contiguous module IDs
    intensity: float     # mean activation across blob modules
    width: int           # number of modules in blob


@dataclass
class StrokeReading:
    """Output of StrokeDetector when a stroke is detected."""
    centroid: float      # pad-level centroid this tick (body-axis units)
    velocity: float      # body-units/second, signed (+ = head→tail, − = tail→head)
    speed: float         # abs(velocity)
    direction: str       # "head_to_tail" | "tail_to_head"
    intensity: float     # mean pad activation of the active region
    confidence: float    # R² of the linear fit over the centroid window
    side: str            # which face(s) are active: "top", "left", "right", "top-left", etc.
    blobs: list[TouchBlob] = field(default_factory=list)


@dataclass
class HoldReading:
    """Output of HoldDetector when a hold is detected.

    A hold is either:
      - one qualifying blob spanning ≥2 modules (cradling / wide grip), or
      - ≥2 separate qualifying blobs (two hands at different locations).
    In both cases the global centroid is stationary (not stroking).
    """
    q_blobs: list[TouchBlob]  # qualifying blobs (each module ≥2 contiguous active pads)
    centroid: float            # global weighted pad centroid (body-axis units)
    duration: float            # seconds since hold onset
    intensity: float           # mean active-pad activation
    side: str                  # which face(s) are active


@dataclass
class CradleReading:
    """Output of CradleDetector when ≥4 modules are simultaneously touched."""
    modules: list[int]   # qualifying module IDs in body order
    centroid: float      # weighted centroid across active modules
    intensity: float     # mean per-module peak activation
    duration: float      # seconds since cradle onset
    side: str            # active face(s)


class StrokeDetector:
    """
    Detects a stroking gesture along the robot body.

    Call update() every control tick. Returns a StrokeReading when touch is
    moving fast enough to be a stroke, or None otherwise.
    """

    def __init__(self) -> None:
        self._window: deque[tuple[float, float]] = deque(maxlen=WINDOW_FRAMES)
        self._last_touch_t: float | None = None
        self._current_direction: str | None = None
        self._last_reading: StrokeReading | None = None

    def update(self, state: RobotState) -> StrokeReading | None:
        """Process one tick of RobotState. Returns StrokeReading or None."""
        centroid, intensity = _pad_centroid(state)

        if centroid is None:
            if (self._last_touch_t is None or
                    state.timestamp - self._last_touch_t >= TOUCH_GAP_GRACE_S):
                self._window.clear()
                self._last_touch_t = None
                self._current_direction = None
                self._last_reading = None
                return None
            # Within grace gap — return last valid reading so the emitter sees
            # a continuous stroke rather than a None that triggers a downgrade.
            return self._last_reading

        self._last_touch_t = state.timestamp
        self._window.append((state.timestamp, centroid))

        if len(self._window) < MIN_WINDOW_FRAMES:
            return None

        velocity, r_squared = _linear_fit(list(self._window))

        if abs(velocity) < VELOCITY_THRESHOLD or r_squared < MIN_CONFIDENCE:
            return None

        # A press keeps the centroid near the contact location — the linear fit can
        # still show a high slope from sensor noise or settling.  Require the centroid
        # to have actually moved over the window so that presses and local oscillations
        # are not mistaken for strokes.
        if abs(centroid - self._window[0][1]) < MIN_STROKE_TRAVEL:
            return None

        curr_dir = "head_to_tail" if velocity > 0 else "tail_to_head"

        if self._current_direction is None:
            self._current_direction = curr_dir
        elif curr_dir != self._current_direction:
            # Direction reversed — reset so the next stroke starts fresh.
            self._window.clear()
            self._window.append((state.timestamp, centroid))
            self._current_direction = None
            self._last_reading = None
            return None

        activations = {mid: s.touch_total for mid, s in state.sensors.items()}
        blobs = _find_blobs(activations)

        self._last_reading = StrokeReading(
            centroid=centroid,
            velocity=velocity,
            speed=abs(velocity),
            direction=curr_dir,
            intensity=intensity,
            confidence=r_squared,
            side=_active_side(state),
            blobs=blobs,
        )
        return self._last_reading


class HoldDetector:
    """
    Detects static contact on the robot body.

    Fires when the global touch centroid is not moving (velocity < VELOCITY_THRESHOLD)
    AND there is at least one qualifying blob.  A qualifying blob is a contiguous run
    of modules where each module has ≥2 adjacent active cap pads — this filters single
    isolated pad taps.

    The ContactClassifier downstream determines the sub-type (touch / squeeze / hold /
    twist / restrict / wrench) based on blob count and motor state.

    Mutually exclusive with StrokeDetector by construction.
    """

    def __init__(self) -> None:
        self._centroid_window: deque[tuple[float, float]] = deque(maxlen=WINDOW_FRAMES)
        self._hold_start: float | None = None

    def reset(self) -> None:
        """Clear velocity history — call after a stroke ends so stale fast frames don't delay hold detection."""
        self._centroid_window.clear()
        self._hold_start = None

    def update(self, state: RobotState) -> HoldReading | None:
        """Process one tick of RobotState. Returns HoldReading or None."""
        centroid, intensity = _pad_centroid(state)

        if centroid is None:
            self._centroid_window.clear()
            self._hold_start = None
            return None

        self._centroid_window.append((state.timestamp, centroid))

        if len(self._centroid_window) < MIN_WINDOW_FRAMES:
            return None

        velocity, _ = _linear_fit(list(self._centroid_window))
        if abs(velocity) >= VELOCITY_THRESHOLD:
            self._hold_start = None
            return None

        q_blobs = _find_qualifying_blobs(state, PAD_THRESHOLD, TOUCH_THRESHOLD)
        if not q_blobs:
            self._hold_start = None
            return None

        now = state.timestamp
        if self._hold_start is None:
            self._hold_start = now

        return HoldReading(
            q_blobs=q_blobs,
            centroid=centroid,
            duration=now - self._hold_start,
            intensity=intensity,
            side=_active_side(state),
        )


CRADLE_MODULE_COUNT: int = 4   # minimum simultaneous qualifying modules to detect cradle


class CradleDetector:
    """
    Detects when ≥4 modules are simultaneously touched.

    Fires when _find_qualifying_modules returns at least CRADLE_MODULE_COUNT IDs.
    Returns a CradleReading each tick while the condition holds, or None.
    Has highest gesture priority — checked before stroke and hold.
    """

    def __init__(self) -> None:
        self._start: float | None = None

    def reset(self) -> None:
        self._start = None

    def update(self, state: RobotState) -> CradleReading | None:
        """Process one tick. Returns CradleReading or None."""
        q_mods = _find_qualifying_modules(state, PAD_THRESHOLD)
        if len(q_mods) < CRADLE_MODULE_COUNT:
            self._start = None
            return None

        now = state.timestamp
        if self._start is None:
            self._start = now

        activations: dict[int, float] = {}
        for mid in q_mods:
            sens = state.sensors.get(mid)
            if sens is None:
                continue
            all_pads = (*sens.touch_right_pads, *sens.touch_left_pads, *sens.touch_middle_pads)
            activations[mid] = max(all_pads)

        sorted_mods = sorted(q_mods)
        total_w = sum(activations.values())
        centroid = (
            sum(m * activations.get(m, 0.0) for m in sorted_mods) / total_w
            if total_w > 0 else float(sorted_mods[len(sorted_mods) // 2])
        )
        intensity = total_w / len(sorted_mods) if sorted_mods else 0.0

        return CradleReading(
            modules=sorted_mods,
            centroid=centroid,
            intensity=intensity,
            duration=now - self._start,
            side=_active_side(state),
        )


# ------------------------------------------------------------------
# Helpers
# ------------------------------------------------------------------

def _face_has_contiguous_pair(
    pads: tuple[float, ...],
    adjacency: tuple[tuple[int, int], ...],
    threshold: float,
) -> bool:
    """True if any two physically adjacent pads in `pads` are both above threshold."""
    return any(pads[a] >= threshold and pads[b] >= threshold for a, b in adjacency)


def _find_qualifying_modules(state: RobotState, pad_threshold: float) -> set[int]:
    """Return module IDs that have ≥2 contiguous active pads.

    Checks intra-face adjacency on all three faces, then cross-module boundary
    adjacency (tail pad of module N + head pad of module N+1 qualifies both).
    """
    qualified: set[int] = set()

    for mod_id, sens in state.sensors.items():
        if (
            _face_has_contiguous_pair(sens.touch_right_pads, _SIDE_ADJACENCY, pad_threshold) or
            _face_has_contiguous_pair(sens.touch_left_pads, _SIDE_ADJACENCY, pad_threshold) or
            _face_has_contiguous_pair(sens.touch_middle_pads, _MIDDLE_ADJACENCY, pad_threshold)
        ):
            qualified.add(mod_id)

    for mod_id, sens in state.sensors.items():
        next_sens = state.sensors.get(mod_id + 1)
        if next_sens is None:
            continue
        if (
            (sens.touch_right_pads[_CROSS_SIDE_TAIL] >= pad_threshold and
             next_sens.touch_right_pads[_CROSS_SIDE_HEAD] >= pad_threshold) or
            (sens.touch_left_pads[_CROSS_SIDE_TAIL] >= pad_threshold and
             next_sens.touch_left_pads[_CROSS_SIDE_HEAD] >= pad_threshold) or
            (any(sens.touch_middle_pads[pi] >= pad_threshold for pi in _CROSS_MID_TAIL) and
             any(next_sens.touch_middle_pads[pi] >= pad_threshold for pi in _CROSS_MID_HEAD))
        ):
            qualified.add(mod_id)
            qualified.add(mod_id + 1)

    return qualified


def _find_qualifying_blobs(
    state: RobotState,
    pad_threshold: float,
    touch_threshold: float,  # noqa: ARG001 — kept for API symmetry with _find_blobs callers
) -> list[TouchBlob]:
    """Return TouchBlobs built from qualifying modules only.

    Only contiguous runs of qualifying modules (those with ≥2 contiguous active
    pads) become blobs.  Non-qualifying modules break blob continuity even if
    they have general touch activity.

    Uses max-pad value as the per-module activation signal so that modules with
    only 2 active pads (whose touch_total would be diluted across 14 pads) are
    still represented faithfully.
    """
    q_mods = _find_qualifying_modules(state, pad_threshold)
    activations: dict[int, float] = {}
    for mid in q_mods:
        sens = state.sensors.get(mid)
        if sens is None:
            continue
        all_pads = (*sens.touch_right_pads, *sens.touch_left_pads, *sens.touch_middle_pads)
        activations[mid] = max(all_pads)
    return _find_blobs(activations)


def _pad_centroid(state: RobotState) -> tuple[float | None, float]:
    """
    Weighted centroid of all active pads along the body axis.

    Returns (centroid, mean_intensity) where centroid is in body-axis units
    (module_id + sub-position within module) and mean_intensity is the
    average active-pad value.  Returns (None, 0.0) when no pad is above threshold.
    """
    weighted_sum = 0.0
    weight_sum = 0.0
    active_sum = 0.0
    n_active = 0
    for mod_id, sens in state.sensors.items():
        all_pads = (*sens.touch_right_pads, *sens.touch_left_pads, *sens.touch_middle_pads)
        for pad_idx, val in enumerate(all_pads):
            if val >= PAD_THRESHOLD:
                body_pos = mod_id + _PAD_BODY_SUB[pad_idx]
                weighted_sum += body_pos * val
                weight_sum += val
                active_sum += val
                n_active += 1
    if weight_sum < 1e-6:
        return None, 0.0
    return weighted_sum / weight_sum, active_sum / n_active


def _find_blobs(activations: dict[int, float]) -> list[TouchBlob]:
    """Group contiguous active modules into blobs."""
    if not activations:
        return []

    sorted_ids = sorted(activations)
    blobs: list[TouchBlob] = []
    run: list[int] = []

    for mod_id in sorted_ids:
        if activations[mod_id] >= TOUCH_THRESHOLD:
            if run and mod_id != run[-1] + 1:
                blobs.append(_make_blob(run, activations))
                run = []
            run.append(mod_id)
        elif run:
            blobs.append(_make_blob(run, activations))
            run = []

    if run:
        blobs.append(_make_blob(run, activations))

    return blobs


def _make_blob(modules: list[int], activations: dict[int, float]) -> TouchBlob:
    vals = [activations[m] for m in modules]
    total = sum(vals)
    centroid = sum(m * a for m, a in zip(modules, vals)) / total if total > 0 else float(modules[0])
    return TouchBlob(
        centroid=centroid,
        modules=list(modules),
        intensity=total / len(modules),
        width=len(modules),
    )


def _active_side(state: RobotState) -> str:
    """Return which face(s) have pads above PAD_THRESHOLD: top/left/right combinations."""
    has_right = has_left = has_top = False
    for sens in state.sensors.values():
        if any(v >= PAD_THRESHOLD for v in sens.touch_right_pads):
            has_right = True
        if any(v >= PAD_THRESHOLD for v in sens.touch_left_pads):
            has_left = True
        if any(v >= PAD_THRESHOLD for v in sens.touch_middle_pads):
            has_top = True
    parts = []
    if has_top:
        parts.append("top")
    if has_left:
        parts.append("left")
    if has_right:
        parts.append("right")
    return "-".join(parts) if parts else "none"


def any_contact(state: RobotState) -> tuple[float | None, float, str]:
    """Return (centroid, intensity, side) for current pad activity.

    centroid is None when no pad is above PAD_THRESHOLD (no contact).
    """
    centroid, intensity = _pad_centroid(state)
    side = _active_side(state) if centroid is not None else "none"
    return centroid, intensity, side


def qualifying_contact(state: RobotState) -> tuple[float | None, str]:
    """Return (centroid, side) for contact qualifying as real touch.

    Requires at least one module with ≥2 physically adjacent pads above
    PAD_THRESHOLD.  Filters single floating/noisy pads that would otherwise
    pass any_contact.  Returns (None, "none") when no qualifying contact.
    """
    q_blobs = _find_qualifying_blobs(state, PAD_THRESHOLD, TOUCH_THRESHOLD)
    if not q_blobs:
        return None, "none"
    total_w = sum(b.intensity * b.width for b in q_blobs)
    centroid = (
        sum(b.centroid * b.intensity * b.width for b in q_blobs) / total_w
        if total_w > 0 else q_blobs[0].centroid
    )
    return centroid, _active_side(state)


def _linear_fit(window: list[tuple[float, float]]) -> tuple[float, float]:
    """
    Fit a line to (timestamp, centroid) pairs.

    Returns (velocity, r_squared).
    """
    n = len(window)
    ts = [w[0] for w in window]
    cs = [w[1] for w in window]

    t_mean = sum(ts) / n
    c_mean = sum(cs) / n

    ss_tt = sum((t - t_mean) ** 2 for t in ts)
    ss_ct = sum((c - c_mean) * (t - t_mean) for c, t in zip(cs, ts))

    if ss_tt < 1e-9:
        return 0.0, 0.0

    slope = ss_ct / ss_tt

    ss_cc = sum((c - c_mean) ** 2 for c in cs)
    if ss_cc < 1e-9:
        r_squared = 1.0
    else:
        residuals = sum((c - (c_mean + slope * (t - t_mean))) ** 2 for c, t in zip(cs, ts))
        r_squared = max(0.0, 1.0 - residuals / ss_cc)

    return slope, r_squared
