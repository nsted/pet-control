"""
Stroke detector — tracks a hand moving along the robot body.

Computes a weighted pad-level centroid along the body axis each tick and fits a
linear velocity over a rolling window.  Working at pad resolution means even a
stroke across two neighbouring pads on the same module is detectable.

No FSR data is used; capacitive pad readings are sufficient.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field

from petctl.types import RobotState

PAD_THRESHOLD: float = 0.35     # minimum per-pad value to contribute to centroid
TOUCH_THRESHOLD: float = 0.08   # minimum touch_total for module-level blob grouping
VELOCITY_THRESHOLD: float = 0.8  # body-units/second — below this is not a stroke
WINDOW_FRAMES: int = 15          # rolling window depth (~0.75 s at 20 Hz)
MIN_WINDOW_FRAMES: int = 5       # minimum frames before fitting

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


class StrokeDetector:
    """
    Detects a stroking gesture along the robot body.

    Call update() every control tick. Returns a StrokeReading when touch is
    moving fast enough to be a stroke, or None otherwise.
    """

    def __init__(self) -> None:
        self._window: deque[tuple[float, float]] = deque(maxlen=WINDOW_FRAMES)

    def update(self, state: RobotState) -> StrokeReading | None:
        """Process one tick of RobotState. Returns StrokeReading or None."""
        centroid, intensity = _pad_centroid(state)

        if centroid is None:
            self._window.clear()
            return None

        self._window.append((state.timestamp, centroid))

        if len(self._window) < MIN_WINDOW_FRAMES:
            return None

        velocity, r_squared = _linear_fit(list(self._window))

        if abs(velocity) < VELOCITY_THRESHOLD:
            return None

        activations = {mid: s.touch_total for mid, s in state.sensors.items()}
        blobs = _find_blobs(activations)

        return StrokeReading(
            centroid=centroid,
            velocity=velocity,
            speed=abs(velocity),
            direction="head_to_tail" if velocity > 0 else "tail_to_head",
            intensity=intensity,
            confidence=r_squared,
            side=_active_side(state),
            blobs=blobs,
        )


# ------------------------------------------------------------------
# Helpers
# ------------------------------------------------------------------

def _pad_centroid(state: RobotState) -> tuple[float | None, float]:
    """
    Weighted centroid of all active pads along the body axis.

    Returns (centroid, mean_intensity) where centroid is in body-axis units
    (module_id + sub-position within module) and mean_intensity is the
    average active-pad value.  Returns (None, 0.0) when no pad is above threshold.
    """
    weighted_sum = 0.0
    total_weight = 0.0
    n_active = 0
    for mod_id, sens in state.sensors.items():
        all_pads = (*sens.touch_right_pads, *sens.touch_left_pads, *sens.touch_middle_pads)
        for pad_idx, val in enumerate(all_pads):
            if val >= PAD_THRESHOLD:
                body_pos = mod_id + _PAD_BODY_SUB[pad_idx]
                weighted_sum += body_pos * val
                total_weight += val
                n_active += 1
    if total_weight < 1e-6:
        return None, 0.0
    return weighted_sum / total_weight, total_weight / n_active


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
