"""
Stroke detector — tracks a hand moving along the robot body.

Classifies touch blobs (contiguous active modules) and fits a linear
velocity to the primary blob's centroid over a rolling time window.
No FSR data is used; capacitive touch_total per module is sufficient.
"""

from __future__ import annotations

import time
from collections import deque
from dataclasses import dataclass, field

from petctl.types import RobotState

TOUCH_THRESHOLD: float = 0.08   # minimum touch_total to count as active
VELOCITY_THRESHOLD: float = 1.5  # modules/second — below this is not a stroke
WINDOW_FRAMES: int = 15          # rolling window depth (~0.75s at 20 Hz)
MIN_WINDOW_FRAMES: int = 5       # minimum frames before fitting


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
    centroid: float      # primary blob centroid this tick
    velocity: float      # modules/second, signed (+ = head→tail, − = tail→head)
    speed: float         # abs(velocity)
    direction: str       # "head_to_tail" | "tail_to_head"
    intensity: float     # primary blob intensity
    confidence: float    # R² of the linear fit over the centroid window
    blobs: list[TouchBlob] = field(default_factory=list)


class StrokeDetector:
    """
    Detects a stroking gesture along the robot body.

    Call update() every control tick. Returns a StrokeReading when the
    primary touch blob is moving fast enough to be a stroke, or None otherwise.
    """

    def __init__(self) -> None:
        # Rolling buffer of (timestamp, centroid) for the primary blob
        self._window: deque[tuple[float, float]] = deque(maxlen=WINDOW_FRAMES)

    def update(self, state: RobotState) -> StrokeReading | None:
        """Process one tick of RobotState. Returns StrokeReading or None."""
        activations = {
            mod_id: sens.touch_total
            for mod_id, sens in state.sensors.items()
        }

        blobs = _find_blobs(activations)

        if not blobs:
            self._window.clear()
            return None

        primary = max(blobs, key=lambda b: b.intensity)
        self._window.append((state.timestamp, primary.centroid))

        if len(self._window) < MIN_WINDOW_FRAMES:
            return None

        velocity, r_squared = _linear_fit(list(self._window))

        if abs(velocity) < VELOCITY_THRESHOLD:
            return None

        return StrokeReading(
            centroid=primary.centroid,
            velocity=velocity,
            speed=abs(velocity),
            direction="head_to_tail" if velocity > 0 else "tail_to_head",
            intensity=primary.intensity,
            confidence=r_squared,
            blobs=blobs,
        )


# ------------------------------------------------------------------
# Helpers
# ------------------------------------------------------------------

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


def _linear_fit(window: list[tuple[float, float]]) -> tuple[float, float]:
    """
    Fit a line to (timestamp, centroid) pairs.

    Returns (velocity, r_squared).
    velocity is in modules/second; r_squared is the coefficient of determination.
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
