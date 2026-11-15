"""Hold-break diagnostic (hug tasks, task knob ``hold_diag`` > 0, default off): why do holds that START not COMPLETE?

A hold *starts* on the first hug (wrap) tick of a contiguous run of hug ticks (while the episode's hold is not yet completed). It *completes* when the metric's 3 s duty window is satisfied
(``HugState.just_held``). It *breaks* on the first non-hug tick of a started, not completed sequence. At a break the causes are recorded (multi-label; ``none`` when no cause applies):

* ``run_below_h``  the longest contiguous run of hugging modules fell below the threshold H,
* ``span_below``   the normal span of the run fell below the level's span requirement,
* ``torque_cap``   |tau| >= 0.98 x cap on ANY in-run joint (joints between the run's modules) during the 0.5 s before the break (including the break tick),
* ``bus_over``     the bus current (per-tick mean, A) exceeded 4 A during the 0.5 s before the break.

Pure python / numpy: testable on synthetic data. Per-episode counts are logged as ``hb_*`` metrics; ``summarise`` aggregates rows into fractions.
"""

from __future__ import annotations

from collections import deque
from typing import Any

import numpy as np

CAUSES = ("run_below_h", "span_below", "torque_cap", "bus_over", "none")
TORQUE_FRAC = 0.98
BUS_LIMIT_A = 4.0
WINDOW_S = 0.5


class HoldBreakTracker:
    """Per-episode hold start / break / completion bookkeeping."""

    def __init__(self, control_hz: float) -> None:
        """``control_hz``: control ticks per second (the 0.5 s look-back window)."""
        self.win = max(1, int(round(WINDOW_S * control_hz)))
        self.reset()

    def reset(self) -> None:
        """Clear for a new episode."""
        self._hist: deque[tuple[np.ndarray, float]] = deque(maxlen=self.win)
        self._active = False
        self._joints: list[int] = []
        self.started = self.broken = self.completed = 0
        self.causes = {c: 0 for c in CAUSES}

    def update(self, *, hug_tick: bool, just_held: bool, held: bool, run: int, run_start: int, h: int, span_deg: float, span_min: float | None,
               at_cap: np.ndarray, bus_a: float, n_joints: int) -> None:
        """One control tick. ``at_cap``: (n_joints,) bool, |tau| >= 0.98 cap per joint; ``bus_a``: this tick's bus current."""
        self._hist.append((np.asarray(at_cap, bool), float(bus_a)))
        if held and not just_held:
            return
        if just_held:
            if self._active:
                self.completed += 1
            self._active = False
            return
        if hug_tick:
            if not self._active:
                self._active = True
                self.started += 1
            self._joints = [k - 1 for k in range(run_start + 1, run_start + run) if 1 <= k <= n_joints]
            return
        if not self._active:
            return
        self._active = False
        self.broken += 1
        below_h = run < h
        below_span = span_min is not None and span_deg < span_min - 1e-6
        tq = any(bool(c[self._joints].any()) for c, _ in self._hist) if self._joints else False
        bus = any(b > BUS_LIMIT_A for _, b in self._hist)
        flags = {"run_below_h": below_h, "span_below": below_span, "torque_cap": tq, "bus_over": bus}
        for k, v in flags.items():
            self.causes[k] += int(v)
        self.causes["none"] += int(not any(flags.values()))

    def metrics(self) -> dict[str, float]:
        """Episode metrics ``hb_started`` / ``hb_broken`` / ``hb_completed`` / ``hb_cause_<name>`` (counts; causes are multi-label)."""
        return {"hb_started": float(self.started), "hb_broken": float(self.broken), "hb_completed": float(self.completed), **{f"hb_cause_{k}": float(v) for k, v in self.causes.items()}}


def summarise(rows: list[dict[str, Any]]) -> dict[str, Any]:
    """Aggregate episode metric dicts (training rows or screen ``metrics``): totals of holds started / broken / completed, and the fraction of broken holds with each cause (multi-label)."""
    rows = [r for r in rows if "hb_started" in r]
    tot = {k: float(sum(r.get(f"hb_{k}", 0.0) for r in rows)) for k in ("started", "broken", "completed")}
    b = max(tot["broken"], 1.0)
    return {"episodes": len(rows), **{f"holds_{k}": int(v) for k, v in tot.items()},
            "break_fractions": {c: (float(sum(r.get(f"hb_cause_{c}", 0.0) for r in rows)) / b if tot["broken"] > 0 else None) for c in CAUSES}}
