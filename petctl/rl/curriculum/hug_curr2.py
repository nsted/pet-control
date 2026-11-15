"""Checks of the per-family hug curriculum run (``scripts/rl/hug_curr2_driver.py``, stage ``hug_curr_02``): pure functions over training-episode rows plus the in-run callback.

Families: cylinders (``shape_kind`` 0), boxes (1), rounded boxes (2); ``success`` of a row is its success at the level its family had when the episode started. Rules (trend based, no fixed
threshold on a level the policy has not reached):
- 20-min stop ONLY if ALL hold: curved-family (cyl + rbox) success over its last 150 episodes < 0.03 AND it has not risen vs the 150 before AND the cylinder span p75 (last 150 cylinder episodes) is not
  >= 5 deg above ``SPAN_P75_BASE`` (106 deg, the hug_curr_01 last-300 value). Fewer than 150 curved episodes in the window: no stop.
- Safety stop (every 5 min from 10 min): box success (success at L1, level-independent) over the last 100 box episodes dropped by more than 0.2 from the first 100 box episodes of the run
  (needs 200 box episodes).
- End: screen success >= 0.55 -> final verdict; else any family advanced a level OR curved success rising (last 150 > previous 150 + 0.01) -> CONTINUE_ONE_MORE_HOUR; else STOP_design_change.
Every check records per family: n, level, rolling success, wrap-90 rate, contact rate, span p50/p75, bend (total joint bend inside the run at the peak-span tick) p50/p75.
"""

from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Any

import numpy as np
from stable_baselines3.common.callbacks import BaseCallback

from petctl.rl.curriculum import m2v7
from petctl.rl.curriculum.hug_long import read_rows_tolerant
from petctl.rl.hug_levels import FAMILIES, family_level_name, read_family_levels

CHECK1_MIN = 20.0
SAFETY_FIRST_MIN, SAFETY_EVERY_MIN = 10.0, 5.0
CURVED = ("cyl", "rbox")
CURVED_MIN_SUCCESS = 0.03
CURVED_WINDOW = 150
SPAN_P75_BASE, SPAN_P75_RISE = 106.0, 5.0
BOX_WINDOW, BOX_DROP = 100, 0.2
RISING_EPS = 0.01
SCREEN_DONE = 0.55


def family_rows(rows: list[dict[str, Any]], families: tuple[str, ...]) -> list[dict[str, Any]]:
    """Rows (chronological) of the given shape families."""
    idx = {FAMILIES.index(f) for f in families}
    return [r for r in rows if int(r.get("shape_kind", -1)) in idx]


def _pct(vals: list[float], q: float) -> float | None:
    return float(np.percentile(vals, q)) if vals else None


def family_table(rows: list[dict[str, Any]], levels: dict[str, int], window: int = 100) -> dict[str, dict[str, Any]]:
    """Per family over its last ``window`` episodes: ``n``, ``level`` (name), ``success`` (own-level), ``wrap90_rate``, ``contact_rate``, span p50 / p75, bend p50 / p75."""
    out: dict[str, dict[str, Any]] = {}
    for f in FAMILIES:
        r = family_rows(rows, (f,))[-window:]
        span = [float(x.get("normal_span_deg", 0.0)) for x in r]
        bend = [float(x["bend_run_deg"]) for x in r if "bend_run_deg" in x]
        out[f] = {"n": len(r), "level": family_level_name(f, levels.get(f, 0)), "success": float(np.mean([float(x.get("success", 0.0)) for x in r])) if r else 0.0,
                  "wrap90_rate": float(np.mean([m2v7.reached_state(x) for x in r])) if r else 0.0, "contact_rate": float(np.mean([float(x.get("contact", 0.0)) for x in r])) if r else 0.0,
                  "span_p50": _pct(span, 50), "span_p75": _pct(span, 75), "bend_p50": _pct(bend, 50), "bend_p75": _pct(bend, 75)}
    return out


def curved_success(rows: list[dict[str, Any]], window: int = CURVED_WINDOW) -> tuple[float | None, float | None]:
    """Success of the last ``window`` curved (cyl + rbox) episodes and of the ``window`` before them (None where fewer than ``window`` episodes exist)."""
    c = family_rows(rows, CURVED)
    f = lambda r: float(np.mean([float(x.get("success", 0.0)) for x in r])) if len(r) >= window else None  # noqa: E731
    return f(c[-window:]), f(c[-2 * window:-window])


def cyl_span_p75(rows: list[dict[str, Any]], window: int = CURVED_WINDOW) -> float | None:
    """p75 of the peak normal span of the last ``window`` cylinder episodes (None without cylinder episodes)."""
    return _pct([float(x.get("normal_span_deg", 0.0)) for x in family_rows(rows, ("cyl",))[-window:]], 75)


def curved_stop(rows: list[dict[str, Any]]) -> tuple[bool, dict[str, Any]]:
    """The 20-min rule: returns ``(stop, signals)``; stop only if ALL of: curved success < 0.03, not risen vs the previous window, cylinder span p75 < ``SPAN_P75_BASE`` + ``SPAN_P75_RISE``."""
    last, prev = curved_success(rows)
    p75 = cyl_span_p75(rows)
    low = last is not None and last < CURVED_MIN_SUCCESS
    not_risen = last is not None and (prev is None or last <= prev)
    no_span_progress = p75 is None or p75 < SPAN_P75_BASE + SPAN_P75_RISE
    return bool(low and not_risen and no_span_progress), {"curved_last150": last, "curved_prev150": prev, "cyl_span_p75": p75, "low": low, "not_risen": not_risen, "no_span_progress": no_span_progress}


def box_baseline(rows: list[dict[str, Any]]) -> float | None:
    """Level-independent box success (``success_l1``) of the first ``BOX_WINDOW`` box episodes of the run (None until they exist)."""
    b = family_rows(rows, ("box",))
    return float(np.mean([float(x.get("success_l1", 0.0)) for x in b[:BOX_WINDOW]])) if len(b) >= BOX_WINDOW else None


def box_safety_stop(rows: list[dict[str, Any]]) -> tuple[bool, dict[str, Any]]:
    """Safety rule: last ``BOX_WINDOW`` box episodes' ``success_l1`` below the run-start baseline by more than ``BOX_DROP`` (needs 2 x ``BOX_WINDOW`` box episodes)."""
    b = family_rows(rows, ("box",))
    if len(b) < 2 * BOX_WINDOW:
        return False, {"box_baseline": None, "box_last": None, "n_box": len(b)}
    base, last = box_baseline(rows), float(np.mean([float(x.get("success_l1", 0.0)) for x in b[-BOX_WINDOW:]]))
    assert base is not None
    return bool(last < base - BOX_DROP - 1e-12), {"box_baseline": base, "box_last": last, "n_box": len(b)}


def end_decision(screen_success: float, any_advanced: bool, rows: list[dict[str, Any]]) -> str:
    """End rule: ``final`` (screen >= 0.55) | ``continue`` (a family advanced, or curved success rising) | ``stop`` (design change)."""
    if screen_success >= SCREEN_DONE:
        return "final"
    last, prev = curved_success(rows)
    rising = last is not None and prev is not None and last > prev + RISING_EPS
    return "continue" if any_advanced or rising else "stop"


class Curr2Checks(BaseCallback):
    """In-run checks: the box safety rule every ``SAFETY_EVERY_MIN`` from ``SAFETY_FIRST_MIN``, the curved trend rule once at ``t1_s``. A failed rule stops training (``reason``
    ``curved_trend_20min`` / ``box_safety``); every check records the per-family table (``results`` and ``<out>/checks.jsonl``)."""

    def __init__(self, out: Path, t1_s: float = CHECK1_MIN * 60.0, scale: float = 1.0) -> None:
        """``out``: the run directory (episode log and ``hug_level.json``); ``scale`` shrinks the safety schedule (smoke tests)."""
        super().__init__()
        self.out, self.t1_s = out, t1_s
        self.safety_first, self.safety_every = SAFETY_FIRST_MIN * 60.0 * scale, SAFETY_EVERY_MIN * 60.0 * scale
        self.results: dict[str, Any] = {}
        self.reason: str | None = None
        self._t0 = 0.0
        self._next_safety = self.safety_first

    def _on_training_start(self) -> None:
        self._t0 = time.time()

    def _rows(self) -> list[dict[str, Any]]:
        return read_rows_tolerant(self.out / "episodes.jsonl")

    def _record(self, key: str, res: dict[str, Any]) -> None:
        res = {"timesteps": int(self.num_timesteps), "wall_min": (time.time() - self._t0) / 60.0, **res}
        self.results[key] = res
        with open(self.out / "checks.jsonl", "a") as f:
            f.write(json.dumps({"check": key, **res}) + "\n")
        if key == "check1":
            (self.out / "check1.json").write_text(json.dumps(res, indent=1))

    def run_safety(self) -> bool:
        """Apply the box safety rule now; True = ok."""
        rows = self._rows()
        stop, sig = box_safety_stop(rows)
        self._record("safety", {"ok": not stop, **sig, "table": family_table(rows, read_family_levels(self.out))})
        print(f"[safety] box success_l1 {sig['box_last']} vs baseline {sig['box_baseline']} (n_box {sig['n_box']}): {'FAIL' if stop else 'ok'}", flush=True)
        return not stop

    def run_check1(self) -> bool:
        """Apply the 20-min curved trend rule now; True = ok (no stop)."""
        rows = self._rows()
        stop, sig = curved_stop(rows)
        self._record("check1", {"ok": not stop, **sig, "levels": read_family_levels(self.out), "table": family_table(rows, read_family_levels(self.out))})
        print(f"[check1] 20 min: {json.dumps(sig)}: {'STOP' if stop else 'ok'}; table {json.dumps(self.results['check1']['table'])}", flush=True)
        return not stop

    def _on_step(self) -> bool:
        el = time.time() - self._t0
        if el >= self._next_safety:
            self._next_safety += self.safety_every
            if not self.run_safety():
                self.reason = "box_safety"
                return False
        if "check1" not in self.results and el >= self.t1_s and not self.run_check1():
            self.reason = "curved_trend_20min"
            return False
        return True
