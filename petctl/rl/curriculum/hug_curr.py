"""Checks of the hug success-curriculum run (``scripts/rl/hug_curr_driver.py``, stage ``hug_curr_01``): pure functions over training-episode rows and the in-run callback.

20-min check: training success at L1 (``success_l1``, last 200 episodes; a run that already advanced past L1 passes this part) >= 0.15 AND h_wrap (tick-weighted mean h over L1 wrap ticks of the
episodes that reached run >= 3 and span >= 90 deg, last 300 episodes) >= 0.70. 40-min check (or at the run end if the run is shorter): level 2 reached. Every check also records the
per-shape breakdown (cylinders / boxes / rounded boxes: reach rate and level success). The end screen (n = 30) and the final gate always score L3 (the gate params carry no curriculum).
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
from petctl.rl.hug_levels import read_level

CHECK1_MIN, CHECK2_MIN = 20.0, 40.0
CHECK1_SUCCESS_L1 = 0.15
CHECK1_H_WRAP = 0.70
CHECK1_SUCCESS_WINDOW, CHECK1_WINDOW = 200, 300
LEVEL2 = 2
SHAPE_NAMES = ("cylinders", "boxes", "rounded")


def per_shape_breakdown(rows: list[dict[str, Any]], window: int = CHECK1_WINDOW) -> dict[str, dict[str, float]]:
    """Per shape family (cylinders / boxes / rounded boxes) over the last ``window`` rows: ``n``, ``wrap90_rate`` (run >= 3 and span >= 90 deg; alias ``reach_rate``, the old mislabelled name), ``contact_rate`` (any module touched the object) and ``level_success`` (the row's own-level success)."""
    out: dict[str, dict[str, float]] = {}
    last = rows[-window:]
    for i, name in enumerate(SHAPE_NAMES):
        r = [x for x in last if int(x.get("shape_kind", -1)) == i]
        w90 = float(np.mean([m2v7.reached_state(x) for x in r])) if r else 0.0
        out[name] = {"n": float(len(r)), "wrap90_rate": w90, "reach_rate": w90,        # ``reach_rate`` = old key (alias): run >= 3 AND span >= 90 deg, NOT contact
                     "contact_rate": float(np.mean([float(x.get("contact", 0.0)) for x in r])) if r else 0.0,
                     "level_success": float(np.mean([float(x.get("success", 0.0)) for x in r])) if r else 0.0}
    return out


def h_wrap_l1(rows: list[dict[str, Any]], window: int = CHECK1_WINDOW) -> float | None:
    """Tick-weighted mean h over level-1 wrap ticks of the episodes that reached run >= 3 and span >= 90 deg in the last ``window`` rows (None without wrap ticks)."""
    reach = [r for r in rows[-window:] if m2v7.reached_state(r)]
    n = sum(r.get("h_wrap1_n", 0.0) for r in reach)
    return float(sum(r.get("h_wrap1_mean", 0.0) * r.get("h_wrap1_n", 0.0) for r in reach) / n) if n > 0 else None


def check1(rows: list[dict[str, Any]], level: int) -> tuple[bool, dict[str, Any]]:
    """20-min check (see the module doc); ``level`` = the persisted curriculum level."""
    last = rows[-CHECK1_SUCCESS_WINDOW:]
    s1 = float(np.mean([float(r.get("success_l1", 0.0)) for r in last])) if last else 0.0
    hw = h_wrap_l1(rows)
    ok = (level >= LEVEL2 or s1 >= CHECK1_SUCCESS_L1) and hw is not None and hw >= CHECK1_H_WRAP
    sig = {"n": len(last), "success_l1": s1, "level": level, "h_wrap": hw, "frac_reach": float(np.mean([m2v7.reached_state(r) for r in rows[-CHECK1_WINDOW:]])) if rows else 0.0,
           "per_shape": per_shape_breakdown(rows)}
    return bool(ok), sig


def check2(rows: list[dict[str, Any]], level: int) -> tuple[bool, dict[str, Any]]:
    """40-min check: level 2 reached."""
    return level >= LEVEL2, {"level": level, "per_shape": per_shape_breakdown(rows)}


class CurrChecks(BaseCallback):
    """In-run checks: ``check1`` once at ``t1_s`` of training wall time, ``check2`` once at ``t2_s``. A failed check stops training (``reason`` ``wrap_20min`` / ``level2_40min``); results (with the
    per-shape breakdown) are kept in ``results`` and written to ``<out>/check<k>.json``. ``run_check2(at_end=True)`` applies the 40-min rule to a run that ended early."""

    def __init__(self, out: Path, t1_s: float = CHECK1_MIN * 60.0, t2_s: float = CHECK2_MIN * 60.0) -> None:
        """``out``: the run directory (episode log and ``hug_level.json``)."""
        super().__init__()
        self.out, self.t1_s, self.t2_s = out, t1_s, t2_s
        self.results: dict[str, Any] = {}
        self.reason: str | None = None
        self._t0 = 0.0

    def _on_training_start(self) -> None:
        self._t0 = time.time()

    def _rows(self) -> list[dict[str, Any]]:
        return read_rows_tolerant(self.out / "episodes.jsonl")

    def _record(self, key: str, ok: bool, sig: dict[str, Any], steps: int | None, extra: dict[str, Any] | None = None) -> None:
        res = {"ok": ok, "timesteps": int(steps) if steps is not None else int(self.num_timesteps), "wall_min": (time.time() - self._t0) / 60.0, **sig, **(extra or {})}
        self.results[key] = res
        (self.out / f"{key}.json").write_text(json.dumps(res, indent=1))

    def run_check1(self, steps: int | None = None) -> bool:
        """Apply check 1 now."""
        ok, sig = check1(self._rows(), read_level(self.out))
        self._record("check1", ok, sig, steps)
        print(f"[check1] 20 min: level {sig['level']}, success_l1 {sig['success_l1']:.3f} (>= {CHECK1_SUCCESS_L1}), h_wrap {sig['h_wrap']} (>= {CHECK1_H_WRAP}), per shape {json.dumps(sig['per_shape'])}: {'ok' if ok else 'FAIL'}", flush=True)
        return ok

    def run_check2(self, at_end: bool = False, steps: int | None = None) -> bool:
        """Apply check 2 now."""
        ok, sig = check2(self._rows(), read_level(self.out))
        self._record("check2", ok, sig, steps, {"at_end": at_end})
        print(f"[check2] {'run end' if at_end else '40 min'}: level {sig['level']} (>= {LEVEL2}), per shape {json.dumps(sig['per_shape'])}: {'ok' if ok else 'FAIL'}", flush=True)
        return ok

    def _on_step(self) -> bool:
        el = time.time() - self._t0
        if "check1" not in self.results and el >= self.t1_s and not self.run_check1():
            self.reason = "wrap_20min"
            return False
        if "check2" not in self.results and el >= self.t2_s and not self.run_check2():
            self.reason = "level2_40min"
            return False
        return True
