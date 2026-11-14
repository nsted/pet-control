"""Go / no-go logic of the Regimen v2 long hug run (``scripts/rl/hug_long_driver.py``, stage ``hug_long_01``, L = 8).

Pure functions over training-episode rows (``episodes.jsonl``) and a callback that applies the two in-run checks; the wall-budget planning and the episode reader are those of
``petctl.rl.curriculum.m2v6``, the reach / hold signals those of ``petctl.rl.curriculum.m2v7``.
"""

from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Any

import numpy as np
from stable_baselines3.common.callbacks import BaseCallback

from petctl.rl.curriculum import m2v7
from petctl.rl.target_shapes import GATE_SEED0, PER_SHAPE, held_out_names

BUDGET_MIN = 45.0
CAP_STEPS = 5_000_000
CHECK1_MIN, CHECK2_MIN = 20.0, 40.0            # wall minutes of training
CHECK1_FRAC_HOLD = 0.20                          # of the episodes reaching run >= 3 and span >= 90 deg: fraction keeping the state >= 2 s
CHECK1_H_WRAP = 0.75                             # mean h over wrap ticks
CHECK1_WINDOW = 300
CHECK2_WINDOW = 200
CHECK2_MIN_SUCCESS = 0.10                        # rolling training success (gate definition: held 3 s, 90 % duty)
SCREEN_N = 30
SCREEN_DONE, SCREEN_CONTINUE = 0.55, 0.30
LR = 1.5e-4


def check1(rows: list[dict[str, Any]], window: int = CHECK1_WINDOW) -> tuple[bool, dict[str, Any]]:
    """20-min check: of the last ``window`` episodes that reached run >= 3 and span >= 90 deg, ``frac_hold`` >= 0.20 AND the tick-weighted mean h over wrap ticks >= 0.75 (no reaching episode = fail)."""
    sig = m2v7.p2_signals(rows, window)
    ok = sig["n_reach"] > 0 and sig["frac_hold"] >= CHECK1_FRAC_HOLD and sig["h_wrap"] is not None and sig["h_wrap"] >= CHECK1_H_WRAP
    return bool(ok), sig


def check2(rows: list[dict[str, Any]], window: int = CHECK2_WINDOW) -> tuple[bool, float, int]:
    """40-min check: rolling training success over the last ``window`` episodes >= 0.10; returns ``(ok, rate, n_episodes_used)``."""
    last = rows[-window:]
    rate = float(np.mean([float(r.get("success", 0.0)) for r in last])) if last else 0.0
    return rate >= CHECK2_MIN_SUCCESS, rate, len(last)


def screen_decision(success: float) -> str:
    """Screening verdict: ``final`` (>= 0.55), ``continue`` (0.30 <= s < 0.55: CONTINUE_ONE_MORE_HOUR), ``stop`` (< 0.30)."""
    return "final" if success >= SCREEN_DONE else "continue" if success >= SCREEN_CONTINUE else "stop"


def balanced_screen_seeds(n: int = SCREEN_N, n_shapes: int = 5) -> list[int]:
    """Gate seeds of a screen with ``n // n_shapes`` episodes per held-out shape (seed ``GATE_SEED0 + 20 i + j``, j = 0..per-1; n = 30: 6 per shape)."""
    per = max(1, n // n_shapes)
    return [GATE_SEED0 + PER_SHAPE * i + j for i in range(n_shapes) for j in range(per)]


def per_shape_success(rows: list[dict[str, Any]], n_modules: int) -> dict[str, float]:
    """Success rate per held-out shape of the length's set from screen rows (``{seed, metrics}``)."""
    names, out = held_out_names(n_modules), {}
    for i, nm in enumerate(names):
        r = [x["metrics"].get("success", 0.0) for x in rows if (int(x["seed"]) - GATE_SEED0) // PER_SHAPE == i]
        if r:
            out[nm] = float(np.mean(r))
    return out


def read_rows_tolerant(f: Path) -> list[dict[str, Any]]:
    """Lines of an episode log that is still being written (an incomplete last line is skipped)."""
    rows: list[dict[str, Any]] = []
    try:
        text = f.read_text()
    except OSError:
        return rows
    for line in text.splitlines():
        try:
            rows.append(json.loads(line))
        except ValueError:
            continue
    return rows


class LongRunChecks(BaseCallback):
    """In-run go / no-go of the long hug run: ``check1`` once at ``t1_s`` of training wall time, ``check2`` once at ``t2_s``. A failed check stops training (``reason`` set,
    e.g. ``hold_20min`` / ``success_40min``); every check is recorded in ``results``. ``check2_at_end`` applies the 40-min rule to a run that ended (by its step target) before 40 min."""

    def __init__(self, out: Path, t1_s: float = CHECK1_MIN * 60.0, t2_s: float = CHECK2_MIN * 60.0) -> None:
        """``out``: the run directory (reads its ``episodes.jsonl``); ``t1_s`` / ``t2_s``: wall seconds into training."""
        super().__init__()
        self.out, self.t1_s, self.t2_s = out, t1_s, t2_s
        self.results: dict[str, Any] = {}
        self.reason: str | None = None
        self._t0 = 0.0

    def _on_training_start(self) -> None:
        self._t0 = time.time()

    def _rows(self) -> list[dict[str, Any]]:
        return read_rows_tolerant(self.out / "episodes.jsonl")

    def _steps(self, steps: int | None) -> int:
        return int(steps) if steps is not None else int(self.num_timesteps)

    def run_check1(self, steps: int | None = None) -> bool:
        """Apply check 1 to the episode log now; records ``results['check1']``."""
        ok, sig = check1(self._rows())
        self.results["check1"] = {"ok": ok, "timesteps": self._steps(steps), "wall_min": (time.time() - self._t0) / 60.0, **sig}
        print(f"[check1] 20 min: reached {sig['n_reach']}/{sig['n']}, frac_hold {sig['frac_hold']:.3f} (>= {CHECK1_FRAC_HOLD}), h_wrap {sig['h_wrap']} (>= {CHECK1_H_WRAP}): {'ok' if ok else 'FAIL'}", flush=True)
        return ok

    def run_check2(self, at_end: bool = False, steps: int | None = None) -> bool:
        """Apply check 2 to the episode log now; records ``results['check2']``."""
        ok, rate, n = check2(self._rows())
        self.results["check2"] = {"ok": ok, "rate": rate, "n": n, "timesteps": self._steps(steps), "wall_min": (time.time() - self._t0) / 60.0, "at_end": at_end}
        print(f"[check2] {'run end' if at_end else '40 min'}: rolling success {rate:.3f} over {n} episodes (>= {CHECK2_MIN_SUCCESS}): {'ok' if ok else 'FAIL'}", flush=True)
        return ok

    def _on_step(self) -> bool:
        el = time.time() - self._t0
        if "check1" not in self.results and el >= self.t1_s:
            if not self.run_check1():
                self.reason = "hold_20min"
                return False
        if "check2" not in self.results and el >= self.t2_s:
            if not self.run_check2():
                self.reason = "success_40min"
                return False
        return True
