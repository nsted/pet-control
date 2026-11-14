"""Helpers of the M2 v6 one-hour-run driver (``scripts/rl/m2v6_driver.py``): wall-budget step planning, wall-clock callbacks, go / no-go decisions and the screening gate.

Owner rule: every run has a wall budget INCLUDING its gate. The step count is set from the throughput measured over the first two minutes: ``steps = 0.75 x budget x steps/s`` (capped);
if the projection would exceed the budget the run is cut (steps / episode length), never the evidence (seeds, held-out n, ablations, captions). ``BudgetCallback`` enforces both.
"""

from __future__ import annotations

import json
import time
from collections import deque
from pathlib import Path
from typing import Any

import numpy as np
from stable_baselines3.common.callbacks import BaseCallback

MEASURE_S = 120.0                  # throughput is measured over the first two minutes
BUDGET_FACTOR = 0.75               # steps = factor x budget x steps/s
MAX_STEPS = 2_500_000
TAKEOFF_WINDOW = 200
TAKEOFF_MIN_RATE = 0.10
FLAT_SCREEN_GO = 0.55
HUG_RUN1_GO_DUTY = 0.20
HUG_RUN2_GO = 0.55
HUG_PILOT_MIN_FRAC = 0.05
HUG_PILOT_RUN, HUG_PILOT_SPAN = 3, 90.0
HUG_PILOT_WINDOW = 300


def plan_steps(sps: float, budget_s: float, cap: int = MAX_STEPS, factor: float = BUDGET_FACTOR) -> int:
    """Training steps for a wall budget ``budget_s`` at measured throughput ``sps``: ``factor x budget x sps`` capped at ``cap`` (never below 1)."""
    return max(1, int(min(cap, factor * budget_s * sps)))


def projected_wall_s(steps: int, sps: float) -> float:
    """Wall time to run ``steps`` at ``sps`` steps/s."""
    return steps / max(sps, 1e-9)


def best_n_envs(bench: dict[str, dict[str, float]]) -> int:
    """The n_envs that maximises the slowest config's throughput (the driver uses one n_envs for flat and hug)."""
    ns = sorted({int(n) for d in bench.values() for n in d})
    return max(ns, key=lambda n: min(d.get(str(n), 0.0) for d in bench.values()))


def takeoff_ok(successes: list[float] | deque[float], min_rate: float = TAKEOFF_MIN_RATE, window: int = TAKEOFF_WINDOW) -> tuple[bool, float]:
    """``(ok, rate)``: rolling training success over the last ``window`` episodes (all of them if fewer; none = 0) against ``min_rate``."""
    last = list(successes)[-window:]
    rate = float(np.mean(last)) if last else 0.0
    return rate >= min_rate, rate


def hug_pilot_fraction(rows: list[dict[str, Any]], window: int = HUG_PILOT_WINDOW) -> float:
    """Fraction of the last ``window`` training episodes whose peak run was >= 3 modules and peak normal span >= 90 deg."""
    last = rows[-window:]
    if not last:
        return 0.0
    return float(np.mean([r.get("hug_modules", 0.0) >= HUG_PILOT_RUN and r.get("normal_span_deg", 0.0) >= HUG_PILOT_SPAN for r in last]))


def hug_pilot_go(frac: float) -> bool:
    """Hug pilot go rule: at least 5 % of the training episodes reach run >= 3 and normal span >= 90 deg."""
    return frac >= HUG_PILOT_MIN_FRAC


def screen_go(phase: str, summary: dict[str, float]) -> bool:
    """Go / no-go of a screening gate (n = 30, dr 0.6): flat main success >= 0.55; hug run 1 mean hold_duty > 0.2; hug run 2 success >= 0.55."""
    if phase == "flat_main":
        return summary["success"] >= FLAT_SCREEN_GO
    if phase == "hug_run1":
        return summary["hold_duty_mean"] > HUG_RUN1_GO_DUTY
    if phase == "hug_run2":
        return summary["success"] >= HUG_RUN2_GO
    raise ValueError(f"no screening rule for phase {phase}")


def screen_seeds(n: int, flat: bool) -> list[int]:
    """Held-out-style seeds of a screen: ``10000 + i`` for flat; for hug spread over the whole 100-seed held-out set (``10000 + 3 i``) so every shape appears."""
    return [10_000 + (i if flat else 3 * i) for i in range(n)]


def summarise_rows(rows: list[dict[str, Any]]) -> dict[str, float]:
    """Screening summary of evaluator rows: success rate, mean hold_duty / normal span / hug run, mean return."""
    m = [r["metrics"] for r in rows]
    mean = lambda k: float(np.mean([x.get(k, 0.0) for x in m])) if m else 0.0  # noqa: E731
    return {"n": float(len(rows)), "success": mean("success"), "hold_duty_mean": mean("hold_duty"), "normal_span_mean": mean("normal_span_deg"), "hug_modules_mean": mean("hug_modules"),
            "return_mean": float(np.mean([r["ret"] for r in rows])) if rows else 0.0, "glitch": mean("glitch")}


def best_episode_record(stage_name: str, rows: list[dict[str, Any]], params: dict[str, float], dr_scale: float, episode_s: float | None) -> dict[str, Any]:
    """A ``gate2_<stage>.json``-shaped record (best successful episode by return, per-seed list) so ``contact_sheet`` / ``render_best`` and the watcher can use a screen."""
    wins = [r for r in rows if r["metrics"].get("success", 0.0) > 0.5]
    best = max(wins, key=lambda r: r["ret"]) if wins else None
    return {"stage": stage_name, "screen": True, "dr_scale": dr_scale, "episode_s": episode_s, "eval_task_params": params,
            "best_episode": None if best is None else {"seed": best["seed"], "return": best["ret"], "dr_scale": dr_scale},
            "per_seed": [{"seed": r["seed"], "success": float(r["metrics"].get("success", 0.0)), "return": r["ret"], "steps": r["steps"]} for r in rows]}


class BudgetCallback(BaseCallback):
    """Wall-budget enforcement. After ``measure_s`` of training the throughput is measured and the step target set by ``plan_steps`` (stored in ``info``); the learning-rate schedule is
    re-based to the new target; training stops at the target or when ``budget_s`` of wall time has passed, whichever comes first."""

    def __init__(self, budget_s: float, cap: int, measure_s: float = MEASURE_S, factor: float = BUDGET_FACTOR) -> None:
        """``budget_s``: training wall budget; ``cap``: step cap (also the nominal ``total_timesteps`` given to ``train``)."""
        super().__init__()
        self.budget_s, self.cap, self.measure_s, self.factor = budget_s, cap, measure_s, factor
        self.target: int | None = None
        self.info: dict[str, Any] = {"budget_s": budget_s, "cap": cap}
        self.stopped_at: int | None = None
        self._t0 = 0.0

    def _on_training_start(self) -> None:
        self._t0 = time.time()

    def _on_step(self) -> bool:
        el = time.time() - self._t0
        if self.target is None and el >= self.measure_s:
            sps = self.num_timesteps / max(el, 1e-9)
            self.target = plan_steps(sps, self.budget_s, self.cap, self.factor)
            if self.target < self.num_timesteps:
                self.target = self.num_timesteps
            self.info.update({"measure_s": el, "sps_measured": sps, "target_steps": self.target, "projected_wall_s": projected_wall_s(self.target, sps)})
            old, ratio = self.model.lr_schedule, self.cap / max(self.target, 1)
            self.model.lr_schedule = lambda f, old=old, ratio=ratio: old(max(0.0, 1.0 - (1.0 - f) * ratio))      # progress was measured against ``cap``; re-base it to the real target
            print(f"[budget] {self.num_timesteps} steps in {el:.0f} s = {sps:.0f} steps/s -> target {self.target} steps (projected {self.info['projected_wall_s'] / 60:.1f} min of {self.budget_s / 60:.0f})", flush=True)
        stop = (self.target is not None and self.num_timesteps >= self.target) or el >= self.budget_s
        if stop:
            self.stopped_at = int(self.num_timesteps)
            self.info.update({"stopped_steps": self.stopped_at, "stopped_wall_s": el, "stopped_by": "wall" if el >= self.budget_s else "target"})
            return False
        return True


class WallTakeoffCallback(BaseCallback):
    """Flat pilot takeoff rule: once, at ``check_s`` of wall time, stop if the rolling training success (last 200 episodes) is below 0.10."""

    def __init__(self, check_s: float, window: int = TAKEOFF_WINDOW, min_rate: float = TAKEOFF_MIN_RATE) -> None:
        """``check_s``: wall seconds into the run at which the rule is applied (6 min of the 12-min pilot)."""
        super().__init__()
        self.check_s, self.window, self.min_rate = check_s, window, min_rate
        self._succ: deque[float] = deque(maxlen=window)
        self.checked_at: int | None = None
        self.rate: float | None = None
        self.failed = False
        self._t0 = 0.0

    def _on_training_start(self) -> None:
        self._t0 = time.time()

    def _on_step(self) -> bool:
        for done, info in zip(self.locals["dones"], self.locals["infos"]):
            if done and "metrics" in info:
                self._succ.append(float(info["metrics"].get("success", 0.0)))
        if self.checked_at is None and time.time() - self._t0 >= self.check_s:
            ok, self.rate = takeoff_ok(self._succ, self.min_rate, self.window)
            self.checked_at, self.failed = int(self.num_timesteps), not ok
            print(f"[takeoff] rolling success {self.rate:.3f} over {len(self._succ)} episodes at {self.checked_at} steps ({self.check_s / 60:.0f} min): {'FAIL (stop)' if self.failed else 'ok'}", flush=True)
            return not self.failed
        return True


def read_episodes(out: Path) -> list[dict[str, Any]]:
    """Every line of ``out/episodes.jsonl`` (training episodes with their metrics)."""
    f = out / "episodes.jsonl"
    return [json.loads(x) for x in f.read_text().splitlines() if x.strip()] if f.exists() else []
