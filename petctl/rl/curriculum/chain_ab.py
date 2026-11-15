"""Helpers of the A1 -> A2 -> B1 -> B2 navigation-chain driver (``scripts/rl/chain_ab_driver.py``, plan sec 4.2): gate specs, the lateral-displacement second measurement, the
go / continue / stop decision and the training-DR ramp. Pure functions where possible so they can be tested on synthetic summaries.

Frozen gates (all at n = 100, dr 0.6, Wilson 95% lower bound vs the threshold, plus the bus-current and glitch hard fails of ``gate2``):
  A1 flat_nav      success >= 0.80 (median t_goal reported)
  A2 flat_omni     success >= 0.70 AND median ``lateral_peak_m`` >= 0.4 m over 100 episodes of +-90 deg goals with an 8 s limit (``lateral_gate``)
  B1 terrain_easy  success >= 0.75 at terrain level 0.6
  B2 terrain_full  success >= 0.70 at terrain level 1.0 AND imu_off paired drop (bootstrap CI lower bound) >= 0.15
Lateral metric: peak, over the episode, of the centre-of-mass displacement from its start projected on the axis perpendicular to the INITIAL body heading, signed towards the goal's side.
It is a displacement in the initial lateral axis, so a turn-then-walk policy that is fast enough also scores; it is the measurement of the plan ("lateral displacement >= 0.4 m in 8 s").
"""

from __future__ import annotations

from typing import Any, Callable

import numpy as np
from stable_baselines3.common.callbacks import BaseCallback

from petctl.rl.curriculum.gate2 import Ablation, Gate2Spec
from petctl.rl.curriculum.stages import CHAIN_STAGES, get_stage

LATERAL_MIN_M = 0.4
LATERAL_EPISODE_S = 8.0
LATERAL_N = 100
LATERAL_PARAMS = {"bearing_deg": 90.0, "bearing_min_deg": 90.0, "goal_dist_min": 0.8, "goal_dist_max": 0.8}
IMU_OFF_MIN_DROP = 0.15
DR_START, DR_END, DR_RAMP_FRAC = 0.3, 0.6, 0.5
BUDGET_MIN = 45.0
CAP_STEPS = 4_000_000
NEXT_STAGE: dict[str, str | None] = {"flat_nav": "flat_omni", "flat_omni": "terrain_easy", "terrain_easy": "terrain_full", "terrain_full": None}


def stage_chain() -> tuple[str, ...]:
    """The stage names in order."""
    return CHAIN_STAGES


def gate_spec(stage_name: str, n: int = 100) -> Gate2Spec:
    """The gate2 spec of a chain stage: ``n`` held-out seeds at dr 0.6, no nominal condition, the stage's frozen threshold; B2 adds the paired ``imu_off`` ablation (drop CI lower bound >= 0.15)."""
    if stage_name not in CHAIN_STAGES:
        raise KeyError(f"not a chain stage: {stage_name}")
    abl = (Ablation("imu_off", {"imu_off": 1.0}, min_drop=IMU_OFF_MIN_DROP),) if stage_name == "terrain_full" else ()
    return Gate2Spec(n=n, dr_scale=0.6, nominal=False, ablations=abl)


def lateral_params(stage_name: str = "flat_omni") -> dict[str, float]:
    """Task params of the lateral measurement: the stage's gate distribution with goals at exactly +-90 deg, 0.8 m away."""
    return {**get_stage(stage_name).gate.task_params, **LATERAL_PARAMS}


def lateral_summary(rows: list[dict[str, Any]], minimum: float = LATERAL_MIN_M) -> dict[str, Any]:
    """Median / quartiles of ``lateral_peak_m`` over evaluator rows and the pass flag (median >= ``minimum``)."""
    v = np.array([float(r["metrics"].get("lateral_peak_m", 0.0)) for r in rows])
    if v.size == 0:
        return {"n": 0, "median_m": float("nan"), "passed": False, "min_m": minimum}
    q1, med, q3 = np.percentile(v, [25, 50, 75])
    return {"n": int(v.size), "median_m": float(med), "q1_m": float(q1), "q3_m": float(q3), "frac_ge_min": float(np.mean(v >= minimum)),
            "success_rate": float(np.mean([float(r["metrics"].get("success", 0.0)) > 0.5 for r in rows])), "min_m": minimum, "passed": bool(med >= minimum)}


def lateral_gate(evaluator: Callable[..., list[dict[str, Any]]], n: int = LATERAL_N, seed0: int = 10_000) -> dict[str, Any]:
    """Run the lateral second measurement with ``evaluator`` (built on ``lateral_params`` / 8 s / dr 0.6) on seeds ``seed0 + i``."""
    return lateral_summary(evaluator({}, [seed0 + i for i in range(n)]))


def apply_second_measurement(res: dict[str, Any], stage_name: str, lateral: dict[str, Any] | None) -> dict[str, Any]:
    """Fold the lateral measurement into a gate2 result (A2 only): adds ``res['lateral']``, a fail reason and updates ``passed`` / ``status``."""
    if stage_name != "flat_omni" or lateral is None:
        return res
    res["lateral"] = lateral
    if not lateral["passed"]:
        res["fail_reasons"] = [*res["fail_reasons"], f"lateral displacement median {lateral['median_m']:.3f} m < {lateral['min_m']:.2f} m in {LATERAL_EPISODE_S:g} s (+-90 deg goals)"]
    res["passed"] = not res["fail_reasons"]
    if not res.get("void"):
        res["status"] = "PASS" if res["passed"] else "FAIL"
    return res


# ---------------------------------------------------------------- chain logic
def next_action(lines: list[dict[str, Any]]) -> dict[str, Any]:
    """Decide the next step from the recorded ``gate`` lines (each ``{phase: 'gate', stage, attempt, passed}``).

    Returns ``{"action": "done"}`` when the last stage passed; ``{"action": "stop", "stage"}`` when a stage failed twice; else ``{"action": "train", "stage", "attempt"}`` (attempt 1 = from the
    previous stage's passing run, attempt 2 = the identical continuation of a failed first attempt) or ``{"action": "gate", "stage", "attempt"}`` for a trained run that has no gate line yet
    (``trained`` lines: ``{phase: 'train', stage, attempt}``)."""
    gates = {(d["stage"], int(d["attempt"])): d for d in lines if d.get("phase") == "gate"}
    trained = {(d["stage"], int(d["attempt"])) for d in lines if d.get("phase") == "train"}
    for st in CHAIN_STAGES:
        g1, g2 = gates.get((st, 1)), gates.get((st, 2))
        if g1 is not None and g1["passed"] or g2 is not None and g2["passed"]:
            continue
        for attempt, g in ((1, g1), (2, g2)):
            if g is not None:
                continue
            if (st, attempt) not in trained:
                return {"action": "train", "stage": st, "attempt": attempt}
            return {"action": "gate", "stage": st, "attempt": attempt}
        return {"action": "stop", "stage": st}
    return {"action": "done"}


def passing_attempt(lines: list[dict[str, Any]], stage: str) -> int | None:
    """The attempt number whose gate passed for ``stage`` (first passing), or None."""
    for a in (1, 2):
        if any(d.get("phase") == "gate" and d["stage"] == stage and int(d["attempt"]) == a and d["passed"] for d in lines):
            return a
    return None


# ---------------------------------------------------------------- training DR ramp
def dr_at(steps: int, target: int | None, start: float = DR_START, end: float = DR_END, frac: float = DR_RAMP_FRAC) -> float:
    """Training DR scale at ``steps``: ``start`` until the step target is known, then a linear ramp to ``end`` over the first ``frac`` of the target."""
    if not target or target <= 0:
        return start
    return float(start + (end - start) * min(1.0, max(0.0, steps / (frac * target))))


class DrRampCallback(BaseCallback):
    """Broadcasts the ramped DR scale (``dr_at`` against the budget callback's step target) to every env at the end of each rollout."""

    def __init__(self, budget: Any, start: float = DR_START, end: float = DR_END, frac: float = DR_RAMP_FRAC) -> None:
        """``budget``: the run's ``BudgetCallback`` (its ``target`` is set after the throughput measurement)."""
        super().__init__()
        self.budget, self.start, self.end, self.frac = budget, start, end, frac
        self.last = start

    def _on_step(self) -> bool:
        return True

    def _on_rollout_end(self) -> None:
        self.last = dr_at(self.num_timesteps, self.budget.target, self.start, self.end, self.frac)
        self.training_env.env_method("set_dr_scale", self.last)
