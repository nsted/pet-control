"""Go / no-go logic of the M2 v7 pilot driver (``scripts/rl/m2v7_driver.py``): learned hold channel (action_mode ``hold``), flat control pilot P0, hold-channel flat pilot P1, hug pilot P2.

All rules read the training episode log (``episodes.jsonl`` rows) and are pure functions so they can be tested with synthetic episodes. Wall-budget planning, ``BudgetCallback`` and the
episode reader are those of ``petctl.rl.curriculum.m2v6``.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import numpy as np

P_WINDOW = 100                      # flat pilots: rolling window of episodes
P_MIN_SUCCESS = 0.15                # flat pilots: rolling success needed at the check
P_MAX_RATIO = 0.7                   # flat pilots: median final_dist / dist0 allowed at the check
P0_CHECK_STEPS = 1_500_000
P1_CHECK_STEPS = 2_000_000
P0_CAP, P1_CAP = 2_000_000, 2_000_000
P2_CAP = 2_500_000
TAKEOFF_SUCCESS = 0.10              # takeoff = first 100-episode window with success >= this
P1_TAKEOFF_FACTOR = 1.5             # P1 takeoff must come within this factor of P0's
P2_WINDOW = 300
P2_MIN_FRAC = 0.05                  # approach: fraction of the last 300 episodes with run >= 3 and normal span >= 90 deg
P2_RUN, P2_SPAN_DEG = 3, 90.0
P2_HOLD_MIN_FRAC = 0.30             # holdability: of those, fraction keeping the state for >= 2 s
P2_HOLD_S = 2.0
P2_H_WRAP_MIN = 0.7                 # holdability: mean h over wrap ticks must exceed this


def rows_upto(rows: list[dict[str, Any]], steps: int) -> list[dict[str, Any]]:
    """Episodes finished at or before ``steps`` training steps."""
    return [r for r in rows if r.get("timesteps", 0) <= steps]


def flat_signals(rows: list[dict[str, Any]], check_steps: int, window: int = P_WINDOW) -> dict[str, float]:
    """Flat pilot signals at ``check_steps``: rolling success and median ``final_dist / dist0`` over the last ``window`` episodes finished by then."""
    last = rows_upto(rows, check_steps)[-window:]
    if not last:
        return {"n": 0.0, "success": 0.0, "median_ratio": float("inf")}
    ratio = [r["final_dist"] / max(r["dist0"], 1e-9) for r in last if "final_dist" in r and "dist0" in r]
    return {"n": float(len(last)), "success": float(np.mean([r.get("success", 0.0) for r in last])),
            "median_ratio": float(np.median(ratio)) if ratio else float("inf")}


def flat_go(sig: dict[str, float]) -> bool:
    """Flat pilot signal rule: rolling success >= 0.15 AND median final_dist / dist0 <= 0.7."""
    return sig["n"] > 0 and sig["success"] >= P_MIN_SUCCESS and sig["median_ratio"] <= P_MAX_RATIO


def time_to_takeoff(rows: list[dict[str, Any]], window: int = P_WINDOW, min_success: float = TAKEOFF_SUCCESS) -> int | None:
    """Training step at which the first ``window``-episode window with success >= ``min_success`` ends; None if there is none."""
    succ = [float(r.get("success", 0.0)) for r in rows]
    if len(succ) < window:
        return None
    c = np.concatenate([[0.0], np.cumsum(succ)])
    means = (c[window:] - c[:-window]) / window
    hit = np.flatnonzero(means >= min_success)
    return int(rows[int(hit[0]) + window - 1]["timesteps"]) if hit.size else None


def p1_go(sig: dict[str, float], t_p0: int | None, t_p1: int | None) -> bool:
    """P1 (hold channel, flat): the same signals as P0 AND time-to-takeoff <= 1.5 x P0's."""
    return flat_go(sig) and t_p0 is not None and t_p1 is not None and t_p1 <= P1_TAKEOFF_FACTOR * t_p0


def reached_state(r: dict[str, Any]) -> bool:
    """An episode that reached the pilot state: peak run >= 3 modules and peak normal span >= 90 deg."""
    return r.get("hug_modules", 0.0) >= P2_RUN and r.get("normal_span_deg", 0.0) >= P2_SPAN_DEG


wrap90 = reached_state                # honest name: this is "peak run >= 3 AND peak span >= 90 deg" (a wrap-90 state), not "reaching" (every shape is touched in ~100 % of the episodes)


def p2_signals(rows: list[dict[str, Any]], window: int = P2_WINDOW) -> dict[str, Any]:
    """Hug pilot signals over the last ``window`` training episodes: ``frac_reach`` (run >= 3 and span >= 90), among those ``frac_hold`` (state kept >= 2 s) and the tick-weighted ``h_wrap``
    (mean h over wrap ticks; None without wrap ticks), plus the mean h of all ticks."""
    last = rows[-window:]
    reach = [r for r in last if reached_state(r)]
    n_wrap = sum(r.get("h_wrap_n", 0.0) for r in reach)
    return {"n": len(last), "n_reach": len(reach), "frac_reach": len(reach) / len(last) if last else 0.0,
            "frac_hold": float(np.mean([r.get("state_hold_s", 0.0) >= P2_HOLD_S for r in reach])) if reach else 0.0,
            "h_wrap": float(sum(r.get("h_wrap_mean", 0.0) * r.get("h_wrap_n", 0.0) for r in reach) / n_wrap) if n_wrap > 0 else None,
            "h_mean": float(np.mean([r["h_mean"] for r in last if "h_mean" in r])) if any("h_mean" in r for r in last) else None}


def p2_decision(sig: dict[str, Any]) -> str:
    """``go`` | ``approach`` (fewer than 5 % reach the state) | ``hold_unused`` (approach ok but h over wrap ticks <= 0.7 or no wrap ticks) | ``holdability`` (h is high but < 30 % keep the state >= 2 s)."""
    if sig["frac_reach"] < P2_MIN_FRAC:
        return "approach"
    if sig["h_wrap"] is None or sig["h_wrap"] <= P2_H_WRAP_MIN:
        return "hold_unused"
    if sig["frac_hold"] < P2_HOLD_MIN_FRAC:
        return "holdability"
    return "go"


def flatten(d: Any, prefix: str = "") -> dict[str, Any]:
    """Flatten nested dicts to ``a.b.c`` keys."""
    if not isinstance(d, dict):
        return {prefix: d}
    out: dict[str, Any] = {}
    for k, v in d.items():
        out.update(flatten(v, f"{prefix}.{k}" if prefix else str(k)))
    return out


def config_diff(a: dict[str, Any], b: dict[str, Any], names: tuple[str, str] = ("a", "b")) -> list[str]:
    """Human-readable differences between two config dicts (flattened); keys present in only one are listed too."""
    fa, fb = flatten(a), flatten(b)
    out = []
    for k in sorted(set(fa) | set(fb)):
        va, vb = fa.get(k, "<absent>"), fb.get(k, "<absent>")
        if va != vb:
            out.append(f"{k}: {names[0]}={va!r}  {names[1]}={vb!r}")
    return out


def write_config_diff(ours: Path, reference: Path, out: Path, extra: list[str] | None = None) -> None:
    """Write the differences of two run ``config.json`` files (ours first) plus ``extra`` notes (e.g. the recipe env) into ``out``."""
    d = config_diff(json.loads(ours.read_text()), json.loads(reference.read_text()), ("ours", "ref"))
    out.write_text(f"ours: {ours}\nref : {reference}\n\n" + "\n".join(d) + "\n" + ("\n" + "\n".join(extra) + "\n" if extra else ""))
