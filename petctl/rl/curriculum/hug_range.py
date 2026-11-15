"""Decision rules of the action-range study (``scripts/rl/range_diag.py``, ``scripts/rl/hug_range_driver.py``): pure functions over episode rows / screen results, testable with synthetic data.

Step A (diagnostic): pooled over wrap-attempt ticks (hug run >= 3 modules) of the cyl r3 / r7 gate episodes, the fraction of in-run joint-ticks with |action| >= 0.95 (``saturated_frac``) decides:
>= 20 % -> Step B; < 10 % -> skip B (Step C); 10-20 % -> Step B (the manager's default for ambiguity).
Step B (``hug_scale15``) / Step C (``hug_long_ep``) GO rule: n=30 screen >= 0.15, OR any curved gate shape >= 1/6, OR cylinder training span p75 >= 165 deg.
20-min check of B: stop ONLY if the cylinder span p75 (last 150 cylinder episodes) fell by more than 10 deg vs its value at the start, or box success fell by more than 0.2 (box safety rule).
"""

from __future__ import annotations

from typing import Any

import numpy as np

from petctl.rl.curriculum import hug_curr2 as hc2
from petctl.rl.curriculum.hug_odd import curved_shape_names

SAT_B_MIN, SAT_SKIP_MAX = 0.20, 0.10
GO_SCREEN, GO_CURVED_SHAPE, GO_CYL_SPAN_P75 = 0.15, 1.0 / 6.0, 165.0
SPAN_FALL_DEG = 10.0
HOUR1_FINAL_SUCCESS, SUCCESS_FALL, BOX_FALL = 0.177, 0.05, 0.2      # expert rules for B hours >= 2 (20-min check)
RISING_DELTA, SCREEN_DONE = 0.05, 0.55
ESC_SCREEN, ESC_CURVED, EXTRA_SCREEN = 0.30, 2.0 / 6.0, 0.30         # end rule after hour 3 / Step C'
SCALE15, SCALE15_REF = 1.5, 1.0                 # the new / the old action_scale_rad
RANGE_KEYS = ("wrap_attempt_ticks", "saturated_frac", "sat_any_frac", "sat_all_frac", "target_gap_run", "max_abs_target_run")


def pool_range(rows: list[dict[str, Any]]) -> dict[str, Any]:
    """Pool the per-episode range metrics (rows are ``{metrics}`` gate rows or flat training rows) over wrap-attempt ticks: tick-weighted ``saturated_frac`` / ``sat_any_frac`` / ``sat_all_frac`` /
    ``target_gap_run``, the max of ``max_abs_target_run``, and episode counts."""
    ms = [r.get("metrics", r) for r in rows]
    n = np.array([float(m.get("wrap_attempt_ticks", 0.0)) for m in ms])
    tot = float(n.sum())
    out: dict[str, Any] = {"episodes": len(ms), "episodes_with_wrap_attempt": int((n > 0).sum()), "wrap_attempt_ticks": tot}
    for k in ("saturated_frac", "sat_any_frac", "sat_all_frac", "target_gap_run"):
        v = np.array([float(m.get(k, 0.0)) for m in ms])
        out[k] = float((v * n).sum() / tot) if tot > 0 else None
    out["max_abs_target_run"] = float(max([float(m.get("max_abs_target_run", 0.0)) for m in ms] or [0.0]))
    return out


def range_decision(saturated_frac: float | None) -> tuple[str, str]:
    """``(decision, reason)``: ``B`` (frac >= 0.20), ``skip_B`` (< 0.10), ``B`` with reason ``ambiguous`` for 0.10-0.20 (default: do B). None (no wrap-attempt ticks at all) counts as ambiguous."""
    if saturated_frac is None:
        return "B", "ambiguous: no wrap-attempt ticks measured (default: do B)"
    if saturated_frac >= SAT_B_MIN - 1e-12:
        return "B", f"saturated fraction {saturated_frac:.3f} >= {SAT_B_MIN}"
    if saturated_frac < SAT_SKIP_MAX - 1e-12:
        return "skip_B", f"saturated fraction {saturated_frac:.3f} < {SAT_SKIP_MAX}"
    return "B", f"ambiguous: saturated fraction {saturated_frac:.3f} in [{SAT_SKIP_MAX}, {SAT_B_MIN}) (default: do B)"


def go_rule(screen_success: float, per_shape: dict[str, float], cyl_span_p75: float | None, n_modules: int = 8) -> tuple[bool, dict[str, Any]]:
    """GO if the n=30 screen >= 0.15, OR any curved gate shape (cylinders, rounded box) >= 1/6, OR the cylinder training span p75 >= 165 deg."""
    curved = {n: per_shape.get(n, 0.0) for n in curved_shape_names(n_modules)}
    by_screen = screen_success >= GO_SCREEN - 1e-9
    by_curved = any(v >= GO_CURVED_SHAPE - 1e-9 for v in curved.values())
    by_span = cyl_span_p75 is not None and cyl_span_p75 >= GO_CYL_SPAN_P75
    return bool(by_screen or by_curved or by_span), {"screen": screen_success, "curved": curved, "cyl_span_p75": cyl_span_p75, "by_screen": by_screen, "by_curved": by_curved, "by_span": by_span}


def span_fall_stop(rows: list[dict[str, Any]], base_p75: float | None) -> tuple[bool, dict[str, Any]]:
    """The 20-min rule of Step B: stop if the cylinder span p75 of the last 150 cylinder episodes is more than 10 deg below ``base_p75`` (needs 150 cylinder episodes and a baseline)."""
    p75 = hc2.cyl_span_p75(rows)
    n_cyl = len(hc2.family_rows(rows, ("cyl",)))
    stop = bool(base_p75 is not None and p75 is not None and n_cyl >= hc2.CURVED_WINDOW and p75 < base_p75 - SPAN_FALL_DEG)
    return stop, {"cyl_span_p75": p75, "base_cyl_span_p75": base_p75, "n_cyl": n_cyl, "fall_deg": None if (p75 is None or base_p75 is None) else base_p75 - p75}


def filter_buffer(items: list[dict[str, Any]], min_span: float) -> list[dict[str, Any]]:
    """Start-buffer entries with a recorded normal span >= ``min_span`` (entries without a span are dropped)."""
    return [e for e in items if float(e.get("span", -1.0)) >= min_span]


class SuccessChecks(hc2.Curr2Checks):
    """``Curr2Checks`` with the expert 20-min rule of B hours >= 2 / Step C' (``success_fall_stop``); the box safety rule is inherited."""

    def __init__(self, out: Any, base_success: float, t1_s: float = hc2.CHECK1_MIN * 60.0, scale: float = 1.0) -> None:
        """``base_success``: training success at the end of the previous hour."""
        super().__init__(out, t1_s, scale)
        self.base_success = base_success

    def run_check1(self) -> bool:
        """Apply the success-fall / box-fall rule now; True = ok (no stop)."""
        import json

        rows = self._rows()
        stop, sig = success_fall_stop(rows, self.base_success)
        levels = hc2.read_family_levels(self.out)
        self._record("check1", {"ok": not stop, **sig, "levels": levels, "table": hc2.family_table(rows, levels), "range": pool_range(rows[-300:])})
        print(f"[check1] 20 min: {json.dumps(sig)}: {'STOP' if stop else 'ok'}", flush=True)
        return not stop


class RangeChecks(hc2.Curr2Checks):
    """``Curr2Checks`` with the Step-B 20-min rule (cylinder span p75 fall) instead of the curved-trend rule; the box safety rule is inherited. ``base_p75`` None: the 20-min rule never stops."""

    def __init__(self, out: Any, base_p75: float | None, t1_s: float = hc2.CHECK1_MIN * 60.0, scale: float = 1.0) -> None:
        """``base_p75``: cylinder span p75 at the start of the run."""
        super().__init__(out, t1_s, scale)
        self.base_p75 = base_p75

    def run_check1(self) -> bool:
        """Apply the span-fall rule now; True = ok (no stop)."""
        import json

        rows = self._rows()
        stop, sig = span_fall_stop(rows, self.base_p75)
        levels = hc2.read_family_levels(self.out)
        self._record("check1", {"ok": not stop, **sig, "levels": levels, "table": hc2.family_table(rows, levels), "range": pool_range(rows[-300:])})
        print(f"[check1] 20 min: {json.dumps(sig)}: {'STOP' if stop else 'ok'}", flush=True)
        return not stop


def train_success(rows: list[dict[str, Any]], window: int = hc2.CURVED_WINDOW) -> float | None:
    """Mean own-level training success of the last ``window`` episodes (None with fewer)."""
    return float(np.mean([float(r.get("success", 0.0)) for r in rows[-window:]])) if len(rows) >= window else None


def success_trend(rows: list[dict[str, Any]], min_rows: int = 60) -> dict[str, Any]:
    """Training success of the first and last sixth of the episodes; ``rising`` = last >= first + 0.05 (needs ``min_rows`` episodes)."""
    n = len(rows) // 6
    if len(rows) < min_rows or n == 0:
        return {"first": None, "last": None, "rising": False, "n": len(rows)}
    f = float(np.mean([float(r.get("success", 0.0)) for r in rows[:n]]))
    la = float(np.mean([float(r.get("success", 0.0)) for r in rows[-n:]]))
    return {"first": f, "last": la, "rising": bool(la >= f + RISING_DELTA - 1e-9), "n": len(rows)}


def success_fall_stop(rows: list[dict[str, Any]], base: float = HOUR1_FINAL_SUCCESS) -> tuple[bool, dict[str, Any]]:
    """20-min rule of B hours >= 2: stop if training success (last 150 episodes) is more than 0.05 below ``base`` OR box success fell more than 0.2 (``box_safety_stop``)."""
    cur = train_success(rows)
    box_stop, box_sig = hc2.box_safety_stop(rows)
    succ_stop = cur is not None and cur < base - SUCCESS_FALL - 1e-12
    return bool(succ_stop or box_stop), {"train_success": cur, "base_success": base, "success_stop": bool(succ_stop), "box_stop": bool(box_stop), **box_sig}


def curved_max(per_shape: dict[str, float], n_modules: int = 8) -> float:
    """Best per-shape screen success over the curved gate shapes (cylinders, rounded box)."""
    return max([per_shape.get(n, 0.0) for n in curved_shape_names(n_modules)] or [0.0])


def is_extra_hour(series: str, hour: int) -> bool:
    """The single extra hour after the end rule (scale15 hour 4, long15 hour 2)."""
    return (series == "scale15" and hour >= 4) or (series == "long15" and hour >= 2)


def post_hour_decision(series: str, hour: int, screen: float, per_shape: dict[str, float], rising: bool, n_modules: int = 8) -> str:
    """Expert rule after an hour screen of ``scale15`` (hours >= 2) or ``long15`` -> ``final`` | ``next`` | ``C15`` | ``extra`` | ``escalate``.
    screen >= 0.55: final (n=100 verdict). Extra hour done: escalate. scale15 hour 2: next if screen >= 0.15 OR a curved shape >= 1/6 OR training success rising, else C15 (long-episode changes on scale 1.5).
    scale15 hour >= 3 and long15: screen < 0.30 and no curved shape >= 2/6 -> escalate, else one extra hour."""
    cm = curved_max(per_shape, n_modules)
    if screen >= SCREEN_DONE - 1e-9:
        return "final"
    if is_extra_hour(series, hour):
        return "escalate"
    if series == "scale15" and hour <= 2:
        return "next" if (screen >= GO_SCREEN - 1e-9 or cm >= GO_CURVED_SHAPE - 1e-9 or rising) else "C15"
    return "escalate" if (screen < ESC_SCREEN - 1e-9 and cm < ESC_CURVED - 1e-9) else "extra"
