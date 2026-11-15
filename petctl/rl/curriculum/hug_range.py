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
