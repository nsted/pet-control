#!/usr/bin/env python3
"""Compare two `petctl run --record` recordings (e.g. in-process vs driver/brain split).

Motions like `--control sine` are wall-clock driven (and their clock starts before
the Rerun viewer finishes launching), so two runs differ by an arbitrary start
phase. For every servo this finds the time offset (within +-max-lag, default one
sine period) that best aligns B to A, then reports the RMS error after alignment
for measured joint positions and for commanded positions. The residual measures
shape / amplitude / dynamics agreement, not start phase.

It also reports, per recording, the tracking lag from commanded to measured
position (servo 1) -- the number that shows added loop latency.

Exit status 1 if any aligned RMS error exceeds --tol (radians).

    python scripts/compare_recordings.py in_process.jsonl split.jsonl [--tol 0.05]
"""

from __future__ import annotations

import argparse
import json
import sys

import numpy as np

DEFAULT_MAX_LAG_S = 3.0
TRIM_S = 1.0        # ignore the first/last second (startup and shutdown transients)
GRID_HZ = 100.0


def load(path: str) -> list[dict]:
    with open(path) as f:
        return [json.loads(line) for line in f if line.strip()]


def series(rows: list[dict], servo: str, kind: str) -> tuple[np.ndarray, np.ndarray]:
    """(t since first row [s], value) for one servo; kind is 'pos' (measured) or 'cmd'."""
    t0 = rows[0]["timestamp"]
    ts: list[float] = []
    vs: list[float] = []
    for r in rows:
        if kind == "pos":
            v = r["servo_positions"].get(servo)
        else:
            v = next((c["position"] for c in r["commands"] if str(c["servo_id"]) == servo), None)
        if v is not None:
            ts.append(r["timestamp"] - t0)
            vs.append(v)
    return np.asarray(ts), np.asarray(vs)


def aligned_rms(
    a: tuple[np.ndarray, np.ndarray], b: tuple[np.ndarray, np.ndarray], max_lag: float = DEFAULT_MAX_LAG_S
) -> tuple[float, float, float]:
    """Return (rms_unaligned, rms_best, best_lag_s) of B resampled onto A's grid."""
    (ta, va), (tb, vb) = a, b
    t_end = min(ta[-1], tb[-1]) - TRIM_S - max_lag
    t_start = TRIM_S + max_lag
    if t_end <= t_start:
        raise SystemExit("recordings are too short to compare")
    grid = np.arange(t_start, t_end, 1.0 / GRID_HZ)
    ref = np.interp(grid, ta, va)
    best = (np.inf, 0.0)
    for lag in np.arange(-max_lag, max_lag + 1e-9, 0.01):
        err = float(np.sqrt(np.mean((np.interp(grid + lag, tb, vb) - ref) ** 2)))
        if err < best[0]:
            best = (err, float(lag))
    unaligned = float(np.sqrt(np.mean((np.interp(grid, tb, vb) - ref) ** 2)))
    return unaligned, best[0], best[1]


def tracking_lag_ms(rows: list[dict], servo: str = "1") -> float:
    """Lag (ms) at which measured position best matches commanded position within one recording."""
    tc, vc = series(rows, servo, "cmd")
    tp, vp = series(rows, servo, "pos")
    grid = np.arange(TRIM_S + 0.5, min(tc[-1], tp[-1]) - TRIM_S - 0.5, 1.0 / GRID_HZ)
    cmd = np.interp(grid, tc, vc)
    best = (np.inf, 0.0)
    for lag in np.arange(0.0, 0.5 + 1e-9, 0.005):
        err = float(np.sqrt(np.mean((np.interp(grid + lag, tp, vp) - cmd) ** 2)))
        if err < best[0]:
            best = (err, float(lag))
    return best[1] * 1000.0


def describe(rows: list[dict]) -> str:
    dur = rows[-1]["timestamp"] - rows[0]["timestamp"]
    peak = max(abs(v) for r in rows for v in r["servo_positions"].values())
    pt = rows[-1].get("power_telemetry") or {}
    return (f"{len(rows)} rows, {dur:.1f}s, {len(rows) / dur:.1f} ticks/s, "
            f"max|pos|={peak:.3f} rad, power={pt.get('system_state', '?')}")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("a")
    ap.add_argument("b")
    ap.add_argument("--tol", type=float, default=0.05, help="max aligned RMS error, rad (default 0.05)")
    ap.add_argument("--max-lag", type=float, default=DEFAULT_MAX_LAG_S, help="alignment search window, s (default 3.0)")
    args = ap.parse_args()

    A, B = load(args.a), load(args.b)
    print(f"A: {describe(A)}\nB: {describe(B)}")
    servos = sorted(set(A[-1]["servo_positions"]) & set(B[-1]["servo_positions"]), key=int)
    worst = 0.0
    print(f"\n{'servo':>5} {'kind':>4} {'rms raw':>9} {'rms aligned':>12} {'lag (ms)':>9}")
    for sid in servos:
        for kind in ("pos", "cmd"):
            sa, sb = series(A, sid, kind), series(B, sid, kind)
            if len(sa[0]) < 10 or len(sb[0]) < 10:
                continue
            raw, best, lag = aligned_rms(sa, sb, args.max_lag)
            worst = max(worst, best)
            print(f"{sid:>5} {kind:>4} {raw:9.4f} {best:12.4f} {lag * 1000:9.0f}")
    print(f"\ntracking lag cmd->pos (servo 1): A {tracking_lag_ms(A):.0f} ms, B {tracking_lag_ms(B):.0f} ms")
    verdict = "PASS" if worst <= args.tol else "FAIL"
    print(f"\nworst aligned RMS = {worst:.4f} rad (tol {args.tol}) -> {verdict}")
    return 0 if worst <= args.tol else 1


if __name__ == "__main__":
    sys.exit(main())
