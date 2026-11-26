"""Does a gait work the same when the body starts INVERTED (rolled 180 deg about its long axis)? Owner hypothesis (2026-11-26): even lengths are near-symmetric under that roll, odd lengths (tail rolled 180 deg) are not.

    python scripts/rl/inverted_gaits.py --lengths 10 11 12 13 --top 16
Takes the fastest ``--top`` open-loop gaits of each length from data/rl/cpg_sweep/L<n>.json, evaluates each upright and inverted (same CPG parameters), and compares |vx|, |vy|, |yaw rate| and speed
(left / right and yaw sign flip when the body is upside-down, so signed lateral numbers are compared as magnitudes). Also reports how many inverted rollouts turn the body back over.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np

from petctl.rl.gait_eval import build_sim, evaluate_gait


def main() -> int:
    """CLI entry point."""
    ap = argparse.ArgumentParser()
    ap.add_argument("--lengths", type=int, nargs="+", default=[10, 11, 12, 13])
    ap.add_argument("--top", type=int, default=16)
    ap.add_argument("--sweep", type=Path, default=Path("data/rl/cpg_sweep"))
    ap.add_argument("--out", type=Path, default=Path("data/rl/inverted_gaits.json"))
    a = ap.parse_args()
    out = {}
    print("| n | gaits | corr(vx up, vx inv) | median speed inv / up | corr(|vy|) | corr(|yaw|) | inverted runs that flip back upright | median |vx| inv / up |")
    print("|---|---|---|---|---|---|---|---|")
    for n in a.lengths:
        rows = [r for r in json.loads((a.sweep / f"L{n}.json").read_text()) if r["ok"] and not r["flipped"]]
        top = sorted(rows, key=lambda r: -r["speed"])[:a.top]
        sim = build_sim(n)
        up, inv = [], []
        for r in top:
            p = np.array(r["params"])
            up.append(evaluate_gait(p, n, seconds=12.0, sim=sim))
            inv.append(evaluate_gait(p, n, seconds=12.0, sim=sim, start_orient=math.pi))
        c = lambda f: float(np.corrcoef([f(u) for u in up], [f(v) for v in inv])[0, 1])          # noqa: E731
        flips_back = sum(v.up_min < 0.2 and False or (v.up_min > -0.5) for v in inv)                # an inverted start has up_min ~ -1; > -0.5 means it turned over at some point
        ratio = float(np.median([v.speed / max(u.speed, 1e-9) for u, v in zip(up, inv)]))
        vx_ratio = float(np.median([abs(v.vx) / max(abs(u.vx), 1e-9) for u, v in zip(up, inv)]))
        print(f"| {n} | {len(top)} | {c(lambda r: r.vx):+.2f} | {ratio:.2f} | {c(lambda r: abs(r.vy)):+.2f} | {c(lambda r: abs(r.yaw_rate)):+.2f} | {flips_back}/{len(top)} | {vx_ratio:.2f} |", flush=True)
        out[n] = {"up": [u.to_dict() for u in up], "inv": [v.to_dict() for v in inv], "params": [r["params"] for r in top]}
    a.out.write_text(json.dumps(out))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
