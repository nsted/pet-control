"""Collect labelled proprioceptive data on the edge scene: unprotected wander episodes (they walk off, so every approach to the edge is recorded), per-tick features and the overhang label.

    python scripts/rl/edge_collect.py --episodes 300 --workers 9 --out data/rl/edge/wander_data.npz
"""

from __future__ import annotations

import argparse
from multiprocessing import get_context
from pathlib import Path

import numpy as np

from edge_sim import Scene, Wander

_S: dict = {}


def work(seed: int) -> dict:
    """One recorded episode."""
    sc = _S.setdefault("sc", Scene())
    lg = sc.run(Wander(), seed, 60.0, record=True)
    T = lg.ticks
    return {k: np.array(v) for k, v in T.items()} | {"fell": lg.fell}


def main() -> int:
    """CLI entry point."""
    ap = argparse.ArgumentParser()
    ap.add_argument("--episodes", type=int, default=300)
    ap.add_argument("--workers", type=int, default=9)
    ap.add_argument("--seed0", type=int, default=5000)
    ap.add_argument("--out", type=Path, required=True)
    a = ap.parse_args()
    with get_context("spawn").Pool(a.workers) as pool:
        res = pool.map(work, range(a.seed0, a.seed0 + a.episodes), chunksize=2)
    a.out.parent.mkdir(parents=True, exist_ok=True)
    lens = np.array([len(r["overhang"]) for r in res])
    np.savez_compressed(a.out, lens=lens, fell=np.array([r["fell"] for r in res]), **{k: np.concatenate([r[k] for r in res]) for k in ("tau", "err", "qd", "up", "gyro", "overhang", "dist_edge", "phase", "params")})
    print(f"{len(res)} episodes, {lens.sum()} ticks, fell {np.mean([r['fell'] for r in res]):.2f}; overhang>0 on {np.mean(np.concatenate([r['overhang'] for r in res]) > 0):.3f} of ticks")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
