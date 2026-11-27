"""Phase 8 baseline: how do the robust library gaits (n = 11) do on rough ground? Each elite of data/rl/cpg_elite_labels_n11.json is rolled out open loop (14 s, gait_eval metrics) on flat ground and on generated terrain
(rolling hills, ridges, mixed) at two difficulty levels; reports the speed kept (terrain speed / flat speed, median over moving elites), the fraction of elites that still move > 5 cm/s, and the best gait per terrain.

    python scripts/rl/terrain_gaits.py --workers 9
"""

from __future__ import annotations

import argparse
import json
from multiprocessing import get_context
from pathlib import Path

import numpy as np

from petctl.rl.gait_eval import build_sim, evaluate_gait
from petctl.sim.terrain import generate_terrain

CONDS = [("flat", None), ("rolling 0.4", dict(level=0.4, rolling=True)), ("rolling 0.8", dict(level=0.8, rolling=True)), ("ridges 0.4", dict(level=0.4, ridges=True)),
         ("ridges 0.8", dict(level=0.8, ridges=True)), ("mixed 0.5", dict(level=0.5)), ("mixed 1.0", dict(level=1.0))]
_SIMS: dict = {}


def work(args: tuple[int, int, list[float]]) -> dict:
    """One elite on one condition."""
    ci, n, params = args
    name, kw = CONDS[ci]
    if ci not in _SIMS:
        _SIMS[ci] = build_sim(n, terrain=generate_terrain(seed=1, **kw)) if kw else build_sim(n)
    r = evaluate_gait(np.array(params), n, seconds=14.0, sim=_SIMS[ci])
    return {"ci": ci, "ok": r.ok, "speed": r.speed if r.ok else 0.0, "flipped": bool(r.flipped) if r.ok else True}


def main() -> int:
    """CLI entry point."""
    ap = argparse.ArgumentParser()
    ap.add_argument("--workers", type=int, default=9)
    ap.add_argument("--n", type=int, default=11)
    a = ap.parse_args()
    rows = [r for r in json.loads(Path(f"data/rl/cpg_elite_labels_n{a.n}.json").read_text()) if float(np.hypot(r["v"][0], r["v"][1])) > 0.05]
    jobs = [(ci, a.n, r["params"]) for ci in range(len(CONDS)) for r in rows]
    with get_context("spawn").Pool(a.workers) as pool:
        res = pool.map(work, jobs, chunksize=8)
    by = {ci: np.array([x["speed"] for x in res if x["ci"] == ci]) for ci in range(len(CONDS))}
    flat = by[0]
    print(f"{len(rows)} moving elites (flat speed median {100 * np.median(flat):.1f} cm/s)")
    out = {}
    for ci, (name, _) in enumerate(CONDS):
        keep = by[ci] / np.maximum(flat, 1e-6)
        out[name] = {"median_kept": float(np.median(keep)), "frac_moving": float(np.mean(by[ci] > 0.05)), "best_cm_s": float(100 * by[ci].max())}
        print(f"  {name:12s} median speed kept {np.median(keep):.2f}   still > 5 cm/s: {np.mean(by[ci] > 0.05):.2f}   best {100 * by[ci].max():.1f} cm/s")
    Path("data/rl/terrain_gaits_n11.json").write_text(json.dumps(out, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
