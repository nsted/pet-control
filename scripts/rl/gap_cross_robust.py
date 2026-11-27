"""Robust dynamic gap crossing: CMA-ES where each candidate is scored over FOUR starts (nominal, start heading +0.15 rad, -0.15 rad, floor friction x1.3). Score = mean of the per-start scores (progress capped at 0.3,
+1.0 + speed bonus on success) + 1.0 when ALL four succeed. Found 2026-11-27 with gap_robust.py: the open-loop crossings of gap_cross.py only worked from the exact search start (1/6 perturbed starts).

    python scripts/rl/gap_cross_robust.py --gap 10 --evals 600 --out data/rl/gap_cross/robust_n11_g10.json
Starts: the saved crossing of the same gap (if any) plus the three MAP-Elites-style seeds of gap_cross.starts.
"""

from __future__ import annotations

import argparse
import json
import time
from multiprocessing import get_context
from pathlib import Path

import cma
import numpy as np

import gap_cross as gc

CONDS = [(0.0, 1.0), (0.15, 1.0), (-0.15, 1.0), (0.0, 1.3)]            # (start heading offset, friction scale)


def work(args: tuple[int, int, list[float]]) -> dict:
    """Score one unit-box candidate over the four conditions."""
    n, gap, u = args
    x = gc.LO + np.clip(np.array(u), 0, 1) * (gc.HI - gc.LO)
    sim = gc.sim_for(n, gap)
    floor = [g for g in range(sim.model.ngeom) if sim.model.geom_bodyid[g] == 0 and sim.model.geom_type[g] == 0]
    mu0 = sim.model.geom_friction[floor, 0].copy()
    rs = []
    for dyaw, mu in CONDS:
        y = x.copy()
        y[-1] += dyaw
        sim.model.geom_friction[floor, 0] = mu0 * mu
        rs.append(gc.rollout(n, gap, y))
    sim.model.geom_friction[floor, 0] = mu0
    allok = all(r["success"] for r in rs)
    return {"score": float(np.mean([r["score"] for r in rs])) + (1.0 if allok else 0.0), "success": allok, "n_ok": int(sum(r["success"] for r in rs))}


def main() -> int:
    """CLI entry point."""
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=11)
    ap.add_argument("--gap", type=int, default=10)
    ap.add_argument("--evals", type=int, default=600)
    ap.add_argument("--workers", type=int, default=9)
    ap.add_argument("--out", type=Path, required=True)
    a = ap.parse_args()
    starts = [(nm, x) for nm, x in gc.starts(a.n, Path("data/rl/map_elites"))]
    prev = Path(f"data/rl/gap_cross/n{a.n}_g{a.gap}.json")
    if prev.exists():
        r = json.loads(prev.read_text())
        starts.insert(0, ("previous crossing", np.array(r["params"] + [r["yaw0"]])))
    pool, best, t0 = get_context("spawn").Pool(a.workers), None, time.time()
    for name, x0 in starts:
        es = cma.CMAEvolutionStrategy(((x0 - gc.LO) / (gc.HI - gc.LO)).clip(0, 1).tolist(), 0.12, {"bounds": [0, 1], "popsize": 18, "seed": 1, "verbose": -9})
        n_ev, top = 0, None
        while n_ev < a.evals and not es.stop():
            c = es.ask()
            res = pool.map(work, [(a.n, a.gap, ci.tolist()) for ci in c])
            es.tell(c, [-r["score"] for r in res])
            n_ev += len(c)
            i = int(np.argmax([r["score"] for r in res]))
            if top is None or res[i]["score"] > top[0]:
                top = (res[i]["score"], c[i].tolist(), res[i])
            if top[2]["success"]:
                break
        print(f"[{time.time() - t0:4.0f}s] gap {a.gap} start '{name}': score {top[0]:.2f}, {top[2]['n_ok']}/4 conditions succeed ({n_ev} evals)", flush=True)
        if best is None or top[0] > best[0]:
            best = (top[0], top[1], top[2], name)
        if best[2]["success"]:
            break
    pool.close()
    x = gc.LO + np.clip(np.array(best[1]), 0, 1) * (gc.HI - gc.LO)
    a.out.parent.mkdir(parents=True, exist_ok=True)
    a.out.write_text(json.dumps({"n_modules": a.n, "gap_cm": a.gap, "success": best[2]["success"], "n_ok": best[2]["n_ok"], "score": best[0], "start": best[3], "params": x[:-1].tolist(), "yaw0": float(x[-1])}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
