"""Dynamic self-righting search (Phase 4 target): the body starts INVERTED (rolled 180 deg about its long axis) or on its SIDE (90 deg) and a CPG motion must turn it over and keep it upright.

    python scripts/rl/self_right.py --n 11 --evals 900 --out data/rl/self_right/n11.json
CMA-ES over the 11 CPG parameters, three starts (the best forward, sideways and rolling cells of the MAP-Elites archive). A rollout is 20 s; success = at EVERY moment of the LAST 5 s the median module up-component is
> 0.6 and the typical worst module > 0.1 (a continuously rolling body is not self-righted) (whole-body upright, not just a head or an IMU), from BOTH start orientations (inverted and side) -- the score is the worse of the two. Flipping during the motion is allowed.
"""

from __future__ import annotations

import argparse
import json
import math
import time
from multiprocessing import get_context
from pathlib import Path

import cma
import numpy as np

from petctl.rl.cpg import CPG, PARAM_BOUNDS, PARAM_NAMES
from petctl.rl.gait_eval import build_sim

LO = np.array([PARAM_BOUNDS[k][0] for k in PARAM_NAMES])
HI = np.array([PARAM_BOUNDS[k][1] for k in PARAM_NAMES])
DT, SECONDS, TAIL_S = 0.05, 20.0, 5.0
STARTS = (math.pi, math.pi / 2)
_SIM: dict = {}


def upright(n: int, p: np.ndarray, orient: float, sim=None, record: bool = False) -> dict:
    """Roll out ``p`` from ``orient`` (roll about the long axis, rad); returns whole-body upright statistics over the last TAIL_S seconds."""
    sim = sim or _SIM.setdefault(n, build_sim(n))
    nj = n - 1
    sim.reset()
    sim.data.qpos[3:7] = [math.cos(orient / 2), math.sin(orient / 2), 0.0, 0.0]
    sim.data.qpos[2] += 0.04
    sim.step(np.zeros(nj), 0.8)
    cpg = CPG(nj, smooth_hz=0.0)
    cpg.reset(p)
    d, ups, first = sim.data, [], None
    for i in range(int(SECONDS / DT)):
        sim.step(cpg.step(DT, p), DT)
        if not np.all(np.isfinite(d.qpos)):
            return {"score": -1.0, "success": False}
        u = np.array([d.xmat[sim.m0_id + k].reshape(3, 3)[2, 2] for k in range(n)])
        ups.append(u)
    ups = np.array(ups)
    tail = ups[-int(TAIL_S / DT):]
    per_tick_med = np.median(tail, axis=1)
    med, worst = float(per_tick_med.min()), float(np.median(tail.min(axis=1)))                  # med: the WORST moment's whole-body median (a rolling body that is only upright on average fails)
    ok = med > 0.6 and worst > 0.1
    t_up = next((i * DT for i in range(len(ups)) if np.median(ups[i]) > 0.7), SECONDS)            # first time the body median is upright
    return {"score": med + 0.5 * worst, "success": bool(ok), "median_up": med, "worst_up": worst, "t_upright_s": t_up}


def work(args: tuple[int, list[float]]) -> dict:
    """Pool worker: worst result over the start orientations."""
    n, u = args
    p = LO + np.clip(np.array(u), 0, 1) * (HI - LO)
    rs = [upright(n, p, o) for o in STARTS]
    w = min(rs, key=lambda r: r["score"])
    return {**w, "success": all(r["success"] for r in rs), "per_start": rs}


def main() -> int:
    """CLI entry point."""
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=11)
    ap.add_argument("--evals", type=int, default=900)
    ap.add_argument("--workers", type=int, default=9)
    ap.add_argument("--archive", type=Path, default=Path("data/rl/map_elites"))
    ap.add_argument("--out", type=Path, required=True)
    a = ap.parse_args()
    cells = json.loads((a.archive / f"n{a.n}_s0_v2.json").read_text())["cells"]
    pool, best, t0 = get_context("spawn").Pool(a.workers), None, time.time()
    seeds = [max(cells, key=lambda c: c["metrics"]["speed"] * (abs(c["metrics"]["roll_rate"]) > 2)),
             max(cells, key=lambda c: c["metrics"]["vx"]), max(cells, key=lambda c: abs(c["metrics"]["vy"]))]
    for si, c in enumerate(seeds):
        es = cma.CMAEvolutionStrategy(((np.array(c["params"]) - LO) / (HI - LO)).clip(0, 1).tolist(), 0.25, {"bounds": [0, 1], "popsize": 18, "seed": si + 1, "verbose": -9})
        n_ev, top = 0, None
        while n_ev < a.evals // 3 and not es.stop():
            xs = es.ask()
            res = pool.map(work, [(a.n, x.tolist()) for x in xs])
            es.tell(xs, [-r["score"] for r in res])
            n_ev += len(xs)
            i = int(np.argmax([r["score"] for r in res]))
            if top is None or res[i]["score"] > top[0]:
                top = (res[i]["score"], xs[i].tolist(), res[i])
            if top[2]["success"]:
                break
        print(f"[{time.time() - t0:4.0f}s] start {si}: score {top[0]:.2f} success={top[2]['success']} ({n_ev} evals)", flush=True)
        if best is None or top[0] > best[0]:
            best = top
    pool.close()
    p = LO + np.clip(np.array(best[1]), 0, 1) * (HI - LO)
    a.out.parent.mkdir(parents=True, exist_ok=True)
    a.out.write_text(json.dumps({"n_modules": a.n, "success": best[2]["success"], "score": best[0], "params": p.tolist(), "per_start": best[2]["per_start"]}))
    print(json.dumps(best[2]["per_start"]))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
