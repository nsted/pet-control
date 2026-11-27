"""Two-stage self-righting (Phase 4 target, second attempt): a CPG motion runs until the body is upright, then the controller STOPS it and eases to a straight rest pose and holds.

    python scripts/rl/self_right2.py --n 11 --evals 1200 --out data/rl/self_right/n11_v2.json
Stage 1 trigger: median module up-component > 0.8 for 0.3 s (computable on the real robot from the head IMU + joint angles). Stage 2: parameters ease (1.5 s) to zero amplitude, joints to q = 0, hold.
Success (both from INVERTED and from the SIDE start): during the last 5 s of a 20 s run the whole-body median up-component stays > 0.6 at every tick and the typical worst module > 0.1. A body that
merely rolls through upright fails. Search: CMA-ES over the 11 CPG parameters from three MAP-Elites seeds.
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


def run(n: int, p: np.ndarray, orient: float, sim=None, frames: list | None = None) -> dict:
    """One two-stage rollout from ``orient``; returns the success statistics and the trigger time."""
    sim = sim or _SIM.setdefault(n, build_sim(n))
    nj = n - 1
    sim.reset()
    sim.data.qpos[3:7] = [math.cos(orient / 2), math.sin(orient / 2), 0.0, 0.0]
    sim.data.qpos[2] += 0.04
    sim.step(np.zeros(nj), 0.8)
    cpg = CPG(nj, smooth_hz=0.0)
    cpg.reset(p)
    d, ups, t_stop, ok_ticks, fade = sim.data, [], None, 0, 1.0
    for i in range(int(SECONDS / DT)):
        if t_stop is None:
            sim.step(cpg.step(DT, p), DT)
        else:
            fade = max(0.0, fade - DT / 1.5)                                              # stage 2: amplitudes and biases ease to zero
            sim.step(cpg.step(DT, p) * fade, DT)
        if not np.all(np.isfinite(d.qpos)):
            return {"score": -1.0, "success": False}
        up = np.array([d.xmat[sim.m0_id + k].reshape(3, 3)[2, 2] for k in range(n)])
        ups.append(up)
        if frames is not None:
            frames.append((i * DT, float(np.median(up)), float(up.min()), t_stop is not None))
        if t_stop is None:
            ok_ticks = ok_ticks + 1 if np.median(up) > 0.8 else 0
            if ok_ticks >= 6:
                t_stop = i * DT
    ups = np.array(ups)
    tail = ups[-int(TAIL_S / DT):]
    med, worst = float(np.median(tail, axis=1).min()), float(np.median(tail.min(axis=1)))
    return {"score": med + 0.5 * worst, "success": bool(med > 0.6 and worst > 0.1), "min_median_up": med, "worst_up": worst, "t_stop_s": t_stop}


def work(args: tuple[int, list[float]]) -> dict:
    """Pool worker: worst result over both start orientations."""
    n, u = args
    p = LO + np.clip(np.array(u), 0, 1) * (HI - LO)
    rs = [run(n, p, o) for o in STARTS]
    w = min(rs, key=lambda r: r["score"])
    return {**w, "success": all(r["success"] for r in rs), "per_start": rs}


def main() -> int:
    """CLI entry point."""
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=11)
    ap.add_argument("--evals", type=int, default=1200)
    ap.add_argument("--workers", type=int, default=9)
    ap.add_argument("--archive", type=Path, default=Path("data/rl/map_elites"))
    ap.add_argument("--out", type=Path, required=True)
    a = ap.parse_args()
    cells = json.loads((a.archive / f"n{a.n}_s0_v2.json").read_text())["cells"]
    seeds = [max(cells, key=lambda c: c["metrics"]["speed"] * (abs(c["metrics"]["roll_rate"]) > 2)), max(cells, key=lambda c: c["metrics"]["vx"]), max(cells, key=lambda c: abs(c["metrics"]["vy"]))]
    pool, best, t0 = get_context("spawn").Pool(a.workers), None, time.time()
    for si, c in enumerate(seeds):
        es = cma.CMAEvolutionStrategy(((np.array(c["params"]) - LO) / (HI - LO)).clip(0, 1).tolist(), 0.3, {"bounds": [0, 1], "popsize": 18, "seed": si + 1, "verbose": -9})
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
        if best[2]["success"]:
            break
    pool.close()
    p = LO + np.clip(np.array(best[1]), 0, 1) * (HI - LO)
    a.out.parent.mkdir(parents=True, exist_ok=True)
    a.out.write_text(json.dumps({"n_modules": a.n, "success": best[2]["success"], "score": best[0], "params": p.tolist(), "per_start": best[2]["per_start"]}))
    print(json.dumps(best[2]["per_start"]))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
