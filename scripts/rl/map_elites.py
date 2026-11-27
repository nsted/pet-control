"""MAP-Elites over the 11 CPG parameters (Phase 4): a library of gaits that differ in HOW they move, per body length.

    python scripts/rl/map_elites.py --n 11 --evals 5000 --seed 0 --workers 9 --out data/rl/map_elites/n11_s0.json
Descriptors (open-loop rollout of ``gait_eval.evaluate_gait``): travel direction in the body frame (8 bins over the circle, only when speed > 2 cm/s, else its own 'static' bin), whole-body roll rate
(5 bins: <-2, -2..-0.5, |r|<0.5, 0.5..2, >2 rad/s), yaw rate (3 bins: <-0.4, |w|<0.4, >0.4 rad/s). Fitness = speed + 0.03 (|roll| + |yaw|) so pure rotation cells are not worth zero.
Start: random unit-box samples (+ optional seed files of CMA results); then Gaussian mutation of random elites (sigma 0.08, 0.2 with p 0.2) and 10% fresh samples. Reports coverage, QD score
(sum of fitness over cells) and the best speed per direction, which is what the length study compares.
"""

from __future__ import annotations

import argparse
import json
import math
import time
from multiprocessing import get_context
from pathlib import Path

import numpy as np

from petctl.rl.cpg import PARAM_BOUNDS, PARAM_NAMES
from petctl.rl.gait_eval import build_sim, evaluate_gait

LO = np.array([PARAM_BOUNDS[k][0] for k in PARAM_NAMES])
HI = np.array([PARAM_BOUNDS[k][1] for k in PARAM_NAMES])
_SIM: dict = {}


def cell_of(m: dict) -> tuple[int, int, int]:
    """Descriptor bin of a metrics dict: (direction 0-7 or 8 = static, roll bin 0-4, yaw bin 0-2)."""
    if m["speed"] > 0.02:
        a = math.atan2(m["vy"], m["vx"])
        d = int(((a + math.pi / 8) % (2 * math.pi)) // (math.pi / 4))
    else:
        d = 8
    r = m["roll_rate"]
    rb = 0 if r < -2 else 1 if r < -0.5 else 2 if r <= 0.5 else 3 if r <= 2 else 4
    w = m["yaw_rate"]
    return d, rb, 0 if w < -0.4 else 1 if w <= 0.4 else 2


def fitness(m: dict) -> float:
    """Speed plus a small credit for rotation."""
    return m["speed"] + 0.03 * (abs(m["roll_rate"]) + abs(m["yaw_rate"]))


def work(args: tuple[int, list[float], float]) -> dict | None:
    """Evaluate one unit-box candidate (worker)."""
    n, u, seconds = args
    if n not in _SIM:
        _SIM[n] = build_sim(n)
    r = evaluate_gait(LO + np.clip(np.array(u), 0, 1) * (HI - LO), n, seconds=seconds, sim=_SIM[n])
    return r.to_dict() if r.ok else None


def main() -> int:
    """CLI entry point."""
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=11)
    ap.add_argument("--evals", type=int, default=5000)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--workers", type=int, default=9)
    ap.add_argument("--batch", type=int, default=72)
    ap.add_argument("--seconds", type=float, default=12.0)
    ap.add_argument("--init", type=Path, nargs="*", default=[], help="CMA result files whose params seed the archive")
    ap.add_argument("--out", type=Path, required=True)
    a = ap.parse_args()
    rng = np.random.default_rng(a.seed)
    arch: dict[tuple, dict] = {}
    curve, n_ev, t0 = [], 0, time.time()
    pool = get_context("spawn").Pool(a.workers)
    seeds = [((np.array(json.loads(p.read_text())["params"]) - LO) / (HI - LO)).tolist() for p in a.init]
    while n_ev < a.evals:
        if n_ev == 0:
            cands = seeds + [rng.random(len(LO)).tolist() for _ in range(a.batch - len(seeds))]
        else:
            keys = list(arch)
            cands = []
            for _ in range(a.batch):
                if rng.random() < 0.1:
                    cands.append(rng.random(len(LO)).tolist())
                else:
                    parent = np.array(arch[keys[rng.integers(len(keys))]]["u"])
                    cands.append(np.clip(parent + rng.normal(0, 0.2 if rng.random() < 0.2 else 0.08, len(LO)), 0, 1).tolist())
        for u, m in zip(cands, pool.map(work, [(a.n, c, a.seconds) for c in cands])):
            if m is None:
                continue
            k, f = cell_of(m), fitness(m)
            if k not in arch or f > arch[k]["f"]:
                arch[k] = {"u": u, "f": f, "m": m}
        n_ev += len(cands)
        curve.append((n_ev, len(arch), sum(e["f"] for e in arch.values())))
        if len(curve) % 10 == 0:
            print(f"[{time.time() - t0:5.0f}s] {n_ev} evals, {len(arch)} cells, QD {curve[-1][2]:.2f}, best speed {max(e['m']['speed'] for e in arch.values()) * 100:.1f} cm/s", flush=True)
    pool.close()
    out = {"n_modules": a.n, "seed": a.seed, "evals": n_ev, "curve": curve,
           "cells": [{"cell": list(k), "fitness": e["f"], "params": (LO + np.array(e["u"]) * (HI - LO)).tolist(), "metrics": e["m"]} for k, e in arch.items()]}
    a.out.parent.mkdir(parents=True, exist_ok=True)
    a.out.write_text(json.dumps(out))
    print(f"wrote {a.out}: {len(arch)} cells, QD {curve[-1][2]:.2f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
