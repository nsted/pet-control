"""Targeted pose optimisation (cross-entropy method, sim only): push one pose type as far as the body, the real torque cap and gravity allow.

Objectives (all with a "can actually be held" requirement: settled, modest hold effort), over the 7 joint targets with NO angle range (search space +-2 pi is a size, not a limit):
    reach   highest module centre            bridge  middle modules raised above the ends        coil   smallest body extent        low   lowest profile

    python scripts/rl/pose_targeted.py --out data/rl/motion_repertoire/poses_targeted.json
"""

from __future__ import annotations

import argparse
import json
import os
import time
from multiprocessing import Pool
from pathlib import Path

import numpy as np

import pose_repertoire as pr

OBJ = {"arch": lambda d: d["mid_clear"] - 2.0 * max(0.0, d["ends_up"]) - 3.0 * max(0.0, 0.30 - d["span"]), "reach": lambda d: d["top"], "bridge": lambda d: d["bridge"] - 0.3 * max(0.0, 3 - d["grounded"]) * 0.0, "coil": lambda d: -d["compact"], "low": lambda d: -d["top"]}
MIN_FIT = float(os.environ.get("POSE_MIN_FIT", "1.5"))     # held-pose requirement: fitness = 1/(0.05 + effort) >= 1.5, i.e. effort <= ~0.62 (torque + residual motion); 0 = no effort limit (only the actuator cap limits torque)
ONLY = [o for o in os.environ.get("POSE_OBJ", "").split(",") if o]


def score(args):
    theta, name = args
    desc, fit = pr.evaluate_pose(theta, 5.0)
    if fit < 0:
        return -9.0, desc, fit
    return OBJ[name](desc) - (0.0 if fit >= MIN_FIT else 0.5), desc, fit


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--gens", type=int, default=60)
    ap.add_argument("--pop", type=int, default=48)
    ap.add_argument("--workers", type=int, default=3)
    ap.add_argument("--out", type=Path, default=Path("data/rl/motion_repertoire/poses_targeted.json"))
    ap.add_argument("--seed-archive", type=Path, default=Path("data/rl/motion_repertoire/poses_unbounded.json"))
    a = ap.parse_args()
    rng = np.random.default_rng(0)
    seeds = json.load(open(a.seed_archive))["cells"] if a.seed_archive.exists() else []
    out, t0 = {}, time.time()
    with Pool(a.workers) as pool:
        for name in (ONLY or [o for o in OBJ if o != "arch"]):
            key = {"arch": "bridge", "reach": "top", "bridge": "bridge", "coil": "compact", "low": "top"}[name]
            sign = 1 if name in ("arch", "reach", "bridge") else -1
            best_seed = max(seeds, key=lambda c: sign * c["desc"][key]) if seeds else None
            mean = np.array(best_seed["theta"]) if best_seed else np.zeros(pr.NJ)
            std = np.full(pr.NJ, 1.0)
            best = (-1e9, None, None, None)
            for g in range(a.gens):
                cand = [mean.copy()] + [np.clip(rng.normal(mean, std), -2 * np.pi, 2 * np.pi) for _ in range(a.pop - 1)]
                res = pool.map(score, [(c, name) for c in cand])
                sc = np.array([r[0] for r in res])
                for c, r in zip(cand, res):
                    if r[0] > best[0]:
                        best = (r[0], c.copy(), r[1], r[2])
                el = np.array([cand[i] for i in np.argsort(-sc)[:max(4, a.pop // 6)]])
                mean, std = 0.5 * mean + 0.5 * el.mean(0), np.maximum(0.7 * std + 0.3 * el.std(0), 0.03)
            out[name] = {"theta": best[1].tolist(), "desc": best[2], "fitness": best[3], "score": best[0]}
            print(json.dumps({"objective": name, "score": round(best[0], 3), "desc": {k: round(v, 3) for k, v in best[2].items()}, "fitness": round(best[3], 2), "elapsed_min": round((time.time() - t0) / 60, 1)}), flush=True)
            a.out.write_text(json.dumps(out, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
