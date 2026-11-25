"""Motion repertoire by quality-diversity search (MAP-Elites), sim only.

Searches the 36-parameter wave controller of ``ridge_gait_search`` for a *vocabulary* of motions instead of one gait. Every candidate runs ``--seconds`` on flat ground from the clean straight
start pose; its behaviour descriptor (measured in the start body frame) is

    vx, vy   forward / sideways speed of the body centre (m/s)       -> forward wriggling, sidewinding, backing up
    yaw      turn rate of the head-centre axis (rad/s)                -> turning, pivoting
    roll     rotation of the head module about its long axis (rev/s)  -> rolling
    extent   mean head-to-centre distance (m)                         -> stretched vs coiled poses
    height   peak head height above its start height (m)              -> hoisting / rearing poses

The archive keeps, per cell of a coarse grid over the descriptor, the most efficient controller found (speed per torque effort). Output: JSON archive with parameters and descriptors.

    python scripts/rl/motion_repertoire.py --iters 150 --workers 4 --out data/rl/motion_repertoire/archive.json
"""

from __future__ import annotations

import argparse
import json
import time
from multiprocessing import Pool
from pathlib import Path

import mujoco
import numpy as np

import ridge_gait_search as rg

BINS = {"vx": (-0.08, 0.08), "vy": (-0.08, 0.08), "yaw": (-0.5, 0.5), "roll": (-0.3, 0.3), "extent": (0.05, 0.45), "height": (0.0, 0.12)}
N_BIN = 5
KEYS = tuple(BINS)

_env = None
_fwd = None


def _ensure_env():
    global _env, _fwd
    if _env is None:
        _env, _fwd = rg._make_env()
    return _env, _fwd


def descriptor(theta: np.ndarray, seconds: float) -> tuple[dict[str, float], float]:
    """Run ``theta`` on flat ground for ``seconds`` and return (descriptor dict, efficiency fitness)."""
    env, fwd = _ensure_env()
    env.reset(seed=1)
    env.sim.set_terrain(rg.ridge_terrain(env, fwd, 0.0))
    m, d = env.sim.model, env.sim.data
    lat = np.array([-fwd[1], fwd[0]])
    scale = float(env.cfg.action_scale_rad)
    head = lambda: d.site_xpos[env.sim.head_id][:2] - d.subtree_com[1][:2]          # noqa: E731
    p0, h0 = np.array(d.subtree_com[1][:2], float), head().copy()
    ang_prev = np.arctan2(h0 @ lat, h0 @ fwd)
    yaw = roll_acc = 0.0
    up_prev = None
    ext, tq, n, t, done = 0.0, 0.0, 0, 0.0, False
    hz0, hpk = float(d.site_xpos[env.sim.head_id][2]), 0.0
    hb = int(m.body("m0").id)
    while not done and t < seconds:
        q = rg.controller(theta, t)
        a = np.concatenate([q / scale, [-1.0]])
        _, _, te, tr, _ = env.step(a)
        done = te or tr
        t += env.dt
        h = head()
        ang = np.arctan2(h @ lat, h @ fwd)
        yaw += (ang - ang_prev + np.pi) % (2 * np.pi) - np.pi
        ang_prev = ang
        R = d.xmat[hb].reshape(3, 3)
        up = R[:, 2]
        if up_prev is not None:                                                      # rotation of the head's up-vector about the start heading axis
            f3 = np.array([fwd[0], fwd[1], 0.0])
            a1 = np.arctan2(np.dot(up, np.cross(f3, [0, 0, 1])), up[2])
            a0 = np.arctan2(np.dot(up_prev, np.cross(f3, [0, 0, 1])), up_prev[2])
            roll_acc += (a1 - a0 + np.pi) % (2 * np.pi) - np.pi
        up_prev = up.copy()
        ext += float(np.linalg.norm(h))
        hpk = max(hpk, float(d.site_xpos[env.sim.head_id][2]) - hz0)
        tq += float(np.mean(np.abs(env.sim.joint_torque())))
        n += 1
    dp = np.array(d.subtree_com[1][:2], float) - p0
    T = max(t, 1e-6)
    desc = {"vx": float(dp @ fwd) / T, "vy": float(dp @ lat) / T, "yaw": yaw / T, "roll": roll_acc / (2 * np.pi) / T, "extent": ext / max(n, 1), "height": hpk}
    speed = float(np.hypot(desc["vx"], desc["vy"])) + 0.05 * abs(desc["yaw"]) + 0.2 * abs(desc["roll"])
    fit = speed / (0.05 + tq / max(n, 1))
    if not np.all(np.isfinite(list(desc.values()))) or (done and t < seconds * 0.9):
        fit = -1.0                                                                   # terminated early (flip / glitch): not a usable motion
    return desc, float(fit)


def cell_of(desc: dict[str, float]) -> tuple[int, ...]:
    return tuple(int(np.clip((desc[k] - lo) / (hi - lo) * N_BIN, 0, N_BIN - 1)) for k, (lo, hi) in BINS.items())


def work(args: tuple[np.ndarray, float]):
    theta, seconds = args
    desc, fit = descriptor(theta, seconds)
    return theta, desc, fit


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--iters", type=int, default=150)
    ap.add_argument("--batch", type=int, default=32)
    ap.add_argument("--init", type=int, default=96)
    ap.add_argument("--seconds", type=float, default=20.0)
    ap.add_argument("--workers", type=int, default=4)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--out", type=Path, default=Path("data/rl/motion_repertoire/archive.json"))
    a = ap.parse_args()
    rng = np.random.default_rng(a.seed)
    lo, hi = rg.bounds()
    a.out.parent.mkdir(parents=True, exist_ok=True)
    archive: dict[tuple[int, ...], dict] = {}
    t0 = time.time()

    def insert(theta, desc, fit) -> bool:
        if fit < 0:
            return False
        c = cell_of(desc)
        if c not in archive or fit > archive[c]["fitness"]:
            archive[c] = {"theta": theta.tolist(), "desc": desc, "fitness": fit}
            return True
        return False

    with Pool(a.workers) as pool:
        for it in range(a.iters + 1):
            if it == 0:
                cand = [np.clip(rng.uniform(lo, hi), lo, hi) for _ in range(a.init)]                          # broad random start: wide coverage of motion types
            else:
                keys = list(archive)
                cand = []
                for _ in range(a.batch):
                    p = np.array(archive[keys[rng.integers(len(keys))]]["theta"])
                    if rng.random() < 0.25:                                                                       # crossover with another elite
                        q = np.array(archive[keys[rng.integers(len(keys))]]["theta"])
                        mask = rng.random(p.shape) < 0.5
                        p = np.where(mask, p, q)
                    cand.append(np.clip(p + rng.normal(0, 0.08, p.shape) * (hi - lo), lo, hi))
            res = pool.map(work, [(c, a.seconds) for c in cand])
            added = sum(insert(*r) for r in res)
            best = max((v["fitness"] for v in archive.values()), default=0.0)
            print(json.dumps({"iter": it, "evals": len(res), "added": added, "cells": len(archive), "best_fit": round(best, 3), "elapsed_min": round((time.time() - t0) / 60, 1)}), flush=True)
            if it % 5 == 0 or it == a.iters:
                a.out.write_text(json.dumps({"bins": BINS, "n_bin": N_BIN, "seconds": a.seconds, "cells": [{"cell": list(k), **v} for k, v in archive.items()]}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
