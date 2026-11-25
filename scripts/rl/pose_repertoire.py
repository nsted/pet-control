"""Static pose repertoire by quality-diversity search (MAP-Elites), sim only.

Searches the 7 joint-angle targets (+-1 rad, the policy's commandable range) for body shapes the robot can actually HOLD under the real torque cap: reaching high, bridging,
coiling up and staying low / flat ("burrowing"). Each candidate is commanded for ``--hold`` seconds on flat ground from the clean straight start pose; descriptors are measured over the
final second from the module centres:

    top      highest module centre (m)                                   -> reach (high) vs burrow (low profile)
    bridge   mean rise of the middle modules minus the rise of the ends -> arch / bridge (>0) or sag (<0)
    compact  largest distance between two module centres (m)            -> coil / bunched (small) vs stretched (large)
    grounded number of modules still within 1.5 cm of their rest height  -> how much of the body is off the ground

The archive keeps, per cell, the pose that is most settled and cheapest to hold (low torque, no residual motion). Output: JSON with joint targets and descriptors.

    python scripts/rl/pose_repertoire.py --iters 150 --workers 3 --out data/rl/motion_repertoire/poses.json
"""

from __future__ import annotations

import argparse
import json
import os
import time
from multiprocessing import Pool
from pathlib import Path

import mujoco
import numpy as np

import ridge_gait_search as rg

BINS = {"top": (0.18, 0.42), "bridge": (-0.08, 0.12), "compact": (0.10, 0.45), "grounded": (0.0, 8.01)}
N_BIN = {"top": 6, "bridge": 5, "compact": 6, "grounded": 5}
NJ = 7

_env = None


def _ensure():
    global _env
    if _env is None:
        _env, fwd = rg._make_env()
        _env.flat = rg.ridge_terrain(_env, fwd, 0.0)
        m = _env.sim.model                                                                  # diagnostic overrides (env POSE_KP / POSE_CAP): emulate holding torque (stiff kp) and a different cap
        if os.environ.get("POSE_KP"):
            kp = float(os.environ["POSE_KP"])
            m.actuator_gainprm[:, 0], m.actuator_biasprm[:, 1] = kp, -kp
        if os.environ.get("POSE_CAP"):
            cap = float(os.environ["POSE_CAP"])
            m.actuator_forcerange[:, 0], m.actuator_forcerange[:, 1] = -cap, cap
    return _env


def evaluate_pose(theta: np.ndarray, hold: float = 5.0) -> tuple[dict[str, float], float]:
    """Command ``theta`` (rad) for ``hold`` s; return (descriptor, settledness/economy fitness; -1 if invalid)."""
    env = _ensure()
    sim = env.sim
    sim.set_terrain(env.flat)                                   # clean straight home pose
    m, d = sim.model, sim.data
    if os.environ.get("POSE_CAP"):
        sim.set_cap(np.full(NJ, float(os.environ["POSE_CAP"])))
    mujoco.mj_forward(m, d)
    z_rest = d.xipos[1:9, 2].copy()
    steps = int(hold / 0.05)
    tq, dq, prev = 0.0, 0.0, sim.joint_pos().copy()
    keep = 20
    P = None
    for i in range(steps):
        sim.step(theta, 0.05)
        q = sim.joint_pos()
        if i >= steps - keep:
            tq += float(np.mean(np.abs(sim.joint_torque())))
            dq += float(np.mean(np.abs(q - prev)))
        prev = q.copy()
    P = d.xipos[1:9].copy()
    if not np.all(np.isfinite(P)):
        return {k: 0.0 for k in BINS}, -1.0
    rise = P[:, 2] - z_rest
    mid, ends = rise[2:6].mean(), rise[[0, 1, 6, 7]].mean()
    dist = np.linalg.norm(P[:, None, :] - P[None, :, :], axis=-1)
    desc = {"top": float(P[:, 2].max()), "bridge": float(mid - ends), "compact": float(dist.max()), "grounded": float(np.sum(rise < 0.015))}
    effort = tq / keep + 3.0 * dq / keep                         # torque to hold + residual motion
    return desc, float(1.0 / (0.05 + effort))


def peak_pose(theta: np.ndarray, seconds: float = 20.0) -> np.ndarray:
    """Joint configuration (rad) of a gait at the instant its head is highest (a seed for the static pose search)."""
    env, fwd = rg._make_env()
    env.reset(seed=1)
    env.sim.set_terrain(rg.ridge_terrain(env, fwd, 0.0))
    scale = float(env.cfg.action_scale_rad)
    d = env.sim.data
    best, qbest, t = -1e9, env.sim.joint_pos().copy(), 0.0
    while t < seconds:
        q = rg.controller(theta, t)
        _, _, te, tr, _ = env.step(np.concatenate([np.clip(q / scale, -1.0, 1.0), [-1.0]]))
        t += env.dt
        z = float(d.site_xpos[env.sim.head_id][2])
        if z > best:
            best, qbest = z, env.sim.joint_pos().copy()
        if te or tr:
            break
    return qbest


def cell_of(desc: dict[str, float]) -> tuple[int, ...]:
    return tuple(int(np.clip((desc[k] - lo) / (hi - lo) * N_BIN[k], 0, N_BIN[k] - 1)) for k, (lo, hi) in BINS.items())


def work(args):
    theta, hold = args
    desc, fit = evaluate_pose(theta, hold)
    return theta, desc, fit


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--iters", type=int, default=150)
    ap.add_argument("--batch", type=int, default=64)
    ap.add_argument("--init", type=int, default=256)
    ap.add_argument("--hold", type=float, default=5.0)
    ap.add_argument("--range", type=float, default=2 * 3.141592653589793, help="search-space size for each joint target (+-rad); NOT a robot limit: the robot has no angle range")
    ap.add_argument("--workers", type=int, default=3)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--seed-gaits", type=Path, default=None, help="gait archive JSON: seed the search with the joint configurations at the peak head height of its highest-rising gaits")
    ap.add_argument("--out", type=Path, default=Path("data/rl/motion_repertoire/poses.json"))
    a = ap.parse_args()
    rng = np.random.default_rng(a.seed)
    a.out.parent.mkdir(parents=True, exist_ok=True)
    archive: dict[tuple[int, ...], dict] = {}
    seeds: list[np.ndarray] = []
    if a.seed_gaits is not None:
        gc = sorted(json.load(open(a.seed_gaits))["cells"], key=lambda c: -c["desc"].get("height", 0.0))[:120]
        seeds = [peak_pose(np.array(c["theta"])) for c in gc]
        print(json.dumps({"seeds": len(seeds)}), flush=True)
    t0 = time.time()

    def insert(theta, desc, fit) -> bool:
        if fit < 0:
            return False
        c = cell_of(desc)
        if c not in archive or fit > archive[c]["fitness"]:
            archive[c] = {"theta": [float(x) for x in theta], "desc": desc, "fitness": fit}
            return True
        return False

    with Pool(a.workers) as pool:
        for it in range(a.iters + 1):
            if it == 0:
                cand = [np.clip(rng.uniform(-1, 1, NJ) * a.range * rng.choice([0.3, 0.6, 1.0]), -a.range, a.range) for _ in range(a.init)]
                cand[0] = np.zeros(NJ)
                for i, sd in enumerate(seeds):                                    # structured starts: peak-height configurations of rearing gaits, plus noisy copies
                    cand[1 + i] = sd
                    if 1 + len(seeds) + i < len(cand):
                        cand[1 + len(seeds) + i] = np.clip(sd + rng.normal(0, 0.1, NJ), -1, 1)
            else:
                keys = list(archive)
                cand = []
                for _ in range(a.batch):
                    p = np.array(archive[keys[rng.integers(len(keys))]]["theta"])
                    if rng.random() < 0.3:
                        q = np.array(archive[keys[rng.integers(len(keys))]]["theta"])
                        p = np.where(rng.random(NJ) < 0.5, p, q)
                    cand.append(np.clip(p + rng.normal(0, rng.choice([0.05, 0.15, 0.3]) * a.range, NJ), -a.range, a.range))
            res = pool.map(work, [(c, a.hold) for c in cand])
            added = sum(insert(*r) for r in res)
            print(json.dumps({"iter": it, "evals": len(res), "added": added, "cells": len(archive), "max_top": round(max(v["desc"]["top"] for v in archive.values()), 3), "elapsed_min": round((time.time() - t0) / 60, 1)}), flush=True)
            if it % 5 == 0 or it == a.iters:
                a.out.write_text(json.dumps({"bins": BINS, "n_bin": N_BIN, "hold_s": a.hold, "cells": [{"cell": list(k), **v} for k, v in archive.items()]}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
