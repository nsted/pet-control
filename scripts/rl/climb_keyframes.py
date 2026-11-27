"""Staged climb of a 12 cm platform: a sequence of K keyframe poses (joint angles) with durations, executed with smooth interpolation (no periodic gait). CMA-ES over K x 10 angles + K durations
(+ start heading), scored over 4 starts (nominal, heading +-0.15, friction x1.3) like climb_robust.py. Seeded from the head-lift static pose of statics (lift the head ~19 cm, then place it on the platform).

    python scripts/rl/climb_keyframes.py --evals 1500 --out data/rl/climb/keyframes_n11.json
Success: every module on top of the platform (z > 0.09 m, x in [0.36, 1.34]).
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

import climb_robust as cr

K, NJ = 4, 10
LO = np.array([-math.pi] * (K * NJ) + [0.8] * K + [-0.4])
HI = np.array([math.pi] * (K * NJ) + [3.5] * K + [0.4])
SECONDS, DT = 22.0, 0.05


def targets(x: np.ndarray, t: float) -> np.ndarray:
    """Joint targets at time t: smoothstep through the keyframes (each segment: ease over 60% of its duration, then hold)."""
    kf = x[:K * NJ].reshape(K, NJ)
    dur = x[K * NJ:K * NJ + K]
    prev, t0 = np.zeros(NJ), 0.0
    for k in range(K):
        if t < t0 + dur[k]:
            a = min(1.0, (t - t0) / (0.6 * dur[k]))
            a = a * a * (3 - 2 * a)
            return prev + (kf[k] - prev) * a
        prev, t0 = kf[k], t0 + dur[k]
    return prev


def rollout(n: int, x: np.ndarray, mu: float = 1.0) -> dict:
    """One staged rollout (reuses climb_robust's scene and scoring)."""
    sim = cr.sim_for(n)
    nj = n - 1
    floor = [g for g in range(sim.model.ngeom) if sim.model.geom_bodyid[g] == 0 and sim.model.geom_type[g] == 0]
    mu0 = sim.model.geom_friction[floor, 0].copy()
    sim.model.geom_friction[floor, 0] = mu0 * mu
    try:
        sim.reset()
        d = sim.data
        yaw = float(x[-1])
        d.qpos[0], d.qpos[1] = -0.15, 0.0
        d.qpos[3:7] = [math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)]
        sim.step(np.zeros(nj), 0.8)
        best, t_ok, ok, max_x, max_z = 0.0, SECONDS, False, -1.0, 0.0
        for i in range(int(SECONDS / DT)):
            sim.step(targets(x, i * DT), DT)
            if not np.all(np.isfinite(d.qpos)):
                return {"score": -1.0, "success": False}
            p = d.xipos[sim.m0_id:sim.m0_id + n]
            on = (p[:, 2] > 0.09) & (p[:, 0] > 0.36) & (p[:, 0] < 1.34) & (np.abs(p[:, 1]) < 0.6)
            best = max(best, float(on.mean()))
            max_x = max(max_x, float(p[0, 0]))
            near = p[:, 0] > 0.2
            if near.any():
                max_z = max(max_z, float(p[near, 2].max()))
            if on.all():
                ok, t_ok = True, i * DT
                break
        shaping = 0.15 * float(np.clip(max_x / 0.36, 0.0, 1.0)) + 0.15 * float(np.clip((max_z - 0.04) / 0.08, 0.0, 1.0))
        return {"score": 0.7 * best + shaping + (1.0 + 0.5 * (1 - t_ok / SECONDS) if ok else 0.0), "success": ok, "t": t_ok}
    finally:
        sim.model.geom_friction[floor, 0] = mu0


def work(args: tuple[int, list[float]]) -> dict:
    """Score a unit-box candidate over the four conditions."""
    n, u = args
    x = LO + np.clip(np.array(u), 0, 1) * (HI - LO)
    rs = []
    for dyaw, mu in cr.CONDS:
        y = x.copy()
        y[-1] += dyaw
        rs.append(rollout(n, y, mu))
    allok = all(r["success"] for r in rs)
    return {"score": float(np.mean([r["score"] for r in rs])) + (1.0 if allok else 0.0), "success": allok, "n_ok": int(sum(r["success"] for r in rs))}


def main() -> int:
    """CLI entry point."""
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=11)
    ap.add_argument("--evals", type=int, default=1500)
    ap.add_argument("--restarts", type=int, default=3)
    ap.add_argument("--workers", type=int, default=9)
    ap.add_argument("--out", type=Path, required=True)
    a = ap.parse_args()
    hl = next(np.array(r["q"]) for r in json.loads(Path("data/rl/statics/results.json").read_text()) if r["task"] == "head_lift" and r["n_modules"] == a.n and r["seed"] == 0)
    pool, best, t0 = get_context("spawn").Pool(a.workers), None, time.time()
    for rs_ in range(a.restarts):
        rng = np.random.default_rng(rs_)
        kf = np.stack([0.5 * hl, hl, hl * rng.uniform(0.5, 1.0), 0.5 * hl]) + rng.normal(0, 0.2, (K, NJ))
        x0 = np.concatenate([kf.ravel(), [2.0] * K, [0.0]])
        es = cma.CMAEvolutionStrategy(((x0 - LO) / (HI - LO)).clip(0, 1).tolist(), 0.2, {"bounds": [0, 1], "popsize": 24, "seed": rs_ + 1, "verbose": -9, "tolfun": 1e-12, "tolflatfitness": 200})
        n_ev, top = 0, None
        while n_ev < a.evals and not es.stop():
            c = es.ask()
            res = pool.map(work, [(a.n, ci.tolist()) for ci in c])
            es.tell(c, [-r["score"] for r in res])
            n_ev += len(c)
            i = int(np.argmax([r["score"] for r in res]))
            if top is None or res[i]["score"] > top[0]:
                top = (res[i]["score"], c[i].tolist(), res[i])
            if top[2]["success"]:
                break
        print(f"[{time.time() - t0:4.0f}s] restart {rs_}: score {top[0]:.2f}, {top[2]['n_ok']}/4 succeed ({n_ev} evals)", flush=True)
        if best is None or top[0] > best[0]:
            best = (top[0], top[1], top[2])
        if best[2]["success"]:
            break
    pool.close()
    x = LO + np.clip(np.array(best[1]), 0, 1) * (HI - LO)
    a.out.parent.mkdir(parents=True, exist_ok=True)
    a.out.write_text(json.dumps({"n_modules": a.n, "success": best[2]["success"], "n_ok": best[2]["n_ok"], "score": best[0], "x": x.tolist()}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
