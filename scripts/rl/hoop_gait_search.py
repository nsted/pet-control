"""Search for a hoop-rolling gait (sim only): the body closes into a loop and rolls head over tail along the ground, like a rolling hula-hoop.

Open-loop 43-parameter controller of ``ridge_gait_search`` (offsets, waves, second harmonic, shared frequency, continuous spin rates; NO angle limits), optimised with CEM after a broad random
screening. Measured on flat ground from the clean straight start pose:

    pitch   net revolutions of the head about the body centre in the vertical plane containing the direction of travel (head over tail)
    dist    net displacement of the body centre (m)
    ring    1 - (head-to-tail distance / straight body length): 1 when the ends meet (closed loop)
    lift    mean height of the highest module above its rest height / loop diameter (the loop stands up instead of lying flat)

Fitness rewards travel that comes WITH pitch revolutions (a hoop turns once per circumference), a closed ring and a loop that stands up.

    python scripts/rl/hoop_gait_search.py --workers 8 --out data/rl/motion_repertoire/hoop.json
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

EP_S = 20.0
_env = None
_fwd = None


def _ensure():
    global _env, _fwd
    if _env is None:
        _env, _fwd = rg._make_env()
        _env.flat = rg.ridge_terrain(_env, _fwd, 0.0)
    return _env, _fwd


def rollout(theta: np.ndarray, seconds: float = EP_S, frames: bool = False) -> dict[str, float]:
    env, fwd = _ensure()
    env.reset(seed=1)
    env.sim.set_terrain(env.flat)
    d = env.sim.data
    scale = float(env.cfg.action_scale_rad)
    P0 = d.xipos[1:9].copy()
    body_len = float(np.linalg.norm(P0[0] - P0[7]))
    z0 = P0[:, 2].copy()
    com0 = np.array(d.subtree_com[1], float)
    coms, rels, ring, lift, n, t, done = [], [], 0.0, 0.0, 0, 0.0, False
    while not done and t < seconds:
        q = rg.controller(theta, t)
        _, _, te, tr, _ = env.step(np.concatenate([q / scale, [-1.0]]))
        done = te or tr
        t += env.dt
        com = np.array(d.subtree_com[1], float)
        P = d.xipos[1:9]
        coms.append(com.copy())
        rels.append(np.array(d.site_xpos[env.sim.head_id], float) - com)
        if t > seconds * 0.4:                                                           # score the steady state, not the start-up
            ring += 1.0 - float(np.linalg.norm(P[0] - P[7])) / body_len
            lift += float((P[:, 2] - z0).max())
            n += 1
    out = {"pitch": 0.0, "dist": 0.0, "ring": 0.0, "lift": 0.0, "speed": 0.0, "ok": float(t > seconds * 0.9 and np.all(np.isfinite(rels)))}
    if len(coms) < 10 or not out["ok"]:
        return out
    coms, rels = np.array(coms), np.array(rels)
    disp = coms[-1] - coms[len(coms) // 3]
    horiz = np.array([disp[0], disp[1], 0.0])
    dist = float(np.linalg.norm(horiz))
    u = horiz / dist if dist > 1e-6 else np.array([fwd[0], fwd[1], 0.0])
    phi = np.unwrap(np.arctan2(rels[:, 2], rels @ u))                                   # head angle about the centre in the (travel, up) plane
    pitch = float((phi[-1] - phi[len(phi) // 3]) / (2 * np.pi))
    T = t * (1 - 1 / 3)
    out.update(pitch=pitch, dist=dist, speed=dist / T, ring=ring / max(n, 1), lift=lift / max(n, 1) / (body_len / np.pi))
    return out


def fitness(m: dict[str, float]) -> float:
    if not m["ok"]:
        return -1.0
    turns = abs(m["pitch"])
    return float(m["dist"] + 0.15 * min(turns, 6.0) + 0.5 * max(m["ring"], 0.0) * min(turns, 3.0) / 3.0 + 0.3 * min(max(m["lift"], 0.0), 1.2) * min(turns, 3.0) / 3.0)


def work(theta):
    m = rollout(theta)
    return theta, m, fitness(m)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--workers", type=int, default=8)
    ap.add_argument("--screen", type=int, default=1200)
    ap.add_argument("--iters", type=int, default=40)
    ap.add_argument("--lam", type=int, default=64)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--out", type=Path, default=Path("data/rl/motion_repertoire/hoop.json"))
    a = ap.parse_args()
    rng = np.random.default_rng(a.seed)
    lo, hi = rg.bounds()
    a.out.parent.mkdir(parents=True, exist_ok=True)
    t0 = time.time()
    best = (-1e9, None, None)
    with Pool(a.workers) as pool:
        cand = [np.clip(rng.uniform(lo, hi), lo, hi) for _ in range(a.screen)]
        res = pool.map(work, cand, chunksize=4)
        res.sort(key=lambda r: -r[2])
        print(json.dumps({"stage": "screen", "best": round(res[0][2], 3), "top10": [round(r[2], 2) for r in res[:10]], "min": round(min(r[2] for r in res), 2), "elapsed_min": round((time.time() - t0) / 60, 1)}), flush=True)
        elite = np.array([r[0] for r in res[:24]])
        mean, std = elite.mean(0), np.maximum(elite.std(0), (hi - lo) * 0.02)
        best = (res[0][2], res[0][0], res[0][1])
        for it in range(a.iters):
            cand = [np.clip(rng.normal(mean, std), lo, hi) for _ in range(a.lam)] + [best[1]]
            res = pool.map(work, cand, chunksize=2)
            res.sort(key=lambda r: -r[2])
            if res[0][2] > best[0]:
                best = (res[0][2], res[0][0], res[0][1])
            el = np.array([r[0] for r in res[:max(4, a.lam // 6)]])
            mean, std = 0.5 * mean + 0.5 * el.mean(0), np.maximum(0.7 * std + 0.3 * el.std(0), (hi - lo) * 0.01)
            print(json.dumps({"iter": it, "best": round(best[0], 3), "metrics": {k: round(v, 3) for k, v in best[2].items()}, "elapsed_min": round((time.time() - t0) / 60, 1)}), flush=True)
            a.out.write_text(json.dumps({"fitness": best[0], "metrics": best[2], "theta": best[1].tolist(), "top": [{"theta": r[0].tolist(), "metrics": r[1], "fitness": r[2]} for r in res[:8]]}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
