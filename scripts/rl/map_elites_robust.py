"""Robust MAP-Elites (Phase 4, redo): a gait only counts if it works from SEVERAL conditions. Found 2026-11-27: the first library re-measured identically from the same start (error 0.00) but only 5/34 elites kept
their velocity within 40% at floor friction x1.3 or after a 3 s run of another gait (scripts/rl/gait_repro.py): the gaits were overfit to one start state.

    python scripts/rl/map_elites_robust.py --n 11 --evals 6000 --workers 9 --out data/rl/map_elites/robust_n11.json
Each candidate is measured in CPGEnv (the RL env's own velocity measure, 7 s window after 3 s) under K = 4 conditions: nominal; friction x1.3; after a 3 s pre-run of one of three fixed pre-gaits (an early library
forward gait, a left gait, a roll gait). Descriptors come from the MEDIAN velocity vector over conditions: direction (8 bins + static), speed (3 bins) and yaw rate (3 bins). Fitness = median speed -
mean |v_c - v_median| (dispersion over conditions). Also reports each elite's worst-condition error vs the median.
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
from petctl.rl.cpg_env import CPGEnv

LO = np.array([PARAM_BOUNDS[k][0] for k in PARAM_NAMES])
HI = np.array([PARAM_BOUNDS[k][1] for k in PARAM_NAMES])
_E: dict = {}


def pre_gaits() -> list[np.ndarray]:
    """Three fixed pre-run gaits from the CMA-ES specialists (forward, left, roll)."""
    out = []
    for f in ("forward_n11.json", "left_n11.json", "roll_travel_n11.json"):
        out.append(np.array(json.loads((Path("data/rl/gait_search") / f).read_text())["params"]))
    return out


def measure(e: CPGEnv, params: np.ndarray, mu: float, pre: np.ndarray | None) -> np.ndarray | None:
    """(vx, vy, yaw) of ``params`` in the start heading frame after an optional pre-run; None when the sim goes unstable."""
    e.reset(seed=1)
    e.sim.model.geom_friction[e.floor_geoms, 0] = e.floor_mu0 * mu
    if pre is not None:
        e.target, e.base = pre.copy(), pre.copy()
        e.cpg.reset(pre, 0.0)
        for _ in range(int(3.0 / e.dt)):
            e.step(np.zeros(13))
        e.h0 = float(np.median(e._chord_angles()))
    e.target, e.base = params.copy(), params.copy()
    if pre is None:
        e.cpg.reset(params, 0.0)
    c0, angs = None, []
    for i in range(int(10.0 / e.dt)):
        _o, _r, _t, done, _i = e.step(np.zeros(13))
        if done and e.k < e.episode_ticks:
            return None
        if i == int(3.0 / e.dt):
            c0 = e.hist[-1][0].copy()
        if i >= int(3.0 / e.dt):
            angs.append(e.hist[-1][1].copy())
    t = 7.0
    d = (e.hist[-1][0] - c0)[:2] / t
    h = e.h0
    ua = np.unwrap(np.stack(angs), axis=0)
    dy = ua[-1] - ua[0]
    yaw = float(np.median(dy) / t) if float(np.ptp(dy)) < 0.3 * t else 0.0
    return np.array([math.cos(h) * d[0] + math.sin(h) * d[1], -math.sin(h) * d[0] + math.cos(h) * d[1], yaw])


def evaluate(args: tuple[list[float], int]) -> dict | None:
    """Robust evaluation of one unit-box candidate."""
    u, seed = args
    e = _E.setdefault("env", CPGEnv(randomise=False, use_prior=False, episode_s=11.0, cmd_s=1e6, w_act=0.0))
    pres = _E.setdefault("pre", pre_gaits())
    p = LO + np.clip(np.array(u), 0, 1) * (HI - LO)
    k = pres[seed % 3]
    vs = [measure(e, p, 1.0, None), measure(e, p, 1.3, None), measure(e, p, 1.0, pres[seed % 3]), measure(e, p, 1.0, pres[(seed + 1) % 3])]
    if any(v is None for v in vs):
        return None
    V = np.array(vs)
    med = np.median(V, axis=0)
    disp = float(np.mean(np.linalg.norm(V[:, :2] - med[:2], axis=1)) + 0.05 * np.mean(np.abs(V[:, 2] - med[2])))
    speed = float(np.linalg.norm(med[:2]))
    return {"params": p.tolist(), "med": med.tolist(), "disp": disp, "speed": speed, "fit": speed + 0.03 * abs(med[2]) - 1.0 * disp, "vs": V.tolist()}


def cell(m: dict) -> tuple[int, int, int]:
    """Descriptor cell: direction (0-7, 8 = static), speed bin, yaw bin."""
    vx, vy, w = m["med"]
    sp = m["speed"]
    d = 8 if sp < 0.03 else int(((math.atan2(vy, vx) + math.pi / 8) % (2 * math.pi)) // (math.pi / 4))
    sb = 0 if sp < 0.08 else 1 if sp < 0.15 else 2
    return d, sb, 0 if w < -0.4 else 1 if w <= 0.4 else 2


def main() -> int:
    """CLI entry point."""
    ap = argparse.ArgumentParser()
    ap.add_argument("--evals", type=int, default=6000)
    ap.add_argument("--workers", type=int, default=9)
    ap.add_argument("--batch", type=int, default=72)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--out", type=Path, required=True)
    a = ap.parse_args()
    rng = np.random.default_rng(a.seed)
    arch: dict[tuple, dict] = {}
    seeds = [((np.array(json.loads(f.read_text())["params"]) - LO) / (HI - LO)).tolist() for f in sorted(Path("data/rl/gait_search").glob("*_n11*.json"))]
    pool, n_ev, t0 = get_context("spawn").Pool(a.workers), 0, time.time()
    while n_ev < a.evals:
        if n_ev == 0:
            cands = seeds + [rng.random(len(LO)).tolist() for _ in range(a.batch - len(seeds))]
        else:
            keys = list(arch)
            cands = [rng.random(len(LO)).tolist() if rng.random() < 0.1 else
                     np.clip(np.array(arch[keys[rng.integers(len(keys))]]["u"]) + rng.normal(0, 0.2 if rng.random() < 0.2 else 0.08, len(LO)), 0, 1).tolist() for _ in range(a.batch)]
        for u, m in zip(cands, pool.map(evaluate, [(c, int(rng.integers(3))) for c in cands])):
            if m is None:
                continue
            k = cell(m)
            if k not in arch or m["fit"] > arch[k]["fit"]:
                arch[k] = {**m, "u": u}
        n_ev += len(cands)
        if (n_ev // a.batch) % 10 == 0:
            ok = [e for e in arch.values() if e["disp"] < 0.4 * max(e["speed"], 0.05) and e["speed"] > 0.05]
            print(f"[{time.time() - t0:5.0f}s] {n_ev} evals, {len(arch)} cells, {len(ok)} robust moving elites, best speed {max(e['speed'] for e in arch.values()) * 100:.1f} cm/s", flush=True)
    pool.close()
    out = [{k: v for k, v in e.items() if k != "u"} | {"cell": list(c)} for c, e in arch.items()]
    a.out.parent.mkdir(parents=True, exist_ok=True)
    a.out.write_text(json.dumps(out))
    print(f"wrote {a.out}: {len(out)} cells")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
