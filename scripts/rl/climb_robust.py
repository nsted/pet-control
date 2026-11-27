"""Climbing a 12 cm platform (the sim's fixed obstacle height) from the floor: CMA-ES over the 11 CPG parameters + start heading, scored over 4 starts (nominal, heading +-0.15 rad, friction x1.3).

    python scripts/rl/climb_robust.py --evals 600 --library 5 --out data/rl/climb/robust_n11.json
Scene: one platform x in [0.35, 1.35] m, half width 0.6 m; the body starts on the floor at x = 0 heading +x (+ start yaw parameter). Per start: score = best fraction of modules on top of the platform
(z > 0.09 m, x in the platform) over the rollout (25 s) + 1.0 + speed bonus when ALL modules are on top; the candidate's score = mean over starts + 1.0 if all four succeed.
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
from petctl.sim.model_builder import BuildParams, PetSim

CAP, SECONDS, DT = 0.5, 25.0, 0.05
LO = np.array([PARAM_BOUNDS[k][0] for k in PARAM_NAMES] + [-math.pi])
HI = np.array([PARAM_BOUNDS[k][1] for k in PARAM_NAMES] + [math.pi])
CONDS = [(0.0, 1.0), (0.15, 1.0), (-0.15, 1.0), (0.0, 1.3)]
_SIM: dict = {}


def sim_for(n: int) -> PetSim:
    """Cached sim with one 12 cm platform ahead."""
    if n not in _SIM:
        _SIM[n] = PetSim(BuildParams(n_modules=n, cap_nm=(CAP,) * (n - 1), dt=0.004, obstacles=(("box", 0.85, 0.0, 0.0, 0.5, 0.6),)))
    return _SIM[n]


def rollout(n: int, x: np.ndarray, mu: float = 1.0) -> dict:
    """One rollout; returns score, success and the time of success."""
    sim = sim_for(n)
    nj = n - 1
    floor = [g for g in range(sim.model.ngeom) if sim.model.geom_bodyid[g] == 0 and sim.model.geom_type[g] == 0]
    mu0 = sim.model.geom_friction[floor, 0].copy()
    sim.model.geom_friction[floor, 0] = mu0 * mu
    try:
        sim.reset()
        d = sim.data
        yaw = float(x[-1])
        d.qpos[0], d.qpos[1] = -0.05, 0.0
        d.qpos[3:7] = [math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)]
        sim.step(np.zeros(nj), 0.8)
        cpg = CPG(nj, smooth_hz=0.0)
        cpg.reset(x[:-1], 0.0)
        best, t_ok, ok, max_x, max_z = 0.0, SECONDS, False, -1.0, 0.0
        for i in range(int(SECONDS / DT)):
            sim.step(cpg.step(DT, x[:-1]), DT)
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
        shaping = 0.15 * float(np.clip(max_x / 0.36, 0.0, 1.0)) + 0.15 * float(np.clip((max_z - 0.04) / 0.08, 0.0, 1.0))            # reach the platform, lift a module over its edge
        return {"score": 0.7 * best + shaping + (1.0 + 0.5 * (1 - t_ok / SECONDS) if ok else 0.0), "success": ok, "t": t_ok}
    finally:
        sim.model.geom_friction[floor, 0] = mu0


def work(args: tuple[int, list[float]]) -> dict:
    """Score a unit-box candidate over the four conditions."""
    n, u = args
    x = LO + np.clip(np.array(u), 0, 1) * (HI - LO)
    rs = []
    for dyaw, mu in CONDS:
        y = x.copy()
        y[-1] += dyaw
        rs.append(rollout(n, y, mu))
    allok = all(r["success"] for r in rs)
    return {"score": float(np.mean([r["score"] for r in rs])) + (1.0 if allok else 0.0), "success": allok, "n_ok": int(sum(r["success"] for r in rs))}


def main() -> int:
    """CLI entry point."""
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=11)
    ap.add_argument("--evals", type=int, default=600)
    ap.add_argument("--library", type=int, default=5)
    ap.add_argument("--workers", type=int, default=9)
    ap.add_argument("--out", type=Path, required=True)
    a = ap.parse_args()
    rows = sorted(json.loads(Path(f"data/rl/cpg_elite_labels_n{a.n}.json").read_text()), key=lambda r: -float(np.hypot(r["v"][0], r["v"][1])))[:a.library]
    starts = [(f"library#{i}", np.array(r["params"] + [-float(np.arctan2(r["v"][1], r["v"][0]))])) for i, r in enumerate(rows)]
    for f in ("forward_n11.json", "roll_travel_n11.json", "left_n11.json"):
        starts.append((f, np.array(json.loads((Path("data/rl/gait_search") / f).read_text())["params"] + [0.0])))
    pool, best, t0 = get_context("spawn").Pool(a.workers), None, time.time()
    for name, x0 in starts:
        es = cma.CMAEvolutionStrategy(((x0 - LO) / (HI - LO)).clip(0, 1).tolist(), 0.15, {"bounds": [0, 1], "popsize": 18, "seed": 1, "verbose": -9, "tolfun": 1e-12, "tolflatfitness": 200})
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
        print(f"[{time.time() - t0:4.0f}s] start '{name}': score {top[0]:.2f}, {top[2]['n_ok']}/4 succeed ({n_ev} evals)", flush=True)
        if best is None or top[0] > best[0]:
            best = (top[0], top[1], top[2], name)
        if best[2]["success"]:
            break
    pool.close()
    x = LO + np.clip(np.array(best[1]), 0, 1) * (HI - LO)
    a.out.parent.mkdir(parents=True, exist_ok=True)
    a.out.write_text(json.dumps({"n_modules": a.n, "success": best[2]["success"], "n_ok": best[2]["n_ok"], "score": best[0], "start": best[3], "params": x[:-1].tolist(), "yaw0": float(x[-1])}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
