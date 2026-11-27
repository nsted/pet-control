"""Dynamic gap crossing (Phase 4 target): the body starts fully on one 12 cm platform and must END fully on the other (every module over the far platform, on top of it), crossing a gap by locomotion.

    python scripts/rl/gap_cross.py --n 11 --gap 10 --evals 600 --out data/rl/gap_cross/n11_g10.json
12 search parameters: the 11 CPG parameters + the start heading (yaw about the vertical, so any travel direction of the gait can point across the gap). CMA-ES, three starts seeded from the
MAP-Elites archive of the same length (best forward, best sideways, best rolling-with-travel cell; the sideways/rolling starts are rotated to point +x), 25 s rollouts.
Score (maximised): far-end progress = (x of the rearmost module) - (gap / 2), so a partial crossing still gets a gradient; success = rearmost module beyond the far edge, every module higher than 0.09 m
at the end (no module fell into the gap), no module off the platform sides. Platforms: 1.0 m long, 1.2 m wide.
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

CAP, H, SECONDS, DT = 0.5, 0.12, 25.0, 0.05
LO = np.array([PARAM_BOUNDS[k][0] for k in PARAM_NAMES] + [-math.pi])
HI = np.array([PARAM_BOUNDS[k][1] for k in PARAM_NAMES] + [math.pi])
_SIMS: dict = {}


def sim_for(n: int, gap_cm: int) -> PetSim:
    """Cached sim with two platforms (x in [g/2, g/2+1.0] and mirrored; half width 0.6 m)."""
    if (n, gap_cm) not in _SIMS:
        g = gap_cm / 100.0
        obs = tuple(("box", s * (g / 2 + 0.5), 0.0, 0.0, 0.5, 0.6) for s in (-1.0, 1.0))
        _SIMS[(n, gap_cm)] = PetSim(BuildParams(n_modules=n, cap_nm=(CAP,) * (n - 1), dt=0.004, obstacles=obs))
    return _SIMS[(n, gap_cm)]


def rollout(n: int, gap_cm: int, x: np.ndarray, record: bool = False) -> dict:
    """Run one candidate (parameters + start yaw); returns score, success and diagnostics (and the module positions per tick if ``record``)."""
    sim = sim_for(n, gap_cm)
    nj, g = n - 1, gap_cm / 100.0
    sim.reset()
    d = sim.data
    yaw = float(x[-1])
    d.qpos[0], d.qpos[1], d.qpos[2] = -(g / 2 + 0.55), 0.0, d.qpos[2] + H + 0.004
    d.qpos[3:7] = [math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)]
    sim.step(np.zeros(nj), 0.8)
    cpg = CPG(nj, smooth_hz=0.0)
    cpg.reset(x[:-1])
    frames, success, t_ok, best_rear = [], False, SECONDS, -9.0
    for i in range(int(SECONDS / DT)):
        sim.step(cpg.step(DT, x[:-1]), DT)
        if not np.all(np.isfinite(d.qpos)):
            return {"score": -1.0, "success": False}
        p = d.xipos[sim.m0_id:sim.m0_id + n]
        if record:
            frames.append(p.copy())
        top = bool((p[:, 2] > 0.09).all()) and bool((np.abs(p[:, 1]) < 0.6).all())
        if top:
            best_rear = max(best_rear, float(p[:, 0].min()))
        if top and p[:, 0].min() > g / 2 + 0.01 and p[:, 0].max() < g / 2 + 1.0:             # every module on top of the far platform
            success, t_ok = True, i * DT
            if not record:
                break
    score = min(best_rear - g / 2, 0.3) + (1.0 + 0.5 * (1 - t_ok / SECONDS) if success else 0.0)
    p = d.xipos[sim.m0_id:sim.m0_id + n]
    rear = float(p[:, 0].min())
    out = {"score": score, "success": success, "t_cross_s": t_ok, "best_rear_cm": 100 * best_rear, "min_z_cm": 100 * float(p[:, 2].min())}
    if record:
        out["frames"] = frames
    return out


def work(args: tuple[int, int, list[float]]) -> dict:
    """Pool worker: unit-box candidate -> rollout summary."""
    n, gap, u = args
    return rollout(n, gap, LO + np.clip(np.array(u), 0, 1) * (HI - LO))


def starts(n: int, archive_dir: Path) -> list[tuple[str, np.ndarray]]:
    """Three seeded starts from the MAP-Elites archive of this length (params + start yaw so the travel direction points +x)."""
    arc = json.loads((archive_dir / f"n{n}_s0_v2.json").read_text())["cells"]
    pick = lambda f, key: max((c for c in arc if f(c["metrics"])), key=lambda c: key(c["metrics"]))          # noqa: E731
    out = []
    for name, c in (("forward", pick(lambda m: abs(m["roll_rate"]) < 0.5, lambda m: m["vx"])), ("sideways", pick(lambda m: abs(m["roll_rate"]) < 0.5, lambda m: m["vy"])),
                    ("rolling", pick(lambda m: abs(m["roll_rate"]) > 2.0, lambda m: m["speed"]))):
        m = c["metrics"]
        out.append((name, np.array(c["params"] + [-math.atan2(m["vy"], m["vx"])])))                     # rotate so the gait's travel direction points +x
    return out


def main() -> int:
    """CLI entry point."""
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=11)
    ap.add_argument("--gap", type=int, default=10)
    ap.add_argument("--evals", type=int, default=600)
    ap.add_argument("--workers", type=int, default=9)
    ap.add_argument("--archive", type=Path, default=Path("data/rl/map_elites"))
    ap.add_argument("--out", type=Path, required=True)
    a = ap.parse_args()
    pool = get_context("spawn").Pool(a.workers)
    best, t0, runs = None, time.time(), []
    for name, x0 in starts(a.n, a.archive):
        es = cma.CMAEvolutionStrategy(((x0 - LO) / (HI - LO)).clip(0, 1).tolist(), 0.15, {"bounds": [0, 1], "popsize": 18, "seed": 1, "verbose": -9})
        n_ev, top = 0, None
        while n_ev < a.evals and not es.stop():
            c = es.ask()
            res = pool.map(work, [(a.n, a.gap, ci.tolist()) for ci in c])
            es.tell(c, [-r["score"] for r in res])
            n_ev += len(c)
            i = int(np.argmax([r["score"] for r in res]))
            if top is None or res[i]["score"] > top[0]:
                top = (res[i]["score"], c[i].tolist(), res[i])
            if top[2]["success"]:
                break
        runs.append({"start": name, "evals": n_ev, "score": top[0], "success": top[2]["success"]})
        print(f"[{time.time() - t0:4.0f}s] n={a.n} gap={a.gap} cm start={name}: score {top[0]:.2f} success={top[2]['success']} ({n_ev} evals)", flush=True)
        if best is None or top[0] > best[0]:
            best = (top[0], top[1], top[2], name)
    pool.close()
    x = LO + np.clip(np.array(best[1]), 0, 1) * (HI - LO)
    a.out.parent.mkdir(parents=True, exist_ok=True)
    a.out.write_text(json.dumps({"n_modules": a.n, "gap_cm": a.gap, "success": best[2]["success"], "score": best[0], "start": best[3], "params": x[:-1].tolist(), "yaw0": float(x[-1]), "runs": runs}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
