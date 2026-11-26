"""Real gap-bridging statics (replaces the weak `arch` proxy of statics.py): two 12 cm platforms with a gap between them; the body starts lying across both, then a held pose (J joint targets,
ramped over 1 s, held 5 s) must keep EVERY module clear of the floor in the gap (module centre > FLOOR_CLEAR above the floor) while settled. Widest gap bridged per length, 0.5 N.m cap, sim only.

    python scripts/rl/gap_statics.py --lengths 10 11 12 13 --gaps 10 15 20 25 30 35 40 --workers 8 --out data/rl/statics/gap.json
Gaps are scanned upward per length and the scan stops after the first failure. Score = lowest module-centre height above the floor (m) minus penalties; success = score >= FLOOR_CLEAR and settled.
"""

from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

import numpy as np
from scipy.optimize import differential_evolution

from petctl.sim.model_builder import BuildParams, PetSim

CAP = 0.5
PLATFORM_H = 0.12                    # the sim's fixed obstacle height
FLOOR_CLEAR = 0.06                   # module centres must stay above this over the gap (floor contact would be ~0.02)
SETTLE_S, RAMP_S, HOLD_S = 0.8, 1.0, 5.0
EVALS = 4800
_SIMS: dict[tuple[int, int], PetSim] = {}


def sim_for(n: int, gap_cm: int) -> PetSim:
    """Cached sim with two platforms (half extents 0.20 x 0.30 m) whose inner edges are ``gap_cm`` apart, centred on x = 0."""
    key = (n, gap_cm)
    if key not in _SIMS:
        g = gap_cm / 100.0
        obs = tuple(("box", sgn * (g / 2 + 0.20), 0.0, 0.0, 0.20, 0.30) for sgn in (-1.0, 1.0))
        _SIMS[key] = PetSim(BuildParams(n_modules=n, cap_nm=(CAP,) * (n - 1), dt=0.004, obstacles=obs))
    return _SIMS[key]


def hold(n: int, gap_cm: int, q: np.ndarray) -> dict:
    """Start across the platforms, ramp to ``q``, hold; returns lowest module height over the gap, settledness, penetration."""
    sim = sim_for(n, gap_cm)
    nj = n - 1
    sim.reset()
    d = sim.data
    d.qpos[2] += PLATFORM_H + 0.004
    sim.step(np.zeros(nj), SETTLE_S)
    dt, steps = 0.05, int((RAMP_S + HOLD_S) / 0.05)
    zs, qd, com, pen = [], [], [], []
    for i in range(steps):
        sim.step(min(1.0, (i + 1) * dt / RAMP_S) * q, dt)
        if not np.all(np.isfinite(d.qpos)):
            return {"ok": False}
        if i * dt >= RAMP_S + HOLD_S - 1.0:
            pos = d.xipos[sim.m0_id:sim.m0_id + n]
            gap_mask = np.abs(pos[:, 0]) < gap_cm / 200.0                      # modules whose centre is above the gap
            zs.append(float(pos[gap_mask, 2].min()) if gap_mask.any() else float(pos[:, 2].min()))
            qd.append(float(np.abs(sim.joint_vel()).max()))
            com.append(pos.mean(axis=0).copy())
            pen.append(float(-d.contact.dist[:d.ncon].min()) if d.ncon else 0.0)
    com = np.array(com)
    pos = d.xipos[sim.m0_id:sim.m0_id + n]
    half = gap_cm / 200.0
    # a real bridge: the two END modules rest on OPPOSITE platforms (beyond the gap edges, above platform height) and some module hangs over the gap
    # (first version let the body sit on one platform with its front cantilevered over the gap: a loophole found by the first search, results kept as gap_INVALID_cantilever.json)
    spans = bool(((pos[0, 0] > half and pos[-1, 0] < -half) or (pos[0, 0] < -half and pos[-1, 0] > half)) and min(pos[0, 2], pos[-1, 2]) > 0.8 * PLATFORM_H
                 and bool((np.abs(pos[:, 0]) < half).any()))
    return {"ok": True, "z_min": min(zs), "settled": max(qd) < 0.05 and float(np.linalg.norm(com[-1] - com[0])) < 0.01, "pen": max(pen), "spans": spans}


def score(n: int, gap_cm: int, q: np.ndarray) -> tuple[float, dict]:
    """(objective to maximise, state)."""
    r = hold(n, gap_cm, q)
    if not r["ok"]:
        return -1.0, r
    val = r["z_min"] - (0.05 if not r["settled"] else 0.0) - (0.05 if r["pen"] > 0.004 else 0.0)
    return (val if r["spans"] else -0.5 + 0.1 * min(val, 0.0)), r


class Cost:
    """Picklable negative objective."""

    def __init__(self, n: int, gap_cm: int) -> None:
        self.n, self.gap = n, gap_cm

    def __call__(self, q: np.ndarray) -> float:
        return -score(self.n, self.gap, q)[0]


def run(n: int, gap_cm: int, seed: int, workers: int) -> dict:
    """One DE search for a given length and gap."""
    dim = n - 1
    pop = max(1, EVALS // (40 * dim))
    r = differential_evolution(Cost(n, gap_cm), [(-math.pi, math.pi)] * dim, maxiter=40, popsize=pop, seed=seed, tol=1e-9, polish=False, workers=workers, updating="deferred", init="sobol")
    val, st = score(n, gap_cm, r.x)
    ok = bool(st.get("settled")) and st.get("z_min", 0.0) >= FLOOR_CLEAR and st.get("pen", 1.0) <= 0.004 and bool(st.get("spans"))
    return {"n_modules": n, "gap_cm": gap_cm, "seed": seed, "objective": float(val), "z_min_cm": 100 * float(st.get("z_min", 0.0)), "bridged": ok, "q": [float(x) for x in r.x],
            "settled": bool(st.get("settled")), "spans": bool(st.get("spans")), "pen_mm": 1000 * float(st.get("pen", 0.0)), "evals": int(r.nfev)}


def main() -> int:
    """CLI entry point."""
    ap = argparse.ArgumentParser()
    ap.add_argument("--lengths", type=int, nargs="+", default=[10, 11, 12, 13])
    ap.add_argument("--gaps", type=int, nargs="+", default=[10, 15, 20, 25, 30, 35, 40])
    ap.add_argument("--seeds", type=int, nargs="+", default=[0])
    ap.add_argument("--workers", type=int, default=8)
    ap.add_argument("--out", type=Path, default=Path("data/rl/statics/gap.json"))
    a = ap.parse_args()
    rows = json.loads(a.out.read_text()) if a.out.exists() else []
    done = {(r["n_modules"], r["gap_cm"], r["seed"]): r for r in rows}
    t0 = time.time()
    for n in a.lengths:
        for seed in a.seeds:
            for g in a.gaps:
                rec = done.get((n, g, seed))
                if rec is None:
                    rec = run(n, g, seed, a.workers)
                    rows.append(rec)
                    a.out.parent.mkdir(parents=True, exist_ok=True)
                    a.out.write_text(json.dumps(rows, indent=1))
                    print(f"[{time.time() - t0:5.0f}s] n={n} gap={g} cm seed={seed}: lowest module {rec['z_min_cm']:.1f} cm bridged={rec['bridged']} settled={rec['settled']}", flush=True)
                if not rec["bridged"]:
                    break                                                         # first failure: stop scanning wider gaps for this length / seed
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
