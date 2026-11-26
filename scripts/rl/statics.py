"""Static / quasi-static feasibility per skill and body length at the 0.5 N.m cap (Phase 3). Sim only.

    python scripts/rl/statics.py --tasks head_lift arch self_right --lengths 10 11 12 13 --seeds 0 1 --workers 8 --out data/rl/statics

A pose is a vector of J joint targets (rad; geometry is 2 pi periodic, so the search box is [-pi, pi]) commanded as a 1 s ramp and held; it counts only if it is SETTLED
(joint speed < 0.05 rad/s and centre-of-mass speed < 1 cm/s over the last second), has no module-module / module-floor penetration deeper than 4 mm, and the actuator cap (0.5 N.m) is
the real limit (a pose that needs more torque sags and is scored where it settles). Tasks:
    head_lift   highest held head-site height above its flat-resting height (cm)            -> step-up / reach upper bound
    arch        widest held span between the head-end and tail-end module while every middle module is >= 3 cm clear of the floor (cm) -> gap-bridge upper bound
    self_right  best "uprightness" reached from an inverted / side start by commanding ONE held pose: mean cos(angle(module z axis, rest z axis)) over all modules (1 = fully upright);
                a quasi-static self-righting exists if >= 0.7 for every module and settled
Equal budget per length (fixed number of evaluations); the current proxy of every best pose (GIM3505-8 model, hold) is reported. Cylinder wrap is done by scripts/rl/m1d_odd_length_feasibility.py.
"""

from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

import numpy as np
from scipy.optimize import differential_evolution

from petctl.config import GIM3505_HOLD_A_PER_NM2, GIM3505_IDLE_PER_MOTOR_A
from petctl.sim.bus import IDLE_ELECTRONICS_A
from petctl.sim.model_builder import BuildParams, PetSim

CAP = 0.5
SETTLE_S, RAMP_S, HOLD_S = 0.8, 1.0, 5.0
PEN_LIMIT_M = 0.004
EVALS = 4800                 # DE evaluations per (task, length, seed): popsize x dim fixed so every length gets the same budget
_SIMS: dict[int, PetSim] = {}
_REST: dict[int, tuple[np.ndarray, np.ndarray, float]] = {}


def sim_for(n: int) -> PetSim:
    """One cached sim per length per process."""
    if n not in _SIMS:
        _SIMS[n] = PetSim(BuildParams(n_modules=n, cap_nm=(CAP,) * (n - 1), dt=0.004))
    return _SIMS[n]


def hold(n: int, q: np.ndarray, orient: int = 0) -> dict:
    """Command pose ``q`` (ramped over ``RAMP_S``) from a flat start (or an inverted / side start), hold it, and return the measured state of the last second."""
    sim = sim_for(n)
    nj = n - 1
    if n not in _REST:                                                # reference: the upright flat rest state of this body
        sim.reset()
        sim.step(np.zeros(nj), SETTLE_S)
        d0 = sim.data
        _REST[n] = (d0.xipos[sim.m0_id:sim.m0_id + n, 2].copy(), np.array([d0.xmat[sim.m0_id + k].reshape(3, 3)[:, 2] for k in range(n)]), float(d0.site_xpos[sim.head_id][2]))
    rest_z, rest_up, head_rest = _REST[n]
    sim.reset()
    if orient:
        roll = {1: math.pi, 2: math.pi / 2, 3: -math.pi / 2}[orient]
        sim.data.qpos[3:7] = [math.cos(roll / 2), math.sin(roll / 2), 0.0, 0.0]
        sim.data.qpos[2] += 0.04
    sim.step(np.zeros(nj), SETTLE_S)
    d, m = sim.data, sim.model
    dt, steps = 0.05, int((RAMP_S + HOLD_S) / 0.05)
    rec = {"head": [], "qd": [], "com": [], "pen": [], "tau": []}
    for i in range(steps):
        a = min(1.0, (i + 1) * dt / RAMP_S)
        sim.step(a * q, dt)
        if not np.all(np.isfinite(d.qpos)):
            return {"ok": False}
        if i * dt >= RAMP_S + HOLD_S - 1.0:
            rec["head"].append(float(d.site_xpos[sim.head_id][2]))
            rec["qd"].append(float(np.abs(sim.joint_vel()).max()))
            rec["com"].append(d.xipos[sim.m0_id:sim.m0_id + n].mean(axis=0).copy())
            rec["pen"].append(float(-d.contact.dist[:d.ncon].min()) if d.ncon else 0.0)
            rec["tau"].append(sim.joint_torque().copy())
    com = np.array(rec["com"])
    com_speed = float(np.linalg.norm(com[-1] - com[0])) / 1.0
    pos = d.xipos[sim.m0_id:sim.m0_id + n].copy()
    up = np.array([d.xmat[sim.m0_id + k].reshape(3, 3)[:, 2] for k in range(n)])
    tau = np.array(rec["tau"])
    settled = max(rec["qd"]) < 0.05 and com_speed < 0.01
    return {"ok": True, "settled": bool(settled), "pen": max(rec["pen"]), "head_z": min(rec["head"]), "head_gain": min(rec["head"]) - head_rest, "pos": pos, "rest_z": rest_z,
            "cos_up": np.einsum("ki,ki->k", up, rest_up), "tau_peak": float(np.abs(tau).max()), "tau_rms": float(np.sqrt(np.mean(tau ** 2))),
            "current_a": IDLE_ELECTRONICS_A + GIM3505_IDLE_PER_MOTOR_A * nj + GIM3505_HOLD_A_PER_NM2 * float(np.mean(np.sum(tau ** 2, axis=1)))}


def score(task: str, n: int, q: np.ndarray) -> tuple[float, dict]:
    """(objective to MAXIMISE, measured state) of pose ``q`` for ``task``."""
    orient = {"head_lift": 0, "arch": 0, "self_right_inv": 1, "self_right_side": 2}[task]
    r = hold(n, q, orient)
    if not r["ok"]:
        return -10.0, r
    bad = (not r["settled"]) or r["pen"] > PEN_LIMIT_M
    if task == "head_lift":
        val = r["head_gain"]
    elif task == "arch":
        pos, rest = r["pos"], r["rest_z"]
        mid = slice(n // 4, n - n // 4)
        clear = float(np.min(pos[mid, 2] - rest[mid]))
        ends_ok = max(0.0, float(max(pos[0, 2] - rest[0], pos[-1, 2] - rest[-1])) - 0.015)      # both ends should rest on the floor
        span = float(np.linalg.norm(pos[0, :2] - pos[-1, :2]))
        val = span - 3.0 * max(0.0, 0.03 - clear) - 5.0 * ends_ok
    else:
        val = float(np.mean(r["cos_up"])) if float(np.min(r["cos_up"])) > -2 else -1.0
    return (val - 1.0 if bad else val), r


class Cost:
    """Picklable negative objective (worker processes are spawned on macOS)."""

    def __init__(self, task: str, n: int) -> None:
        self.task, self.n = task, n

    def __call__(self, q: np.ndarray) -> float:
        return -score(self.task, self.n, q)[0]


def run(task: str, n: int, seed: int, workers: int) -> dict:
    """One DE search with a fixed evaluation budget; returns the record of the best pose."""
    dim = n - 1
    pop = max(1, EVALS // (40 * dim))                      # popsize multiplier; maxiter 40 -> about EVALS evaluations whatever the length
    r = differential_evolution(Cost(task, n), [(-math.pi, math.pi)] * dim, maxiter=40, popsize=pop, seed=seed, tol=1e-9, polish=False, workers=workers, updating="deferred", init="sobol")
    val, st = score(task, n, r.x)
    rec = {"task": task, "n_modules": n, "seed": seed, "objective": float(val), "q": [float(x) for x in r.x], "settled": bool(st.get("settled")), "pen_mm": 1000 * float(st.get("pen", 0.0)),
           "tau_peak": st.get("tau_peak"), "current_a": st.get("current_a"), "evals": int(r.nfev)}
    if task == "head_lift":
        rec["head_gain_cm"] = 100 * float(st["head_gain"])
    elif task == "arch":
        pos, rest = st["pos"], st["rest_z"]
        rec["span_cm"] = 100 * float(np.linalg.norm(pos[0, :2] - pos[-1, :2]))
        rec["mid_clear_cm"] = 100 * float(np.min(pos[n // 4:n - n // 4, 2] - rest[n // 4:n - n // 4]))
    else:
        rec["cos_up_min"], rec["cos_up_mean"] = float(np.min(st["cos_up"])), float(np.mean(st["cos_up"]))
    return rec


def main() -> int:
    """CLI entry point."""
    ap = argparse.ArgumentParser()
    ap.add_argument("--tasks", nargs="+", default=["head_lift", "arch", "self_right_inv", "self_right_side"])
    ap.add_argument("--lengths", type=int, nargs="+", default=[10, 11, 12, 13])
    ap.add_argument("--seeds", type=int, nargs="+", default=[0, 1])
    ap.add_argument("--workers", type=int, default=8)
    ap.add_argument("--out", type=Path, default=Path("data/rl/statics"))
    a = ap.parse_args()
    a.out.mkdir(parents=True, exist_ok=True)
    f = a.out / "results.json"
    rows = json.loads(f.read_text()) if f.exists() else []
    done = {(r["task"], r["n_modules"], r["seed"]) for r in rows}
    t0 = time.time()
    for task in a.tasks:
        for n in a.lengths:
            for seed in a.seeds:
                if (task, n, seed) in done:
                    continue
                rec = run(task, n, seed, a.workers)
                rows.append(rec)
                f.write_text(json.dumps(rows, indent=1))
                key = rec.get("head_gain_cm", rec.get("span_cm", rec.get("cos_up_mean")))
                print(f"[{time.time() - t0:5.0f}s] {task:15s} n={n} seed={seed} obj={rec['objective']:+.3f} main={key:.2f} settled={rec['settled']} pen={rec['pen_mm']:.1f}mm I={rec['current_a']:.2f}A", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
