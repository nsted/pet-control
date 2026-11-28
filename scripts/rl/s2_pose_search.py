"""T17: CMA-ES search for HOLDABLE lifted poses for S2 (n = 11, upright, sim only). q* is the pose measured after entry (reached by construction), held by the kp 3.2 reference PD.

    python scripts/rl/s2_pose_search.py --task head_lift --minutes 25 --out data/rl/s2/search/head_lift.json
Objective: head_lift = head-height gain over straight (need >= 5 cm); arch = height of the highest module over straight (need >= 8 cm). Constraints (penalties): last-2-s tau_peak <= 0.32 N.m at the L2 DR
corner (mass x1.2, derate 0.8, cap 0.4), E < 0.1 at nominal and at the corner, no loss. No joint angle limits (standing rule); sigma limits the step only.
"""

from __future__ import annotations

import argparse
import json
import time
from multiprocessing import Pool
from pathlib import Path

import cma
import numpy as np

from petctl.rl.gait_eval import build_sim
from petctl.rl.s2.cells import HOLD_TAU_FRAC, L2_WORST, N_MODULES, NJ, Pose, corner_check, enter_two_phase

_SIM = None
_Z0 = None
TAU_MAX = HOLD_TAU_FRAC * 0.5 * L2_WORST["derate"]


def _sim():
    global _SIM, _Z0
    if _SIM is None:
        _SIM = build_sim(N_MODULES)
        enter_two_phase(_SIM, Pose("straight", np.zeros(NJ), None), "upright")
        _Z0 = (float(_SIM.data.xipos[_SIM.m0_id, 2]), float(_SIM.data.xipos[_SIM.m0_id:_SIM.m0_id + N_MODULES, 2].max()))
    return _SIM


def height(sim, task: str) -> float:
    """Metric in m: head height (head_lift) or highest module centre (arch), over the straight pose."""
    z = sim.data.xipos[sim.m0_id:sim.m0_id + N_MODULES, 2]
    return float((z[0] - _Z0[0]) if task == "head_lift" else (z.max() - _Z0[1]))


def evaluate(args: tuple[np.ndarray, str]) -> dict:
    """Score one candidate vector; returns metrics and the penalised fitness (lower is better)."""
    q, task = args
    sim = _sim()
    p = Pose("cand", np.asarray(q, float), None)
    qs, errs = enter_two_phase(sim, p, "upright")
    e_nom = float(np.array(errs[-20:]).mean())
    lost_nom = bool(np.array(errs[-20:]).max() > 0.6)
    tau_nom = float(getattr(sim, "_s2_tau", 0.0))
    h = height(sim, task)
    ok_c, tau_c, e_c = corner_check(p, "upright", sim, qs)
    pen = 50.0 * max(0.0, tau_c - TAU_MAX) + 10.0 * max(0.0, e_nom - 0.1) + 10.0 * max(0.0, e_c - 0.1) + (5.0 if lost_nom else 0.0) + 20.0 * max(0.0, tau_nom - TAU_MAX)
    return {"fit": -h + pen, "h": h, "tau_nom": tau_nom, "tau_c": tau_c, "e_nom": e_nom, "e_c": e_c, "qstar": [float(x) for x in qs], "q": [float(x) for x in q]}


def main() -> int:
    """CLI entry point."""
    ap = argparse.ArgumentParser()
    ap.add_argument("--task", choices=("head_lift", "arch"), required=True)
    ap.add_argument("--minutes", type=float, default=25.0)
    ap.add_argument("--procs", type=int, default=8)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--out", type=Path, required=True)
    a = ap.parse_args()
    res = json.loads(Path("data/rl/statics/results.json").read_text())
    x0 = np.array(next(r for r in res if r["task"] == a.task and r["n_modules"] == N_MODULES and r["seed"] == 0)["q"], float) * (0.3 if a.task == "head_lift" else 0.5)
    es = cma.CMAEvolutionStrategy(x0, 0.25, {"seed": a.seed + 1, "popsize": 16, "verbose": -9})
    t0, best, gen = time.time(), None, 0
    with Pool(a.procs) as pool:
        while time.time() - t0 < a.minutes * 60:
            xs = es.ask()
            outs = pool.map(evaluate, [(x, a.task) for x in xs])
            es.tell(xs, [o["fit"] for o in outs])
            cur = min(outs, key=lambda o: o["fit"])
            if best is None or cur["fit"] < best["fit"]:
                best = cur
            gen += 1
            ok = best["tau_c"] <= TAU_MAX and best["e_nom"] < 0.1 and best["e_c"] < 0.1
            print(f"gen {gen} t={time.time() - t0:.0f}s best h={best['h'] * 100:.1f} cm tau_c={best['tau_c']:.2f} e_nom={best['e_nom']:.3f} e_c={best['e_c']:.3f} feasible={ok}", flush=True)
            a.out.write_text(json.dumps({"task": a.task, "gen": gen, "best": best, "tau_max": TAU_MAX, "feasible": ok}, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
