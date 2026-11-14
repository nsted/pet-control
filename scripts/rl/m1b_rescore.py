"""M1b: re-score the M1 best poses and scripted coils under the proximity hug definition (15 mm, 3 s window, 90 % duty), and run the dynamic-hold coil sweep.

Run: python scripts/rl/m1b_rescore.py [--rescore] [--sweep] [--workers 8]. Writes data/rl/m1_hug_feasibility/m1b_rescore.json and m1b_sweep.json (sim only).
"""
from __future__ import annotations

import argparse
import json
import multiprocessing as mp
from dataclasses import asdict
from pathlib import Path

import numpy as np

from petctl.rl.hug_search import EvalResult, HugSearchSim, _sim_for
from petctl.rl.hug import hug_threshold

OUT = Path("data/rl/m1_hug_feasibility")
SWEEP_N = (8, 10, 12)
SWEEP_ANGLES = tuple(round(float(a), 2) for a in np.arange(0.20, 1.41, 0.05))
SWEEP_ALPHAS = (0.7, 0.85, 0.95)


def summarise(r: EvalResult, n: int) -> dict[str, float | int | bool]:
    """The numbers reported per evaluation."""
    return {"n": n, "H": hug_threshold(n), "min_run": r.held_modules, "median_run": r.run_median, "mean_run": round(r.mean_modules, 2), "duty": round(r.duty, 3), "hold_ok": r.hold_ok,
            "bus_mean_a": round(r.bus_mean_a, 2), "bus_applied_peak_a": round(r.bus_applied_peak_a, 1), "pen_mm": round(1000 * r.pen_max_m, 1), "feasible": r.feasible,
            "wrap_deg": round(r.wrap_deg, 0)}


def _eval(args: tuple[int, list[float]]) -> dict:
    n, x = args
    r = _sim_for(n).evaluate(np.asarray(x))
    return summarise(r, n)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--rescore", action="store_true")
    ap.add_argument("--sweep", action="store_true")
    ap.add_argument("--fine12", action="store_true", help="finer angle x alpha sweep at n=12")
    ap.add_argument("--workers", type=int, default=8)
    a = ap.parse_args()
    pool = mp.get_context("spawn").Pool(a.workers)
    if a.rescore:
        rows = json.loads((OUT / "table.json").read_text())["rows"]
        jobs = [(r["n_modules"], r["x"]) for r in rows]
        for alpha in (0.7, 0.85, 0.95, 1.0):                                                  # scripted 11-module coil, all joints +0.88
            jobs.append((11, [0.88] * 10 + [0.0, 0.0, alpha]))
        res = pool.map(_eval, jobs)
        out = [{"label": f"M1 best n={n}" if i < len(rows) else f"scripted coil alpha={jobs[i][1][-1]}", **r} for i, ((n, _), r) in enumerate(zip(jobs, res))]
        (OUT / "m1b_rescore.json").write_text(json.dumps(out, indent=1))
        for o in out:
            print(o)
    if a.sweep:
        jobs = [(n, [ang] * (n - 1) + [0.0, 0.0, al]) for n in SWEEP_N for ang in SWEEP_ANGLES for al in SWEEP_ALPHAS]
        res = pool.map(_eval, jobs)
        out = [{"angle": j[1][0], "alpha": j[1][-1], **r} for j, r in zip(jobs, res)]
        (OUT / "m1b_sweep.json").write_text(json.dumps(out, indent=1))
        for n in SWEEP_N:
            best = max((o for o in out if o["n"] == n), key=lambda o: (o["duty"], o["median_run"]))
            print("best", n, best)
    if a.fine12:
        jobs = [(12, [float(ang)] * 11 + [0.0, 0.0, al]) for ang in np.arange(0.70, 1.13, 0.025) for al in (0.86, 0.9, 0.92, 0.94, 0.96, 0.98)]
        res = pool.map(_eval, jobs)
        out = [{"angle": round(j[1][0], 3), "alpha": j[1][-1], **r} for j, r in zip(jobs, res)]
        (OUT / "m1b_sweep_n12_fine.json").write_text(json.dumps(out, indent=1))
        print(len(out), "evals; hold_ok", sum(o["hold_ok"] for o in out), "hold_ok and feasible", sum(o["hold_ok"] and o["feasible"] for o in out))
        for o in sorted(out, key=lambda o: (-o["duty"], -o["median_run"]))[:8]:
            print(o)
        print("max feasible run:", max((o["median_run"], o["angle"], o["alpha"], o["bus_mean_a"]) for o in out if o["feasible"]))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
