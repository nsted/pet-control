"""T14: S2 gate baseline. Zero-delta PD through the frozen PetEnv evaluator: 18 configs (kp in {0.2, 0.4, 0.8, 1.6, 3.2, 5.0} x zeta_m in {0.5, 1, 2}, kd = kd_ref sqrt(kp / kp_ref) x zeta_m) on the
NON-yield decision and test suites at L0 / L2 / L3. Best PD per (cell, level) = lowest sum of E over the cell's non-yield DECISION scenarios among configs that hold both budgets in every scenario; its E and I
are re-measured on the test suite of the same level. Writes data/rl/s2/baseline.json (the file the watchdog reads). Sim only.

    python scripts/rl/s2_baseline.py --levels L0 L2 L3 --procs 8
"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import numpy as np

from petctl.rl.s2 import suite
from petctl.rl.s2.evaluator import run_pd_suite

KPS = (0.2, 0.4, 0.8, 1.6, 3.2, 5.0)
ZETAS = (0.5, 1.0, 2.0)
MEAN_BUDGET_A, PEAK_BUDGET_A = 4.0, 10.0


def main() -> int:
    """CLI entry point."""
    ap = argparse.ArgumentParser()
    ap.add_argument("--levels", nargs="+", default=["L0", "L2", "L3"])
    ap.add_argument("--procs", type=int, default=8)
    ap.add_argument("--out", type=Path, default=Path("data/rl/s2/baseline.json"))
    a = ap.parse_args()
    out: dict = {"grid": {"kp": KPS, "zeta": ZETAS}, "levels": {}}
    t0 = time.time()
    for lv in a.levels:
        out["levels"][lv] = {}
        for split in suite.SPLITS:
            scs = suite.load(split, lv, "nonyield")
            res = {}
            for kp in KPS:
                for z in ZETAS:
                    rs = run_pd_suite(scs, kp, z, procs=a.procs)
                    res[f"{kp}|{z}"] = [{k: v for k, v in r.items() if k != "applied"} for r in rs]
                    print(f"{lv} {split} kp {kp} z {z}: mean E {np.mean([r['E'] for r in rs]):.3f} lost {np.mean([r['lost'] for r in rs]):.2f} t={time.time() - t0:.0f}s", flush=True)
            out["levels"][lv][split] = {"scenarios": [s.id for s in scs], "cells": [[s.pose, s.orient] for s in scs], "configs": res}
        a.out.write_text(json.dumps(out))
    # best PD per cell and level (selected on the decision suite)
    best: dict = {}
    for lv, d in out["levels"].items():
        dec = d["decision"]
        cells = sorted({tuple(c) for c in dec["cells"]})
        best[lv] = {}
        for cell in cells:
            idx = [i for i, c in enumerate(dec["cells"]) if tuple(c) == cell]
            cand = []
            for key, rs in dec["configs"].items():
                sel = [rs[i] for i in idx]
                ok = all(r["I_max_2s"] <= MEAN_BUDGET_A and r["I_peak_tick"] <= PEAK_BUDGET_A for r in sel)
                cand.append((sum(r["E"] for r in sel), key, ok))
            ok_c = [c for c in cand if c[2]]
            pick = min(ok_c or cand)
            best[lv]["|".join(cell)] = {"config": pick[1], "decision_sumE": pick[0], "budgets_ok": bool(ok_c), "all_configs_sumE": {c[1]: c[0] for c in cand}}
    out["best_pd"] = best
    a.out.write_text(json.dumps(out))
    print("wrote", a.out, "t=%.0fs" % (time.time() - t0))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
