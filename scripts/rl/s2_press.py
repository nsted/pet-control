"""T14 (yield part): calibrate the press per yield cell, then run the yield-PD feasibility check (STAGE_2_GATES, Yield PD check). Dry run unless --write (section B is write-once).

Press = the smallest downward force on the head at which the L2 best PD of the cell saturates (|tau| >= 0.95 x derated cap) at least 3 joints during the overload. The yield PD (best PD, the 3 loaded joints at kp 0.2
while the privileged overload flag is on) must reach Y >= 0.80 on the L2 decision yield draws.

    python scripts/rl/s2_press.py [--write]
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from petctl.rl.config import KP_SCALE_MAX, KP_SCALE_MIN
from petctl.rl.s2 import suite
from petctl.rl.s2.cells import LIFTED
from petctl.rl.s2.evaluator import NJ, pd_policy, run_scenario

KP_REF = 0.8
NOMINAL = {"friction": 1.0, "mass": 1.0, "derate": 1.0, "delay_ticks": 1, "noise_mult": 1.0, "zeta_m": 1.0, "start_t_coil": 30.0, "head_mass": 1.0}


def a_of_kp(kp: float) -> float:
    """Inverse of the exponential kp map: the kp action in [-1, 1] that gives stiffness ``kp``."""
    return float(2.0 * np.log((kp / KP_REF) / KP_SCALE_MIN) / np.log(KP_SCALE_MAX / KP_SCALE_MIN) - 1.0)


def best_pd(cell: tuple[str, str]) -> tuple[float, float]:
    """(kp, zeta) of the L2 best PD of ``cell`` chosen on the decision suite (budget-respecting, lowest sum E)."""
    d = json.loads(Path("data/rl/s2/baseline_L0L2.json").read_text())["levels"]["L2"]["decision"]
    idx = [i for i, c in enumerate(d["cells"]) if tuple(c) == cell]
    cand = []
    for key, rs in d["configs"].items():
        sel = [rs[i] for i in idx]
        ok = all(r["I_max_2s"] <= 4.0 and r["I_peak_tick"] <= 10.0 for r in sel)
        cand.append((sum(r["E"] for r in sel), key, ok))
    key = min([c for c in cand if c[2]] or cand)[1]
    kp, z = key.split("|")
    return float(kp), float(z)


def yield_policy(kp: float, loaded: list[int], kp_low: float = 0.2):
    """Best PD gains through the kp channels; the loaded joints drop to ``kp_low`` while the overload flag is on."""
    base = a_of_kp(kp)
    low = a_of_kp(kp_low)

    def pol(obs, env, t):
        a = np.zeros(2 * NJ)
        a[NJ:] = base
        if env.task.overload():
            a[NJ + np.array(loaded)] = low
        return a
    return pol


def scenario(cell: tuple[str, str], press: float, mult: float = 1.0, level: str = "L2") -> dict:
    """A nominal-plant yield scenario at ``mult`` x ``press`` (calibration only; the frozen draws come from the suite)."""
    return {"id": f"press-{cell[0]}-{cell[1]}-{press:.2f}", "split": "decision", "level": level, "pose": cell[0], "orient": cell[1], "family": "yield", "seed": 7, "onset_s": 4.0, "duration_s": 3.0, "body": "head",
            "axis": "-z", "mag": 1.0, "tilt_axis_deg": 0.0, "load_kind": "force", "derate_x": 1.0, "yield_mult": mult, "press_n": press, "plant": dict(NOMINAL)}


def main() -> int:
    """Calibrate, check, optionally write section B."""
    ap = argparse.ArgumentParser()
    ap.add_argument("--write", action="store_true")
    a = ap.parse_args()
    cells = [tuple(c) for c in json.loads((suite.SUITE_DIR / "cells.json").read_text())["cells"] if c[0] in LIFTED]
    table: dict[str, dict] = {}
    for cell in cells:
        kp, z = best_pd(cell)
        lo, hi = 0.2, 60.0
        for _ in range(9):                                                  # bisection on the force
            mid = 0.5 * (lo + hi)
            r = run_scenario(scenario(cell, mid), pd_policy, fixed_kp=kp, zeta=z, check=False)
            lo, hi = (lo, mid) if r["sat_joints_overload_max"] >= 3 else (mid, hi)
        press = hi
        r = run_scenario(scenario(cell, press), pd_policy, fixed_kp=kp, zeta=z, check=False)
        table[f"{cell[0]}|{cell[1]}"] = {"press_n": press, "loaded_joints": r["loaded_joints"], "best_pd": [kp, z], "sat_joints": r["sat_joints_overload_max"]}
        print(cell, "best PD", (kp, z), "press", round(press, 2), "N loaded", r["loaded_joints"], "sat", r["sat_joints_overload_max"], flush=True)
    # yield PD check on the frozen L2 decision draws (press from the table)
    ys, yb = [], []
    for s in suite.load("decision", "L2", "yield_draws"):
        t = table[f"{s.pose}|{s.orient}"]
        d = s.to_dict() | {"press_n": t["press_n"]}
        kp, z = t["best_pd"]
        ry = run_scenario(d, yield_policy(kp, t["loaded_joints"]), fixed_kp=0.0, zeta=z, check=False)
        rb = run_scenario(d, pd_policy, fixed_kp=kp, zeta=z, check=False)
        ys.append(ry["Y"])
        yb.append(rb["Y"])
        print(f"{s.id:50s} mult {s.yield_mult} Y yieldPD {ry['Y']:.2f} Y bestPD {rb['Y']:.2f} return {ry['return']:.0f} vs {rb['return']:.0f}", flush=True)
    print(f"mean Y: yield PD {np.mean(ys):.3f}  best PD {np.mean(yb):.3f}  (gate: yield PD >= 0.80, and >= best PD + 0.15)")
    Path("data/rl/s2/press_table.json").write_text(json.dumps({"table": table, "Y_yield_pd": float(np.mean(ys)), "Y_best_pd": float(np.mean(yb))}, indent=1))
    if a.write:
        suite.write_yield_b({k: {"press_n": v["press_n"], "loaded_joints": v["loaded_joints"]} for k, v in table.items()})
        print("section B written")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
