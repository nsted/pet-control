"""T6: choose the S2 cells (rev. 9 pose rules), cache their settled states, apply the limp suite-validity rule at L2, then freeze SHA set A (STAGE_2_GATES). Sim only.

    python scripts/rl/s2_build_suites.py            # refuses to overwrite an existing manifest
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from petctl.rl.s2.cells import ORIENTS, cache_path, choose_cells, write_cache
from petctl.rl.s2.evaluator import pd_policy, run_scenario
from petctl.rl.s2.scenario import build_suite
from petctl.rl.s2.suite import SUITE_DIR, write_set_a

LIMP_KP = 1e-4


def main() -> int:
    """Pipeline entry point."""
    if (SUITE_DIR / "manifest.json").exists():
        print("manifest exists: frozen")
        return 1
    poses, reps, summ = choose_cells()
    print("poses", summ["poses"], "dropped", summ["dropped"], "no_go", summ["no_go"])
    if summ["no_go"]:
        return 2
    valid = [(r["pose"], r["orient"]) for r in reps if r["valid"]]
    write_cache(poses, valid)
    ny, _ = build_suite("decision", "L2", valid)
    from multiprocessing import Pool
    import petctl.rl.s2.evaluator as ev
    with Pool(8) as p:
        res = p.map(_limp, [s.to_dict() for s in ny])
    per_cell: dict[tuple[str, str], list[float]] = {}
    for s, r in zip(ny, res):
        per_cell.setdefault((s.pose, s.orient), []).append(r)
    cell_E = {k: float(np.mean(v)) for k, v in per_cell.items()}
    keep = [k for k in valid if cell_E[k] >= 0.1]
    excluded = [{"cell": list(k), "limp_E": cell_E[k], "reason": "limp E < 0.1 rad"} for k in valid if cell_E[k] < 0.1]
    H = lambda ks: float(np.mean([np.clip(1 - cell_E[k] / 0.3, 0, 1) for k in ks])) if ks else 1.0
    order = sorted(keep, key=lambda k: cell_E[k])
    while H(keep) > 0.10 and len(keep) > 1:
        k = order.pop(0)
        keep.remove(k)
        excluded.append({"cell": list(k), "limp_E": cell_E[k], "reason": "aggregate limp H > 0.10"})
    nontrivial = [k for k in valid if cell_E[k] >= 0.1]                          # rev. 9: limp-trivial cells leave the 60% denominator
    frac = sum(len(per_cell[k]) for k in keep) / max(1, sum(len(per_cell[k]) for k in nontrivial))
    print("limp E per cell", {f"{k[0]}|{k[1]}": round(v, 3) for k, v in cell_E.items()})
    print("keep", keep, "limp H", round(H(keep), 3), "scenario fraction", round(frac, 2))
    if frac < 0.6 or H(keep) > 0.10 or not keep:
        print("NO-GO: suite validity rule fails; back to the expert")
        return 3
    meta = {"poses": {n: {"source": p.source, "q_commanded": [float(x) for x in p.q]} for n, p in poses.items()}, "cell_reports": reps, "limp_E_per_cell": {f"{k[0]}|{k[1]}": v for k, v in cell_E.items()},
            "excluded": excluded, "gate_file": "docs/plans/STAGE_2_GATES.md rev. 9"}
    man = write_set_a(keep, meta)
    print("frozen", man["counts"])
    return 0


def _limp(sc: dict) -> float:
    from petctl.rl.s2.evaluator import pd_policy, run_scenario
    return run_scenario(sc, pd_policy, fixed_kp=LIMP_KP, check=False)["E"]


if __name__ == "__main__":
    raise SystemExit(main())
