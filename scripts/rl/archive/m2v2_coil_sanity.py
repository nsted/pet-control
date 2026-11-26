"""Step-0(c) sanity: the scripted M1b constant-angle coil, replayed as a policy-like scripted rollout through the SAME gate2 pipeline (``run_gate2``) as a trained policy.

    python scripts/rl/m2v2_coil_sanity.py --lengths 8 10 12 --n 100

Writes ``data/rl/m2v2/coil_sanity.json`` (hug success with Wilson CI per length and per start perturbation, hug_modules / t_hug medians). Sim only.
"""
from __future__ import annotations

import argparse
import json
import sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

from petctl.rl.curriculum.gate2 import Gate2Spec, run_gate2
from petctl.rl.curriculum.stages import get_stage
from petctl.rl.hug_start import ScriptedCoilEvaluator


def _chunk(args: tuple[int, dict[str, float], float, float, dict[str, float], list[int]]) -> list[dict]:
    n, params, dr, ep, ov, seeds = args
    return ScriptedCoilEvaluator(n, params, dr, ep)(ov, seeds)


class PoolScripted:
    """``ScriptedCoilEvaluator`` split over a process pool (results in seed order)."""

    def __init__(self, n: int, params: dict[str, float], dr: float, ep: float, workers: int) -> None:
        self.n, self.params, self.dr, self.ep, self.workers = n, params, dr, ep, workers

    def __call__(self, overrides: dict[str, float], seeds: list[int], leak: bool = False, dr_scale: float | None = None) -> list[dict]:
        dr = self.dr if dr_scale is None else dr_scale
        chunks = [seeds[i::self.workers] for i in range(self.workers) if seeds[i::self.workers]]
        with ProcessPoolExecutor(len(chunks)) as ex:
            rows = [r for part in ex.map(_chunk, [(self.n, self.params, dr, self.ep, overrides, c) for c in chunks]) for r in part]
        return sorted(rows, key=lambda r: r["seed"])


def main() -> int:
    """CLI."""
    ap = argparse.ArgumentParser()
    ap.add_argument("--lengths", type=int, nargs="+", default=[8, 10, 12])
    ap.add_argument("--n", type=int, default=100)
    ap.add_argument("--workers", type=int, default=6)
    ap.add_argument("--out", type=Path, default=Path("data/rl/m2v2/coil_sanity.json"))
    a = ap.parse_args()
    st = get_stage("m2_hug2")
    out: dict[str, dict] = {}
    for n in a.lengths:
        for pert in (0.0, 1.0):
            params = {**st.gate.task_params, "bus_model": 1.0, "hug_perturb_scale": pert}
            ev = PoolScripted(n, params, 0.6, 30.0, a.workers)
            res = run_gate2(Path("."), st, Gate2Spec(n=a.n, dr_scale=0.6, nominal=False, threshold=0.0), evaluator=ev, write=False)
            rb, mm = res["robust"], res["robust"]["metric_median"]
            key = f"n{n}_perturb{pert}"
            out[key] = {"n_modules": n, "perturb_scale": pert, "episodes": rb["n"], "success": rb["rate"], "wilson_lo": rb["wilson_lo"], "wilson_hi": rb["wilson_hi"],
                        "hug_modules_median": mm.get("hug_modules"), "t_hug_median_s": mm.get("t_hug_s"), "bus_mean_a_median": res["current"].get("bus_mean_a_median")}
            print(key, json.dumps(out[key]), flush=True)
            args = a.out
            args.parent.mkdir(parents=True, exist_ok=True)
            args.write_text(json.dumps(out, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
