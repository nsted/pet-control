"""Step A of the action-range study: does the best L=8 policy pin its joints at the +-1.0 rad target cap while it tries to wrap?

Replays ``--run`` (default the best L=8 checkpoint ``data/rl/hug_curr4/hug_curr_02``, ``policy.npz``) deterministically on the L=8 gate shapes cyl r3 (seeds 10000-10019) and cyl r7 (10020-10039)
with the gate task params of hug_curr_02, dr 0.6, 20 s episodes (the env logs per-episode range metrics, see ``PetEnv._range_metrics``). Pools over wrap-attempt ticks (hug run >= 3 modules): the fraction of
in-run joint-ticks with |action| >= 0.95, the mean gap between each in-run joint's commanded target and its measured angle, the max |target| inside the run. Writes ``data/rl/range_diag/result.json`` and
``docs/plans/RANGE_DIAGNOSTIC.md`` with the decision (``petctl.rl.curriculum.hug_range.range_decision``).

    python scripts/rl/range_diag.py
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from typing import Any

sys.path.insert(0, str(Path(__file__).resolve().parent))

from m2_length_sweep import PoolEvaluator  # noqa: E402

from petctl.rl.curriculum import hug_range as hr  # noqa: E402
from petctl.rl.curriculum.stages import get_stage  # noqa: E402
from petctl.rl.target_shapes import GATE_SEED0, PER_SHAPE, held_out_names  # noqa: E402

RUN = Path("data/rl/hug_curr4/hug_curr_02")
OUT = Path("data/rl/range_diag/result.json")
DOC = Path("docs/plans/RANGE_DIAGNOSTIC.md")
SHAPES = ("cyl_r3", "cyl_r7")


def diagnose(run: Path, n: int, workers: int, dr: float = 0.6) -> dict[str, Any]:
    """Run ``n`` episodes of each of ``SHAPES`` and return the per-shape and pooled range numbers plus the decision."""
    stage = get_stage("hug_curr_02")
    params = dict(stage.gate.task_params)
    params.setdefault("bus_model", 1.0)
    params["range_diag"] = 1.0
    names = list(held_out_names(8))
    assert names[:2] == list(SHAPES), names
    ev = PoolEvaluator(run, "policy.npz", params, dr, stage.gate.episode_s, workers)
    res: dict[str, Any] = {}
    all_rows: list[dict[str, Any]] = []
    for name in SHAPES:
        i = names.index(name)
        rows = ev({}, [GATE_SEED0 + PER_SHAPE * i + j for j in range(n)])
        all_rows += rows
        res[name] = {**hr.pool_range(rows), "success": sum(r["metrics"].get("success", 0.0) for r in rows) / len(rows), "normal_span_deg_median": sorted(r["metrics"].get("normal_span_deg", 0.0) for r in rows)[len(rows) // 2]}
    pooled = hr.pool_range(all_rows)
    decision, reason = hr.range_decision(pooled["saturated_frac"])
    return {"run": str(run), "episodes_per_shape": n, "dr": dr, "episode_s": stage.gate.episode_s, "per_shape": res, "pooled": pooled, "decision": decision, "reason": reason,
            "decision_metric": "saturated_frac = mean over wrap-attempt ticks (hug run >= 3) of the fraction of in-run joints with |action| >= 0.95, pooled over both shapes", "time": time.strftime("%Y-%m-%d %H:%M:%S")}


def write_doc(r: dict[str, Any]) -> None:
    """Write ``docs/plans/RANGE_DIAGNOSTIC.md``."""
    def row(name: str, d: dict[str, Any]) -> str:
        f = lambda v, p=3: "n/a" if v is None else f"{v:.{p}f}"  # noqa: E731
        return (f"| {name} | {d['episodes_with_wrap_attempt']}/{d['episodes']} | {d['wrap_attempt_ticks']:.0f} | {f(d['saturated_frac'])} | {f(d['sat_any_frac'])} | {f(d['sat_all_frac'])} | "
                f"{f(d['target_gap_run'])} | {f(d['max_abs_target_run'])} |")
    txt = (f"# Action-range diagnostic (Step A)\n\nPolicy: `{r['run']}` (L=8, action_scale_rad 1.0), {r['episodes_per_shape']} deterministic episodes each of cyl r3 and cyl r7 "
           f"(gate seeds 10000+, dr {r['dr']}, {r['episode_s']:.0f} s, scored by the gate params of hug_curr_02). Wrap-attempt tick = hug run >= 3 modules; in-run joints = the joints between the run's modules; saturated = |action| >= 0.95.\n\n"
           "| shape | episodes with a wrap attempt | wrap-attempt ticks | saturated frac (joint-ticks) | any in-run joint saturated | all in-run joints saturated | mean |target - angle| (rad) | max |target| in run (rad) |\n"
           "|---|---|---|---|---|---|---|---|\n" + "\n".join(row(k, v) for k, v in [*r["per_shape"].items(), ("pooled", r["pooled"])]) + "\n\n"
           f"Metric used for the decision: {r['decision_metric']}.\n\n**Decision: {r['decision']}** ({r['reason']}). Rule: >= 20 % -> Step B (hug_scale15); < 10 % -> skip B, go to Step C; 10-20 % -> Step B.\n\n"
           f"Data: `{OUT}`.\n")
    DOC.write_text(txt)


def main() -> int:
    """CLI entry."""
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--run", type=Path, default=RUN)
    ap.add_argument("-n", type=int, default=20, help="episodes per shape")
    ap.add_argument("--workers", type=int, default=4)
    ap.add_argument("--out", type=Path, default=OUT)
    a = ap.parse_args()
    r = diagnose(a.run, a.n, a.workers)
    a.out.parent.mkdir(parents=True, exist_ok=True)
    a.out.write_text(json.dumps(r, indent=1))
    write_doc(r)
    print(json.dumps({k: r[k] for k in ("pooled", "decision", "reason")}, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
