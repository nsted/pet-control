"""M1: static hug-feasibility search for n_modules 8..15 (sim only). Run: python scripts/rl/m1_hug_feasibility.py [--lengths 8 9 ...] [--budget 4000]

Writes data/rl/m1_hug_feasibility/table.json (one record per length), a contact-sheet PNG per length and, with --md, the markdown table
docs/plans/M1_HUG_FEASIBILITY.md is written by hand from the JSON (see the plan). Protocol: petctl/rl/hug_search.py.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

from petctl.rl.hug_search import render_sheet, search

OUT = Path("data/rl/m1_hug_feasibility")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--lengths", type=int, nargs="+", default=list(range(8, 16)))
    ap.add_argument("--budget", type=int, default=4000, help="evaluations per length (over all restarts)")
    ap.add_argument("--restarts", type=int, default=2)
    ap.add_argument("--workers", type=int, default=8)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--out", type=Path, default=OUT)
    a = ap.parse_args()
    a.out.mkdir(parents=True, exist_ok=True)
    table_p = a.out / "table.json"
    table = {r["n_modules"]: r for r in json.loads(table_p.read_text())["rows"]} if table_p.exists() else {}
    for n in a.lengths:
        rec = search(n, a.budget, a.restarts, a.workers, a.seed)
        table[n] = rec
        render_sheet(n, rec, a.out / f"best_pose_n{n}.png")
        table_p.write_text(json.dumps({"rows": [table[k] for k in sorted(table)]}, indent=1))
        print(f"n={n}: held {rec['held_modules']}/H={rec['H']} reached={rec['h_reached']} bus {rec['bus_mean_a']:.2f}/{rec['bus_applied_peak_a']:.1f} A "
              f"max|q| {rec['max_joint_rad']:.2f} {rec['geometry']} wall {rec['wall_s']:.0f}s", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
