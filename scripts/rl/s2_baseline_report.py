"""Report of the T14 PD baseline files: best PD per (level, cell) chosen on the decision suite (lowest sum E among configs that keep both budgets in every scenario), re-measured on the test suite; headroom
numbers (E, E_dist, lost, current). Writes docs/plans/STAGE_2_BASELINE.md.

    python scripts/rl/s2_baseline_report.py data/rl/s2/baseline_L0L2.json data/rl/s2/baseline_L3.json
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

MEAN_A, PEAK_A = 4.0, 10.0


def main() -> int:
    """Merge the given baseline files and print / write the report."""
    levels: dict = {}
    for f in sys.argv[1:]:
        levels.update(json.loads(Path(f).read_text())["levels"])
    lines = ["# STAGE_2_BASELINE: T14 gate baseline (zero-delta PD through the frozen evaluator)", "",
             "Best PD per cell chosen on the DECISION suite of the same level (lowest sum E among configs that keep I_max_2s <= 4 A and tick peak <= 10 A in every scenario); numbers below are its TEST-suite re-measurement. "
             "E = mean wrapped joint error over the whole episode (rad); E_dist = the 4 s window after onset; lost = fraction of scenarios with a joint > 0.6 rad off for > 1 s; I = demanded bus current (A, proxy).", "",
             "| level | cell | best config (kp, zeta) | E test | E_settle | E_dist | lost | I mean | worst config E | n |", "|---|---|---|---|---|---|---|---|---|---|"]
    summary = {}
    for lv, d in sorted(levels.items()):
        dec, tst = d["decision"], d["test"]
        cells = sorted({tuple(c) for c in dec["cells"]})
        for cell in cells:
            di = [i for i, c in enumerate(dec["cells"]) if tuple(c) == cell]
            ti = [i for i, c in enumerate(tst["cells"]) if tuple(c) == cell]
            cand = []
            for key, rs in dec["configs"].items():
                sel = [rs[i] for i in di]
                ok = all(r["I_max_2s"] <= MEAN_A and r["I_peak_tick"] <= PEAK_A for r in sel)
                cand.append((sum(r["E"] for r in sel), key, ok))
            pick = min([c for c in cand if c[2]] or cand)
            key = pick[1]
            ts = [tst["configs"][key][i] for i in ti]
            worst = max(np.mean([r["E"] for r in [rs[i] for i in di]]) for rs in dec["configs"].values())
            m = lambda k: float(np.mean([r[k] for r in ts]))
            lines.append(f"| {lv} | {cell[0]} {cell[1]} | {key.replace('|', ', ')}{'' if pick[2] else ' (budget violated)'} | {m('E'):.3f} | {m('E_settle'):.3f} | {m('E_dist'):.3f} | {m('lost'):.2f} | {m('I_mean'):.2f} | {worst:.3f} | {len(ts)} |")
            summary.setdefault(lv, []).append((m("E_dist"), m("lost")))
    lines += ["", "Headroom rule (STAGE_2_GATES): no headroom if best-PD E_dist <= 0.03 rad with no losses. Per level, cells meeting it: " +
              "; ".join(f"{lv}: {sum(1 for e, l in v if e <= 0.03 and l == 0)}/{len(v)}" for lv, v in sorted(summary.items()))]
    out = "\n".join(lines)
    Path("docs/plans/STAGE_2_BASELINE.md").write_text(out + "\n")
    print(out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
