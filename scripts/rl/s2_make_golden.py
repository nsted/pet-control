"""T7: write tests/golden/s2_golden.json: per-scenario numbers of scripted PD policies on a few FROZEN S2 scenarios (the golden evaluation test reproduces them). Regenerating needs the expert's OK (it changes
what the test accepts)."""

from __future__ import annotations

import json
from pathlib import Path

from petctl.rl.s2 import suite
from petctl.rl.s2.evaluator import pd_policy, run_scenario

KEYS = ("E", "E_settle", "E_dist", "E_rec", "I_mean", "I_max_2s", "I_peak_tick", "tau_max")
POLICIES = {"pd_kp1.6_z1": (1.6, 1.0), "pd_kp0.4_z1": (0.4, 1.0), "limp": (1e-4, 1.0)}


def main() -> int:
    """Pick one scenario per family of the L2 decision suite for the first lifted cell and run the scripted policies."""
    scs = suite.load("decision", "L2", "nonyield")
    cell = (scs[0].pose, scs[0].orient)
    pick = {}
    for s in scs:
        if (s.pose, s.orient) == ("head lift", "upright") and s.family not in pick:
            pick[s.family] = s
    out = {}
    for fam, s in pick.items():
        out[s.id] = {}
        for name, (kp, z) in POLICIES.items():
            r = run_scenario(s, pd_policy, fixed_kp=kp, zeta=z)
            out[s.id][name] = {k: r[k] for k in KEYS} | {"lost": r["lost"]}
    Path("tests/golden").mkdir(exist_ok=True)
    Path("tests/golden/s2_golden.json").write_text(json.dumps(out, indent=1, sort_keys=True))
    print("wrote", len(out), "scenarios", cell)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
