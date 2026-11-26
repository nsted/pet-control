"""World-frame check of CMA-ES gait results: re-run each gait, measure centre-of-mass displacement in the world frame (independent of evaluate_gait's body-frame metrics), peak height, and body-module height spread.

    python scripts/rl/verify_gaits.py data/rl/gait_search/forward_n*.json
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

from petctl.rl.cpg import CPG
from petctl.rl.gait_eval import CONTROL_DT, build_sim


def main() -> int:
    """CLI entry point."""
    for path in map(Path, sys.argv[1:]):
        res = json.loads(path.read_text())
        n, nj, p = res["n_modules"], res["n_modules"] - 1, np.array(res["params"])
        sim = build_sim(n)
        sim.reset()
        sim.step(np.zeros(nj), 0.8)
        d = sim.data
        cpg = CPG(nj, smooth_hz=0.0)
        cpg.reset(p)
        com = lambda: d.xipos[sim.m0_id:sim.m0_id + n].mean(axis=0).copy()          # noqa: E731
        for _ in range(int(2.0 / CONTROL_DT)):                                       # warmup
            sim.step(cpg.step(CONTROL_DT, p), CONTROL_DT)
        c0, zs, t = com(), [], 10.0
        for _ in range(int(t / CONTROL_DT)):
            sim.step(cpg.step(CONTROL_DT, p), CONTROL_DT)
            zs.append(float(d.xipos[sim.m0_id:sim.m0_id + n, 2].max()))
        c1 = com()
        dx = c1 - c0
        m = res["metrics"]
        print(f"{path.name}: world COM displacement {100 * np.linalg.norm(dx[:2]):.0f} cm in {t:.0f} s = {100 * np.linalg.norm(dx[:2]) / t:.1f} cm/s (search said speed {100 * m['speed']:.1f}); "
              f"max module height {100 * max(zs):.1f} cm, dz {100 * dx[2]:+.1f} cm", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
