"""World-frame check of MAP-Elites cells with large yaw or roll: re-run and measure the head-to-tail axis rotation (unwrapped, 20 Hz) and COM travel, flag cells where the search metric disagrees.

    python scripts/rl/verify_archive.py data/rl/map_elites/n10_s1.json --min-yaw 2.5
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np

from petctl.rl.cpg import CPG
from petctl.rl.gait_eval import CONTROL_DT, build_sim


def main() -> int:
    """CLI entry point."""
    ap = argparse.ArgumentParser()
    ap.add_argument("archive", type=Path)
    ap.add_argument("--min-yaw", type=float, default=2.5)
    a = ap.parse_args()
    arc = json.loads(a.archive.read_text())
    n, nj = arc["n_modules"], arc["n_modules"] - 1
    sim = build_sim(n)
    for x in sorted((c for c in arc["cells"] if abs(c["metrics"]["yaw_rate"]) > a.min_yaw), key=lambda c: -abs(c["metrics"]["yaw_rate"])):
        p = np.array(x["params"])
        sim.reset()
        sim.step(np.zeros(nj), 0.8)
        cpg = CPG(nj, smooth_hz=0.0)
        cpg.reset(p)
        d, ang, c0 = sim.data, [], None
        for i in range(int(12.0 / CONTROL_DT)):
            sim.step(cpg.step(CONTROL_DT, p), CONTROL_DT)
            if i == int(2.0 / CONTROL_DT):
                c0 = d.xipos[sim.m0_id:sim.m0_id + n].mean(axis=0).copy()
            v = d.xipos[sim.m0_id] - d.xipos[sim.m0_id + n - 1]
            ang.append(math.atan2(v[1], v[0]))
        u = np.unwrap(ang)
        k = int(2.0 / CONTROL_DT)
        rate = (u[-1] - u[k]) / (12.0 - 2.0)
        step = float(np.abs(np.diff(u)).max())
        trav = float(np.linalg.norm(d.xipos[sim.m0_id:sim.m0_id + n].mean(axis=0)[:2] - c0[:2])) / 10.0
        print(f"cell {x['cell']}: search yaw {x['metrics']['yaw_rate']:+.2f}, head-tail axis rate {rate:+.2f} rad/s (max step {step:.2f} rad/tick), COM travel {100 * trav:.1f} cm/s")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
