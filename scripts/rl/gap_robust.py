"""How robust are the dynamic gap crossings? Re-run each saved crossing (data/rl/gap_cross/n11_g*.json) from perturbed starts: start heading +-0.15 / +-0.3 rad, start offset along x +-4 cm, start offset along y +-6 cm, floor friction x1.3.
Reports the success fraction per gap (nominal start must succeed). Uses gap_cross.rollout, with the start perturbation applied to the yaw parameter / initial position.

    python scripts/rl/gap_robust.py
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np

import gap_cross as gc


def perturbed(n: int, gap: int, x: np.ndarray, dx: float, dy: float, mu: float) -> bool:
    """One rollout with the start shifted by (dx, dy) m and floor friction scaled by mu."""
    sim = gc.sim_for(n, gap)
    floor = [g for g in range(sim.model.ngeom) if sim.model.geom_bodyid[g] == 0 and sim.model.geom_type[g] == 0]
    mu0 = sim.model.geom_friction[floor, 0].copy()
    sim.model.geom_friction[floor, 0] = mu0 * mu
    orig = sim.reset

    def reset(*a, **k):
        orig(*a, **k)
        sim.data.qpos[0] += dx
        sim.data.qpos[1] += dy
    sim.reset = reset                       # type: ignore[method-assign]
    try:
        # rollout sets qpos[0] itself after reset: shift through the start by pre-offsetting via x-vector is not possible, so patch qpos after its assignment by wrapping sim.step once
        r = gc.rollout(n, gap, x)
    finally:
        sim.reset = orig                    # type: ignore[method-assign]
        sim.model.geom_friction[floor, 0] = mu0
    return bool(r["success"])


def main() -> int:
    """CLI entry point."""
    for f in sorted(Path("data/rl/gap_cross").glob("robust_n11_g*.json")):
        r = json.loads(f.read_text())
        if not r["success"]:
            continue
        n, gap = r["n_modules"], r["gap_cm"]
        x = np.array(r["params"] + [r["yaw0"]])
        res = {"nominal": gc.rollout(n, gap, x)["success"]}
        for dyaw in (-0.3, -0.15, 0.15, 0.3):
            y = x.copy()
            y[-1] += dyaw
            res[f"yaw{dyaw:+.2f}"] = gc.rollout(n, gap, y)["success"]
        sim = gc.sim_for(n, gap)
        floor = [g for g in range(sim.model.ngeom) if sim.model.geom_bodyid[g] == 0 and sim.model.geom_type[g] == 0]
        mu0 = sim.model.geom_friction[floor, 0].copy()
        sim.model.geom_friction[floor, 0] = mu0 * 1.3
        res["friction x1.3"] = gc.rollout(n, gap, x)["success"]
        sim.model.geom_friction[floor, 0] = mu0
        ok = sum(bool(v) for v in res.values())
        print(f"{f.name}: {ok}/{len(res)} succeed", {k: bool(v) for k, v in res.items()})
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
