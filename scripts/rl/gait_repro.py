"""Are the library gaits reproducible? Re-measure each elite's (vx, vy, yaw) in CPGEnv under (a) the nominal start, (b) friction x0.7, (c) friction x1.3, (d) after a 3 s pre-run of a different random gait.
Reports the median relative error of the speed vector and the fraction of elites whose velocity stays within 40% of the label (the tracking-kernel width).

    python scripts/rl/gait_repro.py --n-elites 40
"""

from __future__ import annotations

import argparse

import numpy as np

from petctl.rl.cpg_env import CPGEnv


def measure(e: CPGEnv, params: np.ndarray, mu: float = 1.0, pre: np.ndarray | None = None) -> np.ndarray:
    """(vx, vy) over t = 3..10 s in the start heading frame."""
    e.reset(seed=1)
    e.sim.model.geom_friction[e.floor_geoms, 0] = e.floor_mu0 * mu
    if pre is not None:
        e.target, e.base = pre.copy(), pre.copy()
        e.cpg.reset(pre, 0.0)
        for _ in range(int(3.0 / e.dt)):
            e.step(np.zeros(13))
        e.h0 = float(np.median(e._chord_angles()))
    e.target, e.base = params.copy(), params.copy()
    if pre is None:
        e.cpg.reset(params, 0.0)                                                                      # same initial CPG state as the relabelling
    c0 = None
    for i in range(int(10.0 / e.dt)):
        e.step(np.zeros(13))
        if i == int(3.0 / e.dt):
            c0 = e.hist[-1][0].copy()
    d = (e.hist[-1][0] - c0)[:2] / 7.0
    h = e.h0
    return np.array([np.cos(h) * d[0] + np.sin(h) * d[1], -np.sin(h) * d[0] + np.cos(h) * d[1]])


def main() -> int:
    """CLI entry point."""
    ap = argparse.ArgumentParser()
    ap.add_argument("--n-elites", type=int, default=40)
    a = ap.parse_args()
    e = CPGEnv(randomise=False, use_prior=False, episode_s=40.0, cmd_s=1e6)
    rng = np.random.default_rng(0)
    idx = rng.choice(len(e.elites), min(a.n_elites, len(e.elites)), replace=False)
    err = {"nominal": [], "mu0.7": [], "mu1.3": [], "after other gait": []}
    ok = {k: 0 for k in err}
    used = 0
    for i in idx:
        p = e.elites[i]
        v0 = e.elite_v[i][:2]
        if np.linalg.norm(v0) < 0.05:
            continue
        used += 1
        pre = e.elites[rng.integers(len(e.elites))]
        for name, v in (("nominal", measure(e, p)), ("mu0.7", measure(e, p, 0.7)), ("mu1.3", measure(e, p, 1.3)), ("after other gait", measure(e, p, 1.0, pre))):
            r = float(np.linalg.norm(v - v0) / np.linalg.norm(v0))
            err[name].append(r)
            ok[name] += int(r < 0.4)
    print(f"{used} elites with speed > 5 cm/s; relative velocity error vs the label (median) and fraction within 40%:")
    for k in err:
        print(f"  {k:18s} median error {np.median(err[k]):.2f}   within 40%: {ok[k]}/{used}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
