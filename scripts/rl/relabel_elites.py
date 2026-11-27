"""Relabel the library gaits (MAP-Elites archives, n = 11) by rolling each out INSIDE CPGEnv (open loop, upright, 10 s) and measuring (vx, vy, yaw rate) exactly as the env's reward does (1 s window frames,
heading fixed at the start, chords must agree). The selector prior of CPGEnv uses these labels when data/rl/cpg_elite_labels_n{n}.json exists.

    python scripts/rl/relabel_elites.py --n 11
"""

from __future__ import annotations

import argparse
import json
from multiprocessing import get_context
from pathlib import Path

import numpy as np

from petctl.rl.cpg_env import ARCHIVES, CPGEnv

_ENV: dict = {}


def label(args: tuple[int, list[float]]) -> dict:
    """Roll out one parameter set; returns its env-measured velocity and mean torque."""
    n, params = args
    e = _ENV.setdefault(n, CPGEnv(n_modules=n, randomise=False, use_prior=False, episode_s=11.0, cmd_s=1e6))
    e.reset(seed=1)
    e.target, e.base = np.array(params), np.array(params)
    e.cpg.reset(e.target, 0.0)
    e.cmd = np.zeros(3)
    c_start, angs = None, []
    for i in range(int(10.0 / e.dt)):
        e.step(np.zeros(13))
        if i == int(3.0 / e.dt):
            c_start = e.hist[-1][0].copy()
        if i >= int(3.0 / e.dt):
            angs.append(e.hist[-1][1].copy())
    c_end = e.hist[-1][0]
    t = (int(10.0 / e.dt) - int(3.0 / e.dt)) * e.dt
    d = (c_end - c_start)[:2] / t
    h = e.h0
    v = [float(np.cos(h) * d[0] + np.sin(h) * d[1]), float(-np.sin(h) * d[0] + np.cos(h) * d[1])]
    ua = np.unwrap(np.stack(angs), axis=0)                                                      # unwrapped along every tick (no endpoint aliasing)
    dyaw = ua[-1] - ua[0]
    yaw = float(np.median(dyaw) / t) if float(np.ptp(dyaw)) < 0.3 * t else 0.0                   # chords must agree (0.3 rad per second of window)
    return {"params": params, "v": v + [yaw]}


def main() -> int:
    """CLI entry point."""
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=11)
    a = ap.parse_args()
    ps = []
    for pat in ARCHIVES:
        f = Path(pat.format(n=a.n))
        if f.exists():
            ps += [c["params"] for c in json.loads(f.read_text())["cells"] if c["metrics"]["speed"] > 0.0]
    for f in sorted(Path("data/rl/gait_search").glob(f"*_n{a.n}*.json")):                                 # CMA-ES specialists (forward, reverse, left, right, turns, rolls)
        ps.append(json.loads(f.read_text())["params"])
    with get_context("spawn").Pool(8) as pool:
        out = pool.map(label, [(a.n, p) for p in ps], chunksize=4)
    Path(f"data/rl/cpg_elite_labels_n{a.n}.json").write_text(json.dumps(out))
    v = np.array([o["v"] for o in out])
    print(f"{len(out)} elites relabelled; max |vx| {np.abs(v[:, 0]).max():.2f}, |vy| {np.abs(v[:, 1]).max():.2f}, |yaw| {np.abs(v[:, 2]).max():.2f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
