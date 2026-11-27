"""Library-selector oracle for the CPG env: for each fixed command, run every MAP-Elites elite (n = 11, both seeds) open loop and report the best mean tracking kernel (window velocity kernel after the
first second). A policy that only picks a library gait cannot beat this; a policy that blends / morphs gaits should. Also reports the lazy baseline (zero-speed body).

    python scripts/rl/oracle_cpg.py --out data/rl/cpg_oracle.json
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from petctl.rl.cpg_env import CPGEnv

CMDS = {"stand": (0, 0, 0), "forward": (0.14, 0, 0), "back": (-0.14, 0, 0), "left": (0, 0.14, 0), "right": (0, -0.14, 0), "turn left": (0, 0, 0.8), "turn right": (0, 0, -0.8), "forward + turn left": (0.12, 0, 0.6)}


def kernel(env: CPGEnv, params: np.ndarray, cmd: tuple, seconds: float = 8.0) -> float:
    """Mean velocity kernel over the last 6 s of an open-loop rollout of ``params`` under a fixed command."""
    env.reset(seed=3)
    env.target = params.copy()
    env.cpg.reset(params, 0.0)
    env.cmd = np.array(cmd, float)
    ks, prev = [], 0.0
    for i in range(int(seconds / env.dt)):
        env.step(np.zeros(13))
        cur = env.sums["vel"]
        if i >= int(2.0 / env.dt):
            ks.append((cur - prev) / env.w["vel"])
        prev = cur
    return float(np.mean(ks))


def main() -> int:
    """CLI entry point."""
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", type=Path, default=Path("data/rl/cpg_oracle.json"))
    a = ap.parse_args()
    env = CPGEnv(n_modules=11, randomise=False, episode_s=9.0, cmd_s=1e6)
    out = {}
    for name, cmd in CMDS.items():
        ks = [kernel(env, e, cmd) for e in env.elites]
        out[name] = {"best_elite_kernel": max(ks), "median_elite_kernel": float(np.median(ks))}
        print(f"{name:20s} best elite {max(ks):.2f}  median elite {np.median(ks):.2f}", flush=True)
    out["_mean_best"] = float(np.mean([v["best_elite_kernel"] for v in out.values()]))
    print(f"mean over commands of the best elite: {out['_mean_best']:.2f}; lazy (never moves) baseline 0.375")
    a.out.write_text(json.dumps(out, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
