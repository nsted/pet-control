"""Throughput benchmark for the m2v6 driver: pure env stepping with random actions (no learner) in a SubprocVecEnv, for goal_hug (m2_hug5, n=10) and m2_flat (n=10) at several n_envs.

    python scripts/rl/m2v6_benchmark.py --seconds 60 --n-envs 6 8 10     # writes data/rl/m2v5/benchmark.json {config: {n_envs: steps_per_s}}
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from pathlib import Path

import numpy as np

CONFIGS = {"goal_hug_n10": "m2_hug5", "m2_flat_n10": "m2_flat"}


def bench(stage_name: str, n_envs: int, seconds: float, length: int = 10) -> float:
    """Env steps per second of ``n_envs`` workers stepping with random actions for ``seconds`` seconds (after a short warm-up)."""
    for var in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "VECLIB_MAXIMUM_THREADS", "MKL_NUM_THREADS"):
        os.environ.setdefault(var, "1")
    from stable_baselines3.common.vec_env import SubprocVecEnv

    from petctl.rl.curriculum.runner import build_config
    from petctl.rl.curriculum.stages import get_stage
    from petctl.rl.train import make_env_fn

    cfg = build_config(get_stage(stage_name), {"n_modules": float(length)})
    venv = SubprocVecEnv([make_env_fn(cfg, i, 0, True) for i in range(n_envs)])
    try:
        venv.reset()
        rng = np.random.default_rng(0)
        for _ in range(10):
            venv.step(rng.uniform(-1, 1, (n_envs, cfg.n_joints)))
        t0, steps = time.time(), 0
        while time.time() - t0 < seconds:
            venv.step(rng.uniform(-1, 1, (n_envs, cfg.n_joints)))
            steps += n_envs
        return steps / (time.time() - t0)
    finally:
        venv.close()


def main() -> int:
    """CLI entry."""
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=60.0)
    ap.add_argument("--n-envs", type=int, nargs="+", default=[6, 8, 10])
    ap.add_argument("--out", type=Path, default=Path("data/rl/m2v5/benchmark.json"))
    a = ap.parse_args()
    res: dict[str, dict[str, float]] = json.loads(a.out.read_text()) if a.out.exists() else {}
    for name, stage in CONFIGS.items():
        for n in a.n_envs:
            sps = bench(stage, n, a.seconds)
            res.setdefault(name, {})[str(n)] = round(sps, 1)
            print(f"{name} n_envs={n}: {sps:.0f} steps/s", flush=True)
            a.out.parent.mkdir(parents=True, exist_ok=True)
            a.out.write_text(json.dumps(res, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
