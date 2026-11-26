"""Throughput benchmark: env steps/s of SubprocVecEnv (random actions, DR on) against n_envs, with isotropic vs anisotropic skin friction.

    python scripts/rl/bench_envs.py --n-modules 11 --n-envs 4 6 8 10 12 --steps 1500 --out data/rl/bench_envs.json
Training adds PPO update time on top (the learner runs in the main process), so use the RESULT for ranking n_envs, not as the training speed.
"""

from __future__ import annotations

import argparse
import json
import os
import time
from pathlib import Path

for _v in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "VECLIB_MAXIMUM_THREADS", "MKL_NUM_THREADS"):
    os.environ.setdefault(_v, "1")

import numpy as np

from petctl.rl.config import DomainRand, EnvConfig
from petctl.rl.train import make_env_fn


def bench(n_modules: int, n_envs: int, steps: int, aniso: bool) -> float:
    """Steps per second over ``steps`` vector steps (after a warm-up) for one configuration."""
    from stable_baselines3.common.vec_env import SubprocVecEnv

    cfg = EnvConfig(task="goal_reach", n_modules=n_modules, action_mode="hold", episode_s=15.0, dr=DomainRand(scale=0.3, friction_aniso=(1.0, 2.5) if aniso else (1.0, 1.0)),
                    task_params={"goal_dist_min": 1.0, "goal_dist_max": 2.0, "allow_flip": 1.0, "start_pose_rad": 0.6})
    venv = SubprocVecEnv([make_env_fn(cfg, i, 0) for i in range(n_envs)])
    venv.reset()
    rng = np.random.default_rng(0)

    def act() -> np.ndarray:
        return rng.uniform(-1, 1, (n_envs, cfg.n_actions)).astype(np.float32)

    for _ in range(30):
        venv.step(act())
    t0 = time.time()
    for _ in range(steps):
        venv.step(act())
    dt = time.time() - t0
    venv.close()
    return steps * n_envs / dt


def main() -> int:
    """CLI entry point."""
    ap = argparse.ArgumentParser()
    ap.add_argument("--n-modules", type=int, default=11)
    ap.add_argument("--n-envs", type=int, nargs="+", default=[4, 6, 8, 10, 12])
    ap.add_argument("--steps", type=int, default=1500)
    ap.add_argument("--out", type=Path, default=Path("data/rl/bench_envs.json"))
    a = ap.parse_args()
    res = []
    for n in a.n_envs:
        row = {"n_envs": n}
        for aniso in (False, True):
            row["aniso" if aniso else "iso"] = round(bench(a.n_modules, n, a.steps, aniso), 1)
        print(row, flush=True)
        res.append(row)
    a.out.parent.mkdir(parents=True, exist_ok=True)
    a.out.write_text(json.dumps({"n_modules": a.n_modules, "steps": a.steps, "rows": res}, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
