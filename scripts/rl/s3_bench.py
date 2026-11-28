"""T25: S3 env throughput at n = 11 (loco_cmd, delta_kp, MIT frame, thermal, bus model, kp action) with 10 SubprocVecEnv workers and random actions. Sim only.

    PYTHONPATH=. python scripts/rl/s3_bench.py
"""

from __future__ import annotations

import time

import numpy as np
from stable_baselines3.common.vec_env import SubprocVecEnv

from petctl.rl.config import EnvConfig
from petctl.rl.env import PetEnv

TP = {"mit_frame": 1.0, "thermal": 1.0, "action_unbounded": 1.0, "bus_model": 1.0, "bus_motor": 1.0, "bus_clamp": 0.0, "reply_dr": 1.0}


def make(i: int, tp: dict):
    def f():
        env = PetEnv(EnvConfig(task="loco_cmd", n_modules=11, episode_s=20.0, action_mode="delta_kp", task_params=tp, seed=i))
        return env
    return f


def main() -> int:
    """Measure steps/s for a few configurations."""
    for name, tp in (("full (thermal + bus + MIT frame)", TP), ("lean (no thermal, no bus)", {"mit_frame": 1.0, "action_unbounded": 1.0, "reply_dr": 1.0})):
        venv = SubprocVecEnv([make(i, tp) for i in range(10)])
        venv.reset()
        rng = np.random.default_rng(0)
        n = 400
        t0 = time.time()
        for _ in range(n):
            venv.step(rng.uniform(-1, 1, (10, 20)))
        dt = time.time() - t0
        print(f"{name}: {10 * n / dt:.0f} env steps/s with 10 envs", flush=True)
        venv.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
