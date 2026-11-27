"""PPO in CPG-parameter space (Phase 7): ``CPGEnv`` command tracking. Start through ``scripts/rl/launch_run.py`` (pilot first).

    python scripts/rl/launch_run.py pilot --steps 1000000 -- python scripts/rl/train_cpg.py --steps 1000000 --out data/rl/cpg_pilot1
Std-runaway fix kept: ent_coef 0, log_std_init -1. Watchdog (reward-term logging + stop rules) is on unless PETCTL_WATCHDOG=0.
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import time
from pathlib import Path

import torch
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import CallbackList
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import SubprocVecEnv, VecNormalize

from petctl.rl.cpg_env import CPGEnv


def make(rank: int, seed: int, n: int, kw: dict):
    """Env factory."""
    def f():
        e = CPGEnv(n_modules=n, **kw)
        e.reset(seed=seed + rank)
        return Monitor(e)
    return f


def main() -> int:
    """CLI entry point."""
    ap = argparse.ArgumentParser()
    ap.add_argument("--steps", type=int, default=1_000_000)
    ap.add_argument("--n", type=int, default=11)
    ap.add_argument("--n-envs", type=int, default=9)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--stage", choices=("steady", "switching"), default="switching", help="steady: one command per 20 s episode, upright, 3 s mask; switching: 6 s segments, randomised start")
    ap.add_argument("--resume", type=Path, default=None, help="run directory to continue from (model.zip + vecnormalize.pkl)")
    a = ap.parse_args()
    a.out.mkdir(parents=True, exist_ok=True)
    torch.set_num_threads(2)
    kw = dict(cmd_s=20.0, episode_s=20.0, mask_s=3.0, randomise=False) if a.stage == "steady" else {}
    venv = SubprocVecEnv([make(i, a.seed * 1000, a.n, kw) for i in range(a.n_envs)])
    if a.resume:
        venv = VecNormalize.load(str(a.resume / "vecnormalize.pkl"), venv)
        model = PPO.load(str(a.resume / "model"), env=venv)
    else:
        venv = VecNormalize(venv, norm_obs=True, norm_reward=True, clip_obs=10.0)
        model = PPO("MlpPolicy", venv, learning_rate=3e-4, n_steps=256, batch_size=576, n_epochs=5, gamma=0.99, gae_lambda=0.95, ent_coef=0.0, clip_range=0.2, seed=a.seed,
                    policy_kwargs={"net_arch": [256, 256], "log_std_init": -1.0}, tensorboard_log=None, verbose=0, device="cpu")
    cbs = []
    if os.environ.get("PETCTL_WATCHDOG", "1") != "0":
        from petctl.rl.watchdog import WatchdogCallback
        base = os.environ.get("PETCTL_WATCHDOG_BASELINE")
        cbs.append(WatchdogCallback(a.out, baseline=Path(base) if base else None))
    t0 = time.time()
    model.learn(total_timesteps=a.steps, callback=CallbackList(cbs), progress_bar=False)
    model.save(str(a.out / "model"))
    venv.save(str(a.out / "vecnormalize.pkl"))
    try:
        git = subprocess.check_output(["git", "rev-parse", "--short", "HEAD"], text=True).strip()
    except Exception:
        git = "unknown"
    (a.out / "meta.json").write_text(json.dumps({"git": git, "steps": a.steps, "n_envs": a.n_envs, "seed": a.seed, "n_modules": a.n, "algo": "ppo_cpg", "wall_s": time.time() - t0,
                                                 "watchdog_stop": (a.out / "STOP_watchdog").exists()}, indent=1))
    venv.close()
    print(f"done in {time.time() - t0:.0f}s -> {a.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
