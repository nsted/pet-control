"""Watch a trained gait in the MuJoCo viewer (inference only, loads policy.npz). Run: mjpython scripts/view_policy.py data/rl/<run>."""
from __future__ import annotations

import argparse
import time
from pathlib import Path

import mujoco.viewer  # type: ignore[import-untyped]

from petctl.rl.env import PetEnv
from petctl.rl.policy import GaitPolicy


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("run", type=Path)
    ap.add_argument("--seconds", type=float, default=30.0, help="episode length")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--dr-scale", type=float, default=0.0, help="0 = nominal plant, 1 = full randomisation")
    ap.add_argument("--speed", type=float, default=1.0, help="playback speed (1 = real time)")
    a = ap.parse_args()
    pol = GaitPolicy(a.run)
    pol.cfg.dr.scale, pol.cfg.episode_s = a.dr_scale, a.seconds
    env = PetEnv(pol.cfg)
    obs, _ = env.reset(seed=a.seed)
    info: dict = {}
    with mujoco.viewer.launch_passive(env.sim.model, env.sim.data) as v:
        v.cam.distance, v.cam.elevation, v.cam.azimuth = 0.9, -35, 130
        done = False
        while v.is_running() and not done:
            obs, _, term, trunc, info = env.step(pol.action(obs))
            done = term or trunc
            v.cam.lookat[:] = env.sim.data.subtree_com[1]
            v.sync()
            time.sleep(env.dt / a.speed)
    print("  ".join(f"{k}={x:.3f}" for k, x in info.get("metrics", {}).items()))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
