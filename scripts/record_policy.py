"""Record a trained gait to MP4 (offscreen render, inference only). Run: python scripts/record_policy.py data/rl/<run> out.mp4."""
from __future__ import annotations

import argparse
import subprocess
from pathlib import Path

import mujoco  # type: ignore[import-untyped]
import numpy as np

from petctl.rl.env import PetEnv
from petctl.rl.policy import GaitPolicy


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("run", type=Path)
    ap.add_argument("out", type=Path)
    ap.add_argument("--seconds", type=float, default=15.0)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--dr-scale", type=float, default=0.0)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--size", type=int, nargs=2, default=(960, 540), metavar=("W", "H"))
    a = ap.parse_args()
    pol = GaitPolicy(a.run)
    pol.cfg.dr.scale, pol.cfg.episode_s = a.dr_scale, a.seconds
    env = PetEnv(pol.cfg)
    obs, _ = env.reset(seed=a.seed)
    w, h = a.size
    env.sim.model.vis.global_.offwidth, env.sim.model.vis.global_.offheight = max(w, 640), max(h, 480)
    rend = mujoco.Renderer(env.sim.model, height=h, width=w)
    cam = mujoco.MjvCamera()
    cam.distance, cam.elevation, cam.azimuth = 0.9, -35, 130
    ff = subprocess.Popen(["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{w}x{h}", "-r", str(a.fps),
                           "-i", "-", "-an", "-c:v", "libx264", "-pix_fmt", "yuv420p", str(a.out)], stdin=subprocess.PIPE)
    assert ff.stdin is not None
    frame_dt, next_frame, info = 1.0 / a.fps, 0.0, {}
    done = False
    while not done:
        obs, _, term, trunc, info = env.step(pol.action(obs))
        done = term or trunc
        t = env._t * env.dt
        while next_frame <= t:                                   # real-time playback at the video frame rate
            cam.lookat[:] = env.sim.data.subtree_com[1]
            rend.update_scene(env.sim.data, camera=cam)
            ff.stdin.write(np.ascontiguousarray(rend.render()).tobytes())
            next_frame += frame_dt
    ff.stdin.close()
    ff.wait()
    print(f"wrote {a.out}  " + "  ".join(f"{k}={x:.3f}" for k, x in info.get("metrics", {}).items()))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
