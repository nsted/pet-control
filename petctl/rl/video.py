"""Offscreen MP4 rendering of a trained policy (inference only) and the end-of-training "video of the best" helper."""
from __future__ import annotations

import json
import subprocess
import time
from pathlib import Path

import mujoco
import numpy as np

from petctl.rl.env import PetEnv
from petctl.rl.policy import GaitPolicy

MEDIA_DIR = Path("docs/media")


def record_video(run: Path, out: Path, policy_file: str = "policy.npz", seconds: float | None = None, seed: int = 0,
                 dr_scale: float = 0.0, fps: int = 30, size: tuple[int, int] = (960, 540)) -> dict[str, float]:
    """Roll out ``run/policy_file`` deterministically for one episode, write an MP4 to ``out``, return the episode metrics."""
    pol = GaitPolicy(run, policy_file)
    pol.cfg.dr.scale = dr_scale
    if seconds is not None:
        pol.cfg.episode_s = seconds
    env = PetEnv(pol.cfg)
    obs, _ = env.reset(seed=seed)
    w, h = size
    env.sim.model.vis.global_.offwidth, env.sim.model.vis.global_.offheight = max(w, 640), max(h, 480)
    rend = mujoco.Renderer(env.sim.model, height=h, width=w)
    cam = mujoco.MjvCamera()
    cam.distance, cam.elevation, cam.azimuth = 0.9, -35, 130
    out.parent.mkdir(parents=True, exist_ok=True)
    ff = subprocess.Popen(["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{w}x{h}", "-r", str(fps),
                           "-i", "-", "-an", "-c:v", "libx264", "-pix_fmt", "yuv420p", str(out)], stdin=subprocess.PIPE)
    assert ff.stdin is not None
    frame_dt, next_frame, info = 1.0 / fps, 0.0, {}
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
    return {k: float(x) for k, x in info.get("metrics", {}).items()}


def record_best_video(run: Path, media_dir: Path = MEDIA_DIR) -> Path | None:
    """Write ``<media_dir>/<run>_best_ep<N>_<timestamp>.mp4`` for the run's best policy (``best_policy.npz``, else ``policy.npz``).

    Never raises: a missing ffmpeg or render error is reported and training results are unaffected.
    """
    try:
        best = run / "best.json"
        policy_file = "best_policy.npz" if (run / "best_policy.npz").exists() else "policy.npz"
        ep = json.loads(best.read_text()).get("episode", "x") if best.exists() else "final"
        out = media_dir / f"{run.name}_best_ep{ep}_{time.strftime('%Y%m%d_%H%M%S')}.mp4"
        m = record_video(run, out, policy_file)
        print(f"[video] {out}  " + "  ".join(f"{k}={v:.3f}" for k, v in m.items()), flush=True)
        return out
    except Exception as e:  # noqa: BLE001 - video is a convenience, never fail the run for it
        print(f"[video] could not record best video: {e}", flush=True)
        return None
