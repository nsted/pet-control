"""Captioned video of frozen S2 scenarios under scripted policies (sim only). Each clip: scenario id, policy, running joint error.

    python scripts/rl/render_s2.py --out docs/media/s2_hold/pd_baseline/s2_frozen_pd.mp4
"""

from __future__ import annotations

import argparse
import subprocess
from pathlib import Path

import mujoco
import numpy as np

from petctl.rl.s2 import suite
from petctl.rl.s2.evaluator import env_for, pd_policy
from petctl.rl.video import burn_caption, downscale

CLIPS = [("decision-L2-head_lift-upright-load-0", 0.4, 1.0, "PD kp 0.4"), ("decision-L2-head_lift-upright-load-0", 5.0, 2.0, "best PD kp 5.0 zeta 2"), ("decision-L2-head_lift-upright-load-0", 1e-4, 1.0, "limp (kp ~0)"),
         ("decision-L2-arch_2-upright-tilt-0", 1.6, 1.0, "PD kp 1.6"), ("decision-L2-arch_2-upright-tilt-0", 5.0, 2.0, "best PD kp 5.0 zeta 2")]


def main() -> int:
    """CLI entry point."""
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", type=Path, required=True)
    a = ap.parse_args()
    by = {s.id: s for s in suite.load("decision", "L2", "nonyield")}
    w, h, fps = 1280, 720, 20
    a.out.parent.mkdir(parents=True, exist_ok=True)
    ff = subprocess.Popen(["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{w}x{h}", "-r", str(fps), "-i", "-", "-an", "-c:v", "libx264", "-pix_fmt", "yuv420p", str(a.out)],
                          stdin=subprocess.PIPE)
    for sid, kp, z, label in CLIPS:
        sc = by[sid]
        env = env_for(sc, kp, z)
        env.reset(seed=sc.seed, options={"scenario": sc.to_dict(), "cache_dir": "data/rl/s2/cache"})
        m = env.sim.model
        m.vis.global_.offwidth, m.vis.global_.offheight = w, h
        rend = mujoco.Renderer(m, height=h, width=w)
        cam = mujoco.MjvCamera()
        cam.distance, cam.elevation = 0.9, -18
        qs = env.task.q_star
        for t in range(env.max_steps):
            env.step(np.zeros(20))
            e = float(np.abs(env.sim.joint_pos() - qs).max())
            on = sc.onset_s <= t * env.dt < sc.onset_s + sc.duration_s
            cam.azimuth = 130.0 + 6.0 * t * env.dt
            cam.lookat[:] = env.sim.data.subtree_com[1]
            rend.update_scene(env.sim.data, camera=cam)
            cap = f"S2 hold (frozen {sc.level} scenario {sc.pose}/{sc.orient}/{sc.family}, sim, scripted {label}) | max joint err {e:.2f} rad | {'DISTURBANCE ON' if on else 'quiet'} | t={t * env.dt:4.1f}s"
            ff.stdin.write(burn_caption(downscale(rend.render(), (w, h)), cap).tobytes())
    ff.stdin.close()
    ff.wait()
    print("wrote", a.out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
