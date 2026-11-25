"""Captioned montage of the best held poses (reach / bridge / coil / low) from ``pose_targeted.py``: each pose is commanded, settles for 4 s, then the camera orbits it for 8 s. Sim only.

    python scripts/rl/render_poses.py --poses data/rl/motion_repertoire/poses_targeted.json --out docs/media/poses_montage_20261125.mp4
"""

from __future__ import annotations

import argparse
import json
import subprocess
from pathlib import Path

import mujoco
import numpy as np

import pose_repertoire as pr
from petctl.rl.video import burn_caption, downscale


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--poses", type=Path, default=Path("data/rl/motion_repertoire/poses_targeted.json"))
    ap.add_argument("--out", type=Path, required=True)
    a = ap.parse_args()
    poses = json.load(open(a.poses))
    env = pr._ensure()
    sim = env.sim
    m, d = sim.model, sim.data
    w, h, fps = 1280, 720, 30
    m.vis.global_.offwidth, m.vis.global_.offheight, m.vis.quality.shadowsize = w, h, 8192
    rend = mujoco.Renderer(m, height=h, width=w)
    cam = mujoco.MjvCamera()
    cam.distance, cam.elevation = 1.0, -22
    a.out.parent.mkdir(parents=True, exist_ok=True)
    ff = subprocess.Popen(["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{w}x{h}", "-r", str(fps), "-i", "-", "-an", "-c:v", "libx264", "-pix_fmt", "yuv420p", str(a.out)], stdin=subprocess.PIPE)
    for name in ("reach", "bridge", "coil", "low"):
        if name not in poses:
            continue
        th = np.array(poses[name]["theta"])
        ds = poses[name]["desc"]
        sim.set_terrain(env.flat)
        mujoco.mj_forward(m, d)
        z0 = float(d.xipos[1:9, 2].max())
        cap = f"HELD POSE (no angle limits, flat ground): {name.upper()} | top {ds['top']*100:.1f} cm (rest {z0*100:.1f}) | bridge {ds['bridge']*100:+.1f} cm | extent {ds['compact']*100:.1f} cm | max joint angle {np.abs(th).max():.1f} rad"
        t = 0.0
        for i in range(int(12 * fps)):
            for _ in range(int(0.5 / (fps * 0.05)) or 1):
                pass
            if i % max(1, int(fps * 0.05)) == 0 or True:
                sim.step(th, 1.0 / fps)
            t += 1.0 / fps
            cam.azimuth = 140.0 + (360.0 * max(0.0, t - 4.0) / 8.0)
            cam.lookat[:] = d.subtree_com[1]
            rend.update_scene(d, camera=cam)
            ff.stdin.write(burn_caption(downscale(rend.render(), (w, h)), f"{cap[:150]}  t={t:4.1f}s").tobytes())
    ff.stdin.close()
    ff.wait()
    print("wrote", a.out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
