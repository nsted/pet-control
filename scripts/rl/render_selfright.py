"""Render a self-righting result (data/rl/self_right/*.json) from each start orientation (inverted 180 deg, side 90 deg), captioned with the whole-body upright number.

    python scripts/rl/render_selfright.py data/rl/self_right/n11.json --out docs/media/selfright_n11.mp4
"""

from __future__ import annotations

import argparse
import json
import math
import subprocess
from pathlib import Path

import mujoco
import numpy as np

from petctl.rl.cpg import CPG
from petctl.rl.gait_eval import build_sim
from petctl.rl.video import burn_caption
from self_right import DT, SECONDS, STARTS

W, H, FPS = 1280, 720, 20


def main() -> int:
    """CLI entry point."""
    ap = argparse.ArgumentParser()
    ap.add_argument("result", type=Path)
    ap.add_argument("--out", type=Path, required=True)
    a = ap.parse_args()
    r = json.loads(a.result.read_text())
    n, nj, p = r["n_modules"], r["n_modules"] - 1, np.array(r["params"])
    sim = build_sim(n)
    sim.model.vis.global_.offwidth, sim.model.vis.global_.offheight = W, H
    rend = mujoco.Renderer(sim.model, height=H, width=W)
    cam = mujoco.MjvCamera()
    cam.distance, cam.elevation, cam.azimuth = 1.1, -30.0, 120.0
    ff = subprocess.Popen(["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{W}x{H}", "-r", str(FPS), "-i", "-", "-an", "-c:v", "libx264", "-pix_fmt", "yuv420p", str(a.out)],
                          stdin=subprocess.PIPE)
    assert ff.stdin is not None
    for orient, name in zip(STARTS, ("INVERTED start (180 deg)", "SIDE start (90 deg)")):
        sim.reset()
        sim.data.qpos[3:7] = [math.cos(orient / 2), math.sin(orient / 2), 0.0, 0.0]
        sim.data.qpos[2] += 0.04
        sim.step(np.zeros(nj), 0.8)
        cpg = CPG(nj, smooth_hz=0.0)
        cpg.reset(p)
        d = sim.data
        for i in range(int(SECONDS / DT)):
            sim.step(cpg.step(DT, p), DT)
            up = np.array([d.xmat[sim.m0_id + k].reshape(3, 3)[2, 2] for k in range(n)])
            cam.lookat[:] = d.xipos[sim.m0_id:sim.m0_id + n].mean(axis=0)
            rend.update_scene(d, camera=cam)
            cap = f"{n} modules | dynamic self-righting, {name} | module up-component median {np.median(up):+.2f} worst {up.min():+.2f} (1 = belly down) | open loop CPG, 0.5 N.m cap, sim only | t={i * DT:4.1f}s"
            ff.stdin.write(burn_caption(rend.render(), cap).tobytes())
    ff.stdin.close()
    ff.wait()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
