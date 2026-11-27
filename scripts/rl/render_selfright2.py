"""Render a two-stage self-righting result (self_right2.py) from the inverted and the side start; the caption shows the stage and the whole-body up numbers.

    python scripts/rl/render_selfright2.py data/rl/self_right/n11_v2.json --out docs/media/selfright2_n11.mp4
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
from self_right2 import DT, SECONDS, STARTS

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
        d, t_stop, ok_ticks, fade = sim.data, None, 0, 1.0
        for i in range(int(SECONDS / DT)):
            if t_stop is None:
                sim.step(cpg.step(DT, p), DT)
            else:
                fade = max(0.0, fade - DT / 1.5)
                sim.step(cpg.step(DT, p) * fade, DT)
            up = np.array([d.xmat[sim.m0_id + k].reshape(3, 3)[2, 2] for k in range(n)])
            if t_stop is None:
                ok_ticks = ok_ticks + 1 if np.median(up) > 0.8 else 0
                if ok_ticks >= 6:
                    t_stop = i * DT
            cam.lookat[:] = d.xipos[sim.m0_id:sim.m0_id + n].mean(axis=0)
            rend.update_scene(d, camera=cam)
            stage = "stage 2: UPRIGHT detected, stopping and straightening" if t_stop is not None else "stage 1: CPG rolling motion"
            ff.stdin.write(burn_caption(rend.render(), f"{n} modules | two-stage self-righting, {name} | {stage} | up median {np.median(up):+.2f} worst {up.min():+.2f} | open loop + trigger, 0.5 N.m cap | t={i * DT:4.1f}s").tobytes())
    ff.stdin.close()
    ff.wait()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
