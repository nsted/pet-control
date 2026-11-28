"""Captioned video of the PD pose-hold baseline preview (T8): the hard cases (head lift lost, tall lattice pose at kp x4) next to a held pose. Sim only.

    python scripts/rl/render_pd_hold.py --out docs/media/s2_hold/pd_baseline/pd_hold_preview.mp4
Headroom preview only: it runs ``pose_hold_baseline.run``-style episodes outside PetEnv (no delay, bus or thermal model); the gate baseline is T14.
"""

from __future__ import annotations

import argparse
import math
import subprocess
from pathlib import Path

import mujoco
import numpy as np

from pose_hold_baseline import DT, disturbance_list, enter, poses
from petctl.rl.gait_eval import build_sim
from petctl.rl.video import burn_caption, downscale

CASES = [("head lift", 1.0, "push head 5N"), ("lattice tall", 4.0, "none"), ("lattice tall", 2.0, "push head 5N"), ("lattice mid", 2.0, "push head 5N")]


def main() -> int:
    """CLI entry point."""
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", type=Path, required=True)
    a = ap.parse_args()
    sim = build_sim(11)
    m, d = sim.model, sim.data
    w, h, fps = 1280, 720, 20
    m.vis.global_.offwidth, m.vis.global_.offheight = w, h
    rend = mujoco.Renderer(m, height=h, width=w)
    cam = mujoco.MjvCamera()
    cam.distance, cam.elevation = 1.1, -20
    a.out.parent.mkdir(parents=True, exist_ok=True)
    ff = subprocess.Popen(["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{w}x{h}", "-r", str(fps), "-i", "-", "-an", "-c:v", "libx264", "-pix_fmt", "yuv420p", str(a.out)],
                          stdin=subprocess.PIPE)
    P = poses()
    dists = {x["name"]: x for x in disturbance_list(sim.m0_id)}
    for pname, ks, dname in CASES:
        q, order = P[pname]
        dist = dists[dname]
        kp0, kd0 = sim.gains()
        sim.set_gains(kp0 * ks, kd0 * math.sqrt(ks))
        enter(sim, q, order)
        body = sim.m0_id + dist.get("body", 0)
        t = 0.0
        err_max = 0.0
        for i in range(int(6.0 / DT)):
            d.xfrc_applied[:] = 0.0
            if "f" in dist and i * DT < 3.0:
                d.xfrc_applied[body, :3] = dist["f"]
            sim.step(q, DT)
            t += DT
            e = float(np.abs(sim.joint_pos() - q).max())
            err_max = max(err_max, e)
            cam.azimuth = 140.0 + 20.0 * t
            cam.lookat[:] = d.subtree_com[1]
            rend.update_scene(d, camera=cam)
            state = "LOST" if e > 0.6 else "holding"
            cap = f"S2 hold, PD BASELINE PREVIEW (n=11, sim, headroom preview not gate): pose={pname} kp x{ks:g} disturbance={dname} | max joint err {e:.2f} rad ({state}) | t={t:3.1f}s"
            ff.stdin.write(burn_caption(downscale(rend.render(), (w, h)), cap).tobytes())
        d.xfrc_applied[:] = 0.0
        sim.set_gains(kp0, kd0)
    ff.stdin.close()
    ff.wait()
    print("wrote", a.out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
