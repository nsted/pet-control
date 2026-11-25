"""Render the best hoop-rolling gaits from hoop_gait_search.py (flat ground, open loop, side view so the head-over-tail turning is visible). Sim only.

    python scripts/rl/render_hoop.py --in data/rl/motion_repertoire/hoop.json --out docs/media/hoop_20261125.mp4
"""

from __future__ import annotations

import argparse
import json
import subprocess
from pathlib import Path

import mujoco
import numpy as np

import hoop_gait_search as hg
import ridge_gait_search as rg
from petctl.rl.video import burn_caption, downscale


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--in", dest="inp", type=Path, default=Path("data/rl/motion_repertoire/hoop.json"))
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--seconds", type=float, default=20.0)
    ap.add_argument("--n", type=int, default=2)
    a = ap.parse_args()
    res = json.load(open(a.inp))
    runs = [(np.array(res["theta"]), res["metrics"])] + [(np.array(r["theta"]), r["metrics"]) for r in res["top"][1:a.n]]
    env, _ = hg._ensure()
    w, h, fps = 1280, 720, 30
    m = env.sim.model
    m.vis.global_.offwidth, m.vis.global_.offheight, m.vis.quality.shadowsize = w, h, 8192
    rend = mujoco.Renderer(m, height=h, width=w)
    cam = mujoco.MjvCamera()
    cam.distance, cam.elevation, cam.azimuth = 1.3, -8, 90
    a.out.parent.mkdir(parents=True, exist_ok=True)
    ff = subprocess.Popen(["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{w}x{h}", "-r", str(fps), "-i", "-", "-an", "-c:v", "libx264", "-pix_fmt", "yuv420p", str(a.out)], stdin=subprocess.PIPE)
    scale = float(env.cfg.action_scale_rad)
    for i, (th, mt) in enumerate(runs):
        env.reset(seed=1)
        env.sim.set_terrain(env.flat)
        d = env.sim.data
        cap = f"HOOP-ROLL SEARCH (open loop, flat ground) #{i + 1}: {mt['pitch']:+.1f} head-over-tail turns, {mt['dist']:.2f} m travelled, ring closure {mt['ring']:.2f}"
        t, nxt = 0.0, 0.0
        while t < a.seconds:
            _, _, te, tr, _ = env.step(np.concatenate([rg.controller(th, t) / scale, [-1.0]]))
            t += env.dt
            while nxt <= t:
                cam.lookat[:] = d.subtree_com[1]
                rend.update_scene(d, camera=cam)
                ff.stdin.write(burn_caption(downscale(rend.render(), (w, h)), f"{cap}  t={t:4.1f}s").tobytes())
                nxt += 1.0 / fps
            if te or tr:
                break
    ff.stdin.close()
    ff.wait()
    print("wrote", a.out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
