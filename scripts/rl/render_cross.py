"""Render dynamic gap-crossing results (data/rl/gap_cross/*.json from gap_cross.py) as one captioned video; each clip runs until the crossing succeeds (+2 s) or 25 s.

    python scripts/rl/render_cross.py --out docs/media/gap_cross.mp4 data/rl/gap_cross/n11_g10.json [...]
"""

from __future__ import annotations

import argparse
import json
import math
import subprocess
from pathlib import Path

import mujoco
import numpy as np

from gap_cross import DT, H, SECONDS, sim_for
from petctl.rl.cpg import CPG
from petctl.rl.video import burn_caption

W, Hh, FPS = 1280, 720, 20


def main() -> int:
    """CLI entry point."""
    ap = argparse.ArgumentParser()
    ap.add_argument("results", type=Path, nargs="+")
    ap.add_argument("--out", type=Path, required=True)
    a = ap.parse_args()
    ff = subprocess.Popen(["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{W}x{Hh}", "-r", str(FPS), "-i", "-", "-an", "-c:v", "libx264", "-pix_fmt", "yuv420p", str(a.out)],
                          stdin=subprocess.PIPE)
    assert ff.stdin is not None
    for path in a.results:
        r = json.loads(path.read_text())
        n, gap = r["n_modules"], r["gap_cm"]
        sim = sim_for(n, gap)
        sim.model.vis.global_.offwidth, sim.model.vis.global_.offheight = W, Hh
        rend = mujoco.Renderer(sim.model, height=Hh, width=W)
        cam = mujoco.MjvCamera()
        cam.distance, cam.elevation, cam.azimuth = 1.5, -30.0, 90.0
        g, d, nj = gap / 100.0, sim.data, n - 1
        sim.reset()
        d.qpos[0], d.qpos[2] = -(g / 2 + 0.55), d.qpos[2] + H + 0.004
        d.qpos[3:7] = [math.cos(r["yaw0"] / 2), 0, 0, math.sin(r["yaw0"] / 2)]
        sim.step(np.zeros(nj), 0.8)
        p = np.array(r["params"])
        cpg = CPG(nj, smooth_hz=0.0)
        cpg.reset(p)
        t_end, done_at = SECONDS, None
        for i in range(int(SECONDS / DT)):
            sim.step(cpg.step(DT, p), DT)
            q = d.xipos[sim.m0_id:sim.m0_id + n]
            if done_at is None and (q[:, 2] > 0.09).all() and q[:, 0].min() > g / 2 + 0.01 and q[:, 0].max() < g / 2 + 1.0:
                done_at = i * DT
                t_end = min(t_end, done_at + 2.0)
            if i * DT > t_end:
                break
            cam.lookat[:] = [q[:, 0].mean() * 0.6, 0.0, 0.1]
            rend.update_scene(d, camera=cam)
            state = f"CROSSED at {done_at:.1f} s" if done_at is not None else "crossing..."
            cap = f"{n} modules | dynamic crossing of a {gap} cm gap between 12 cm platforms, starts fully on the left one | {state} | search start '{r['start']}', open loop, 0.5 N.m cap | t={i * DT:4.1f}s"
            ff.stdin.write(burn_caption(rend.render(), cap).tobytes())
        rend.close()
        print(f"{path.name}: crossed at {done_at}", flush=True)
    ff.stdin.close()
    ff.wait()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
