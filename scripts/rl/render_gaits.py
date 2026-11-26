"""Render the best CPG gait per behaviour from the Phase 3 sweep (data/rl/cpg_sweep/L<n>.json) as one captioned video: forward, reverse, left, right, turn left, turn right, roll.
Each gait is replayed exactly as in the sweep (settle 0.8 s, open-loop CPG, 0.5 N.m cap, isotropic friction 1.0), camera follows the body centre. docs/media is git-ignored.

    python scripts/rl/render_gaits.py --n 11 --seconds 14 --out docs/media/gaits_n11.mp4
"""

from __future__ import annotations

import argparse
import json
import subprocess
from pathlib import Path

import mujoco
import numpy as np

from petctl.rl.cpg import CPG
from petctl.rl.gait_eval import CONTROL_DT, build_sim
from petctl.rl.video import burn_caption

W, H, FPS = 1280, 720, 20
BEHAVIOURS = [("forward", lambda r: r["vx"], "vx"), ("reverse", lambda r: -r["vx"], "vx"), ("sideways left", lambda r: r["vy"], "vy"), ("sideways right", lambda r: -r["vy"], "vy"),
              ("turn left", lambda r: r["yaw_rate"], "yaw_rate"), ("turn right", lambda r: -r["yaw_rate"], "yaw_rate"), ("roll", lambda r: abs(r["roll_rate"]), "roll_rate")]


def main() -> int:
    """CLI entry point."""
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=11)
    ap.add_argument("--sweep", type=Path, default=Path("data/rl/cpg_sweep"))
    ap.add_argument("--seconds", type=float, default=14.0)
    ap.add_argument("--out", type=Path, default=Path("docs/media/gaits.mp4"))
    a = ap.parse_args()
    n, nj = a.n, a.n - 1
    rows = [r for r in json.loads((a.sweep / f"L{n}.json").read_text()) if r["ok"] and not r["flipped"]]
    sim = build_sim(n)
    sim.model.vis.global_.offwidth, sim.model.vis.global_.offheight = W, H
    rend = mujoco.Renderer(sim.model, height=H, width=W)
    cam = mujoco.MjvCamera()
    cam.distance, cam.elevation, cam.azimuth = 1.0, -30.0, 120.0
    a.out.parent.mkdir(parents=True, exist_ok=True)
    ff = subprocess.Popen(["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{W}x{H}", "-r", str(FPS), "-i", "-", "-an", "-c:v", "libx264",
                           "-pix_fmt", "yuv420p", str(a.out)], stdin=subprocess.PIPE)
    assert ff.stdin is not None
    names = ("f", "dphi", "psi", "a_yaw", "a_twist")
    for label, key, _ in BEHAVIOURS:
        r = max(rows, key=key)
        p = dict(zip(names, r["params"][:5]))
        cpg = CPG(nj, smooth_hz=0.0)
        cpg.reset(np.array(r["params"]))
        sim.reset()
        sim.step(np.zeros(nj), 0.8)
        d = sim.data
        x0 = d.xipos[sim.m0_id:sim.m0_id + n].mean(axis=0).copy()
        cap = (f"{n} modules | {label} gait | CPG f {p['f']:.1f} Hz, yaw amp {p['a_yaw']:.1f}, twist amp {p['a_twist']:.1f}, lag {p['dphi']:+.1f}, psi {p['psi']:+.2f} | "
               f"sweep result: vx {100 * r['vx']:+.1f} vy {100 * r['vy']:+.1f} cm/s, yaw {r['yaw_rate']:+.2f} roll {r['roll_rate']:+.2f} rad/s | 0.5 N.m cap, open loop, sim only")
        for i in range(int(a.seconds / CONTROL_DT)):
            sim.step(cpg.step(CONTROL_DT, np.array(r["params"])), CONTROL_DT)
            com = d.xipos[sim.m0_id:sim.m0_id + n].mean(axis=0)
            cam.lookat[:] = com
            rend.update_scene(d, camera=cam)
            ff.stdin.write(burn_caption(rend.render(), f"{cap}  t={i * CONTROL_DT:4.1f}s").tobytes())
        print(f"rendered {label}: moved {100 * np.linalg.norm((d.xipos[sim.m0_id:sim.m0_id + n].mean(axis=0) - x0)[:2]):.0f} cm", flush=True)
    ff.stdin.close()
    ff.wait()
    print(f"wrote {a.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
