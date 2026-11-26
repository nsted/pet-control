"""Render gaits found by scripts/rl/gait_search_cma.py (result JSON files) as one captioned video, with a fading trail of the body centre, and print per-module roll rates as a sanity check.

    python scripts/rl/render_params.py --out docs/media/roll_n11.mp4 --seconds 14 data/rl/gait_search/roll_n11.json [more.json ...]
"""

from __future__ import annotations

import argparse
import json
import math
import subprocess
from pathlib import Path

import mujoco
import numpy as np

from petctl.rl.cpg import CPG, PARAM_NAMES
from petctl.rl.gait_eval import CONTROL_DT, _heading, build_sim
from petctl.rl.video import burn_caption

W, H, FPS = 1280, 720, 20


def main() -> int:
    """CLI entry point."""
    ap = argparse.ArgumentParser()
    ap.add_argument("results", type=Path, nargs="+")
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--seconds", type=float, default=14.0)
    a = ap.parse_args()
    a.out.parent.mkdir(parents=True, exist_ok=True)
    ff = subprocess.Popen(["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{W}x{H}", "-r", str(FPS), "-i", "-", "-an", "-c:v", "libx264",
                           "-pix_fmt", "yuv420p", str(a.out)], stdin=subprocess.PIPE)
    assert ff.stdin is not None
    for path in a.results:
        res = json.loads(path.read_text())
        n, nj, p = res["n_modules"], res["n_modules"] - 1, np.array(res["params"])
        m = res["metrics"]
        sim = build_sim(n)
        sim.model.vis.global_.offwidth, sim.model.vis.global_.offheight = W, H
        rend = mujoco.Renderer(sim.model, height=H, width=W)
        cam = mujoco.MjvCamera()
        cam.distance, cam.elevation, cam.azimuth = 1.1, -32.0, 120.0
        sim.reset()
        sim.step(np.zeros(nj), 0.8)
        _, left0, _ = _heading(sim)
        d = sim.data
        cpg = CPG(nj, smooth_hz=0.0)
        cpg.reset(p)
        mods = sorted({0, n // 4, n // 2, (3 * n) // 4, n - 1})
        ang = {k: [] for k in mods}
        cap = (f"{n} modules | CMA-ES gait, objective '{res['objective']}' | f {p[0]:.2f} Hz lag {p[1]:+.2f} psi {p[2]:+.2f} Ay {p[3]:.2f} At {p[4]:.2f} | search result: vx {100 * m['vx']:+.1f} vy {100 * m['vy']:+.1f} cm/s, "
               f"yaw {m['yaw_rate']:+.2f}, whole-body roll {m['roll_rate']:+.2f} rad/s | 0.5 N.m cap, open loop, sim only")
        for i in range(int(a.seconds / CONTROL_DT)):
            sim.step(cpg.step(CONTROL_DT, p), CONTROL_DT)
            cam.lookat[:] = d.xipos[sim.m0_id:sim.m0_id + n].mean(axis=0)
            for k in mods:
                ang[k].append(math.atan2(float(d.xmat[sim.m0_id + k].reshape(3, 3)[:, 2] @ left0), float(d.xmat[sim.m0_id + k].reshape(3, 3)[2, 2])))
            rend.update_scene(d, camera=cam)
            ff.stdin.write(burn_caption(rend.render(), f"{cap}  t={i * CONTROL_DT:4.1f}s").tobytes())
        rates = {k: (np.unwrap(v)[-1] - np.unwrap(v)[0]) / (len(v) * CONTROL_DT) for k, v in ang.items()}
        step = max(float(np.abs(np.diff(np.unwrap(v))).max()) for v in ang.values())
        print(f"{path.name}: per-module net roll rate (rad/s) {({k: round(float(v), 2) for k, v in rates.items()})}, max per-tick roll step {step:.2f} rad; params {dict(zip(PARAM_NAMES, np.round(p, 2)))}", flush=True)
        rend.close()
    ff.stdin.close()
    ff.wait()
    print(f"wrote {a.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
