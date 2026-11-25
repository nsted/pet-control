"""Render a captioned montage of repertoire motions (flat ground, open loop): one segment per selected archive cell. Sim only.

    python scripts/rl/render_repertoire.py --archive data/rl/motion_repertoire/archive.json --out docs/media/repertoire_montage_20261125.mp4
"""

from __future__ import annotations

import argparse
import json
import subprocess
from pathlib import Path

import mujoco
import numpy as np

import ridge_gait_search as rg
from petctl.rl.video import burn_caption, downscale


def pick(cells: list[dict]) -> list[tuple[str, dict]]:
    """A few clearly different motion types from the archive (by descriptor criteria)."""
    def best(label, score, ok=lambda c: True):
        pool = [c for c in cells if ok(c)]
        return (label, max(pool, key=score)) if pool else None
    d = lambda c, k: c["desc"][k]                                                         # noqa: E731
    sp = lambda c: float(np.hypot(d(c, "vx"), d(c, "vy")))                                # noqa: E731
    out = [best("most efficient (rolling + sidewinding)", lambda c: c["fitness"]),
           best("forward wriggle", lambda c: d(c, "vx"), lambda c: abs(d(c, "roll")) < 0.15 and abs(d(c, "yaw")) < 0.15),
           best("backward", lambda c: -d(c, "vx"), lambda c: abs(d(c, "yaw")) < 0.15),
           best("sidewinding", lambda c: abs(d(c, "vy")), lambda c: abs(d(c, "roll")) < 0.15 and abs(d(c, "vx")) < 0.03),
           best("turn in place (pivot)", lambda c: abs(d(c, "yaw")), lambda c: sp(c) < 0.015),
           best("fastest rolling", lambda c: abs(d(c, "roll")))]
    return [o for o in out if o is not None]


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--archive", type=Path, default=Path("data/rl/motion_repertoire/archive.json"))
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--seconds", type=float, default=14.0)
    a = ap.parse_args()
    cells = json.load(open(a.archive))["cells"]
    env, fwd = rg._make_env()
    w, h, fps = 1280, 720, 30
    m = env.sim.model
    m.vis.global_.offwidth, m.vis.global_.offheight, m.vis.quality.shadowsize = w, h, 8192
    rend = mujoco.Renderer(m, height=h, width=w)
    cam = mujoco.MjvCamera()
    cam.distance, cam.elevation, cam.azimuth = 1.1, -30, 140
    a.out.parent.mkdir(parents=True, exist_ok=True)
    ff = subprocess.Popen(["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{w}x{h}", "-r", str(fps), "-i", "-", "-an", "-c:v", "libx264", "-pix_fmt", "yuv420p", str(a.out)], stdin=subprocess.PIPE)
    scale = float(env.cfg.action_scale_rad)
    for label, c in pick(cells):
        env.reset(seed=1)
        env.sim.set_terrain(rg.ridge_terrain(env, fwd, 0.0))
        d = env.sim.data
        th = np.array(c["theta"])
        ds = c["desc"]
        cap = f"MOTION REPERTOIRE (open-loop search, flat ground): {label} | vx {ds['vx']:+.2f} vy {ds['vy']:+.2f} m/s, turn {ds['yaw']:+.2f} rad/s, roll {ds['roll']:+.2f} rev/s"
        t, nxt = 0.0, 0.0
        while t < a.seconds:
            q = rg.controller(th, t)
            _, _, te, tr, _ = env.step(np.concatenate([q / scale, [-1.0]]))
            t += env.dt
            while nxt <= t:
                cam.lookat[:] = d.subtree_com[1]
                rend.update_scene(d, camera=cam)
                ff.stdin.write(burn_caption(downscale(rend.render(), (w, h)), f"{cap[:150]}  t={t:4.1f}s").tobytes())
                nxt += 1.0 / fps
            if te or tr:
                break
    ff.stdin.close()
    ff.wait()
    print("wrote", a.out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
