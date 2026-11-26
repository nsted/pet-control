"""Render the best cylinder-wrap poses (scripts/rl/wrap_statics.py results) as one captioned video: per length, the largest cylinder that was wrapped >= 180 deg (hold_ok), replayed with the
search protocol (free settle, ramp, settle, hold), camera on the cylinder. Run with PETCTL_BUS_MOTOR=gim3505 (the model the search used). docs/media is git-ignored.

    PETCTL_BUS_MOTOR=gim3505 python scripts/rl/render_wrap.py --out docs/media/wrap_n10-13.mp4
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
from pathlib import Path

import mujoco
import numpy as np

from petctl.rl.shape_search import _sim_for
from petctl.rl.target_shapes import make_shape
from petctl.rl.video import burn_caption

W, H, FPS = 1280, 720, 20


def main() -> int:
    """CLI entry point."""
    assert os.environ.get("PETCTL_BUS_MOTOR") == "gim3505", "set PETCTL_BUS_MOTOR=gim3505"
    ap = argparse.ArgumentParser()
    ap.add_argument("--results", type=Path, default=Path("data/rl/statics/wrap.json"))
    ap.add_argument("--lengths", type=int, nargs="+", default=[10, 11, 12, 13])
    ap.add_argument("--out", type=Path, default=Path("docs/media/wrap.mp4"))
    a = ap.parse_args()
    rows = json.loads(a.results.read_text())
    a.out.parent.mkdir(parents=True, exist_ok=True)
    ff = subprocess.Popen(["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{W}x{H}", "-r", str(FPS), "-i", "-", "-an", "-c:v", "libx264",
                           "-pix_fmt", "yuv420p", str(a.out)], stdin=subprocess.PIPE)
    assert ff.stdin is not None
    for n in a.lengths:
        ok = [r for r in rows if r["n_modules"] == n and r["hold_ok"] and r["held_span"] >= 180.0]
        rec = max(ok, key=lambda r: (r["radius_cm"], r["held_span"])) if ok else None
        if rec is None:
            continue
        shape = make_shape("cyl", (rec["radius_cm"] / 100.0,), 0.0)
        hs = _sim_for(n, shape)
        frames: list[np.ndarray] = []
        hs.evaluate(np.array(rec["x"]), record=frames)
        model = hs.sim.model
        model.vis.global_.offwidth, model.vis.global_.offheight = W, H
        rend = mujoco.Renderer(model, height=H, width=W)
        d = mujoco.MjData(model)
        cam = mujoco.MjvCamera()
        cam.distance, cam.elevation, cam.azimuth = 0.75, -40.0, 130.0
        cam.lookat[:] = [rec["post_xy"][0], rec["post_xy"][1], 0.04]
        cap = (f"{n} modules | wraps a {rec['radius_cm']} cm radius cylinder: {rec['held_span']:.0f} deg held, {rec['held_run']} modules in contact | current proxy {rec['bus_mean_a']:.1f} A (GIM3505-8 model, unvalidated) "
               f"| 0.5 N.m cap, single held pose | statics search, sim only")
        for i, qp in enumerate(frames):
            d.qpos[:] = qp
            mujoco.mj_forward(model, d)
            rend.update_scene(d, camera=cam)
            ff.stdin.write(burn_caption(rend.render(), f"{cap}  t={i / FPS:4.1f}s").tobytes())
        rend.close()
        print(f"rendered n={n} r={rec['radius_cm']} cm span {rec['held_span']:.0f}", flush=True)
    ff.stdin.close()
    ff.wait()
    print(f"wrote {a.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
