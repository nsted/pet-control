"""Render the best gap-bridging poses (scripts/rl/gap_statics.py results) as one captioned video: per length, the widest gap that was bridged, replayed exactly as in the search
(start lying across both platforms, 1 s ramp to the pose, 5 s hold), camera follows the body. Reads data/rl/statics/gap.json only. docs/media is git-ignored.

    python scripts/rl/render_gap.py --out docs/media/gap_bridge_progress.mp4 [--lengths 10 11 12 13]
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import gap_statics as gs  # noqa: E402

from petctl.rl.video import burn_caption  # noqa: E402

W, H, FPS = 1280, 720, 20


def main() -> int:
    """CLI entry point."""
    ap = argparse.ArgumentParser()
    ap.add_argument("--results", type=Path, default=Path("data/rl/statics/gap.json"))
    ap.add_argument("--lengths", type=int, nargs="+", default=[10, 11, 12, 13])
    ap.add_argument("--out", type=Path, default=Path("docs/media/gap_bridge_progress.mp4"))
    a = ap.parse_args()
    rows = json.loads(a.results.read_text())
    a.out.parent.mkdir(parents=True, exist_ok=True)
    ff = subprocess.Popen(["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{W}x{H}", "-r", str(FPS), "-i", "-", "-an", "-c:v", "libx264",
                           "-pix_fmt", "yuv420p", str(a.out)], stdin=subprocess.PIPE)
    assert ff.stdin is not None
    for n in a.lengths:
        ok = [r for r in rows if r["n_modules"] == n and r["bridged"]]
        rec = max(ok, key=lambda r: r["gap_cm"]) if ok else None
        if rec is None:
            continue
        g = rec["gap_cm"]
        sim = gs.sim_for(n, g)
        sim.model.vis.global_.offwidth, sim.model.vis.global_.offheight = W, H
        rend = mujoco.Renderer(sim.model, height=H, width=W)
        cam = mujoco.MjvCamera()
        cam.distance, cam.elevation, cam.azimuth = 1.3, -18.0, 100.0
        q = np.array(rec["q"])
        sim.reset()
        sim.data.qpos[2] += gs.PLATFORM_H + 0.004
        sim.step(np.zeros(n - 1), gs.SETTLE_S)
        cap = f"{n} modules | widest gap bridged between two 12 cm platforms: {g} cm | lowest module over the gap {rec['z_min_cm']:.1f} cm | 0.5 N.m cap, single held pose | statics search, sim only"
        dt, total = 1.0 / FPS, int((gs.RAMP_S + gs.HOLD_S) / (1.0 / FPS)) + 10
        for i in range(total):
            sim.step(min(1.0, (i + 1) * dt / gs.RAMP_S) * q, dt)
            cam.lookat[:] = [0.0, 0.0, 0.12]
            rend.update_scene(sim.data, camera=cam)
            ff.stdin.write(burn_caption(rend.render(), f"{cap}  t={i * dt:4.1f}s").tobytes())
        rend.close()
        print(f"rendered n={n} gap={g} cm", flush=True)
    ff.stdin.close()
    ff.wait()
    print(f"wrote {a.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
