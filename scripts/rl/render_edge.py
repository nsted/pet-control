"""Render edge-avoidance episodes (docs/plans/EDGE_AVOIDANCE.md) as a captioned video: platform scene, one clip per episode, caption = policy, mode, time, modules overhanging (the viewer's label, not a sensor).

    python scripts/rl/render_edge.py --policy wander --seeds 90000 90001 90002 --out docs/media/edge_wander.mp4
    python scripts/rl/render_edge.py --policy oracle --snap --halt 1.0 --seeds ... --out docs/media/edge_oracle.mp4
"""

from __future__ import annotations

import argparse
import subprocess
from pathlib import Path

import mujoco
import numpy as np

from edge_reflex import Detector, Reflex
from edge_sim import Scene, Stand, Wander
from petctl.rl.video import burn_caption

W, H, FPS = 1280, 720, 20


def main() -> int:
    """CLI entry point."""
    ap = argparse.ArgumentParser()
    ap.add_argument("--policy", choices=("wander", "stand", "oracle", "detector"), default="wander")
    ap.add_argument("--seeds", type=int, nargs="+", default=[90000, 90001, 90002])
    ap.add_argument("--seconds", type=float, default=40.0)
    ap.add_argument("--detector", type=Path, default=Path("data/rl/edge/detector2_look0.npz"))
    ap.add_argument("--snap", action="store_true")
    ap.add_argument("--halt", type=float, default=0.0)
    ap.add_argument("--dist", type=float, default=None, help="oracle: trigger when the COM is within this distance of the edge")
    ap.add_argument("--out", type=Path, required=True)
    a = ap.parse_args()
    sc = Scene()
    sc.sim.model.vis.global_.offwidth, sc.sim.model.vis.global_.offheight = W, H
    rend = mujoco.Renderer(sc.sim.model, height=H, width=W)
    cam = mujoco.MjvCamera()
    cam.distance, cam.elevation, cam.azimuth = 2.6, -38.0, 100.0
    ff = subprocess.Popen(["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{W}x{H}", "-r", str(FPS), "-i", "-", "-an", "-c:v", "libx264", "-pix_fmt", "yuv420p", str(a.out)],
                          stdin=subprocess.PIPE)
    assert ff.stdin is not None
    for seed in a.seeds:
        ctl = Wander() if a.policy == "wander" else Stand() if a.policy == "stand" else Reflex(Detector(a.detector) if a.policy == "detector" else None, oracle=(a.policy == "oracle"), snap=a.snap, halt_s=a.halt, oracle_dist=a.dist)
        state = {"fell_t": None}

        def cb(scene: Scene, t: float, mode: str) -> None:
            p = scene.positions()
            oh = int(((np.abs(p[:, 0]) > scene.half) | (np.abs(p[:, 1]) > scene.half)).sum())
            if p[:, 2].min() < scene.H - 0.08 and state["fell_t"] is None:
                state["fell_t"] = t
            cam.lookat[:] = [0.0, 0.0, scene.H]
            rend.update_scene(scene.sim.data, camera=cam)
            status = f"FELL at {state['fell_t']:.1f} s" if state["fell_t"] is not None else "on the platform"
            ff.stdin.write(burn_caption(rend.render(), f"11 modules | edge avoidance, {100 * scene.H:.0f} cm platform | policy {a.policy}{' snap' if a.snap else ''} | mode {mode} | modules over the edge {oh} | {status} | t={t:4.1f}s").tobytes())

        lg = sc.run(ctl, seed, a.seconds, frame_cb=cb)
        print(f"seed {seed}: fell={lg.fell} t={lg.seconds:.1f}s path={lg.path_m:.1f} m", flush=True)
        for _ in range(int(0.8 * FPS)):                                                                  # hold the last frame briefly
            ff.stdin.write(burn_caption(rend.render(), "end of episode").tobytes())
    ff.stdin.close()
    ff.wait()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
