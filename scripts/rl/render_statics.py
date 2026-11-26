"""Render the best static poses found by scripts/rl/statics.py as one captioned video (sim only; reads data/rl/statics/results.json, never touches the search).

    python scripts/rl/render_statics.py --n 11 --out docs/media/statics_n11_progress.mp4
Each pose is replayed exactly as in the search (settle 0.8 s, 1 s ramp to the pose, hold 5 s), camera follows the body centre. docs/media is git-ignored.
"""

from __future__ import annotations

import argparse
import json
import math
import subprocess
import sys
from pathlib import Path

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import statics  # noqa: E402

from petctl.rl.video import burn_caption  # noqa: E402

W, H, FPS = 1280, 720, 20
TITLES = {"head_lift": "highest held head lift", "arch": "widest held arch (gap-bridge proxy)", "self_right_inv": "best single held pose from an INVERTED start",
          "self_right_side": "best single held pose from a SIDE start"}


def pick(rows: list[dict], task: str, n: int) -> dict | None:
    """Best record (highest objective) of ``task`` at length ``n``."""
    c = [r for r in rows if r["task"] == task and r["n_modules"] == n]
    return max(c, key=lambda r: r["objective"]) if c else None


def main() -> int:
    """CLI entry point."""
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=11)
    ap.add_argument("--results", type=Path, default=Path("data/rl/statics/results.json"))
    ap.add_argument("--out", type=Path, default=Path("docs/media/statics_progress.mp4"))
    a = ap.parse_args()
    rows = json.loads(a.results.read_text())
    n = a.n
    sim = statics.sim_for(n)
    sim.model.vis.global_.offwidth, sim.model.vis.global_.offheight = W, H
    rend = mujoco.Renderer(sim.model, height=H, width=W)
    cam = mujoco.MjvCamera()
    cam.distance, cam.elevation, cam.azimuth = 1.1, -22.0, 125.0
    a.out.parent.mkdir(parents=True, exist_ok=True)
    ff = subprocess.Popen(["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{W}x{H}", "-r", str(FPS), "-i", "-", "-an", "-c:v", "libx264",
                           "-pix_fmt", "yuv420p", str(a.out)], stdin=subprocess.PIPE)
    assert ff.stdin is not None
    nj, dt = n - 1, 1.0 / FPS
    for task in ("head_lift", "arch", "self_right_inv", "self_right_side"):
        rec = pick(rows, task, n)
        if rec is None:
            continue
        q = np.array(rec["q"])
        orient = {"self_right_inv": 1, "self_right_side": 2}.get(task, 0)
        sim.reset()
        if orient:
            roll = {1: math.pi, 2: math.pi / 2}[orient]
            sim.data.qpos[3:7] = [math.cos(roll / 2), math.sin(roll / 2), 0.0, 0.0]
            sim.data.qpos[2] += 0.04
        sim.step(np.zeros(nj), statics.SETTLE_S)
        main_val = rec.get("head_gain_cm", rec.get("span_cm"))
        result = f"head +{main_val:.1f} cm" if task == "head_lift" else (f"span {main_val:.1f} cm" if task == "arch" else f"mean uprightness {rec['cos_up_mean']:.2f}, worst module {rec['cos_up_min']:+.2f}")
        cap = f"{n} modules | {TITLES[task]} | 0.5 N.m cap, single held pose | result: {result} | statics search, sim only"
        total = int((statics.RAMP_S + statics.HOLD_S) / dt) + 10
        for i in range(total):
            frac = min(1.0, (i + 1) * dt / statics.RAMP_S)
            sim.step(frac * q, dt)
            cam.lookat[:] = sim.data.xipos[sim.m0_id:sim.m0_id + n].mean(axis=0)
            rend.update_scene(sim.data, camera=cam)
            ff.stdin.write(burn_caption(rend.render(), f"{cap}  t={i * dt:4.1f}s").tobytes())
        print(f"rendered {task}: {result}", flush=True)
    ff.stdin.close()
    ff.wait()
    print(f"wrote {a.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
