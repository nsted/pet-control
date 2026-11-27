"""Render staged pose entries (data/rl/pose_stage_n11.json): the joint order, one joint folded per second, then the hold. Picks the tallest, most compact and a few structured (repeating-motif) poses.

    python scripts/rl/render_pose_entry.py --out docs/media/pose_entry_n11.mp4
"""

from __future__ import annotations

import argparse
import json
import math
import subprocess
from pathlib import Path

import mujoco
import numpy as np

from petctl.rl.gait_eval import build_sim
from petctl.rl.video import burn_caption
from pose_stage import STEP_S

W, H, FPS = 1280, 720, 20


def pick(res: list[dict], k_tall: int = 3, k_compact: int = 2, k_motif: int = 2) -> list[tuple[str, dict]]:
    """Choose the poses to show."""
    v = [r for r in res if r.get("valid")]
    out = [("tallest", r) for r in sorted(v, key=lambda r: -r["height_cm"])[:k_tall]]
    out += [("most compact (>=6 modules lifted)", r) for r in sorted([r for r in v if r["n_off_floor"] >= 6], key=lambda r: r["footprint_cm2"])[:k_compact]]
    motif = [r for r in v if any(r["seq"][i] == r["seq"][i + p] for p in (2, 3, 4) for i in range(len(r["seq"]) - p)) and len(set(r["seq"])) >= 3 and r["n_off_floor"] >= 6]
    seen = {tuple(r["seq"]) for _, r in out}
    out += [("repeating motif", r) for r in sorted(motif, key=lambda r: -r["height_cm"]) if tuple(r["seq"]) not in seen][:k_motif]
    return out


def main() -> int:
    """CLI entry point."""
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", type=Path, default=Path("data/rl/pose_stage_n11.json"))
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--n", type=int, default=11)
    a = ap.parse_args()
    picks = pick(json.loads(a.src.read_text()))
    n, nj = a.n, a.n - 1
    sim = build_sim(n)
    sim.model.vis.global_.offwidth, sim.model.vis.global_.offheight = W, H
    rend = mujoco.Renderer(sim.model, height=H, width=W)
    cam = mujoco.MjvCamera()
    cam.distance, cam.elevation, cam.azimuth = 0.9, -25.0, 130.0
    ff = subprocess.Popen(["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{W}x{H}", "-r", str(FPS), "-i", "-", "-an", "-c:v", "libx264", "-pix_fmt", "yuv420p", str(a.out)],
                          stdin=subprocess.PIPE)
    assert ff.stdin is not None
    d = sim.data

    def frame(cap: str) -> None:
        cam.lookat[:] = d.xipos[sim.m0_id:sim.m0_id + n].mean(axis=0)
        rend.update_scene(d, camera=cam)
        ff.stdin.write(burn_caption(rend.render(), cap).tobytes())

    for label, r in picks:
        seq, order = r["seq"], r["order"]
        q_t = np.array(seq, float) * math.pi / 2
        sim.reset()
        sim.step(np.zeros(nj), 0.5)
        cur = np.zeros(nj)
        base = f"{n} modules | pose {seq} (x90 deg) | {label}: {r['height_cm']:.0f} cm tall, {r['n_off_floor']} modules lifted, peak torque {r['tau_peak']:.2f} N.m | order {r['order_name']}"
        for k, j in enumerate(order):
            for i in range(int(STEP_S * FPS)):
                cur[j] = cur[j] + (q_t[j] - cur[j]) * (1.0 / (int(STEP_S * FPS) - i))
                sim.step(cur, 1.0 / FPS)
                frame(f"{base} | folding joint {j} ({k + 1}/{nj})")
        for i in range(3 * FPS):
            sim.step(q_t, 1.0 / FPS)
            frame(f"{base} | HOLD")
    ff.stdin.close()
    ff.wait()
    print(f"wrote {a.out}: {len(picks)} poses")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
