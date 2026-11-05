"""Record a trained gait to MP4 (offscreen render, inference only). Run: python scripts/record_policy.py data/rl/<run> out.mp4."""
from __future__ import annotations

import argparse
from pathlib import Path

from petctl.rl.video import record_video


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("run", type=Path)
    ap.add_argument("out", type=Path)
    ap.add_argument("--seconds", type=float, default=15.0)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--dr-scale", type=float, default=0.0)
    ap.add_argument("--policy-file", default="policy.npz")
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--size", type=int, nargs=2, default=(960, 540), metavar=("W", "H"))
    a = ap.parse_args()
    m = record_video(a.run, a.out, a.policy_file, a.seconds, a.seed, a.dr_scale, a.fps, tuple(a.size))
    print(f"wrote {a.out}  " + "  ".join(f"{k}={x:.3f}" for k, x in m.items()))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
