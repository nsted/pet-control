"""Render a MAP-Elites archive as one captioned video: the best straight gait in each travel direction, the fastest rolling cells and the fastest turning cells.

    python scripts/rl/render_archive.py data/rl/map_elites/n11_s0.json --out docs/media/me_n11_s0.mp4 --seconds 8
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import tempfile
from pathlib import Path

DIRS = ["forward", "forward-left", "left", "back-left", "reverse", "back-right", "right", "forward-right"]


def main() -> int:
    """CLI entry point."""
    ap = argparse.ArgumentParser()
    ap.add_argument("archive", type=Path)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--seconds", type=float, default=8.0)
    ap.add_argument("--rolls", type=int, default=3)
    ap.add_argument("--turns", type=int, default=2)
    a = ap.parse_args()
    arc = json.loads(a.archive.read_text())
    cells = arc["cells"]
    picks = []
    for d in range(8):
        c = [x for x in cells if x["cell"] == [d, x["cell"][1], x["cell"][2]] and x["cell"][0] == d and x["cell"][1] == 2 and x["cell"][2] == 1]
        if c:
            picks.append((f"straight {DIRS[d]}", c[0]))
    rolls = sorted([x for x in cells if x["cell"][1] in (0, 4)], key=lambda x: -x["metrics"]["speed"])[:a.rolls]
    picks += [(f"rolling {x['metrics']['roll_rate']:+.1f} rad/s", x) for x in rolls]
    turns = sorted([x for x in cells if x["cell"][2] in (0, 2) and x["cell"][1] == 2], key=lambda x: -abs(x["metrics"]["yaw_rate"]))[:a.turns]
    picks += [(f"turning {x['metrics']['yaw_rate']:+.1f} rad/s", x) for x in turns]
    tmp = Path(tempfile.mkdtemp(prefix="me_render_"))
    files = []
    for i, (label, x) in enumerate(picks):
        f = tmp / f"{i:02d}.json"
        f.write_text(json.dumps({"n_modules": arc["n_modules"], "objective": f"MAP-Elites {label}", "params": x["params"], "metrics": x["metrics"]}))
        files.append(str(f))
    return subprocess.call([sys.executable, "scripts/rl/render_params.py", "--out", str(a.out), "--seconds", str(a.seconds), *files])


if __name__ == "__main__":
    raise SystemExit(main())
