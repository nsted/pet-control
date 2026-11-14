"""Render ONE best-episode video per STAGE of the m2v6 schedule (owner rule: one best video per stage at the stage end, nothing else is rendered).

m2_flat3: when the flat main run has its screening line (best successful screening episode of the flat main run). m2_hug6: when the final held-out gate line exists (best successful gate episode of hug run 2).
Exits when the driver has gone and every stage that can be rendered has been (or the schedule stopped). Markers ``<root>/rendered_<stage>`` make it idempotent.

    nohup python scripts/rl/m2v6_render_watcher.py --length 10 > data/rl/m2v6/watcher.log 2>&1 &
"""
from __future__ import annotations

import argparse
import json
import subprocess
import time
from pathlib import Path
from typing import Any

from petctl.rl import video
from petctl.rl.curriculum.gate2 import render_best

POLL_S = 60


def _running() -> bool:
    return subprocess.run(["pgrep", "-f", "m2v6_driver"], capture_output=True).returncode == 0


def _lines(root: Path) -> list[dict[str, Any]]:
    f = root / "summary.jsonl"
    return [json.loads(x) for x in f.read_text().splitlines() if x.strip()] if f.exists() else []


def _render(root: Path, stage: str, run: Path) -> None:
    mark = root / f"rendered_{stage}"
    if mark.exists():
        return
    try:
        out = render_best(run, stage)
        print("rendered", out, flush=True)
    except ValueError as e:                                  # no successful episode to show
        print(f"no video for {stage}: {e}", flush=True)
        out = None
    mark.write_text(str(out))


def main() -> None:
    """Poll until the driver is gone, rendering each stage's video once its stage end has been recorded."""
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", type=Path, default=Path("data/rl/m2v6"))
    ap.add_argument("--length", type=int, default=10)
    a = ap.parse_args()
    base = a.root / f"L{a.length}"
    while True:
        alive = _running()
        ph = {d["phase"] for d in _lines(a.root) if d.get("go")}
        if "flat_main" in ph:
            _render(a.root, "m2_flat3", base / "flat_main")
        if "final" in ph:
            _render(a.root, "m2_hug6", base / "hug_run2")
        if not alive and ((a.root / "DONE").exists() or any(a.root.glob("STOP_*"))):
            return
        time.sleep(POLL_S)


if __name__ == "__main__":
    main()
