"""Render ONE best-episode video for the M2 v3 hug stage (owner rule: one best video per STAGE, at the end of the stage; nothing else is rendered).

Waits until the m2v3 driver has exited, then renders the best successful gate episode of the best m2_hug3 run (highest hug success, then hold_duty median).

    nohup python scripts/rl/m2v3_render_watcher.py > data/rl/m2v3/watcher.log 2>&1 &
"""
from __future__ import annotations

import json
import subprocess
import time
from pathlib import Path
from typing import Any

from petctl.rl import video
from petctl.rl.curriculum.gate2 import _gate_episode
from petctl.rl.curriculum.stages import get_stage

ROOT = Path("data/rl/m2v3")
STAGE = "m2_hug3"


def _running() -> bool:
    return subprocess.run(["pgrep", "-f", "m2v3_driver"], capture_output=True).returncode == 0


def _lines() -> list[dict[str, Any]]:
    f = ROOT / "summary.jsonl"
    return [json.loads(x) for x in f.read_text().splitlines() if x.strip()] if f.exists() else []


def main() -> None:
    """Poll until the driver is gone, then render the stage video once."""
    while _running():
        time.sleep(120)
    rows = [d for d in _lines() if d["task"] == STAGE and d.get("success") is not None]
    for best in sorted(rows, key=lambda d: (d["success"], d.get("hold_duty_median") or 0.0), reverse=True):
        if best["success"] <= 0.0:
            break
        run = ROOT / str(best["L"]) / STAGE / f"seed{best['seed']}"
        res, seed = _gate_episode(run, STAGE, None)
        st = get_stage(STAGE)
        out = video.MEDIA_DIR / f"{STAGE}_STAGE_best_L{best['L']}_seed{best['seed']}_{time.strftime('%Y%m%d_%H%M%S')}.mp4"
        label = f"M2 v3 stage {STAGE}: best run L={best['L']} seed{best['seed']} (hug success {best['success']:.2f}); {st.description}"[:200]
        video.record_video(run, out, seconds=res.get("episode_s"), seed=seed, dr_scale=float(res["dr_scale"]), task_params=res["eval_task_params"], label=label,
                           replace_task_params=True, distance=0.9)
        print("rendered", out, flush=True)
        return
    print("no successful episode to render", STAGE, flush=True)


if __name__ == "__main__":
    main()
