"""Render ONE best-episode video per finished M2 v2 STAGE (owner rule: one best video per stage, never per run; at most 10 renders/hour). Polls the v2 summary until done.

  m2_hug2:  rendered as soon as the L=8 hug v2 run (seed0) has its gate result: its best successful gate episode.
  m2_flat2: rendered when all three L=8 flat runs exist (the sweep's m2_flat seed0 as the prior + the v2 seeds 1 and 2; seeds that failed takeoff do not count and are replaced by the
            next seed): the best episode of the best run (highest success, then lowest median time to goal).

    nohup python scripts/rl/m2_render_watcher.py > data/rl/m2v2/watcher.log 2>&1 &
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

ROOT = Path("data/rl/m2v2")
PRIOR_FLAT8 = ("m2_flat", Path("data/rl/m2_sweep/8/m2_flat/seed0"))      # (stage name of its gate2 json, run dir): the L=8 seed0 flat run of the v1 sweep
FLAT_RUNS_NEEDED = 3


def _running() -> bool:
    return subprocess.run(["pgrep", "-f", "m2v2_driver"], capture_output=True).returncode == 0


def _lines() -> list[dict[str, Any]]:
    f = ROOT / "summary.jsonl"
    return [json.loads(x) for x in f.read_text().splitlines() if x.strip()] if f.exists() else []


def _prior() -> dict[str, Any] | None:
    """The v1 L=8 flat seed0 gate result as a summary-like row."""
    f = PRIOR_FLAT8[1] / "gate2_m2_flat.json"
    if not f.exists():
        return None
    r = json.loads(f.read_text())["robust"]
    return {"success": r["rate"], "t_goal_median_s": r["metric_median"].get("time_to_goal_s"), "stage": PRIOR_FLAT8[0], "run": PRIOR_FLAT8[1], "label": "m2_flat sweep seed0 (prior)"}


def candidates(task: str, lines: list[dict[str, Any]]) -> list[dict[str, Any]] | None:
    """The finished L=8 runs of ``task`` to choose the stage's best from, or None while the stage is not complete."""
    if task == "m2_hug2":
        runs = [d for d in lines if d["task"] == task and d["L"] == 8 and d.get("success") is not None]
        return [{**d, "stage": task, "run": ROOT / "8" / task / f"seed{d['seed']}", "label": f"{task} L=8 seed{d['seed']}"} for d in runs] or None
    runs = [d for d in lines if d["task"] == task and d["L"] == 8 and d.get("success") is not None]
    rows = [{**d, "stage": task, "run": ROOT / "8" / task / f"seed{d['seed']}", "label": f"{task} L=8 seed{d['seed']}"} for d in runs]
    prior = _prior()
    if prior is not None:
        rows.append(prior)
    return rows if len(rows) >= FLAT_RUNS_NEEDED else None


def render_stage(task: str, rows: list[dict[str, Any]]) -> Path | None:
    """Best run of ``rows`` (highest success, then fastest), its best successful gate episode -> one video. None if no run has a successful episode."""
    for best in sorted(rows, key=lambda d: (d["success"], -(d.get("t_goal_median_s") or d.get("t_hug_median_s") or 1e9)), reverse=True):
        if best["success"] <= 0.0:
            break
        run: Path = best["run"]
        res, seed = _gate_episode(run, best["stage"], None)
        st = get_stage(best["stage"])
        out = video.MEDIA_DIR / f"{task}_STAGE_best_{run.parent.parent.name}_{run.parent.name}_{run.name}_{time.strftime('%Y%m%d_%H%M%S')}.mp4"
        label = f"M2 v2 stage {task}: best run {best['label']} (success {best['success']:.2f}); {st.description}"[:200]
        video.record_video(run, out, seconds=res.get("episode_s"), seed=seed, dr_scale=float(res["dr_scale"]), task_params=res["eval_task_params"], label=label,
                           replace_task_params=True, distance=0.9 if task == "m2_hug2" else 2.0)
        return out
    return None


def main() -> None:
    """Poll; render each v2 stage once when its runs are done."""
    done: set[str] = set()
    final = False
    while True:
        lines = _lines()
        for task in ("m2_hug2", "m2_flat2"):
            if task in done:
                continue
            rows = candidates(task, lines)
            if rows is None:
                continue
            try:
                out = render_stage(task, rows)
                print("rendered" if out else "no successful episode to render", task, out, flush=True)
            except Exception as e:  # noqa: BLE001 - keep the watcher alive; the manager reads the log
                print("render failed", task, repr(e), flush=True)
            done.add(task)
        if len(done) == 2 or final:
            break
        if not _running():
            time.sleep(5)
            final = not _running()                                     # the driver is gone: one last pass over its final summary lines, then exit
            continue
        time.sleep(120)


if __name__ == "__main__":
    main()
