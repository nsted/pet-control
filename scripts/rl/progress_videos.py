"""Regimen v2 progress videos: every 30 min render ONE captioned video of the BEST RUN SO FAR for each ACTIVE stage (owner rule; no per-hour cap).

Stages = (root, family) over ``data/rl/m2v*/`` and ``data/rl/hug_long*/``; family is ``hug`` when the run's config task contains "hug", else ``flat``. A run is ACTIVE when its
``status.json`` / ``progress.json`` / ``episodes.jsonl`` changed within ``--active-s`` (15 min). Best run = highest rolling success over the last 100 episodes
among the stage's runs started in the last 24 h (ties: most steps). Its newest policy (``policy.npz`` or ``progress_policy.npz``, whichever is newer) is
rendered once per tick. A stage is complete (skipped; the stage-end watcher renders its single best video) when its root has DONE_* / STOP_* markers and no
driver process is alive. Marker files ``<root>/progress_ticks/<family>_<time>.json`` make ticks idempotent across restarts. Exits after ``--idle-exit-s`` with
no active stage.

    nohup python scripts/rl/progress_videos.py > data/rl/progress_videos.log 2>&1 &
    python scripts/rl/progress_videos.py --dry-run                       # print what would be rendered
    python scripts/rl/progress_videos.py --once --run data/rl/m2v7_L8/L8/P1 --run data/rl/m2v7_L8/L8/P2_hw6    # smoke test: treat these runs as active, render now
"""
from __future__ import annotations

import argparse
import json
import subprocess
import time
import traceback
from pathlib import Path
from typing import Any

from petctl.rl import video

DATA = Path("data/rl")
CANDIDATE_SEEDS = [10_000 + i for i in range(6)]
CAMERA_M = {"flat": 1.6, "hug": 0.9}
TAIL_BYTES = 400_000
ACTIVITY_FILES = ("status.json", "progress.json", "episodes.jsonl")
POLICY_FILES = ("policy.npz", "progress_policy.npz")


def log(msg: str) -> None:
    """Timestamped stdout line."""
    print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)


def _json(p: Path) -> dict[str, Any]:
    try:
        return json.loads(p.read_text())
    except (OSError, ValueError):
        return {}


def _mtime(p: Path) -> float:
    try:
        return p.stat().st_mtime
    except OSError:
        return 0.0


def _birth(p: Path) -> float:
    try:
        st = p.stat()
        return float(getattr(st, "st_birthtime", st.st_mtime))
    except OSError:
        return 0.0


def tail_episodes(run: Path, n: int = 100) -> list[dict[str, Any]]:
    """Last ``n`` complete lines of ``run/episodes.jsonl`` (reads only the file tail)."""
    f = run / "episodes.jsonl"
    try:
        with f.open("rb") as fh:
            fh.seek(0, 2)
            size = fh.tell()
            fh.seek(max(0, size - TAIL_BYTES))
            raw = fh.read().decode("utf-8", "ignore").splitlines()
    except OSError:
        return []
    rows: list[dict[str, Any]] = []
    for line in raw[1:] if size > TAIL_BYTES else raw:
        try:
            rows.append(json.loads(line))
        except ValueError:
            continue
    return rows[-n:]


def family_of(run: Path) -> str:
    """``hug`` if the run's task contains "hug", else ``flat``."""
    return "hug" if "hug" in str(_json(run / "config.json").get("task", "")) else "flat"


def run_info(run: Path, root: Path) -> dict[str, Any] | None:
    """Summary of one run directory (None when it has no episodes or no policy yet): rolling success, steps, wall minutes, newest policy file."""
    rows = tail_episodes(run)
    pols = [(_mtime(run / n), n) for n in POLICY_FILES if (run / n).exists()]
    if not rows or not pols:
        return None
    prog, stat = _json(run / "progress.json"), _json(run / "status.json")
    steps = max(int(rows[-1].get("timesteps", 0)), int(prog.get("timesteps", 0)), int(stat.get("timesteps", 0)))
    wall_min = float(stat["wall_min"]) if "wall_min" in stat else float(rows[-1].get("wall_s", 0.0)) / 60.0
    if "wall_s" in prog and prog.get("timesteps", 0) >= stat.get("timesteps", 0):
        wall_min = float(prog["wall_s"]) / 60.0
    succ = [float(r.get("success", 0.0)) for r in rows]
    return {"run": run, "root": root, "family": family_of(run), "rolling_success": sum(succ) / len(succ), "n_episodes": len(succ), "steps": steps,
            "wall_min": wall_min, "policy_file": max(pols)[1], "start": _birth(run / "config.json") or _birth(run / "episodes.jsonl"),
            "last_activity": max(_mtime(run / n) for n in ACTIVITY_FILES), "stage": _json(run / "meta.json").get("stage"),
            "final": bool(stat.get("final", False))}


def run_dirs(data: Path) -> list[tuple[Path, Path]]:
    """All ``(root, run_dir)`` pairs under ``data/m2v*/`` (a run dir holds an ``episodes.jsonl``)."""
    return [(r, e.parent) for r in sorted([*data.glob("m2v*"), *data.glob("hug_long*"), *data.glob("hug_curr*")]) if r.is_dir() for e in sorted(r.rglob("episodes.jsonl"))]


def driver_alive() -> bool:
    """True when a Regimen driver process is running."""
    return subprocess.run(["pgrep", "-f", r"scripts/rl/(m2v[0-9a-z_]*|hug_long_|hug_curr_)driver"], capture_output=True).returncode == 0


def root_complete(root: Path, alive: bool) -> bool:
    """A stage root is complete: DONE_*/STOP_* marker present and no live driver."""
    return (any(root.glob("DONE_*")) or any(root.glob("STOP_*"))) and not alive


def discover(data: Path, now: float, active_s: float = 900.0, window_s: float = 86400.0, alive: bool | None = None,
             forced: list[Path] | None = None) -> dict[tuple[str, str], dict[str, Any]]:
    """Active stages -> ``{(root name, family): {"best": run_info, "runs": [...], "stage_start": t}}``.

    ``forced`` runs count as active (and as started now) regardless of file times / completion markers (smoke tests).
    """
    alive = driver_alive() if alive is None else alive
    infos: list[dict[str, Any]] = []
    for root, run in run_dirs(data):
        is_forced = forced is not None and any(run.resolve() == f.resolve() for f in forced)
        info = run_info(run, root)
        if info is None:
            continue
        if is_forced:
            info["forced"] = True
        elif root_complete(root, alive) or now - info["last_activity"] > active_s:
            continue
        infos.append(info)
    stages: dict[tuple[str, str], dict[str, Any]] = {}
    for info in infos:
        stages.setdefault((info["root"].name, info["family"]), {"runs": [], "root": info["root"]})["runs"].append(info)
    for st in stages.values():
        recent = [i for i in st["runs"] if i.get("forced") or now - i["start"] <= window_s] or st["runs"]
        st["best"] = max(recent, key=lambda i: (i["rolling_success"], i["steps"]))
        st["stage_start"] = min(i["start"] for i in recent)
    return stages


def last_tick(root: Path, family: str) -> float:
    """Time of the newest progress tick marker of this stage (0 if none)."""
    return max([_mtime(p) for p in (root / "progress_ticks").glob(f"{family}_*.json")] or [0.0])


def due(st: dict[str, Any], family: str, now: float, period_s: float) -> bool:
    """A render is due: ``period_s`` since the last tick (or since the stage became active, for the first one)."""
    return now - max(last_tick(st["root"], family), st["stage_start"]) >= period_s


def pick_seed(run: Path, policy_file: str, distance: float) -> tuple[int, bool]:
    """First of the 6 candidate seeds whose (render-free) rollout succeeds; else 10000 and False."""
    for seed in CANDIDATE_SEEDS:
        try:
            if video.rollout_metrics(run, policy_file, seed).get("success", 0.0) >= 0.5:
                return seed, True
        except Exception as e:  # noqa: BLE001
            log(f"probe seed {seed} failed: {e}")
    return CANDIDATE_SEEDS[0], False


def episode_note(metrics: dict[str, float], reached: bool, seed: int, found: bool, family: str) -> str:
    """Final-hold caption built ONLY from the recorded episode's own metrics."""
    ok = metrics.get("success", 0.0) >= 0.5
    bits = [f"seed {seed}: {'SUCCESS' if ok else 'not successful'}"]
    if "final_dist" in metrics:
        bits.append(f"final dist {metrics['final_dist']:.2f} m")
    if family == "hug":
        if "hug_modules" in metrics:
            bits.append(f"hug modules {metrics['hug_modules']:.0f}")
        if "wrap_deg" in metrics:
            bits.append(f"wrap {metrics['wrap_deg']:.0f} deg")
    return "Regimen v2 PROGRESS episode: " + ", ".join(bits) + ("" if found else " (no successful episode yet)")


def render_stage(st: dict[str, Any], family: str, out_dir: Path) -> Path:
    """Render the stage's best-run-so-far video and write the tick marker; returns the mp4 path."""
    b = st["best"]
    run, pf = b["run"], b["policy_file"]
    seed, found = pick_seed(run, pf, CAMERA_M[family])
    cfg = _json(run / "config.json")
    stage = b["stage"] or b["root"].name
    label = (f"Regimen v2 PROGRESS - {stage} {run.name} (L={cfg.get('n_modules', '?')}) - best run so far: success {b['rolling_success']:.2f} "
             f"at {b['steps']} steps, {b['wall_min']:.0f} min into training" + ("" if found else " - no successful episode yet"))
    out = out_dir / f"progress_{b['root'].name}_{family}_{run.name}_{time.strftime('%Y%m%d_%H%M%S')}.mp4"
    m = video.record_video(run, out, pf, seed=seed, distance=CAMERA_M[family], label=label,
                           final_note=lambda met, reached: episode_note(met, reached, seed, found, family))
    tick = st["root"] / "progress_ticks"
    tick.mkdir(parents=True, exist_ok=True)
    (tick / f"{family}_{time.strftime('%Y%m%d_%H%M%S')}.json").write_text(json.dumps({"video": str(out), "run": str(run), "policy_file": pf, "seed": seed,
                                                                                    "found_success": found, "label": label, "episode_metrics": m}, indent=1))
    return out


def tick(data: Path, period_s: float, dry: bool, force_due: bool, forced: list[Path] | None, out_dir: Path, active_s: float) -> int:
    """One pass over the active stages; returns how many stages are active."""
    now = time.time()
    stages = discover(data, now, active_s, forced=forced)
    for (rootname, family), st in sorted(stages.items()):
        b = st["best"]
        is_due = force_due or due(st, family, now, period_s)
        log(f"stage {rootname}/{family}: {len(st['runs'])} active run(s), best {b['run']} success {b['rolling_success']:.2f} steps {b['steps']} "
            f"policy {b['policy_file']} - {'DUE' if is_due else 'not due'}")
        if not is_due or dry:
            continue
        try:
            log(f"rendered {render_stage(st, family, out_dir)}")
        except Exception:  # noqa: BLE001 - keep the loop alive
            log("render failed:\n" + traceback.format_exc())
    return len(stages)


def main() -> None:
    """CLI entry (see module docstring)."""
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", type=Path, default=DATA)
    ap.add_argument("--period-s", type=float, default=1800.0)
    ap.add_argument("--poll-s", type=float, default=60.0)
    ap.add_argument("--active-s", type=float, default=900.0)
    ap.add_argument("--idle-exit-s", type=float, default=7200.0)
    ap.add_argument("--out-dir", type=Path, default=video.MEDIA_DIR)
    ap.add_argument("--once", action="store_true", help="one pass, render every active stage now (ignores the period)")
    ap.add_argument("--dry-run", action="store_true", help="print what would be rendered, render nothing")
    ap.add_argument("--run", type=Path, action="append", help="treat this run dir as active (smoke tests); repeatable")
    a = ap.parse_args()
    last_active = time.time()
    while True:
        try:
            n = tick(a.data, a.period_s, a.dry_run, a.once, a.run, a.out_dir, a.active_s)
        except Exception:  # noqa: BLE001
            log("tick failed:\n" + traceback.format_exc())
            n = 1
        if a.once or a.dry_run:
            return
        last_active = time.time() if n else last_active
        if time.time() - last_active > a.idle_exit_s:
            log("no active stage for the idle window; exiting")
            return
        time.sleep(a.poll_s)


if __name__ == "__main__":
    main()
