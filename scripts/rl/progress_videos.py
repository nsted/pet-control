"""Regimen v2 progress videos: for EVERY training run, at least ONE and at most THREE videos per 30 min of the run's wall time (the owner monitors progress only through these videos).

A run is a directory with an ``episodes.jsonl`` under ``data/rl/m2v*/``, ``hug_long*``, ``hug_curr*``, ``odd*`` and ``hug_range*``; family is ``hug`` when its config task contains "hug", else ``flat``. A run is ALIVE
while its ``status.json`` / ``progress.json`` / ``episodes.jsonl`` changed within ``--active-s`` (15 min) and its ``status.json`` is not final. Per alive run: the first video is due ``--first-s`` (12 min) after the run
starts (so a run of >= 12 min always gets a midpoint video and none starts later than 15 min), then one every ``--period-s`` (25 min; never more than 3 per 30 min, never fewer than 1). When a run finishes
(final ``status.json`` / ``meta.json``) or stops (no activity for ``--active-s``) ONE FINAL video of its newest policy is rendered (``policy.npz`` or ``progress_policy.npz``, whichever is newer), once per run, if the run
ended within ``--recent-s`` (2 h). The video is always rendered, with the caption "no successful episode yet" when none of the 6 candidate seeds succeeds; the seed shown is then the one with the largest span / wrap
(hug) or the smallest final distance (flat). Marker files ``<run>/progress_ticks/<kind>_<time>.json`` make the schedule idempotent across restarts. The watcher stays alive across run boundaries (exits only after
``--idle-exit-s``, default 3 h, with no alive run); ``scripts/rl/progress_videos_watchdog.sh`` restarts it if it dies.

    nohup scripts/rl/progress_videos_watchdog.sh > data/rl/progress_videos.log 2>&1 &
    python scripts/rl/progress_videos.py --dry-run                       # print what would be rendered
    python scripts/rl/progress_videos.py --once --run data/rl/m2v7_L8/L8/P1    # smoke test: treat these runs as alive, render now
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
from petctl.rl.curriculum.hug_odd import body_note
from petctl.rl.hug_levels import level_caption

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
            "final": bool(stat.get("final", False)) or (run / "meta.json").exists()}


def run_dirs(data: Path) -> list[tuple[Path, Path]]:
    """All ``(root, run_dir)`` pairs under the study roots of ``data`` (a run dir holds an ``episodes.jsonl``)."""
    pats = ("m2v*", "hug_long*", "hug_curr*", "odd*", "hug_range*", "hug_anneal*", "chain_ab*")
    return [(r, e.parent) for r in sorted(x for pat in pats for x in data.glob(pat)) if r.is_dir() for e in sorted(r.rglob("episodes.jsonl")) if ".aborted" not in str(e.parent)]


def ticks(run: Path) -> list[tuple[float, str]]:
    """``(mtime, kind)`` of the video tick markers of ``run`` (kind ``progress`` | ``final``)."""
    return [(_mtime(p), p.name.split("_")[0]) for p in (run / "progress_ticks").glob("*.json")]


def discover_runs(data: Path, now: float, active_s: float = 900.0, recent_s: float = 7200.0, forced: list[Path] | None = None) -> list[dict[str, Any]]:
    """Runs that need attention: ``alive`` ones (fresh files, not final) and just-ended ones (final / inactive within ``recent_s``) that have no final video yet. Each entry is a ``run_info`` plus
    ``alive``, ``ticks`` (marker list) and ``forced``. ``forced`` runs count as alive and started now (smoke tests)."""
    out: list[dict[str, Any]] = []
    for root, run in run_dirs(data):
        info = run_info(run, root)
        if info is None:
            continue
        is_forced = forced is not None and any(run.resolve() == f.resolve() for f in forced)
        info["forced"], info["ticks"] = is_forced, ticks(run)
        info["alive"] = is_forced or (now - info["last_activity"] <= active_s and not info["final"])
        ended_recently = now - info["last_activity"] <= recent_s
        has_final = any(k == "final" for _, k in info["ticks"])
        if info["alive"] or (ended_recently and not has_final):
            out.append(info)
    return out


def next_kind(info: dict[str, Any], now: float, first_s: float, period_s: float) -> str | None:
    """What this run needs now: ``final`` (ended, no final video yet), ``progress`` (alive and due) or None."""
    kinds = info["ticks"]
    if not info["alive"]:
        return None if any(k == "final" for _, k in kinds) else "final"
    last = max([t for t, _ in kinds] or [0.0])
    start = now if info.get("forced") else info["start"]
    if info.get("forced"):
        return "progress"
    return "progress" if (now - last >= period_s if kinds else now - start >= first_s) else None


def pick_seed(run: Path, policy_file: str, family: str = "flat") -> tuple[int, bool]:
    """First of the 6 candidate seeds whose (render-free) rollout succeeds (``True``); otherwise the seed with the largest span (hug) or smallest final distance (flat) (``False``)."""
    best, best_score = CANDIDATE_SEEDS[0], -1e18
    for seed in CANDIDATE_SEEDS:
        try:
            m = video.rollout_metrics(run, policy_file, seed)
        except Exception as e:  # noqa: BLE001
            log(f"probe seed {seed} failed: {e}")
            continue
        if m.get("success", 0.0) >= 0.5:
            return seed, True
        score = float(m.get("normal_span_deg", m.get("wrap_deg", 0.0))) if family == "hug" else -float(m.get("final_dist", 1e9))
        if score > best_score:
            best, best_score = seed, score
    return best, False


def shape_label(m: dict[str, float]) -> str:
    """Factual shape label of an episode from its recorded metrics (``shape_kind`` 0 cyl / 1 box / 2 rbox, size params ``shape_a`` / ``shape_b`` / ``shape_rc`` in m)."""
    k, a, b, rc = int(m.get("shape_kind", -1)), m.get("shape_a", 0.0) * 100, m.get("shape_b", 0.0) * 100, m.get("shape_rc", 0.0) * 100
    if k == 0:
        return f"shape cylinder r {a:.1f} cm"
    if k == 1:
        return f"shape box {2 * a:.1f}x{2 * b:.1f} cm"
    if k == 2:
        return f"shape rounded box {2 * a:.1f}x{2 * b:.1f} cm rc {rc:.1f}"
    return "shape unknown"


def episode_note(metrics: dict[str, float], reached: bool, seed: int, found: bool, family: str) -> str:
    """Final-hold caption built ONLY from the recorded episode's own metrics."""
    ok = metrics.get("success", 0.0) >= 0.5
    bits = [f"seed {seed}: {'SUCCESS' if ok else 'not successful'}"]
    if "final_dist" in metrics:
        bits.append(f"final dist {metrics['final_dist']:.2f} m")
    if family == "hug":
        if "shape_kind" in metrics:
            bits.append(shape_label(metrics))
        if "success_l3" in metrics:
            bits.append("scored at the gate definition L3")
        if "hug_modules" in metrics:
            bits.append(f"hug modules {metrics['hug_modules']:.0f}")
        if "wrap_deg" in metrics:
            bits.append(f"wrap {metrics['wrap_deg']:.0f} deg")
    return "Regimen v2 PROGRESS episode: " + ", ".join(bits) + ("" if found else " (no successful episode yet)")


def render_run(info: dict[str, Any], kind: str, out_dir: Path) -> Path:
    """Render one captioned video of the run's newest policy (``kind`` ``progress`` | ``final``) and write the tick marker; returns the mp4 path."""
    run, family = info["run"], info["family"]
    pf = info["policy_file"]
    if kind == "final" and (run / "policy.npz").exists() and (run / "meta.json").exists():
        pf = "policy.npz"
    seed, found = pick_seed(run, pf, family)
    cfg = _json(run / "config.json")
    stage = info["stage"] or info["root"].name
    scale = float(cfg.get("action_scale_rad", 1.0))
    label = (f"Regimen v2 {'FINAL' if kind == 'final' else 'PROGRESS'} - {stage} {run.name} (L={cfg.get('n_modules', '?')}) - "
             f"{'run end' if kind == 'final' else 'run so far'}: success {info['rolling_success']:.2f} (last {info['n_episodes']} episodes) at {info['steps']} steps, {info['wall_min']:.0f} min into training"
             + (f" - training {level_caption(run)}" if level_caption(run) else "") + f" - joint target cap +-{scale:g} rad" + ("" if found else " - no successful episode yet")
             + (f" - body: {body_note(int(cfg['n_modules']))}" if cfg.get("n_modules") not in (None, 8) else ""))
    out = out_dir / f"{kind}_{info['root'].name}_{family}_{run.name}_{time.strftime('%Y%m%d_%H%M%S')}.mp4"
    m = video.record_video(run, out, pf, seed=seed, distance=CAMERA_M[family], label=label,
                           final_note=lambda met, reached: episode_note(met, reached, seed, found, family))
    tick = run / "progress_ticks"
    tick.mkdir(parents=True, exist_ok=True)
    (tick / f"{kind}_{time.strftime('%Y%m%d_%H%M%S')}.json").write_text(json.dumps({"video": str(out), "run": str(run), "policy_file": pf, "seed": seed, "found_success": found,
                                                                                  "label": label, "episode_metrics": m}, indent=1))
    return out


def tick(data: Path, a: argparse.Namespace, forced: list[Path] | None) -> int:
    """One pass over the runs; returns how many runs are alive."""
    now = time.time()
    runs = discover_runs(data, now, a.active_s, a.recent_s, forced)
    for info in runs:
        kind = "progress" if (a.once and info["alive"]) else next_kind(info, now, a.first_s, a.period_s)
        log(f"run {info['run']}: {'alive' if info['alive'] else 'ended'}, success {info['rolling_success']:.2f}, steps {info['steps']}, policy {info['policy_file']} - {kind or 'not due'}")
        if kind is None or a.dry_run:
            continue
        try:
            log(f"rendered {render_run(info, kind, a.out_dir)}")
        except Exception:  # noqa: BLE001 - keep the loop alive
            log("render failed:\n" + traceback.format_exc())
    return sum(1 for r in runs if r["alive"])


def main() -> None:
    """CLI entry (see module docstring)."""
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", type=Path, default=DATA)
    ap.add_argument("--first-s", type=float, default=720.0, help="first video this long after a run starts")
    ap.add_argument("--period-s", type=float, default=1500.0, help="then one video per this period (25 min: 1 - 2 per 30 min)")
    ap.add_argument("--poll-s", type=float, default=60.0)
    ap.add_argument("--active-s", type=float, default=900.0)
    ap.add_argument("--recent-s", type=float, default=7200.0, help="a run that ended within this window gets its final video")
    ap.add_argument("--idle-exit-s", type=float, default=10800.0)
    ap.add_argument("--out-dir", type=Path, default=video.MEDIA_DIR)
    ap.add_argument("--once", action="store_true", help="one pass, render every alive run now (and the final video of ended runs)")
    ap.add_argument("--dry-run", action="store_true", help="print what would be rendered, render nothing")
    ap.add_argument("--run", type=Path, action="append", help="treat this run dir as alive (smoke tests); repeatable")
    a = ap.parse_args()
    last_active = time.time()
    while True:
        try:
            n = tick(a.data, a, a.run)
        except Exception:  # noqa: BLE001
            log("tick failed:\n" + traceback.format_exc())
            n = 1
        if a.once or a.dry_run:
            return
        last_active = time.time() if n else last_active
        if time.time() - last_active > a.idle_exit_s:
            log("no alive run for the idle window; exiting")
            return
        time.sleep(a.poll_s)


if __name__ == "__main__":
    main()
