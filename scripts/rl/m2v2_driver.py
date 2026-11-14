"""M2 v2 driver (Regimen v2, after the L=8 sweep): hug v2 and the two-phase flat recipe, ONE training job at a time (8 envs), gate2 n=100 per finished run.

Order (resumable by (L, task, seed) summary lines in ``<root>/summary.jsonl``; a finished training (``meta.json``) is not retrained):
  (a) m2_hug2  L=8 seed0, 3M steps, warm start from the L=8 sweep m2_flat seed0 (STOP marker ``STOP_hug_failed`` and exit if its hug success < 0.05)
  (b) m2_flat2 L=8 seeds 1, 2 (the sweep's m2_flat seed0 counts as a prior, not retrained)
  (c) L=10: m2_flat2 x 3 seeds, then m2_hug2 x 1 seed warm-started from the best flat seed
  (d) L=12: same as (c)
A flat run whose rolling training success is < 0.10 at 0.6M steps (phase 1) is killed, logged as ``takeoff_fail`` and replaced by the next seed (at most seed 5).
Terrain and L = 9 / 11 are skipped. No videos (the render watcher makes one per stage). Sim only.

    nohup python scripts/rl/m2v2_driver.py > data/rl/m2v2/driver.log 2>&1 &
    python scripts/rl/m2v2_driver.py --root data/rl/m2v2_smoke --scale 0.002 --gate-n 4        # smoke test
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
import traceback
from pathlib import Path
from typing import Any

sys.path.insert(0, str(Path(__file__).resolve().parent))

from m2_length_sweep import RECIPE_ENV, gate_one, summarise  # noqa: E402

from petctl.rl.curriculum.early_stop import HealthStopCallback  # noqa: E402
from petctl.rl.curriculum.m2v2 import TakeoffCallback, best_seed, next_seed, run_note, summary_lines  # noqa: E402
from petctl.rl.curriculum.runner import build_config  # noqa: E402
from petctl.rl.curriculum.stages import FLAT2_PHASE1, FLAT2_PHASE1_ENT, FLAT2_PHASE1_STEPS, Stage, get_stage  # noqa: E402
from petctl.rl.hug_start import FLAG_ENV, StartMixCallback  # noqa: E402

PRIOR_FLAT8 = Path("data/rl/m2_sweep/8/m2_flat/seed0")
STOP_MARKER = "STOP_hug_failed"
HUG_STOP_BELOW = 0.05


def run_dir(root: Path, length: int, task: str, seed: int) -> Path:
    """``<root>/<L>/<task>/seed<k>``."""
    return root / str(length) / task / f"seed{seed}"


def _meta(out: Path) -> dict[str, Any]:
    m = json.loads((out / "meta.json").read_text())
    return {"steps": m["steps"], "wall_s": m["wall_s"], "steps_per_s": m["steps"] / max(m["wall_s"], 1e-9),
            "early_stop": (out / "early_stop.txt").read_text() if (out / "early_stop.txt").exists() else None}


def _finished(out: Path) -> bool:
    return (out / "meta.json").exists() and (out / "policy.npz").exists()


def train_flat2(length: int, seed: int, out: Path, scale: float, n_envs: int) -> dict[str, Any]:
    """Two-phase flat recipe: phase 1 (0.7M, bearing 45, goal 0.6-1.2 m, dr 0.3, ent 0.01) with the takeoff check, phase 2 (1.3M, the stage's own distribution, ent 0.005)
    continuing the same policy and normalisation statistics. Returns ``{steps, wall_s, steps_per_s, early_stop, takeoff, takeoff_fail}``."""
    from petctl.rl.train import train

    if _finished(out):
        return {**_meta(out), "takeoff": json.loads((out / "phase1" / "takeoff.json").read_text()) if (out / "phase1" / "takeoff.json").exists() else {}}
    stage, p1 = get_stage("m2_flat2"), out / "phase1"
    os.environ.update(RECIPE_ENV)
    s1, s2 = int(FLAT2_PHASE1_STEPS * scale), int((stage.steps - FLAT2_PHASE1_STEPS) * scale)
    wall = 0.0
    if (p1 / "takeoff_fail.txt").exists():
        return {"takeoff_fail": (p1 / "takeoff_fail.txt").read_text(), "steps": s1, "wall_s": 0.0, "steps_per_s": 0.0, "early_stop": None, "takeoff": json.loads((p1 / "takeoff.json").read_text())}
    if not _finished(p1):
        cfg1 = build_config(stage, {"n_modules": float(length), **FLAT2_PHASE1})
        take, health, t0 = TakeoffCallback(int(600_000 * scale)), HealthStopCallback(), time.time()
        train(cfg1, s1, n_envs, p1, seed, "ppo", None, FLAT2_PHASE1_ENT, None, video=False, asymmetric=True, extra_callbacks=[health, take])
        wall += time.time() - t0
        info = {"rate": take.rate, "checked_at": take.checked_at, "failed": take.failed}
        (p1 / "takeoff.json").write_text(json.dumps(info))
        if health.reason:
            (p1 / "early_stop.txt").write_text(health.reason)
        if take.failed:
            msg = f"takeoff failure: rolling training success {take.rate:.3f} < 0.10 at {take.checked_at} steps (phase 1)"
            (p1 / "takeoff_fail.txt").write_text(msg)
            return {"takeoff_fail": msg, "steps": int(take.checked_at or 0), "wall_s": wall, "steps_per_s": (take.checked_at or 0) / max(wall, 1e-9), "early_stop": None, "takeoff": info}
    info = json.loads((p1 / "takeoff.json").read_text()) if (p1 / "takeoff.json").exists() else {}
    health, t0 = HealthStopCallback(), time.time()
    cfg2 = build_config(stage, {"n_modules": float(length)})
    train(cfg2, s2, n_envs, out, seed + 100, "ppo", None, stage.ent_coef, p1, video=False, asymmetric=True, extra_callbacks=[health], warm_copy_ret=True)
    wall += time.time() - t0
    if health.reason:
        (out / "early_stop.txt").write_text(health.reason)
    m = json.loads((out / "meta.json").read_text())
    m.update({"steps": s1 + s2, "wall_s": wall, "phases": [s1, s2]})
    (out / "meta.json").write_text(json.dumps(m, indent=1))
    return {"steps": s1 + s2, "wall_s": wall, "steps_per_s": (s1 + s2) / max(wall, 1e-9), "early_stop": health.reason, "takeoff": info}


def train_hug2(length: int, seed: int, out: Path, init_from: Path, scale: float, n_envs: int) -> dict[str, Any]:
    """m2_hug2 from the warm start ``init_from`` (block-mapped observation columns); the start-mix switch flag lives in the run directory."""
    from petctl.rl.train import train

    if _finished(out):
        return _meta(out)
    stage = get_stage("m2_hug2")
    os.environ.update(RECIPE_ENV)
    flag = (out / "hug_mix_switched.flag").resolve()
    out.mkdir(parents=True, exist_ok=True)
    if flag.exists():
        flag.unlink()
    os.environ[FLAG_ENV] = str(flag)
    cfg, health, mix, t0 = build_config(stage, {"n_modules": float(length)}), HealthStopCallback(), StartMixCallback(flag), time.time()
    steps = int(stage.steps * scale)
    try:
        train(cfg, steps, n_envs, out, seed, "ppo", None, stage.ent_coef, init_from, video=False, asymmetric=True, extra_callbacks=[health, mix])
    finally:
        os.environ.pop(FLAG_ENV, None)
    wall = time.time() - t0
    if health.reason:
        (out / "early_stop.txt").write_text(health.reason)
    m = json.loads((out / "meta.json").read_text())
    m.update({"steps": steps, "wall_s": wall, "mix_switched_at": mix.switched_at, "init_from": str(init_from)})
    (out / "meta.json").write_text(json.dumps(m, indent=1))
    return {"steps": steps, "wall_s": wall, "steps_per_s": steps / max(wall, 1e-9), "early_stop": health.reason, "mix_switched_at": mix.switched_at}


def next_job(root: Path) -> dict[str, Any] | None:
    """The next run of the ORDER (a)-(d), or None when everything is done. ``{task, L, seed, init}``."""
    lines = summary_lines(root)
    have = {(d["L"], d["task"], int(d["seed"])) for d in lines}
    if (8, "m2_hug2", 0) not in have:
        return {"task": "m2_hug2", "L": 8, "seed": 0, "init": PRIOR_FLAT8}
    s = next_seed(lines, 8, "m2_flat2", first_seed=1, need=2)
    if s is not None:
        return {"task": "m2_flat2", "L": 8, "seed": s}
    for L in (10, 12):
        s = next_seed(lines, L, "m2_flat2", first_seed=0, need=3)
        if s is not None:
            return {"task": "m2_flat2", "L": L, "seed": s}
        if (L, "m2_hug2", 0) not in have:
            b = best_seed(lines, L, "m2_flat2")
            if b is None:
                return {"task": "m2_hug2", "L": L, "seed": 0, "init": None}          # no usable flat policy: recorded as skipped
            return {"task": "m2_hug2", "L": L, "seed": 0, "init": run_dir(root, L, "m2_flat2", b)}
    return None


def execute(job: dict[str, Any], root: Path, scale: float, gate_n: int, n_envs: int, workers: int) -> dict[str, Any]:
    """Train (or resume) + gate one job; returns its summary line (also appended to the summary file by the caller)."""
    L, task, seed = job["L"], job["task"], job["seed"]
    stage: Stage = get_stage(task)
    out = run_dir(root, L, task, seed)
    out.mkdir(parents=True, exist_ok=True)
    print(f"[run] L={L} {task} seed{seed} -> {out}  ({time.strftime('%H:%M:%S')})", flush=True)
    meta: dict[str, Any] = {}
    try:
        if task == "m2_hug2":
            if job["init"] is None:
                return {**summarise(L, task, seed, None, {}, "skipped", "no flat policy to warm-start from"), "phase": "single"}
            meta = train_hug2(L, seed, out, Path(job["init"]), scale, n_envs)
            status, extra = ("early_stopped" if meta.get("early_stop") else "ok"), {"phase": "single", "init_from": str(job["init"]), "mix_switched_at": meta.get("mix_switched_at")}
            phases = [out]
        else:
            meta = train_flat2(L, seed, out, scale, n_envs)
            extra = {"phase": "two_phase", "takeoff": meta.get("takeoff")}
            phases = [out / "phase1", out]
            if meta.get("takeoff_fail"):
                print(f"[note] {run_note(out, [out / 'phase1'])}", flush=True)
                return {**summarise(L, task, seed, None, meta, "takeoff_fail", meta["takeoff_fail"]), **extra}
            status = "early_stopped" if meta.get("early_stop") else "ok"
        note = run_note(out, phases)
        (out / "note.txt").write_text(note + "\n")
        print(f"[note] {note}", flush=True)
        res = gate_one(stage, out, gate_n, workers)
        line = {**summarise(L, task, seed, res, meta, status, meta.get("early_stop") or ""), **extra}
        if task == "m2_hug2":
            line.update({"hug_success": res["robust"]["rate"], "hug_success_ci": [res["robust"]["wilson_lo"], res["robust"]["wilson_hi"]], "t_hug_median_s": line.get("t_hug_median_s")})
    except Exception as e:  # noqa: BLE001
        traceback.print_exc()
        line = summarise(L, task, seed, None, meta, "failed", f"{type(e).__name__}: {e}")
    return line


def main() -> int:
    """CLI entry."""
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--root", type=Path, default=Path("data/rl/m2v2"))
    ap.add_argument("--scale", type=float, default=1.0, help="multiplies every step count (smoke tests)")
    ap.add_argument("--gate-n", type=int, default=100)
    ap.add_argument("--n-envs", type=int, default=8)
    ap.add_argument("--eval-workers", type=int, default=4)
    ap.add_argument("--max-jobs", type=int, default=0, help="stop after this many jobs (0 = all; smoke tests)")
    a = ap.parse_args()
    root: Path = a.root
    root.mkdir(parents=True, exist_ok=True)
    if (root / STOP_MARKER).exists():
        print(f"[stop] {root / STOP_MARKER} exists: remove it to continue", flush=True)
        return 0
    done = 0
    while True:
        job = next_job(root)
        if job is None or (a.max_jobs and done >= a.max_jobs):
            break
        line = execute(job, root, a.scale, a.gate_n, a.n_envs, a.eval_workers)
        with open(root / "summary.jsonl", "a") as f:
            f.write(json.dumps(line) + "\n")
        print(f"[done] {json.dumps(line)}", flush=True)
        done += 1
        if job["task"] == "m2_hug2" and job["L"] == 8 and (line.get("success") is None or line["success"] < HUG_STOP_BELOW):
            (root / STOP_MARKER).write_text(f"m2_hug2 L=8 seed0 hug success {line.get('success')} < {HUG_STOP_BELOW}: manager checkpoint\n")
            print(f"[stop] hug v2 failed at L=8 (success {line.get('success')}): wrote {root / STOP_MARKER}", flush=True)
            return 0
    print("[m2v2 complete]", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
