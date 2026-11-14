"""M2 v3 driver (Regimen v2, after hug v2 failed to hold): hug v3, ONE training job at a time (8 envs), gate2 n=100 per finished run.

Order (resumable by (L, task, seed) summary lines in ``<root>/summary.jsonl``; a finished training (``meta.json``) is not retrained):
  (a) m2_hug3 L=10 seed0 (no flat policy for L=10 yet: trained from scratch, no warm start)
  (b) m2_hug3 L=8  seed0 (warm start from the L=8 sweep m2_flat seed0)
  (c) seed1 on whichever length held (success Wilson lower bound > 0 and hold_duty median > 0.5; the better one if both); neither held: marker ``STOP_none_held``
  (d) the driver stops after (c) for a manager checkpoint.
Stop rule: at 1.0M steps (end of phase 1) rolling hold_duty < 0.3 -> the run stops itself, term sums + two contact sheets (best / worst gate episode of a 20-episode eval) are
dumped and the marker ``STOP_hold_failed`` is written (the driver exits). No videos here (the render watcher makes one per stage). Sim only.

    nohup python scripts/rl/m2v3_driver.py > data/rl/m2v3/driver.log 2>&1 &
    python scripts/rl/m2v3_driver.py --root data/rl/m2v3_smoke --scale 0.01 --gate-n 4        # smoke test
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

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

from m2_length_sweep import RECIPE_ENV, gate_one, summarise  # noqa: E402

from petctl.rl.curriculum.early_stop import HealthStopCallback  # noqa: E402
from petctl.rl.curriculum.gate2 import contact_sheet  # noqa: E402
from petctl.rl.curriculum.m2v2 import run_note, summary_lines  # noqa: E402
from petctl.rl.curriculum.runner import build_config  # noqa: E402
from petctl.rl.curriculum.stages import get_stage  # noqa: E402
from petctl.rl.hug_start import FLAG_ENV, V3_PHASE1_STEPS, HugV3PhaseCallback  # noqa: E402

TASK = "m2_hug3"
PRIOR_FLAT8 = Path("data/rl/m2_sweep/8/m2_flat/seed0")
STOP_HOLD = "STOP_hold_failed"
STOP_NONE = "STOP_none_held"
HELD_DUTY_MEDIAN = 0.5


def run_dir(root: Path, length: int, seed: int) -> Path:
    """``<root>/<L>/m2_hug3/seed<k>``."""
    return root / str(length) / TASK / f"seed{seed}"


def init_for(length: int) -> Path | None:
    """Warm-start source: the length's best flat policy (only L=8 has one; L=10: none, from scratch)."""
    return PRIOR_FLAT8 if length == 8 else None


def held(line: dict[str, Any]) -> bool:
    """A run 'held': hug success Wilson lower bound > 0 and hold_duty median > 0.5."""
    return (line.get("wilson_lo") or 0.0) > 1e-9 and (line.get("hold_duty_median") or 0.0) > HELD_DUTY_MEDIAN


def next_job(root: Path) -> dict[str, Any] | None:
    """The next run of the order (a)-(c), or None when the driver is to stop (finished, or no length held: the caller writes ``STOP_none_held``)."""
    lines = [d for d in summary_lines(root) if d["task"] == TASK]
    have = {(d["L"], int(d["seed"])) for d in lines}
    for L in (10, 8):
        if (L, 0) not in have:
            return {"L": L, "seed": 0, "init": init_for(L)}
    winners = [d for d in lines if int(d["seed"]) == 0 and held(d)]
    if not winners:
        return None
    best = max(winners, key=lambda d: (d["success"], d.get("hold_duty_median") or 0.0))
    if (best["L"], 1) not in have:
        return {"L": best["L"], "seed": 1, "init": init_for(best["L"])}
    return None


def train_hug3(length: int, seed: int, out: Path, init_from: Path | None, scale: float, n_envs: int) -> dict[str, Any]:
    """m2_hug3 (log_std -1.6, ent 0, start-mix phases with the stop rule). Returns steps / wall / early_stop / mix_switched_at / stop_hold (rule text or None)."""
    from petctl.rl.train import train

    stage = get_stage(TASK)
    steps = int(stage.steps * scale)
    if (out / "meta.json").exists() and (out / "policy.npz").exists():
        m = json.loads((out / "meta.json").read_text())
        return {"steps": m["steps"], "wall_s": m["wall_s"], "steps_per_s": m["steps"] / max(m["wall_s"], 1e-9), "early_stop": None,
                "mix_switched_at": m.get("mix_switched_at"), "stop_hold": m.get("stop_hold")}
    os.environ.update(RECIPE_ENV)
    flag = (out / "hug_mix_switched.flag").resolve()
    out.mkdir(parents=True, exist_ok=True)
    if flag.exists():
        flag.unlink()
    os.environ[FLAG_ENV] = str(flag)
    cfg, health, t0 = build_config(stage, {"n_modules": float(length)}), HealthStopCallback(), time.time()
    phase = HugV3PhaseCallback(flag, phase1_steps=int(V3_PHASE1_STEPS * scale))
    try:
        train(cfg, steps, n_envs, out, seed, "ppo", None, stage.ent_coef, init_from, video=False, asymmetric=True, extra_callbacks=[health, phase], log_std_init=stage.log_std_init)
    finally:
        os.environ.pop(FLAG_ENV, None)
    wall = time.time() - t0
    if health.reason:
        (out / "early_stop.txt").write_text(health.reason)
    stop = f"rolling hold_duty {phase.rate:.3f} < 0.3 at {phase.checked_at} steps" if phase.stopped else None
    m = json.loads((out / "meta.json").read_text())
    m.update({"steps": int(m["steps"]) if not phase.stopped else int(phase.checked_at or 0), "wall_s": wall, "mix_switched_at": phase.switched_at, "init_from": str(init_from), "stop_hold": stop,
              "log_std_init": stage.log_std_init})
    (out / "meta.json").write_text(json.dumps(m, indent=1))
    return {"steps": m["steps"], "wall_s": wall, "steps_per_s": m["steps"] / max(wall, 1e-9), "early_stop": health.reason, "mix_switched_at": phase.switched_at, "stop_hold": stop}


def dump_stop(out: Path, root: Path, workers: int) -> None:
    """Stop-rule dump: mean term sums of the last 200 training episodes, and two contact-sheet PNGs (best / worst episode of a 20-episode gate eval; no videos)."""
    rows = [json.loads(x) for x in (out / "episodes.jsonl").read_text().splitlines() if x.strip()][-200:]
    terms = {k: float(np.nanmean([r[k] for r in rows if k in r])) for k in sorted({k for r in rows for k in r if k.startswith("term_")})}
    (root / "stop_term_sums.json").write_text(json.dumps({"episodes": len(rows), "mean_term_sums": terms}, indent=1))
    print(f"[stop] mean term sums (last {len(rows)} episodes): {json.dumps(terms)}", flush=True)
    try:
        res = gate_one(get_stage(TASK), out, 20, workers)
        ps = sorted(res["per_seed"], key=lambda r: r["return"])
        for tag, r in (("worst", ps[0]), ("best", ps[-1])):
            png = contact_sheet(out, TASK, seed=int(r["seed"]), out=root / f"stop_sheet_{tag}.png")
            print(f"[stop] {tag} sheet (return {r['return']:.1f}): {png}", flush=True)
    except Exception as e:  # noqa: BLE001
        traceback.print_exc()
        print(f"[stop] sheets failed: {type(e).__name__}: {e}", flush=True)


def execute(job: dict[str, Any], root: Path, scale: float, gate_n: int, n_envs: int, workers: int) -> tuple[dict[str, Any], bool]:
    """Train (or resume) + gate one job; returns (summary line, stop-rule fired)."""
    L, seed = job["L"], job["seed"]
    out = run_dir(root, L, seed)
    out.mkdir(parents=True, exist_ok=True)
    print(f"[run] L={L} {TASK} seed{seed} init={job['init']} -> {out}  ({time.strftime('%H:%M:%S')})", flush=True)
    meta: dict[str, Any] = {}
    try:
        meta = train_hug3(L, seed, out, job["init"], scale, n_envs)
        if meta.get("stop_hold"):
            dump_stop(out, root, workers)
            return {**summarise(L, TASK, seed, None, meta, "stop_hold_failed", meta["stop_hold"]), "init_from": str(job["init"])}, True
        note = run_note(out, [out])
        (out / "note.txt").write_text(note + "\n")
        print(f"[note] {note}", flush=True)
        res = gate_one(get_stage(TASK), out, gate_n, workers)
        mm = res["robust"]["metric_median"]
        line = {**summarise(L, TASK, seed, res, meta, "early_stopped" if meta.get("early_stop") else "ok", meta.get("early_stop") or ""), "init_from": str(job["init"]),
                "mix_switched_at": meta.get("mix_switched_at"), "hug_success": res["robust"]["rate"], "hug_success_ci": [res["robust"]["wilson_lo"], res["robust"]["wilson_hi"]],
                "hold_duty_median": mm.get("hold_duty"), "post_force_n_median": mm.get("post_force_n")}
        return line, False
    except Exception as e:  # noqa: BLE001
        traceback.print_exc()
        return summarise(L, TASK, seed, None, meta, "failed", f"{type(e).__name__}: {e}"), False


def main() -> int:
    """CLI entry."""
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--root", type=Path, default=Path("data/rl/m2v3"))
    ap.add_argument("--scale", type=float, default=1.0, help="multiplies every step count (smoke tests)")
    ap.add_argument("--gate-n", type=int, default=100)
    ap.add_argument("--n-envs", type=int, default=8)
    ap.add_argument("--eval-workers", type=int, default=4)
    a = ap.parse_args()
    root: Path = a.root
    root.mkdir(parents=True, exist_ok=True)
    for m in (STOP_HOLD, STOP_NONE):
        if (root / m).exists():
            print(f"[stop] {root / m} exists: remove it to continue", flush=True)
            return 0
    while True:
        job = next_job(root)
        if job is None:
            lines = [d for d in summary_lines(root) if d["task"] == TASK]
            if len(lines) >= 2 and not any(held(d) for d in lines if int(d["seed"]) == 0):
                (root / STOP_NONE).write_text("no length held (success CI lower bound > 0 and hold_duty median > 0.5): manager checkpoint\n")
                print(f"[stop] none held: wrote {root / STOP_NONE}", flush=True)
            else:
                print("[m2v3 checkpoint] orders (a)-(c) done: manager checkpoint", flush=True)
            return 0
        line, stopped = execute(job, root, a.scale, a.gate_n, a.n_envs, a.eval_workers)
        with open(root / "summary.jsonl", "a") as f:
            f.write(json.dumps(line) + "\n")
        print(f"[done] {json.dumps(line)}", flush=True)
        if stopped:
            (root / STOP_HOLD).write_text(f"{TASK} L={job['L']} seed{job['seed']}: {line['note']}: manager checkpoint\n")
            print(f"[stop] stop rule fired: wrote {root / STOP_HOLD}", flush=True)
            return 0


if __name__ == "__main__":
    sys.exit(main())
