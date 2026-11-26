"""``hug_anneal`` pilot: late-stage fine-tune of the best hug_scale15 checkpoint (hour 6, n=100: 0.12), L=8, resumable, ONE training job at a time.

Stage ``hug_anneal`` = hug_scale15 (action_scale 1.5, action_obs_scale 1.5, no actor rescale, hold_weight 10, hold_ramp 1, mix 0.50/0.20/0.30, per-family ladder, DR 0.3, 20 s episodes, gamma .997, n_steps 1024,
asym critic) with: log-std RESET to -2.0 at the warm start and then learned, ent_coef 0, learning rate linear 1.5e-4 -> 2e-5 over the run (PETCTL_LR_FINAL), hold-break diagnostic (``hold_diag``) logged in
episodes.jsonl. Warm start from data/rl/hug_range/scale15_h6 (model, vecnormalize, reward stats, per-family levels) + the scale15 start buffer. Steps = 0.75 x 45 min x measured steps/s (cap 4M).
Evaluation is deterministic (GaitPolicy.action is the policy mean). 20-min check: box safety recorded, never stops; stop only if training success fell > 0.05 below the run's own start value (first 150 episodes).
END of an hour: n=60 screen on the L=8 gate set (12 per shape, DR 0.6, L3) + hold-break summary. GO if success >= 0.25 OR a curved shape other than cyl_r3 > 0 -> up to 2 more identical hours (screen after each;
NO-GO there = park), then n=100 gate2 (DONE). NO-GO after hour 1 -> ``STOP_ANNEAL_PARK_HUG``. Every phase writes a line to <root>/summary.jsonl.

    nohup python scripts/rl/hug_anneal_driver.py > data/rl/hug_anneal/driver.log 2>&1 &
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
import sys
from pathlib import Path
from typing import Any

sys.path.insert(0, str(Path(__file__).resolve().parent))

import hug_curr2_driver as hd  # noqa: E402
import hug_range_driver as hrd  # noqa: E402
import m2v6_driver as d6  # noqa: E402
from m2_length_sweep import PoolEvaluator  # noqa: E402

from petctl.rl import hold_diag  # noqa: E402
from petctl.rl.curriculum import hug_curr2 as hc2  # noqa: E402
from petctl.rl.curriculum import hug_long as hl  # noqa: E402
from petctl.rl.curriculum import hug_range as hr  # noqa: E402
from petctl.rl.curriculum import m2v6  # noqa: E402
from petctl.rl.curriculum.stages import get_stage  # noqa: E402
from petctl.rl.hug_levels import LEVEL_FILE  # noqa: E402
from petctl.rl.records import HugFamilyLevelCallback  # noqa: E402

STAGE, L = "hug_anneal", 8
SRC = Path("data/rl/hug_range/scale15_h6")
SRC_BUFFER = Path("data/rl/hug_range/scale15_hug_buffer.json")
LR0, LR_FINAL = 1.5e-4, 2e-5
SCREEN_N, MAX_HOURS = 60, 3
BUDGET_MIN, CAP_STEPS = 45.0, 4_000_000
log, record, lines, phase_line = d6.log, d6.record, d6.lines, hrd.phase_line


def screen(out: Path, n: int, workers: int) -> dict[str, Any]:
    """Deterministic screen on the L=8 gate set (n // 5 per shape), DR 0.6, gate (L3) definition, with the range and hold-break diagnostics on."""
    stage = get_stage(STAGE)
    params = dict(stage.gate.task_params)
    params.setdefault("bus_model", 1.0)
    params.update({"range_diag": 1.0, "hold_diag": 1.0})
    assert "hug_curriculum" not in params and "shape_mix" not in params
    rows = PoolEvaluator(out, "policy.npz", params, 0.6, stage.gate.episode_s, workers)({}, hl.balanced_screen_seeds(n))
    (out / f"gate2_{STAGE}.json").write_text(json.dumps(m2v6.best_episode_record(STAGE, rows, params, 0.6, stage.gate.episode_s), indent=1))
    s = {**m2v6.summarise_rows(rows), "per_shape": hl.per_shape_success(rows, L), "range": hr.pool_range(rows), "hold_breaks": hold_diag.summarise([r["metrics"] for r in rows])}
    (out / "screen.json").write_text(json.dumps(s, indent=1))
    log(f"screen {out.name}: {json.dumps(s)}")
    return s


def train_hour(root: Path, hour: int, init: Path, n_envs: int, sc: float) -> tuple[Path, str | None]:
    """Train hour ``hour`` (resumable); returns ``(run dir, stop reason)``."""
    out, buf, phase = root / f"anneal_h{hour}", root / "anneal_hug_buffer.json", f"train_anneal_h{hour}"
    prev = phase_line(root, phase)
    if prev is not None:
        return out, prev.get("stop_reason")
    hrd.aside_unfinished(out)
    out.mkdir(parents=True, exist_ok=True)
    if not buf.exists():
        shutil.copy(SRC_BUFFER, buf)
        log(f"[buffer] {SRC_BUFFER} copied to {buf}")
    lv = init / LEVEL_FILE
    if not (out / LEVEL_FILE).exists() and lv.exists():
        shutil.copy(lv, out / LEVEL_FILE)
        log(f"[levels] per-family levels copied from {lv}")
    checks = hr.AnnealChecks(out, hc2.CHECK1_MIN * 60.0 * sc, sc)
    levels = HugFamilyLevelCallback(out)
    keys = ("PETCTL_LR", "PETCTL_LR_CONST", "PETCTL_LR_FINAL", "PETCTL_PROGRESS_EXPORT_S")
    old_env = {k: os.environ.get(k) for k in keys}
    os.environ.update({"PETCTL_LR": str(LR0), "PETCTL_LR_FINAL": str(LR_FINAL)})
    os.environ.pop("PETCTL_LR_CONST", None)
    os.environ.setdefault("PETCTL_PROGRESS_EXPORT_S", "900")
    try:
        info = d6.train_phase(STAGE, out, L, {"hold_ramp": 1.0}, 0.0, 600 + hour, init, BUDGET_MIN, CAP_STEPS, n_envs, sc, extra=[levels, checks], buffer=buf, copy_ret=True, keep_log_std=False)
    finally:
        for k, v in old_env.items():
            if v is None:
                os.environ.pop(k, None)
            else:
                os.environ[k] = v
    rows = hl.read_rows_tolerant(out / "episodes.jsonl")
    reason = checks.reason
    record(root, {"phase": phase, "go": reason is None, "stop_reason": reason, "stage": STAGE, "checks": checks.results, "family": hd.family_dump(out), "range_last300": hr.pool_range(rows[-300:]),
                  "train_success": hr.success_trend(rows), "hold_breaks_train": hold_diag.summarise(rows), "hold_breaks_train_last300": hold_diag.summarise(rows[-300:]), **info})
    return out, reason


def park(root: Path, why: str) -> int:
    """``STOP_ANNEAL_PARK_HUG`` + a summary line."""
    record(root, {"phase": "stop_ANNEAL_PARK_HUG", "go": False, "why": why})
    (root / "STOP_ANNEAL_PARK_HUG").write_text(why + "\n")
    hrd.log_entry("hug_anneal PARK", f"- {why}\n- Data: {root}")
    log(f"[STOP_ANNEAL_PARK_HUG] {why}")
    return 0


def main() -> int:
    """CLI entry."""
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--root", type=Path, default=Path("data/rl/hug_anneal"))
    ap.add_argument("--n-envs", type=int, default=0)
    ap.add_argument("--scale", type=float, default=1.0, help="multiplies every wall budget (smoke tests)")
    ap.add_argument("--screen-n", type=int, default=SCREEN_N)
    ap.add_argument("--final-n", type=int, default=100)
    ap.add_argument("--eval-workers", type=int, default=4)
    a = ap.parse_args()
    root, sc = a.root, a.scale
    root.mkdir(parents=True, exist_ok=True)
    for m in root.glob("STOP_*"):
        log(f"[stop] {m} exists: remove it to continue")
        return 0
    if (root / "DONE").exists():
        log("[done] DONE exists")
        return 0
    n_envs = a.n_envs or m2v6.best_n_envs(json.loads(d6.BENCH.read_text()))
    log(f"hug_anneal L={L} n_envs={n_envs} root={root} source={SRC}")
    init = SRC
    for hour in range(1, MAX_HOURS + 1):
        out, reason = train_hour(root, hour, init, n_envs, sc)
        sline = phase_line(root, f"screen_anneal_h{hour}")
        if sline is None:
            s = screen(out, a.screen_n, a.eval_workers)
            go, sig = hr.anneal_go(s["success"], s["per_shape"], L)
            sline = {"phase": f"screen_anneal_h{hour}", "go": go, "signals": sig, "screen": s, "family": hd.family_dump(out), "train_stop_reason": reason, "stage": STAGE}
            record(root, sline)
        s = sline["screen"]
        log_msg = (f"- hour {hour}: stop reason {reason}; screen n={int(s['n'])} (L=8 gate set, DR 0.6, L3, deterministic): success {s['success']:.3f}, per shape {json.dumps(s['per_shape'])}; "
                   f"hold_duty {s['hold_duty_mean']:.3f}, span {s['normal_span_mean']:.0f}; hold breaks {json.dumps(s.get('hold_breaks'))}; GO signals {json.dumps(sline['signals'])}\n- Data: {root}")
        hrd.log_entry(f"hug_anneal hour {hour} screen", log_msg)
        if reason is not None:
            return park(root, f"hug_anneal hour {hour} 20-min check failed ({reason}); screen {s['success']:.3f}")
        if not sline["go"]:
            return park(root, f"hug_anneal hour {hour} NO-GO: screen {s['success']:.3f}, per shape {json.dumps(s['per_shape'])}")
        if hour == 1:
            (root / "GO_ANNEAL_CONTINUE").write_text(f"hour 1 GO: screen {s['success']:.3f}; the driver continues with up to 2 more identical hours\n")
        if s["success"] >= hc2.SCREEN_DONE:
            break
        init = out
    # GO through the loop (or screen >= 0.55): n=100 gate2 of the last hour's policy
    last = out
    final_verdict(root, last, hour, a.final_n, a.eval_workers)
    return 0


def final_verdict(root: Path, out: Path, hour: int, n: int, workers: int) -> None:
    """n=100 gate (per-shape floor, capacitive_off) of the hour's policy; writes ``DONE`` and the curriculum log entry."""
    from petctl.rl.curriculum.gate2 import run_gate2
    from petctl.rl.curriculum.m2v5 import gate2_spec_hug5
    from petctl.rl.target_shapes import held_out_names

    stage = get_stage(STAGE)
    spec = gate2_spec_hug5(L)
    spec.n = n
    params = dict(stage.gate.task_params)
    params.setdefault("bus_model", 1.0)
    params["hold_diag"] = 1.0
    log(f"final gate n={n} (anneal hour {hour})")
    res = run_gate2(out, stage, spec, evaluator=PoolEvaluator(out, "policy.npz", params, spec.dr_scale, stage.gate.episode_s, workers))
    groups = {g: round(v["rate"], 2) for g, v in res.get("groups", {}).items()}
    rb = res["robust"]
    record(root, {"phase": "final", "go": True, "hour": hour, "passed": res["passed"], "success": rb["rate"], "wilson_lo": rb["wilson_lo"], "groups": res.get("groups"),
                  "ablations": {k: v.get("drop") for k, v in res.get("ablations", {}).items()}, "fail_reasons": res["fail_reasons"]})
    hrd.log_entry(f"hug_anneal final gate after hour {hour}", f"- n={res['n']} (L=8 held-out {list(held_out_names(L))}, dr {res['dr_scale']}): success {rb['rate']:.3f} (Wilson {rb['wilson_lo']:.3f}-{rb['wilson_hi']:.3f}); "
                  f"per shape {json.dumps(groups)}; passed={res['passed']} reasons={res['fail_reasons']}\n- Data: {root}")
    (root / "DONE").write_text(f"anneal hour {hour}: final gate passed={res['passed']}\n")
    log("[hug_anneal complete: DONE]")


if __name__ == "__main__":
    sys.exit(main())
