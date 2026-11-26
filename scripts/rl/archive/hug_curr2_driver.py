"""Regimen v2 per-family hug success curriculum at L = 8 (stage ``hug_curr_02``): ONE training job (10 envs), 45 min wall budget of training + n=30 screening gate (+ the n=100 verdict on a pass),
resumable from ``<root>/summary.jsonl``.

Warm start: ``data/rl/hug_curr/hug_curr_01`` (model + vecnormalize, trained log-std and reward-normalisation statistics copied; its start buffer copied in). Per shape family (cyl / box / rbox)
training-only levels, stored as a dict in ``<root>/hug_curr_02/hug_level.json`` (broadcast to the workers once per rollout): cylinders and rounded boxes C0 (span 100 / run 4 / hold 1.0 s) -> C1 (125 / 4 / 1.5 s)
-> L2 (150 / 4 / 2 s) -> L3 (gate); boxes L1 (120 / 3 / 1.5 s) -> L2 -> L3; a family advances at rolling success >= 0.30 over its last 100 episodes, never demotes. Shape mix cyl 0.50 / box 0.20 /
rbox 0.30, cylinder radius 1.5-9 cm, 0.5 cm exclusion margin around the gate shapes. Steps = 0.75 x 45 min x steps/s measured over the first 2 minutes (cap 4M).

Checks (``petctl.rl.curriculum.hug_curr2``): 20 min stop ONLY if curved-family success < 0.03 over its last 150 episodes AND not risen vs the 150 before AND cylinder span p75 not >= 5 deg above 106;
separate safety stop if box success drops by more than 0.2 from its level at the start. No 40-min stop. Any stop writes ``STOP_<reason>``, the term sums, the per-family table and two contact sheets (no video).
End: n=30 screen at L3 on the L=8 gate set (6 per shape, dr 0.6): >= 0.55 -> n=100 verdict (gate2_spec_hug5(8), capacitive_off ablation) -> DONE; else a family advanced or curved success rising ->
CONTINUE_ONE_MORE_HOUR (nothing launched); else STOP_design_change (first candidate: hold reward growing with the consecutive hold ticks). Sim only.

    PETCTL_PROGRESS_EXPORT_S=900 nohup python scripts/rl/hug_curr2_driver.py > data/rl/hug_curr2/driver.log 2>&1 &
    python scripts/rl/hug_curr2_driver.py --root data/rl/hug_curr2_smoke --scale 0.01 --screen-n 5 --final-n 5 --force-go     # smoke test (wall budgets x scale)
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
import sys
import time
from pathlib import Path
from typing import Any

sys.path.insert(0, str(Path(__file__).resolve().parent))

import m2v6_driver as d6  # noqa: E402
from m2_length_sweep import PoolEvaluator  # noqa: E402

from petctl.rl.curriculum import hug_curr2 as hc2  # noqa: E402
from petctl.rl.curriculum import hug_long as hl  # noqa: E402
from petctl.rl.curriculum import m2v6  # noqa: E402
from petctl.rl.curriculum.gate2 import run_gate2  # noqa: E402
from petctl.rl.curriculum.m2v5 import gate2_spec_hug5  # noqa: E402
from petctl.rl.curriculum.stages import get_stage  # noqa: E402
from petctl.rl.hug_levels import LEVEL_FILE, read_family_levels  # noqa: E402
from petctl.rl.records import HugFamilyLevelCallback  # noqa: E402
from petctl.rl.target_shapes import held_out_names  # noqa: E402

STAGE, L = "hug_curr_02", 8
INIT = Path("data/rl/hug_curr/hug_curr_01")
INIT_BUFFER = Path("data/rl/hug_curr/hug_buffer.json")
BUDGET_MIN, CAP_STEPS = 45.0, 4_000_000
log, finished, lines, record, stop = d6.log, d6.finished, d6.lines, d6.record, d6.stop


def go_line(root: Path, phase: str) -> dict[str, Any] | None:
    """The recorded line of ``phase`` (the last one) or None."""
    found = [d for d in lines(root) if d["phase"] == phase]
    return found[-1] if found else None


def screen30(out: Path, n: int, workers: int) -> dict[str, Any]:
    """Screening gate on the L=8 held-out set, always scored at L3 (the gate params carry no curriculum): ``n`` episodes, ``n // 5`` per shape, dr 0.6."""
    stage = get_stage(STAGE)
    params = dict(stage.gate.task_params)
    params.setdefault("bus_model", 1.0)
    assert "hug_curriculum" not in params and "shape_mix" not in params
    ev = PoolEvaluator(out, "policy.npz", params, 0.6, stage.gate.episode_s, workers)
    rows = ev({}, hl.balanced_screen_seeds(n))
    (out / f"gate2_{STAGE}.json").write_text(json.dumps(m2v6.best_episode_record(STAGE, rows, params, 0.6, stage.gate.episode_s), indent=1))
    s = {**m2v6.summarise_rows(rows), "per_shape": hl.per_shape_success(rows, L)}
    (out / "screen.json").write_text(json.dumps(s, indent=1))
    log(f"screen {out.name}: {json.dumps(s)}")
    return s


def family_dump(out: Path) -> dict[str, Any]:
    """Per-family table of the training episodes plus the persisted levels and advances."""
    rows = hl.read_rows_tolerant(out / "episodes.jsonl")
    lv = json.loads((out / LEVEL_FILE).read_text()) if (out / LEVEL_FILE).exists() else {}
    return {"levels": lv.get("names"), "history": lv.get("history"), "table": hc2.family_table(rows, read_family_levels(out)), "curved_success": hc2.curved_success(rows),
            "cyl_span_p75": hc2.cyl_span_p75(rows)}


def dump_stop(out: Path, root: Path, reason: str, workers: int) -> None:
    """Stop dump: term sums + two contact sheets (``d6.dump_stop``) and the per-family table."""
    d6.dump_stop(out, root, reason, workers, STAGE)
    fd = family_dump(out)
    (root / f"stop_{reason}_per_family.json").write_text(json.dumps(fd, indent=1))
    log(f"[stop] per-family table (training): {json.dumps(fd)}")


def append_log_entry(root: Path, train: dict[str, Any], screen: dict[str, Any], final: dict[str, Any], out: Path) -> None:
    """Append the factual curriculum-log entry of the finished run."""
    rb = final["robust"]
    fd = family_dump(out)
    txt = (f"\n## {time.strftime('%Y-%m-%d')} hug_curr_02 (L=8) final gate\n\n"
           f"- Training: {train.get('steps')} steps, {train.get('wall_s', 0) / 60:.1f} min, {train.get('steps_per_s', 0):.0f} steps/s (continuation of hug_curr_01, per-family success curriculum, final levels {fd['levels']}, "
           f"advances {json.dumps(fd['history'])}); screen n=30 (L3): success {screen['success']:.3f}, per shape {json.dumps(screen['per_shape'])}\n"
           f"- Final gate n={final['n']} (L=8 held-out set {list(held_out_names(L))}, dr {final['dr_scale']}): success {rb['rate']:.3f} (Wilson {rb['wilson_lo']:.3f}-{rb['wilson_hi']:.3f}); per shape "
           f"{json.dumps({g: round(v['rate'], 2) for g, v in final.get('groups', {}).items()})}; ablations {json.dumps({k: v.get('drop', {}).get('drop') for k, v in final.get('ablations', {}).items()})}; "
           f"passed={final['passed']} reasons={final['fail_reasons']}\n- Data: {root}\n")
    with open(d6.LOG, "a") as f:
        f.write(txt)


def main() -> int:
    """CLI entry."""
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--root", type=Path, default=Path("data/rl/hug_curr2"))
    ap.add_argument("--init", type=Path, default=INIT)
    ap.add_argument("--init-levels", type=Path, default=None, help="hug_level.json of a previous run (continuation keeps its per-family levels)")
    ap.add_argument("--init-buffer", type=Path, default=INIT_BUFFER)
    ap.add_argument("--n-envs", type=int, default=0, help="0: the best n_envs of data/rl/m2v5/benchmark.json")
    ap.add_argument("--scale", type=float, default=1.0, help="multiplies every wall budget and check time (smoke tests)")
    ap.add_argument("--screen-n", type=int, default=hl.SCREEN_N)
    ap.add_argument("--final-n", type=int, default=100)
    ap.add_argument("--eval-workers", type=int, default=4)
    ap.add_argument("--hold-ramp", type=float, default=0.0, help="hold_ramp task knob (1: hold pays hold_weight x min(2, 1 + hold_ticks / hold_need)); 0 = unchanged")
    ap.add_argument("--force-go", action="store_true", help="smoke tests only: record the check values but never stop on them")
    a = ap.parse_args()
    root, sc = a.root, a.scale
    root.mkdir(parents=True, exist_ok=True)
    for m in [*root.glob("STOP_*"), *root.glob("CONTINUE_*")]:
        log(f"[stop] {m} exists: remove it to continue")
        return 0
    if (root / "DONE").exists():
        log("[done] DONE exists")
        return 0
    n_envs = a.n_envs or m2v6.best_n_envs(json.loads(d6.BENCH.read_text()))
    out, buf = root / STAGE, root / "hug_buffer.json"
    log(f"hug_curr2 L={L} n_envs={n_envs} root={root} init={a.init} lr={hl.LR} const budget={BUDGET_MIN} min cap={CAP_STEPS}")

    # ---- training with the in-run checks
    if go_line(root, "train") is None:
        if out.exists() and not finished(out):                                  # an interrupted run: keep it aside (its episode log is appended to otherwise)
            aside = out.with_name(f"{out.name}.aborted_{time.strftime('%Y%m%d_%H%M%S')}")
            out.rename(aside)
            log(f"[resume] unfinished run moved to {aside}")
        if not buf.exists() and a.init_buffer.exists():
            shutil.copy(a.init_buffer, buf)
            log(f"[buffer] initial buffer copied from {a.init_buffer} ({len(json.loads(buf.read_text()))} states)")
        out.mkdir(parents=True, exist_ok=True)
        if a.init_levels is not None and a.init_levels.exists() and not (out / LEVEL_FILE).exists():              # continuation: keep the per-family levels reached so far
            shutil.copy(a.init_levels, out / LEVEL_FILE)
            log(f"[levels] initial per-family levels copied from {a.init_levels}")
        checks = hc2.Curr2Checks(out, hc2.CHECK1_MIN * 60.0 * sc, sc)
        levels = HugFamilyLevelCallback(out)
        old_env = {k: os.environ.get(k) for k in ("PETCTL_LR", "PETCTL_LR_CONST", "PETCTL_PROGRESS_EXPORT_S")}
        os.environ.update({"PETCTL_LR": str(hl.LR), "PETCTL_LR_CONST": "1"})
        os.environ.setdefault("PETCTL_PROGRESS_EXPORT_S", "900")
        try:
            info = d6.train_phase(STAGE, out, L, {"hold_ramp": a.hold_ramp}, get_stage(STAGE).ent_coef, 300, a.init, BUDGET_MIN, CAP_STEPS, n_envs, sc, extra=[levels, checks], buffer=buf, copy_ret=True, keep_log_std=True)
        finally:
            for k, v in old_env.items():
                if v is None:
                    os.environ.pop(k, None)
                else:
                    os.environ[k] = v
        reason = checks.reason
        fd = family_dump(out)
        record(root, {"phase": "train", "go": reason is None or a.force_go, "stop_reason": reason, "checks": checks.results, "family": fd,
                      "buffer_states": len(json.loads(buf.read_text())) if buf.exists() else 0, **info})
        if reason is not None and not a.force_go:
            dump_stop(out, root, reason, a.eval_workers)
            return stop(root, reason, f"hug_curr_02 check failed ({reason}): {json.dumps({k: {x: y for x, y in v.items() if x != 'table'} for k, v in checks.results.items()})}")

    # ---- screen (always L3)
    train_line = go_line(root, "train")
    assert train_line is not None
    sc_line = go_line(root, "screen")
    if sc_line is None:
        s = screen30(out, a.screen_n, a.eval_workers)
        rows = hl.read_rows_tolerant(out / "episodes.jsonl")
        advanced = bool(json.loads((out / LEVEL_FILE).read_text()).get("history")) if (out / LEVEL_FILE).exists() else False
        verdict = "final" if a.force_go else hc2.end_decision(s["success"], advanced, rows)
        sc_line = {"phase": "screen", "go": verdict == "final", "verdict": verdict, "screen": s, "family": family_dump(out)}
        record(root, sc_line)
    s, verdict = sc_line["screen"], sc_line["verdict"]
    if verdict == "continue":
        (root / "CONTINUE_ONE_MORE_HOUR").write_text(f"screen success {s['success']:.3f} < {hc2.SCREEN_DONE}; a family advanced or curved success is rising: the manager decides whether to continue for one more hour\n")
        log(f"[continue] screen {s['success']:.3f}: wrote CONTINUE_ONE_MORE_HOUR")
        return 0
    if verdict == "stop":
        dump_stop(out, root, "design_change", a.eval_workers)
        return stop(root, "design_change", f"screen success {s['success']:.3f} < {hc2.SCREEN_DONE}, no family advanced and curved success not rising; per shape {json.dumps(s['per_shape'])}. "
                    "First candidate: hold reward growing with the consecutive hold ticks")

    # ---- final n=100 verdict (capacitive_off ablation)
    stage = get_stage(STAGE)
    spec = gate2_spec_hug5(L)
    spec.n = a.final_n
    params = dict(stage.gate.task_params)
    params.setdefault("bus_model", 1.0)
    log(f"final gate n={spec.n}")
    res = run_gate2(out, stage, spec, evaluator=PoolEvaluator(out, "policy.npz", params, spec.dr_scale, stage.gate.episode_s, a.eval_workers))
    record(root, {"phase": "final", "go": True, "passed": res["passed"], "success": res["robust"]["rate"], "wilson_lo": res["robust"]["wilson_lo"], "groups": res.get("groups"),
                  "ablations": {k: v.get("drop") for k, v in res.get("ablations", {}).items()}, "fail_reasons": res["fail_reasons"]})
    if not a.force_go:                                                          # smoke tests must not write the curriculum log
        append_log_entry(root, train_line, s, res, out)
    (root / "DONE").write_text(f"final gate passed={res['passed']}\n")
    log("[hug_curr2 complete: DONE]")
    return 0


if __name__ == "__main__":
    sys.exit(main())
