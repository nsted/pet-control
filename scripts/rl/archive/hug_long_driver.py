"""Regimen v2 long hug run at L = 8 (stage ``hug_long_01``): ONE training job (10 envs), wall budget 45 min of training + n=30 screening gate (+ the n=100 final verdict on a pass), resumable
from ``<root>/summary.jsonl`` (a finished training, ``meta.json`` + ``policy.npz``, is never retrained; an interrupted one is moved aside and restarted).

Training continues the m2_hug7 hold-weight-6 pilot (``--init``: ``data/rl/m2v7_L8/L8/P2_hw6``; its reward-normalisation statistics and trained log-std are copied, no h-bias re-init) under the stage
``hug_long_01`` (hold_weight 10, 50 % of the shape draws from the phase-1 size ranges, dr 0.3, straight 70 % / buffer 30 % starts, constant learning rate 1.5e-4). Steps = 0.75 x 45 min x steps/s measured
over the first 2 minutes (cap 5M). The pilot's start buffer is copied in as the initial buffer.

Go / no-go (each stop writes ``STOP_<reason>``, the mean term sums of the last 200 episodes and two contact-sheet PNGs, no video):
  20 min  among the last 300 episodes that reach run >= 3 and span >= 90 deg: frac_hold (state kept >= 2 s) >= 0.20 AND mean h over wrap ticks >= 0.75   else STOP_hold_20min
  40 min  rolling training success (last 200 episodes; held 3 s at 90 % duty) >= 0.10 (a run whose step target ends before 40 min is judged by the same rule at its end)   else STOP_success_40min
  end     n=30 screen on the L=8 held-out set (6 per shape, dr 0.6): >= 0.55 -> n=100 final (gate2_spec_hug5(8): per-shape floor, capacitive_off ablation) and DONE;
          0.30 <= s < 0.55 -> CONTINUE_ONE_MORE_HOUR (the manager decides; nothing is launched); < 0.30 -> STOP_screen_low.
Progress videos come from ``scripts/rl/progress_videos.py`` (every 30 min). Sim only.

    PETCTL_PROGRESS_EXPORT_S=900 nohup python scripts/rl/hug_long_driver.py > data/rl/hug_long/driver.log 2>&1 &
    python scripts/rl/hug_long_driver.py --root data/rl/hug_long_smoke --scale 0.01 --screen-n 5 --final-n 5 --force-go     # smoke test (wall budgets x scale)
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

from petctl.rl.curriculum import hug_long as hl  # noqa: E402
from petctl.rl.curriculum import m2v6  # noqa: E402
from petctl.rl.curriculum.gate2 import run_gate2  # noqa: E402
from petctl.rl.curriculum.m2v5 import gate2_spec_hug5  # noqa: E402
from petctl.rl.curriculum.stages import get_stage  # noqa: E402
from petctl.rl.target_shapes import held_out_names  # noqa: E402

STAGE, L = "hug_long_01", 8
INIT = Path("data/rl/m2v7_L8/L8/P2_hw6")
INIT_BUFFER = Path("data/rl/m2v7_L8/L8/hug_buffer_P2_hw6.json")
log, finished, lines, record, stop = d6.log, d6.finished, d6.lines, d6.record, d6.stop


def go_line(root: Path, phase: str) -> dict[str, Any] | None:
    """The recorded line of ``phase`` (the last one) or None."""
    found = [d for d in lines(root) if d["phase"] == phase]
    return found[-1] if found else None


def screen30(out: Path, n: int, workers: int) -> dict[str, Any]:
    """Screening gate on the L=8 held-out set: ``n`` episodes, ``n // 5`` per shape (seeds 10000 + 20 i + j), dr 0.6, the stage's gate distribution. Writes ``screen.json`` and ``gate2_<stage>.json``."""
    stage = get_stage(STAGE)
    params = dict(stage.gate.task_params)
    params.setdefault("bus_model", 1.0)
    ev = PoolEvaluator(out, "policy.npz", params, 0.6, stage.gate.episode_s, workers)
    rows = ev({}, hl.balanced_screen_seeds(n))
    (out / f"gate2_{STAGE}.json").write_text(json.dumps(m2v6.best_episode_record(STAGE, rows, params, 0.6, stage.gate.episode_s), indent=1))
    s = {**m2v6.summarise_rows(rows), "per_shape": hl.per_shape_success(rows, L)}
    (out / "screen.json").write_text(json.dumps(s, indent=1))
    log(f"screen {out.name}: {json.dumps(s)}")
    return s


def append_log_entry(root: Path, train: dict[str, Any], screen: dict[str, Any], final: dict[str, Any]) -> None:
    """Append the factual curriculum-log entry of the finished run."""
    rb = final["robust"]
    txt = (f"\n## {time.strftime('%Y-%m-%d')} hug_long_01 (L=8) final gate\n\n"
           f"- Training: {train.get('steps')} steps, {train.get('wall_s', 0) / 60:.1f} min, {train.get('steps_per_s', 0):.0f} steps/s (continuation of m2_hug7 hw6 pilot, constant lr 1.5e-4); screen n=30: success {screen['success']:.3f}, per shape {json.dumps(screen['per_shape'])}\n"
           f"- Final gate n={final['n']} (L=8 held-out set {list(held_out_names(L))}, dr {final['dr_scale']}): success {rb['rate']:.3f} (Wilson {rb['wilson_lo']:.3f}-{rb['wilson_hi']:.3f}); per shape "
           f"{json.dumps({g: round(v['rate'], 2) for g, v in final.get('groups', {}).items()})}; ablations {json.dumps({k: v.get('drop', {}).get('drop') for k, v in final.get('ablations', {}).items()})}; "
           f"passed={final['passed']} reasons={final['fail_reasons']}\n- Data: {root}\n")
    with open(d6.LOG, "a") as f:
        f.write(txt)


def main() -> int:
    """CLI entry."""
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--root", type=Path, default=Path("data/rl/hug_long"))
    ap.add_argument("--init", type=Path, default=INIT)
    ap.add_argument("--init-buffer", type=Path, default=INIT_BUFFER)
    ap.add_argument("--n-envs", type=int, default=0, help="0: the best n_envs of data/rl/m2v5/benchmark.json")
    ap.add_argument("--scale", type=float, default=1.0, help="multiplies every wall budget and check time (smoke tests)")
    ap.add_argument("--screen-n", type=int, default=hl.SCREEN_N)
    ap.add_argument("--final-n", type=int, default=100)
    ap.add_argument("--eval-workers", type=int, default=4)
    ap.add_argument("--force-go", action="store_true", help="smoke tests only: record the go / no-go values but never stop on them")
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
    log(f"hug_long L={L} n_envs={n_envs} root={root} init={a.init} lr={hl.LR} const")

    # ---- training with the in-run checks
    if go_line(root, "train") is None:
        if out.exists() and not finished(out):                                  # an interrupted run: keep it aside (its episode log is appended to otherwise)
            aside = out.with_name(f"{out.name}.aborted_{time.strftime('%Y%m%d_%H%M%S')}")
            out.rename(aside)
            log(f"[resume] unfinished run moved to {aside}")
        if not buf.exists() and a.init_buffer.exists():
            shutil.copy(a.init_buffer, buf)
            log(f"[buffer] initial buffer copied from {a.init_buffer} ({len(json.loads(buf.read_text()))} states)")
        checks = hl.LongRunChecks(out, hl.CHECK1_MIN * 60.0 * sc, hl.CHECK2_MIN * 60.0 * sc)
        old_env = {k: os.environ.get(k) for k in ("PETCTL_LR", "PETCTL_LR_CONST")}
        os.environ.update({"PETCTL_LR": str(hl.LR), "PETCTL_LR_CONST": "1"})
        try:
            info = d6.train_phase(STAGE, out, L, {}, get_stage(STAGE).ent_coef, 300, a.init, hl.BUDGET_MIN, hl.CAP_STEPS, n_envs, sc, extra=[checks], buffer=buf, copy_ret=True, keep_log_std=True)
        finally:
            for k, v in old_env.items():
                if v is None:
                    os.environ.pop(k, None)
                else:
                    os.environ[k] = v
        reason = checks.reason
        if reason is None and "check2" not in checks.results:                   # the step target ended the run before the 40-min check: same rule at the end
            if not checks.run_check2(at_end=True, steps=info["steps"]):
                reason = "success_40min"
        record(root, {"phase": "train", "go": reason is None or a.force_go, "stop_reason": reason, "checks": checks.results, "buffer_states": len(json.loads(buf.read_text())) if buf.exists() else 0, **info})
        if reason is not None and not a.force_go:
            d6.dump_stop(out, root, reason, a.eval_workers, STAGE)
            return stop(root, reason, f"hug_long check failed ({reason}): {json.dumps(checks.results)}")

    # ---- screen
    train_line = go_line(root, "train")
    assert train_line is not None
    sc_line = go_line(root, "screen")
    if sc_line is None:
        s = screen30(out, a.screen_n, a.eval_workers)
        verdict = "final" if a.force_go else hl.screen_decision(s["success"])
        sc_line = {"phase": "screen", "go": verdict == "final", "verdict": verdict, "screen": s}
        record(root, sc_line)
    s, verdict = sc_line["screen"], sc_line["verdict"]
    if verdict == "continue":
        (root / "CONTINUE_ONE_MORE_HOUR").write_text(f"screen success {s['success']:.3f} in [0.30, 0.55): the manager decides whether to continue for one more hour: manager checkpoint\n")
        log(f"[continue] screen {s['success']:.3f}: wrote CONTINUE_ONE_MORE_HOUR")
        return 0
    if verdict == "stop":
        d6.dump_stop(out, root, "screen_low", a.eval_workers, STAGE)
        return stop(root, "screen_low", f"screen success {s['success']:.3f} < {hl.SCREEN_CONTINUE}")

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
        append_log_entry(root, train_line, s, res)
    (root / "DONE").write_text(f"final gate passed={res['passed']}\n")
    log("[hug_long complete: DONE]")
    return 0


if __name__ == "__main__":
    sys.exit(main())
