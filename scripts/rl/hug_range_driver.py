"""Action-range study at L = 8, chained and resumable: Step A (diagnostic) -> Step B (stage ``hug_scale15``, +2 automatic hours on GO) -> Step C (stage ``hug_long_ep``) -> escalation.
ONE training job at a time (10 envs), state in ``<root>/summary.jsonl`` (EVERY phase writes a line, so a manager wait loop wakes on each one), markers ``STOP_*`` / ``DONE`` / ``ESCALATE``.

STEP A  ``range_diag.diagnose`` (no training): best L=8 checkpoint ``data/rl/hug_curr4/hug_curr_02``, cyl r3 + cyl r7, 20 episodes each at dr 0.6: fraction of wrap-attempt ticks (hug run >= 3) with the in-run
        joints saturated (|action| >= 0.95), mean |target - angle| of in-run joints, max |target|. >= 20 % -> B; < 10 % -> skip B (go to C); 10-20 % -> B. Writes data/rl/range_diag/result.json and docs/plans/RANGE_DIAGNOSTIC.md.
STEP B  stage ``hug_scale15`` = hug_curr_02 with action_scale_rad 1.5 (hold_weight 10, hold_ramp 1, lr 1.5e-4 constant, ent 0.002, DR 0.3, 20 s episodes, mix cyl 0.50 / box 0.20 / rbox 0.30, per-family ladder,
        levels + buffer + log-std + reward-norm stats preserved); warm start from the +-1.0 policy with the 7 joint output channels (weights, bias) multiplied by 1/1.5 and the observed action history kept in the old
        units (action_obs_scale 1.5), so the commanded targets of actions within +-1 are unchanged. Steps = 0.75 x 45 min x measured steps/s (cap 4M). 20-min check: stop only if the cylinder span p75 fell by more than
        10 deg vs the start or box success fell by more than 0.2. End: n=30 screen on the L=8 gate set at L3 (dr 0.6): GO if screen >= 0.15 OR any curved gate shape >= 1/6 OR cylinder training span p75 >= 165 deg.
        GO -> two more identical 45-min hours (levels preserved, screen after each). Any screen >= 0.55 -> n=100 verdict (gate2_spec_hug5(8), capacitive_off) -> DONE. NO-GO -> Step C.
STEP C  stage ``hug_long_ep``: hug_curr_02 recipe, action_scale 1.0, warm from hug_curr4, episodes 30 s, start-buffer probability 0.5, buffer storing only states with normal span >= 120 deg (initial buffer = the
        entries of data/rl/hug_curr4/hug_buffer.json with span >= 120), 45 min + n=30 screen, same GO rule / continuation as B (box safety stop only). If C fails: STOP_ESCALATE_OWNER with the L=8, L=9 and range tables.

    nohup python scripts/rl/hug_range_driver.py > data/rl/hug_range/driver.log 2>&1 &
    python scripts/rl/hug_range_driver.py --root data/rl/hug_range_smoke --scale 0.01 --screen-n 5 --final-n 5 --force-go --skip-diag      # smoke test (wall budgets x scale)
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

import hug_curr2_driver as hd  # noqa: E402
import m2v6_driver as d6  # noqa: E402
import range_diag as rd  # noqa: E402
from m2_length_sweep import PoolEvaluator  # noqa: E402

from petctl.rl.curriculum import hug_curr2 as hc2  # noqa: E402
from petctl.rl.curriculum import hug_long as hl  # noqa: E402
from petctl.rl.curriculum import hug_range as hr  # noqa: E402
from petctl.rl.curriculum import m2v6  # noqa: E402
from petctl.rl.curriculum.gate2 import run_gate2  # noqa: E402
from petctl.rl.curriculum.m2v5 import gate2_spec_hug5  # noqa: E402
from petctl.rl.curriculum.stages import get_stage  # noqa: E402
from petctl.rl.hug_levels import LEVEL_FILE  # noqa: E402
from petctl.rl.records import HugFamilyLevelCallback  # noqa: E402
from petctl.rl.target_shapes import held_out_names  # noqa: E402

L = 8
SRC = Path("data/rl/hug_curr4/hug_curr_02")
SRC_BUFFER = Path("data/rl/hug_curr4/hug_buffer.json")
BUDGET_MIN, CAP_STEPS, MAX_HOURS = 45.0, 4_000_000, 3
SERIES = {"scale15": "hug_scale15", "long_ep": "hug_long_ep"}
log, finished, record, lines = d6.log, d6.finished, d6.record, d6.lines


def phase_line(root: Path, phase: str) -> dict[str, Any] | None:
    """The last recorded line of ``phase`` or None."""
    found = [d for d in lines(root) if d["phase"] == phase]
    return found[-1] if found else None


def halt(root: Path, reason: str, why: str) -> int:
    """Record a ``stop_<reason>`` phase line (wakes the manager's wait loop) and write ``STOP_<reason>``."""
    record(root, {"phase": f"stop_{reason}", "go": False, "why": why})
    return d6.stop(root, reason, why)


def aside_unfinished(out: Path) -> None:
    """An interrupted run (no meta.json / policy.npz) is moved aside: its episode log is appended to otherwise."""
    if out.exists() and not finished(out):
        aside = out.with_name(f"{out.name}.aborted_{time.strftime('%Y%m%d_%H%M%S')}")
        out.rename(aside)
        log(f"[resume] unfinished run moved to {aside}")


def screen(out: Path, stage_name: str, n: int, workers: int) -> dict[str, Any]:
    """n=30-style screen on the L=8 held-out set, always scored at L3 (the gate params carry no curriculum), dr 0.6, 20 s episodes."""
    stage = get_stage(stage_name)
    params = dict(stage.gate.task_params)
    params.setdefault("bus_model", 1.0)
    params["range_diag"] = 1.0
    assert "hug_curriculum" not in params and "shape_mix" not in params
    rows = PoolEvaluator(out, "policy.npz", params, 0.6, stage.gate.episode_s, workers)({}, hl.balanced_screen_seeds(n))
    (out / f"gate2_{stage_name}.json").write_text(json.dumps(m2v6.best_episode_record(stage_name, rows, params, 0.6, stage.gate.episode_s), indent=1))
    s = {**m2v6.summarise_rows(rows), "per_shape": hl.per_shape_success(rows, L), "range": hr.pool_range(rows)}
    (out / "screen.json").write_text(json.dumps(s, indent=1))
    log(f"screen {out.name}: {json.dumps(s)}")
    return s


def source_cyl_span_p75() -> float | None:
    """Cylinder span p75 over the last 150 cylinder episodes of the warm-start source run (the 'value at start' of the 20-min rule)."""
    return hc2.cyl_span_p75(hl.read_rows_tolerant(SRC / "episodes.jsonl"))


def prepare_buffer(series: str, buf: Path) -> None:
    """Initial start buffer of a series: scale15 = a copy of the source buffer; long_ep = its entries with normal span >= 120 deg."""
    if buf.exists():
        return
    items = json.loads(SRC_BUFFER.read_text()) if SRC_BUFFER.exists() else []
    if series == "long_ep":
        items = hr.filter_buffer(items, 120.0)
    buf.parent.mkdir(parents=True, exist_ok=True)
    buf.write_text(json.dumps(items))
    log(f"[buffer] {series}: {len(items)} initial states written to {buf}")


def train_hour(root: Path, series: str, hour: int, init: Path, n_envs: int, sc: float, force: bool) -> tuple[Path, str | None]:
    """Train hour ``hour`` of a series (resumable); returns ``(run dir, stop reason or None)``."""
    stage_name = SERIES[series]
    out, buf = root / f"{series}_h{hour}", root / f"{series}_hug_buffer.json"
    phase = f"train_{series}_h{hour}"
    prev = phase_line(root, phase)
    if prev is not None:
        return out, prev.get("stop_reason")
    aside_unfinished(out)
    out.mkdir(parents=True, exist_ok=True)
    prepare_buffer(series, buf)
    lv_src = SRC / LEVEL_FILE if hour == 1 else init / LEVEL_FILE
    if not (out / LEVEL_FILE).exists() and lv_src.exists():
        shutil.copy(lv_src, out / LEVEL_FILE)
        log(f"[levels] per-family levels copied from {lv_src}")
    base_p75 = source_cyl_span_p75() if (series == "scale15" and hour == 1) else None
    if series == "scale15" and hour == 1:
        checks: hc2.Curr2Checks = hr.RangeChecks(out, base_p75, hc2.CHECK1_MIN * 60.0 * sc, sc)
    else:
        checks = hc2.Curr2Checks(out, 1e12, sc)                                          # box safety only
    levels = HugFamilyLevelCallback(out)
    rescale = hr.SCALE15_REF / hr.SCALE15 if (series == "scale15" and hour == 1) else None
    old_env = {k: os.environ.get(k) for k in ("PETCTL_LR", "PETCTL_LR_CONST", "PETCTL_PROGRESS_EXPORT_S")}
    os.environ.update({"PETCTL_LR": str(hl.LR), "PETCTL_LR_CONST": "1"})
    os.environ.setdefault("PETCTL_PROGRESS_EXPORT_S", "900")
    try:
        info = d6.train_phase(stage_name, out, L, {"hold_ramp": 1.0}, get_stage(stage_name).ent_coef, 400 + hour, init, BUDGET_MIN, CAP_STEPS, n_envs, sc, extra=[levels, checks], buffer=buf,
                              copy_ret=True, keep_log_std=True, rescale_joints=rescale)
    finally:
        for k, v in old_env.items():
            if v is None:
                os.environ.pop(k, None)
            else:
                os.environ[k] = v
    rows = hl.read_rows_tolerant(out / "episodes.jsonl")
    reason = None if force else checks.reason
    fd = hd.family_dump(out)
    record(root, {"phase": phase, "go": reason is None, "stop_reason": reason, "stage": stage_name, "checks": checks.results, "family": fd, "range_last300": hr.pool_range(rows[-300:]),
                  "base_cyl_span_p75": base_p75, "buffer_states": len(json.loads(buf.read_text())) if buf.exists() else 0, "rescale_joints": rescale, **info})
    return out, reason


def final_verdict(root: Path, out: Path, series: str, hour: int, n: int, workers: int, force: bool) -> None:
    """n=100 verdict (per-shape floor, capacitive_off) of the hour's policy; writes ``DONE``."""
    stage = get_stage(SERIES[series])
    spec = gate2_spec_hug5(L)
    spec.n = n
    params = dict(stage.gate.task_params)
    params.setdefault("bus_model", 1.0)
    log(f"final gate n={n} ({series} hour {hour})")
    res = run_gate2(out, stage, spec, evaluator=PoolEvaluator(out, "policy.npz", params, spec.dr_scale, stage.gate.episode_s, workers))
    groups = {g: round(v["rate"], 2) for g, v in res.get("groups", {}).items()}
    record(root, {"phase": "final", "go": True, "series": series, "hour": hour, "passed": res["passed"], "success": res["robust"]["rate"], "wilson_lo": res["robust"]["wilson_lo"], "groups": res.get("groups"),
                  "ablations": {k: v.get("drop") for k, v in res.get("ablations", {}).items()}, "fail_reasons": res["fail_reasons"]})
    rb = res["robust"]
    if not force:                                                                    # smoke tests must not write the curriculum log
        log_entry(f"{SERIES[series]} final gate after hour {hour}", f"- Final gate n={res['n']} (L=8 held-out set {list(held_out_names(L))}, dr {res['dr_scale']}): success {rb['rate']:.3f} "
                  f"(Wilson {rb['wilson_lo']:.3f}-{rb['wilson_hi']:.3f}); per shape {json.dumps(groups)}; passed={res['passed']} reasons={res['fail_reasons']}\n- Data: {root}")
    (root / "DONE").write_text(f"{series} hour {hour}: final gate passed={res['passed']}\n")
    log("[hug_range complete: DONE]")


def log_entry(title: str, body: str) -> None:
    """Append a factual entry to the curriculum log."""
    with open(d6.LOG, "a") as f:
        f.write(f"\n## {time.strftime('%Y-%m-%d %H:%M')} action-range study: {title}\n\n{body}\n")


def run_series(root: Path, series: str, n_envs: int, sc: float, a: argparse.Namespace) -> str:
    """Hour 1 + screen (GO rule), then on GO two more automatic hours with a screen after each. Returns ``done`` | ``nogo`` | ``exhausted`` | ``stopped``."""
    stage_name = SERIES[series]
    init = SRC
    for hour in range(1, MAX_HOURS + 1):
        out, reason = train_hour(root, series, hour, init, n_envs, sc, a.force_go)
        sline = phase_line(root, f"screen_{series}_h{hour}")
        if sline is None:
            s = screen(out, stage_name, a.screen_n, a.eval_workers)
            fd = hd.family_dump(out)
            go, sig = hr.go_rule(s["success"], s["per_shape"], fd["cyl_span_p75"], L)
            sline = {"phase": f"screen_{series}_h{hour}", "go": go or a.force_go, "signals": sig, "screen": s, "family": fd, "train_stop_reason": reason, "stage": stage_name}
            record(root, sline)
        s = sline["screen"]
        if reason is not None and hour > 1:
            halt(root, reason, f"{stage_name} hour {hour} check failed ({reason}); screen {s['success']:.3f}, per shape {json.dumps(s['per_shape'])}")
            return "stopped"
        if s["success"] >= hc2.SCREEN_DONE:
            final_verdict(root, out, series, hour, a.final_n, a.eval_workers, a.force_go)
            return "done"
        if hour == 1 and (reason is not None or not sline["go"]):
            log_entry(f"{stage_name} hour 1 NO-GO", f"- stop reason {reason}; screen n=30 (L3): {s['success']:.3f}, per shape {json.dumps(s['per_shape'])}; GO signals {json.dumps(sline['signals'])}\n- Data: {root}")
            return "nogo"
        init = out
    return "exhausted"


def escalation_text(root: Path) -> str:
    """The three tables of the owner escalation: L=8 screens, L=9 screens, range diagnostic (+ this study's screens)."""
    def screens(path: Path, key: str) -> list[str]:
        out = []
        for d in lines(path):
            if d["phase"].startswith(key):
                out.append(f"  {d['phase']}: success {d['screen']['success']:.3f}, per shape {json.dumps(d['screen'].get('per_shape'))}")
        return out
    parts = ["L=8 screens (n=30, L3; data/rl/hug_curr*/summary.jsonl):", *screens(Path("data/rl/hug_curr4"), "screen"), "  hours 1-3 per the curriculum log: 0.033 / 0.067 / 0.033",
             "L=9 screens (data/rl/odd9/summary.jsonl):", *screens(Path("data/rl/odd9"), "screen_h")]
    try:
        r = json.loads(rd.OUT.read_text())
        parts += ["Range diagnostic (data/rl/range_diag/result.json):", f"  pooled {json.dumps(r['pooled'])}", f"  decision {r['decision']}: {r['reason']}"]
    except (OSError, ValueError, KeyError):
        parts += ["Range diagnostic: not available"]
    parts += ["This study's screens:", *screens(root, "screen_")]
    return "\n".join(parts)


def escalate(root: Path, why: str) -> int:
    """STOP_ESCALATE_OWNER (+ ``ESCALATE``) with the three tables."""
    text = f"{why}\n\n{escalation_text(root)}\n"
    (root / "ESCALATE").write_text(text)
    log_entry("ESCALATE to the owner", text)
    return halt(root, "ESCALATE_OWNER", why)


def main() -> int:
    """CLI entry."""
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--root", type=Path, default=Path("data/rl/hug_range"))
    ap.add_argument("--n-envs", type=int, default=0, help="0: the best n_envs of data/rl/m2v5/benchmark.json")
    ap.add_argument("--scale", type=float, default=1.0, help="multiplies every wall budget and check time (smoke tests)")
    ap.add_argument("--screen-n", type=int, default=hl.SCREEN_N)
    ap.add_argument("--final-n", type=int, default=100)
    ap.add_argument("--eval-workers", type=int, default=4)
    ap.add_argument("--diag-n", type=int, default=20)
    ap.add_argument("--force-go", action="store_true", help="smoke tests only: record the go / no-go values but never stop on them")
    ap.add_argument("--skip-diag", action="store_true", help="smoke tests only: skip Step A and run Step B")
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
    log(f"hug_range L={L} n_envs={n_envs} root={root} source={SRC}")

    # ---- STEP A
    dline = phase_line(root, "diag")
    if dline is None:
        if a.skip_diag:
            dline = {"phase": "diag", "go": True, "decision": "B", "reason": "skipped (smoke test)"}
        else:
            r = rd.diagnose(SRC, a.diag_n, a.eval_workers)
            rd.OUT.parent.mkdir(parents=True, exist_ok=True)
            rd.OUT.write_text(json.dumps(r, indent=1))
            rd.write_doc(r)
            dline = {"phase": "diag", "go": r["decision"] == "B", "decision": r["decision"], "reason": r["reason"], "pooled": r["pooled"], "per_shape": r["per_shape"]}
        record(root, dline)
    path = dline["decision"]

    # ---- STEP B
    if path == "B":
        res = run_series(root, "scale15", n_envs, sc, a)
        if res == "done":
            return 0
        if res == "stopped":
            return 0
        if res == "exhausted":
            return halt(root, "scale15_after3", "hug_scale15 ran 3 hours and no screen reached 0.55; Step C is not run after a GO (manager decides)")
        log("[B] NO-GO: going to Step C")
    else:
        log("[A] range not saturated: Step B skipped, going to Step C")

    # ---- STEP C
    res = run_series(root, "long_ep", n_envs, sc, a)
    if res in ("done", "stopped"):
        return 0
    if res == "exhausted":
        return halt(root, "long_ep_after3", "hug_long_ep ran 3 hours and no screen reached 0.55")
    return escalate(root, "Step C (hug_long_ep) NO-GO: the range (B) and long-episode (C) design changes both failed")


if __name__ == "__main__":
    sys.exit(main())
