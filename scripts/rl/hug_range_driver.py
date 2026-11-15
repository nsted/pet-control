"""Action-range study at L = 8, chained and resumable: Step A (diagnostic) -> Step B (stage ``hug_scale15``, +2 automatic hours on GO) -> Step C (stage ``hug_long_ep``) -> escalation.
ONE training job at a time (10 envs), state in ``<root>/summary.jsonl`` (EVERY phase writes a line, so a manager wait loop wakes on each one), markers ``STOP_*`` / ``DONE`` / ``ESCALATE``.

STEP A  ``range_diag.diagnose`` (no training): best L=8 checkpoint ``data/rl/hug_curr4/hug_curr_02``, cyl r3 + cyl r7, 20 episodes each at dr 0.6: fraction of wrap-attempt ticks (hug run >= 3) with the in-run
        joints saturated (|action| >= 0.95), mean |target - angle| of in-run joints, max |target|. >= 20 % -> B; < 10 % -> skip B (go to C); 10-20 % -> B. Writes data/rl/range_diag/result.json and docs/plans/RANGE_DIAGNOSTIC.md.
STEP B  stage ``hug_scale15`` = hug_curr_02 with action_scale_rad 1.5 (hold_weight 10, hold_ramp 1, lr 1.5e-4 constant, ent 0.002, DR 0.3, 20 s episodes, mix cyl 0.50 / box 0.20 / rbox 0.30, per-family ladder,
        levels + buffer + log-std + reward-norm stats preserved); warm start from the +-1.0 policy with the 7 joint output channels (weights, bias) multiplied by 1/1.5 and the observed action history kept in the old
        units (action_obs_scale 1.5), so the commanded targets of actions within +-1 are unchanged. Steps = 0.75 x 45 min x measured steps/s (cap 4M). 20-min check: stop only if the cylinder span p75 fell by more than
        10 deg vs the start or box success fell by more than 0.2. End: n=30 screen on the L=8 gate set at L3 (dr 0.6): GO if screen >= 0.15 OR any curved gate shape >= 1/6 OR cylinder training span p75 >= 165 deg.
        GO -> two more identical 45-min hours (levels preserved, screen after each). Any screen >= 0.55 -> n=100 verdict (gate2_spec_hug5(8), capacitive_off) -> DONE. NO-GO -> Step C.
POST-H1 (expert rules, hours >= 2 of B; ``run_post``): 20-min check stops only if training success (last 150 eps) < hour-1 final 0.177 - 0.05 or box success fell > 0.2. After each screen: >= 0.55 -> n=100 verdict;
        hour 2: continue if screen >= 0.15 / curved shape >= 1/6 / training success rising (last sixth >= first + 0.05) else Step C' (stage ``hug_long15`` = hug_long_ep changes on scale 1.5, warm from the latest scale15
        hour, one hour); after hour 3 or C': screen < 0.30 and no curved shape >= 2/6 -> ESCALATE_OWNER, else ONE extra hour, then decide at 0.55 (else escalate). The hour-1 NO-GO counts as overruled when an
        ``expert_override`` line exists or scale15_h2 exists.
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
SERIES = {"scale15": "hug_scale15", "long_ep": "hug_long_ep", "long15": "hug_long15"}
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
    s15 = buf.parent / "scale15_hug_buffer.json"
    if series == "long15" and s15.exists():                                           # Step C': the latest scale15 buffer
        items = json.loads(s15.read_text())
    if series in ("long_ep", "long15"):
        items = hr.filter_buffer(items, 120.0)
    buf.parent.mkdir(parents=True, exist_ok=True)
    buf.write_text(json.dumps(items))
    log(f"[buffer] {series}: {len(items)} initial states written to {buf}")


def base_success(root: Path, series: str, hour: int) -> float:
    """Training success the 20-min rule of hour ``hour`` compares to: scale15 hours >= 2 use the hour-1 final 0.177; long15 hour 1 the last scale15 hour's final, later hours the previous long15 hour's."""
    if series == "scale15":
        return hr.HOUR1_FINAL_SUCCESS
    prev = phase_line(root, f"train_long15_h{hour - 1}") if hour > 1 else last_train_line(root, "scale15")
    v = ((prev or {}).get("train_success") or {}).get("last")
    return float(v) if v is not None else hr.HOUR1_FINAL_SUCCESS


def last_train_line(root: Path, series: str) -> dict[str, Any] | None:
    """The train line of the highest finished hour of ``series`` or None."""
    found = [d for d in lines(root) if d["phase"].startswith(f"train_{series}_h")]
    return found[-1] if found else None


def train_hour(root: Path, series: str, hour: int, init: Path, n_envs: int, sc: float, force: bool, force_c15: bool = False) -> tuple[Path, str | None]:
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
    lv_src = SRC / LEVEL_FILE if (hour == 1 and series != "long15") else init / LEVEL_FILE
    if not (out / LEVEL_FILE).exists() and lv_src.exists():
        shutil.copy(lv_src, out / LEVEL_FILE)
        log(f"[levels] per-family levels copied from {lv_src}")
    base_p75 = source_cyl_span_p75() if (series == "scale15" and hour == 1) else None
    if series == "scale15" and hour == 1:
        checks: hc2.Curr2Checks = hr.RangeChecks(out, base_p75, hc2.CHECK1_MIN * 60.0 * sc, sc)
    elif series == "long15" and force_c15:                                               # --force-c15: base = this run's own first 150 episodes
        checks = hr.StartSuccessChecks(out, hc2.CHECK1_MIN * 60.0 * sc, sc)
    elif series == "long15" or (series == "scale15" and hour >= 2):                      # expert rule: success fall > 0.05 below the previous hour's final / box fall > 0.2
        checks = hr.SuccessChecks(out, base_success(root, series, hour), hc2.CHECK1_MIN * 60.0 * sc, sc)
    else:
        checks = hc2.Curr2Checks(out, 1e12, sc)                                          # box safety only
    levels = HugFamilyLevelCallback(out)
    rescale = hr.SCALE15_REF / hr.SCALE15 if (series == "scale15" and hour == 1) else None
    old_env = {k: os.environ.get(k) for k in ("PETCTL_LR", "PETCTL_LR_CONST", "PETCTL_PROGRESS_EXPORT_S")}
    os.environ.update({"PETCTL_LR": str(hl.LR), "PETCTL_LR_CONST": "1"})
    os.environ.setdefault("PETCTL_PROGRESS_EXPORT_S", "900")
    try:
        info = d6.train_phase(stage_name, out, L, {"hold_ramp": 1.0}, get_stage(stage_name).ent_coef, (500 if series == "long15" else 400) + hour, init, BUDGET_MIN, CAP_STEPS, n_envs, sc, extra=[levels, checks], buffer=buf,
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
                  "base_cyl_span_p75": base_p75, "train_success": hr.success_trend(rows), "buffer_states": len(json.loads(buf.read_text())) if buf.exists() else 0, "rescale_joints": rescale, **info})
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
        if series == "scale15" and hour == 1 and reason is None and (sline["go"] or h1_overridden(root)):
            return run_post(root, 2, out, n_envs, sc, a)
        if hour == 1 and (reason is not None or not sline["go"]):
            log_entry(f"{stage_name} hour 1 NO-GO", f"- stop reason {reason}; screen n=30 (L3): {s['success']:.3f}, per shape {json.dumps(s['per_shape'])}; GO signals {json.dumps(sline['signals'])}\n- Data: {root}")
            return "nogo"
        init = out
    return "exhausted"


def h1_overridden(root: Path) -> bool:
    """The expert overruled the hour-1 NO-GO (an ``expert_override`` line, or hour 2 was already started)."""
    return phase_line(root, "expert_override") is not None or (root / "scale15_h2").exists()


def run_post(root: Path, hour: int, init: Path, n_envs: int, sc: float, a: argparse.Namespace) -> str:
    """Expert rules after Step B hour 1: hours are identical continuations (scale 1.5, no further rescale); every hour writes train / screen / decision lines. Returns ``done`` | ``stopped``."""
    series = "scale15"
    while True:
        out, reason = train_hour(root, series, hour, init, n_envs, sc, a.force_go)
        stage_name = SERIES[series]
        sline = phase_line(root, f"screen_{series}_h{hour}")
        if sline is None:
            s = screen(out, stage_name, a.screen_n, a.eval_workers)
            fd = hd.family_dump(out)
            trend = hr.success_trend(hl.read_rows_tolerant(out / "episodes.jsonl"))
            _, sig = hr.go_rule(s["success"], s["per_shape"], fd["cyl_span_p75"], L)
            sline = {"phase": f"screen_{series}_h{hour}", "go": True, "signals": sig, "trend": trend, "screen": s, "family": fd, "train_stop_reason": reason, "stage": stage_name}
            record(root, sline)
        s = sline["screen"]
        rising = bool(sline.get("trend", {}).get("rising"))
        if reason is not None:
            halt(root, reason, f"{stage_name} hour {hour} check failed ({reason}); screen {s['success']:.3f}, per shape {json.dumps(s['per_shape'])}")
            return "stopped"
        dec = hr.post_hour_decision(series, hour, s["success"], s["per_shape"], rising, L)
        record(root, {"phase": f"decision_{series}_h{hour}", "go": dec != "escalate", "decision": dec, "screen": s["success"], "rising": rising, "curved_max": hr.curved_max(s["per_shape"], L)})
        if dec == "final":
            final_verdict(root, out, series, hour, a.final_n, a.eval_workers, a.force_go)
            return "done"
        if dec == "escalate":
            escalate(root, f"{stage_name} hour {hour}: screen {s['success']:.3f}, curved max {hr.curved_max(s['per_shape'], L):.3f}: end rule -> stop the hug line")
            return "stopped"
        if dec == "C15":
            series, hour = "long15", 1
        else:                                                                         # next | extra: identical continuation
            hour += 1
        init = out


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
    parts += ["This study's screens (scale 1.5 hours / Step C'; per shape, span / saturation / trend in summary.jsonl):", *screens(root, "screen_")]
    for d in lines(root):
        if d["phase"].startswith("train_"):
            tr = d.get("train_success") or {}
            fam = d.get("family") or {}
            rg = d.get("range_last300") or {}
            parts.append(f"  {d['phase']}: cyl span p75 {fam.get('cyl_span_p75')}, saturated_frac {rg.get('saturated_frac')}, train success first/last sixth {tr.get('first')}/{tr.get('last')}")
    return "\n".join(parts)


def escalate(root: Path, why: str) -> int:
    """STOP_ESCALATE_OWNER (+ ``ESCALATE``) with the three tables."""
    text = f"{why}\n\n{escalation_text(root)}\n"
    (root / "ESCALATE").write_text(text)
    log_entry("ESCALATE to the owner", text)
    return halt(root, "ESCALATE_OWNER", why)


def run_force_c15(root: Path, n_envs: int, sc: float, a: argparse.Namespace) -> int:
    """``--force-c15`` (expert override of the scale15 end rule): ONE hour of Step C' (``hug_long15``) warm from ``scale15_h3``, 20-min SuccessChecks (base = first 150 episodes of the run), n=30 screen,
    ``screen_long15_h1`` line, then STOP_long15_after1 (the manager decides). Resumable: finished train / screen phases are reused."""
    s3 = phase_line(root, "screen_scale15_h3")
    if s3 is None:
        log("[force-c15] no screen_scale15_h3 line in summary.jsonl: nothing to continue from")
        return 1
    marker = root / "STOP_scale15_after3"
    if marker.exists():
        dest = Path("data/rl/contended_runs")
        dest.mkdir(parents=True, exist_ok=True)
        marker.rename(dest / f"STOP_scale15_after3.{time.strftime('%Y%m%d_%H%M%S')}")
        log(f"[force-c15] {marker} moved aside into {dest}")
    record(root, {"phase": "expert_override_c15", "go": True, "why": "manager override of the scale15 hour-3 end rule: one hour of Step C' (hug_long15)", "screen_scale15_h3": s3["screen"]["success"]})
    init = root / "scale15_h3"
    out, reason = train_hour(root, "long15", 1, init, n_envs, sc, a.force_go, force_c15=True)
    sline = phase_line(root, "screen_long15_h1")
    if sline is None:
        s = screen(out, "hug_long15", a.screen_n, a.eval_workers)
        fd = hd.family_dump(out)
        trend = hr.success_trend(hl.read_rows_tolerant(out / "episodes.jsonl"))
        _, sig = hr.go_rule(s["success"], s["per_shape"], fd["cyl_span_p75"], L)
        sline = {"phase": "screen_long15_h1", "go": True, "signals": sig, "trend": trend, "screen": s, "family": fd, "train_stop_reason": reason, "stage": "hug_long15"}
        record(root, sline)
    s = sline["screen"]
    return halt(root, "long15_after1", f"hug_long15 hour 1 done (stop reason {reason}); screen {s['success']:.3f}, per shape {json.dumps(s['per_shape'])}; the manager decides")


def ceiling_passed(ceiling: str) -> bool:
    """True once the wall clock is at or past today's ``HH:MM``."""
    return time.strftime("%H:%M") >= ceiling


def fallback_table(root: Path) -> str:
    """One row per scale15 screen: success, per shape, normal span, hold duty."""
    rows = []
    for d in lines(root):
        if d["phase"].startswith("screen_scale15_h"):
            s = d["screen"]
            rows.append(f"  {d['phase']}: success {s['success']:.3f}, span {s.get('normal_span_mean', 0):.0f}, hold_duty {s.get('hold_duty_mean', 0):.3f}, per shape {json.dumps(s.get('per_shape'))}")
    return "\n".join(rows)


def run_fallback(root: Path, n_envs: int, sc: float, a: argparse.Namespace) -> int:
    """``--fallback-scale15``: identical scale15 hours (h4, h5, ... each warm from the previous hour, from ``scale15_h3``), train + n=30 screen per hour; screen >= 0.55 -> n=100 verdict (DONE);
    past ``--ceiling`` (wall clock) -> ESCALATE_OWNER with the per-hour table. Resumable: finished train / screen phases are reused."""
    if phase_line(root, "screen_scale15_h3") is None:
        log("[fallback] no screen_scale15_h3 line in summary.jsonl")
        return 1
    marker = root / "STOP_long15_after1"
    if marker.exists():
        dest = Path("data/rl/contended_runs")
        dest.mkdir(parents=True, exist_ok=True)
        marker.rename(dest / f"STOP_long15_after1.{time.strftime('%Y%m%d_%H%M%S')}")
        log(f"[fallback] {marker} moved aside into {dest}")
    if phase_line(root, "fallback_scale15") is None:
        record(root, {"phase": "fallback_scale15", "go": True, "why": "expert rule: Step C' was worse (0.033 vs 0.133); identical scale15 continuations from scale15_h3", "ceiling": a.ceiling})
    hour = 4
    while True:
        out, reason = train_hour(root, "scale15", hour, root / f"scale15_h{hour - 1}", n_envs, sc, a.force_go)
        sline = phase_line(root, f"screen_scale15_h{hour}")
        if sline is None:
            s = screen(out, "hug_scale15", a.screen_n, a.eval_workers)
            fd = hd.family_dump(out)
            trend = hr.success_trend(hl.read_rows_tolerant(out / "episodes.jsonl"))
            _, sig = hr.go_rule(s["success"], s["per_shape"], fd["cyl_span_p75"], L)
            sline = {"phase": f"screen_scale15_h{hour}", "go": True, "signals": sig, "trend": trend, "screen": s, "family": fd, "train_stop_reason": reason, "stage": "hug_scale15"}
            record(root, sline)
        s = sline["screen"]
        if reason is not None:
            halt(root, reason, f"hug_scale15 hour {hour} check failed ({reason}); screen {s['success']:.3f}, per shape {json.dumps(s['per_shape'])}")
            return 0
        if s["success"] >= hc2.SCREEN_DONE:
            final_verdict(root, out, "scale15", hour, a.final_n, a.eval_workers, a.force_go)
            return 0
        if ceiling_passed(a.ceiling):
            escalate(root, f"fallback scale15: ceiling {a.ceiling} passed after hour {hour} (screen {s['success']:.3f}); owner decides\n{fallback_table(root)}")
            return 0
        hour += 1


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
    ap.add_argument("--force-c15", action="store_true", help="expert override: move STOP_scale15_after3 aside and run ONE hour of Step C' (hug_long15, warm from scale15_h3, 20-min SuccessChecks with the run's own start success), n=30 screen, then STOP_long15_after1")
    ap.add_argument("--fallback-scale15", action="store_true", help="expert fallback: move STOP_long15_after1 aside and loop identical scale15 hours (h4, h5, ... from scale15_h3) with n=30 screens; >= 0.55 -> n=100 verdict, past --ceiling -> ESCALATE_OWNER")
    ap.add_argument("--ceiling", default="16:41", help="HH:MM wall clock after which the fallback chain stops with ESCALATE_OWNER (default 16:41)")
    a = ap.parse_args()
    root, sc = a.root, a.scale
    root.mkdir(parents=True, exist_ok=True)
    if a.fallback_scale15:
        return run_fallback(root, a.n_envs or m2v6.best_n_envs(json.loads(d6.BENCH.read_text())), sc, a)
    if a.force_c15:
        if (root / "STOP_long15_after1").exists():
            log("[force-c15] STOP_long15_after1 exists: remove it to continue")
            return 0
        return run_force_c15(root, a.n_envs or m2v6.best_n_envs(json.loads(d6.BENCH.read_text())), sc, a)
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
