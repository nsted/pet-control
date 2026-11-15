"""Odd-length study driver (L = 9 by default): flat hold-channel pilot (cold start) -> per-family hug curriculum (stage ``hug_curr_02``, hold_ramp 1) in up to three automatic 45-min hours.
ONE training job at a time (10 envs), resumable from ``<root>/summary.jsonl``. The L = 8 driver (``hug_curr2_driver.py``) is not modified.

Body: the 9-module robot is the GENERATED URDF variant (``resolve_urdf_dir`` / ``petctl.sim.urdf_gen``, ``tail_flip`` auto); the physical tail orientation is an unconfirmed owner assumption and is
written into every log entry and marker.

STEP 1  flat P1 pilot (m2_flat4, action_mode hold, cold start, DR 0.3, bearing 45, goals 0.6-1.2 m, ent 0.01, log_std -1.0, 20 s episodes): 25 min budget, steps = 0.75 x budget x measured steps/s (cap 6M).
        GO if the rolling success (last 100 episodes) >= 0.15 at the end. NO-GO: ONE retry (new seed, 45 min budget, cap 12M); a second failure writes STOP_L<n>_flat.
STEP 2  hug hours (stage hug_curr_02, hold_weight 10, hold_ramp 1, constant lr 1.5e-4, per-family levels, buffer starts, 45 min budget, cap 4M steps). Hour 1 warm-starts from the flat policy (name-block
        mapping, levels fresh, empty buffer) and applies the 20-min trend rule + box safety stop of hug_curr_02; hours 2-3 continue from the previous hour (model, vecnormalize, buffer, levels; box safety only).
        After each hour an n=30 screen (6 per shape, the length's held-out set, dr 0.6): >= 0.55 -> n=100 verdict (gate2_spec_hug5(N), per-shape floor 0.5, capacitive_off) -> DONE_L<n>.
        Else hour 2 always follows hour 1; hour 3 follows hour 2 only if its screen >= 0.15 or a curved gate shape scored >= 2/6, else STOP_L<n>_hug; after hour 3 STOP_L<n>_after3.

    nohup python scripts/rl/hug_curr_odd_driver.py --length 9 --root data/rl/odd9 > data/rl/odd9/driver.log 2>&1 &
    python scripts/rl/hug_curr_odd_driver.py --root data/rl/odd9_smoke --scale 0.01 --screen-n 5 --final-n 5 --force-go          # smoke test (wall budgets x scale)
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
from m2_length_sweep import PoolEvaluator  # noqa: E402

from petctl.rl.curriculum import hug_curr2 as hc2  # noqa: E402
from petctl.rl.curriculum import hug_long as hl  # noqa: E402
from petctl.rl.curriculum import hug_odd as ho  # noqa: E402
from petctl.rl.curriculum import m2v6  # noqa: E402
from petctl.rl.curriculum.gate2 import run_gate2  # noqa: E402
from petctl.rl.curriculum.m2v5 import gate2_spec_hug5  # noqa: E402
from petctl.rl.curriculum.stages import FLAT4_PILOT, FLAT4_PILOT_ENT, get_stage  # noqa: E402
from petctl.rl.hug_levels import LEVEL_FILE  # noqa: E402
from petctl.rl.records import HugFamilyLevelCallback  # noqa: E402
from petctl.rl.target_shapes import held_out_names  # noqa: E402

STAGE = hd.STAGE
log, finished, record, stop, lines = d6.log, d6.finished, d6.record, d6.stop, d6.lines


def aside_unfinished(out: Path) -> None:
    """An interrupted run (no meta.json / policy.npz) is moved aside: its episode log is appended to otherwise."""
    if out.exists() and not finished(out):
        aside = out.with_name(f"{out.name}.aborted_{time.strftime('%Y%m%d_%H%M%S')}")
        out.rename(aside)
        log(f"[resume] unfinished run moved to {aside}")


def screen_n(out: Path, length: int, n: int, workers: int) -> dict[str, Any]:
    """Screening gate on the length's held-out set, always scored at L3 (the gate params carry no curriculum): ``n`` episodes, ``n // 5`` per shape, dr 0.6."""
    stage = get_stage(STAGE)
    params = dict(stage.gate.task_params)
    params.setdefault("bus_model", 1.0)
    assert "hug_curriculum" not in params and "shape_mix" not in params
    ev = PoolEvaluator(out, "policy.npz", params, 0.6, stage.gate.episode_s, workers)
    rows = ev({}, hl.balanced_screen_seeds(n))
    (out / f"gate2_{STAGE}.json").write_text(json.dumps(m2v6.best_episode_record(STAGE, rows, params, 0.6, stage.gate.episode_s), indent=1))
    s = {**m2v6.summarise_rows(rows), "per_shape": hl.per_shape_success(rows, length)}
    (out / "screen.json").write_text(json.dumps(s, indent=1))
    log(f"screen {out.name}: {json.dumps(s)}")
    return s


def log_entry(root: Path, length: int, title: str, body: str) -> None:
    """Append a factual entry to the curriculum log (always with the body note)."""
    with open(d6.LOG, "a") as f:
        f.write(f"\n## {time.strftime('%Y-%m-%d')} L={length} odd-length study: {title}\n\n{body}\n- Body: {ho.body_note(length)}\n- Data: {root}\n")


def flat_stage(root: Path, base: Path, length: int, n_envs: int, sc: float, force: bool) -> Path | None:
    """Flat P1 pilot with one retry; returns the run dir of the go pilot or None (no-go twice)."""
    attempts = (("P1", ho.FLAT_BUDGET_MIN, ho.FLAT_CAP, 0), ("P1_retry", ho.RETRY_BUDGET_MIN, ho.RETRY_CAP, 1))
    for tag, budget, cap, seed in attempts:
        prev = [d for d in lines(root) if d["phase"] == f"flat_{tag}"]
        if prev:
            if prev[-1]["go"]:
                return base / tag
            continue
        out = base / tag
        aside_unfinished(out)
        info = d6.train_phase("m2_flat4", out, length, FLAT4_PILOT, FLAT4_PILOT_ENT, seed, None, budget, cap, n_envs, sc, action_mode="hold")
        rows = m2v6.read_episodes(out)
        go, sig = ho.flat_pilot_go(rows, info["steps"])
        record(root, {"phase": f"flat_{tag}", "go": go or force, "signals": sig, "time_to_takeoff": m2v7_takeoff(rows), "body": ho.body_note(length), **info})
        if go or force:
            return out
        log(f"[flat] {tag} no-go: rolling success {sig['success']:.3f} (< {ho.FLAT_GO_SUCCESS})")
    return None


def m2v7_takeoff(rows: list[dict[str, Any]]) -> int | None:
    """Step of the first 100-episode window with success >= 0.10 (recorded, informational)."""
    from petctl.rl.curriculum import m2v7
    return m2v7.time_to_takeoff(rows)


def hug_hour(root: Path, base: Path, length: int, hour: int, init: Path, n_envs: int, sc: float, a: argparse.Namespace) -> tuple[Path, str | None]:
    """Train hug hour ``hour`` (resumable); returns ``(run dir, stop reason or None)``."""
    out, buf = base / f"hug_h{hour}", base / "hug_buffer.json"
    if any(d["phase"] == f"train_h{hour}" for d in lines(root)):
        return out, None
    aside_unfinished(out)
    out.mkdir(parents=True, exist_ok=True)
    if hour > 1 and not (out / LEVEL_FILE).exists() and (init / LEVEL_FILE).exists():
        shutil.copy(init / LEVEL_FILE, out / LEVEL_FILE)
        log(f"[levels] per-family levels copied from {init}")
    t1 = hc2.CHECK1_MIN * 60.0 * sc if hour == 1 else 1e12                       # the 20-min trend rule applies to hour 1 only; box safety to every hour
    checks = hc2.Curr2Checks(out, t1, sc)
    levels = HugFamilyLevelCallback(out)
    old_env = {k: os.environ.get(k) for k in ("PETCTL_LR", "PETCTL_LR_CONST", "PETCTL_PROGRESS_EXPORT_S")}
    os.environ.update({"PETCTL_LR": str(hl.LR), "PETCTL_LR_CONST": "1"})
    os.environ.setdefault("PETCTL_PROGRESS_EXPORT_S", "900")
    try:
        cont = hour > 1                                                          # continuation: keep the trained log-std and reward-normalisation statistics
        info = d6.train_phase(STAGE, out, length, {"hold_ramp": a.hold_ramp}, get_stage(STAGE).ent_coef, 300 + hour, init, ho.HUG_BUDGET_MIN, ho.HUG_CAP, n_envs, sc,
                              extra=[levels, checks], buffer=buf, copy_ret=cont, keep_log_std=cont)
    finally:
        for k, v in old_env.items():
            if v is None:
                os.environ.pop(k, None)
            else:
                os.environ[k] = v
    reason = checks.reason
    fd = hd.family_dump(out)
    record(root, {"phase": f"train_h{hour}", "go": reason is None or a.force_go, "stop_reason": reason, "checks": checks.results, "family": fd, "hold_ramp": a.hold_ramp,
                  "buffer_states": len(json.loads(buf.read_text())) if buf.exists() else 0, "body": ho.body_note(length), **info})
    if reason is not None and not a.force_go:
        hd.dump_stop(out, root, reason, a.eval_workers)
        return out, reason
    return out, None


def main() -> int:
    """CLI entry."""
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--root", type=Path, default=Path("data/rl/odd9"))
    ap.add_argument("--length", type=int, default=9, help="body length N (module count); the held-out set is target_shapes.held_out_set(N)")
    ap.add_argument("--n-envs", type=int, default=0, help="0: the best n_envs of data/rl/m2v5/benchmark.json")
    ap.add_argument("--scale", type=float, default=1.0, help="multiplies every wall budget and check time (smoke tests)")
    ap.add_argument("--screen-n", type=int, default=hl.SCREEN_N)
    ap.add_argument("--final-n", type=int, default=100)
    ap.add_argument("--eval-workers", type=int, default=4)
    ap.add_argument("--hold-ramp", type=float, default=1.0, help="hold_ramp task knob (1 = hold pays hold_weight x min(2, 1 + hold_ticks / hold_need))")
    ap.add_argument("--force-go", action="store_true", help="smoke tests only: record the go / no-go values but never stop on them")
    a = ap.parse_args()
    root, L, sc = a.root, a.length, a.scale
    root.mkdir(parents=True, exist_ok=True)
    for m in root.glob("STOP_*"):
        log(f"[stop] {m} exists: remove it to continue")
        return 0
    if (root / f"DONE_L{L}").exists():
        log(f"[done] DONE_L{L} exists")
        return 0
    n_envs = a.n_envs or m2v6.best_n_envs(json.loads(d6.BENCH.read_text()))
    base = root / f"L{L}"
    log(f"odd-length study L={L} n_envs={n_envs} root={root} hold_ramp={a.hold_ramp} ({ho.body_note(L) or 'owner 8-module body'})")

    # ---- STEP 1 flat pilot (cold start, hold channel)
    flat = flat_stage(root, base, L, n_envs, sc, a.force_go)
    if flat is None:
        log_entry(root, L, "flat P1 pilot NO-GO twice", "- " + "; ".join(f"{d['phase']}: rolling success {d['signals']['success']:.3f} at {d['steps']} steps" for d in lines(root) if d["phase"].startswith("flat_")))
        return stop(root, f"L{L}_flat", f"L={L} flat P1 pilot failed twice (rolling success < {ho.FLAT_GO_SUCCESS}); {ho.body_note(L)}")

    # ---- STEP 2 hug hours
    init = flat
    for hour in range(1, ho.MAX_HOURS + 1):
        out, reason = hug_hour(root, base, L, hour, init, n_envs, sc, a)
        if reason is not None:
            return stop(root, reason, f"hug hour {hour} check failed ({reason}); {ho.body_note(L)}")
        scl = [d for d in lines(root) if d["phase"] == f"screen_h{hour}"]
        if scl:
            s, decision = scl[-1]["screen"], scl[-1]["decision"]
        else:
            s = screen_n(out, L, a.screen_n, a.eval_workers)
            decision = "final" if a.force_go and hour == 1 else ho.hour_decision(hour, s["success"], s["per_shape"], L)
            record(root, {"phase": f"screen_h{hour}", "go": decision in ("final", "next"), "decision": decision, "screen": s, "family": hd.family_dump(out), "body": ho.body_note(L)})
        if decision == "final":
            return final_verdict(root, out, L, hour, a)
        if decision == "next":
            init = out
            continue
        screens = {f"h{d['phase'][-1]}": {"success": d["screen"]["success"], "per_shape": d["screen"]["per_shape"]} for d in lines(root) if d["phase"].startswith("screen_h")}
        marker = f"L{L}_hug" if decision == "stop" else f"L{L}_after3"
        log_entry(root, L, f"hug stop ({marker})", f"- screens (n=30, L3, {list(held_out_names(L))}): {json.dumps(screens)}")
        return stop(root, marker, f"hug hour {hour} screen {s['success']:.3f}, per shape {json.dumps(s['per_shape'])}; screens {json.dumps(screens)}; {ho.body_note(L)}")
    return 0


def final_verdict(root: Path, out: Path, L: int, hour: int, a: argparse.Namespace) -> int:
    """n=100 verdict (per-shape floor, capacitive_off) of hour ``hour``'s policy; writes DONE_L<n>."""
    stage = get_stage(STAGE)
    spec = gate2_spec_hug5(L)
    spec.n = a.final_n
    params = dict(stage.gate.task_params)
    params.setdefault("bus_model", 1.0)
    log(f"final gate n={spec.n} (hour {hour}, L={L})")
    res = run_gate2(out, stage, spec, evaluator=PoolEvaluator(out, "policy.npz", params, spec.dr_scale, stage.gate.episode_s, a.eval_workers))
    groups = {g: round(v["rate"], 2) for g, v in res.get("groups", {}).items()}
    record(root, {"phase": "final", "go": True, "hour": hour, "passed": res["passed"], "success": res["robust"]["rate"], "wilson_lo": res["robust"]["wilson_lo"], "groups": res.get("groups"),
                  "ablations": {k: v.get("drop") for k, v in res.get("ablations", {}).items()}, "fail_reasons": res["fail_reasons"], "body": ho.body_note(L)})
    if not a.force_go:                                                           # smoke tests must not write the curriculum log
        rb = res["robust"]
        log_entry(root, L, f"final gate after hug hour {hour}",
                  f"- Final gate n={res['n']} (L={L} held-out set {list(held_out_names(L))}, dr {res['dr_scale']}): success {rb['rate']:.3f} (Wilson {rb['wilson_lo']:.3f}-{rb['wilson_hi']:.3f}); per shape "
                  f"{json.dumps(groups)}; ablations {json.dumps({k: v.get('drop', {}).get('drop') for k, v in res.get('ablations', {}).items()})}; passed={res['passed']} reasons={res['fail_reasons']}")
    (root / f"DONE_L{L}").write_text(f"final gate passed={res['passed']}; {ho.body_note(L)}\n")
    log(f"[odd-length study complete: DONE_L{L}]")
    return 0


if __name__ == "__main__":
    sys.exit(main())
