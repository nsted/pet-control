"""M2 v6 one-hour-run driver (Regimen v2): delta actions, self-bootstrapping hug starts, automatic go / no-go. ONE training job at a time with the best n_envs of the benchmark.

Schedule (``--length L``, default 10; resumable by the phase lines of ``<root>/summary.jsonl``; a finished training (``meta.json`` + ``policy.npz``) is not retrained):
  1. flat_pilot  m2_flat3, 12 min (<= 600k steps, dr 0.3, bearing 45, goals 0.6-1.2 m, ent 0.01). GO if the rolling training success >= 0.10 at 6 min (one seed retry; else STOP_flat_takeoff).
  2. flat_main   continues the pilot (dr 0.6, any bearing, goals 0.6-1.5 m, ent 0.005), 45 min training + n=30 screen (dr 0.6, 20 s). GO if screen success >= 0.55 else STOP_flat_main.
  3. hug_pilot   m2_hug6 warm-started from the flat main (block-mapped columns, new target block zero), 12 min (<= 600k steps, dr 0.3). GO if >= 5 % of the last 300 training episodes reach run >= 3
                 and normal span >= 90 deg; else STOP_hug_pilot (term sums + two contact-sheet PNGs, no video).
  4. hug_run1    continues the pilot (dr 0.6), 45 min + n=30 screen. GO if the mean hold_duty > 0.2 else STOP_hug_run1.
  5. hug_run2    continuation with identical hyper-parameters, 45 min + n=30 screen. GO if success >= 0.55 else STOP_hug_run2.
  6. final       n=100 held-out-shape gate (``m2v5.gate2_spec_hug5``: per-shape floor, capacitive_off ablation), the log entry, marker DONE. Stage videos: ``m2v6_render_watcher.py``.
Run wall budget rule (owner): measure steps/s over the first 2 minutes, steps = 0.75 x budget x steps/s (cap 2.5M, pilots 600k); training also stops at the wall budget. Evidence is never cut.
A failed go / no-go writes ``STOP_<reason>`` into the root and exits so the manager can consult the expert. Sim only.

    nohup python scripts/rl/m2v6_driver.py --length 10 > data/rl/m2v6/driver.log 2>&1 &
    python scripts/rl/m2v6_driver.py --root data/rl/m2v6_smoke --scale 0.01 --screen-n 4 --final-n 5       # smoke test (wall budgets x scale)
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

from m2_length_sweep import RECIPE_ENV, PoolEvaluator  # noqa: E402

from petctl.rl.curriculum import m2v6  # noqa: E402
from petctl.rl.curriculum.early_stop import HealthStopCallback  # noqa: E402
from petctl.rl.curriculum.gate2 import contact_sheet, run_gate2  # noqa: E402
from petctl.rl.curriculum.m2v5 import gate2_spec_hug5  # noqa: E402
from petctl.rl.curriculum.runner import build_config  # noqa: E402
from petctl.rl.curriculum.stages import FLAT3_PILOT, FLAT3_PILOT_ENT, HUG6_PILOT, get_stage, stage_variant  # noqa: E402
from petctl.rl.hug_buffer import BUFFER_ENV  # noqa: E402

BENCH = Path("data/rl/m2v5/benchmark.json")
PILOT_BUDGET_MIN, PILOT_CAP = 12.0, 2_500_000      # cap per the owner rule (0.75 x budget x steps/s, max 2.5M); the earlier 600k cap was sized for a 700 steps/s machine
MAIN_BUDGET_MIN = 45.0
TAKEOFF_AT_MIN = 6.0
LOG = Path("docs/plans/CURRICULUM_LOG.md")


def log(msg: str) -> None:
    """Timestamped progress line."""
    print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)


def finished(out: Path) -> bool:
    """A completed training: ``meta.json`` and ``policy.npz`` exist."""
    return (out / "meta.json").exists() and (out / "policy.npz").exists()


def train_phase(stage_name: str, out: Path, length: int, overrides: dict[str, float], ent: float, seed: int, init_from: Path | None, budget_min: float, cap: int, n_envs: int,
                scale: float, extra: list[Any] | None = None, buffer: Path | None = None, copy_ret: bool = False, action_mode: str | None = None, keep_log_std: bool = False, rescale_joints: float | None = None) -> dict[str, Any]:
    """Train one phase under the wall-budget rule (``keep_log_std``: with ``init_from``, keep the source's trained log-std instead of re-applying the stage's); returns ``{steps, wall_s, sps, budget...}`` (read back from ``phase.json`` when already finished)."""
    from petctl.rl.train import train

    info_f = out / "phase.json"
    if finished(out) and info_f.exists():
        return json.loads(info_f.read_text())
    stage = get_stage(stage_name) if action_mode is None else stage_variant(stage_name, action_mode)       # action_mode: the m2v7 stages take it as a parameter
    cfg = build_config(stage, {"n_modules": float(length), **overrides})
    os.environ.update(RECIPE_ENV)
    if buffer is not None:
        os.environ[BUFFER_ENV] = str(buffer.resolve())
    budget = BudgetCallbackFor(budget_min * 60.0 * scale, cap, scale)
    health = HealthStopCallback()
    t0 = time.time()
    try:
        train(cfg, cap, n_envs, out, seed, "ppo", None, ent, init_from, video=False, asymmetric=True, extra_callbacks=[health, budget, *(extra or [])], warm_copy_ret=copy_ret,
              log_std_init=None if keep_log_std else stage.log_std_init, rescale_joints=rescale_joints)
    finally:
        os.environ.pop(BUFFER_ENV, None)
    wall = time.time() - t0
    steps = int(budget.stopped_at if budget.stopped_at is not None else json.loads((out / "status.json").read_text())["timesteps"])       # a takeoff stop ends the run before the budget does
    m = json.loads((out / "meta.json").read_text())
    m.update({"steps": steps, "wall_s": wall, "init_from": str(init_from), "n_envs": n_envs, "stage": stage_name})
    (out / "meta.json").write_text(json.dumps(m, indent=1))
    if health.reason:
        (out / "early_stop.txt").write_text(health.reason)
    info = {"steps": steps, "wall_s": wall, "steps_per_s": steps / max(wall, 1e-9), "budget": budget.info, "early_stop": health.reason, "n_envs": n_envs}
    info_f.write_text(json.dumps(info, indent=1))
    return info


def BudgetCallbackFor(budget_s: float, cap: int, scale: float) -> m2v6.BudgetCallback:  # noqa: N802
    """Budget callback whose throughput measurement window shrinks with ``scale`` (smoke tests)."""
    return m2v6.BudgetCallback(budget_s, cap, measure_s=max(5.0, m2v6.MEASURE_S * scale))


def screen(stage_name: str, out: Path, n: int, workers: int, flat: bool) -> dict[str, float]:
    """Screening gate: ``n`` held-out-style seeds, robust DR 0.6, the stage's gate distribution; writes ``gate2_<stage>.json`` (best-episode record) and returns the summary. Go / no-go use only."""
    stage = get_stage(stage_name)
    params = dict(stage.gate.task_params)
    params.setdefault("bus_model", 1.0)
    ev = PoolEvaluator(out, "policy.npz", params, 0.6, stage.gate.episode_s, workers)
    rows = ev({}, m2v6.screen_seeds(n, flat))
    (out / f"gate2_{stage_name}.json").write_text(json.dumps(m2v6.best_episode_record(stage_name, rows, params, 0.6, stage.gate.episode_s), indent=1))
    s = m2v6.summarise_rows(rows)
    (out / "screen.json").write_text(json.dumps(s, indent=1))
    log(f"screen {out.name}: {json.dumps(s)}")
    return s


def dump_stop(out: Path, root: Path, tag: str, workers: int, stage_name: str = "m2_hug6") -> None:
    """Stop dump: mean term sums of the last 200 training episodes and two contact-sheet PNGs (best / worst of a 20-episode screen); no video."""
    rows = m2v6.read_episodes(out)[-200:]
    terms = {k: float(np.nanmean([r[k] for r in rows if k in r])) for k in sorted({k for r in rows for k in r if k.startswith("term_")})}
    (root / f"stop_{tag}_term_sums.json").write_text(json.dumps({"episodes": len(rows), "mean_term_sums": terms}, indent=1))
    log(f"[stop] mean term sums (last {len(rows)} episodes): {json.dumps(terms)}")
    try:
        screen(stage_name, out, 20, workers, False)
        res = json.loads((out / f"gate2_{stage_name}.json").read_text())
        ps = sorted(res["per_seed"], key=lambda r: r["return"])
        for name, r in (("worst", ps[0]), ("best", ps[-1])):
            png = contact_sheet(out, stage_name, seed=int(r["seed"]), out=root / f"stop_{tag}_sheet_{name}.png")
            log(f"[stop] {name} sheet (return {r['return']:.1f}): {png}")
    except Exception as e:  # noqa: BLE001
        traceback.print_exc()
        log(f"[stop] sheets failed: {type(e).__name__}: {e}")


def lines(root: Path) -> list[dict[str, Any]]:
    """Phase lines of ``summary.jsonl``."""
    f = root / "summary.jsonl"
    return [json.loads(x) for x in f.read_text().splitlines() if x.strip()] if f.exists() else []


def record(root: Path, line: dict[str, Any]) -> None:
    """Append one phase line."""
    line = {**line, "time": time.strftime("%Y-%m-%d %H:%M:%S")}
    with open(root / "summary.jsonl", "a") as f:
        f.write(json.dumps(line) + "\n")
    log(f"[phase] {json.dumps(line)}")


def stop(root: Path, reason: str, why: str) -> int:
    """Write the marker ``STOP_<reason>``; the driver then exits (manager checkpoint)."""
    (root / f"STOP_{reason}").write_text(why + ": manager checkpoint\n")
    log(f"[stop] {why}: wrote {root / f'STOP_{reason}'}")
    return 0


def done_line(root: Path, phase: str) -> dict[str, Any] | None:
    """The recorded go line of ``phase`` or None."""
    for d in lines(root):
        if d["phase"] == phase and d.get("go"):
            return d
    return None


def append_log_entry(root: Path, length: int, final: dict[str, Any], spans: dict[str, Any]) -> None:
    """Append the factual curriculum-log entry of the finished schedule."""
    rb = final["robust"]
    txt = (f"\n## {time.strftime('%Y-%m-%d')} m2v6 one-hour schedule (L={length}) final gate\n\n"
           f"- Runs: {json.dumps(spans)}\n"
           f"- Final gate n={final['n']} held-out shapes, dr {final['dr_scale']}: success {rb['rate']:.3f} (Wilson {rb['wilson_lo']:.3f}-{rb['wilson_hi']:.3f}); per shape "
           f"{json.dumps({g: round(v['rate'], 2) for g, v in final.get('groups', {}).items()})}; ablations {json.dumps({k: v.get('drop', {}).get('drop') for k, v in final.get('ablations', {}).items()})}; "
           f"passed={final['passed']} reasons={final['fail_reasons']}\n- Data: {root}\n")
    with open(LOG, "a") as f:
        f.write(txt)


def main() -> int:
    """CLI entry."""
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--root", type=Path, default=Path("data/rl/m2v6"))
    ap.add_argument("--length", type=int, default=10)
    ap.add_argument("--n-envs", type=int, default=0, help="0: the best n_envs of data/rl/m2v5/benchmark.json")
    ap.add_argument("--scale", type=float, default=1.0, help="multiplies every wall budget (smoke tests)")
    ap.add_argument("--screen-n", type=int, default=30)
    ap.add_argument("--final-n", type=int, default=100, help="held-out gate episodes (100 = 5 shapes x 20; smaller only for smoke tests)")
    ap.add_argument("--eval-workers", type=int, default=4)
    ap.add_argument("--force-go", action="store_true", help="smoke tests only: record the go / no-go values but never stop on them")
    a = ap.parse_args()
    root, L, sc = a.root, a.length, a.scale
    root.mkdir(parents=True, exist_ok=True)
    for m in root.glob("STOP_*"):
        log(f"[stop] {m} exists: remove it to continue")
        return 0
    if (root / "DONE").exists():
        log("[done] schedule already complete")
        return 0
    n_envs = a.n_envs or m2v6.best_n_envs(json.loads(BENCH.read_text()))
    log(f"m2v6 L={L} n_envs={n_envs} root={root}")
    base = root / f"L{L}"
    pilot_dirs = [base / "flat_pilot" / f"seed{k}" for k in (0, 1)]
    flat_main, hug_pilot, run1, run2 = base / "flat_main", base / "hug_pilot", base / "hug_run1", base / "hug_run2"
    buf = base / "hug_buffer.json"
    spans: dict[str, Any] = {}

    # ---- 1. flat pilot (takeoff rule, one seed retry)
    if done_line(root, "flat_pilot") is None:
        for k, out in enumerate(pilot_dirs):
            if any(d["phase"] == "flat_pilot" and d["seed"] == k for d in lines(root)):
                continue
            take = m2v6.WallTakeoffCallback(TAKEOFF_AT_MIN * 60.0 * sc)
            log(f"flat pilot seed{k} -> {out}")
            info = train_phase("m2_flat3", out, L, FLAT3_PILOT, FLAT3_PILOT_ENT, k, None, PILOT_BUDGET_MIN, PILOT_CAP, n_envs, sc, extra=[take])
            if take.checked_at is None:                                  # the pilot ended (step cap) before the 6-min check: apply the rule to its final rolling success
                ok, take.rate = m2v6.takeoff_ok([float(r.get("success", 0.0)) for r in m2v6.read_episodes(out)])
                take.checked_at, take.failed = info["steps"], not ok
                log(f"[takeoff] pilot ended before the check time: rolling success {take.rate:.3f} over the last {m2v6.TAKEOFF_WINDOW} episodes: {'ok' if ok else 'FAIL'}")
            go = not take.failed or a.force_go
            record(root, {"phase": "flat_pilot", "seed": k, "go": go, "rolling_success": take.rate, "checked_at": take.checked_at, **info})
            if go:
                break
        if done_line(root, "flat_pilot") is None:
            return stop(root, "flat_takeoff", "flat pilot failed the takeoff rule (rolling training success < 0.10 at 6 min) on both seeds")
    pilot = pilot_dirs[int(done_line(root, "flat_pilot")["seed"])]  # type: ignore[index]

    # ---- 2. flat main
    if done_line(root, "flat_main") is None:
        info = train_phase("m2_flat3", flat_main, L, {}, get_stage("m2_flat3").ent_coef, 100, pilot, MAIN_BUDGET_MIN, m2v6.MAX_STEPS, n_envs, sc, copy_ret=True)
        s = screen("m2_flat3", flat_main, a.screen_n, a.eval_workers, True)
        go = m2v6.screen_go("flat_main", s) or a.force_go
        record(root, {"phase": "flat_main", "seed": 100, "go": go, "screen": s, **info})
        if not go:
            return stop(root, "flat_main", f"flat main screen success {s['success']:.2f} < {m2v6.FLAT_SCREEN_GO}")

    # ---- 3. hug pilot
    if done_line(root, "hug_pilot") is None:
        info = train_phase("m2_hug6", hug_pilot, L, HUG6_PILOT, get_stage("m2_hug6").ent_coef, 200, flat_main, PILOT_BUDGET_MIN, PILOT_CAP, n_envs, sc, buffer=buf)
        frac = m2v6.hug_pilot_fraction(m2v6.read_episodes(hug_pilot))
        go = m2v6.hug_pilot_go(frac) or a.force_go
        record(root, {"phase": "hug_pilot", "seed": 200, "go": go, "frac_run3_span90": frac, "buffer_states": len(json.loads(buf.read_text())) if buf.exists() else 0, **info})
        if not go:
            dump_stop(hug_pilot, root, "hug_pilot", a.eval_workers)
            return stop(root, "hug_pilot", f"hug pilot: only {100 * frac:.1f} % of training episodes reached run >= 3 and span >= 90 deg (< 5 %)")

    # ---- 4. hug run 1, 5. hug run 2 (continuations with identical hyper-parameters)
    for phase, out, init, seed in (("hug_run1", run1, hug_pilot, 201), ("hug_run2", run2, run1, 202)):
        if done_line(root, phase) is not None:
            continue
        info = train_phase("m2_hug6", out, L, {}, get_stage("m2_hug6").ent_coef, seed, init, MAIN_BUDGET_MIN, m2v6.MAX_STEPS, n_envs, sc, buffer=buf, copy_ret=True)
        s = screen("m2_hug6", out, a.screen_n, a.eval_workers, False)
        go = m2v6.screen_go(phase, s) or a.force_go
        record(root, {"phase": phase, "seed": seed, "go": go, "screen": s, "buffer_states": len(json.loads(buf.read_text())) if buf.exists() else 0, **info})
        if not go:
            return stop(root, phase, f"{phase} screen {json.dumps(s)} failed its go rule")

    # ---- 6. final gate
    stage = get_stage("m2_hug6")
    spec = gate2_spec_hug5()
    spec.n = a.final_n
    params = dict(stage.gate.task_params)
    params.setdefault("bus_model", 1.0)
    log(f"final gate n={spec.n}")
    res = run_gate2(run2, stage, spec, evaluator=PoolEvaluator(run2, "policy.npz", params, spec.dr_scale, stage.gate.episode_s, a.eval_workers))
    record(root, {"phase": "final", "go": True, "passed": res["passed"], "success": res["robust"]["rate"], "wilson_lo": res["robust"]["wilson_lo"], "groups": res.get("groups"),
                  "ablations": {k: v.get("drop") for k, v in res.get("ablations", {}).items()}, "fail_reasons": res["fail_reasons"]})
    for p in ("flat_pilot", "flat_main", "hug_pilot", "hug_run1", "hug_run2"):
        d = done_line(root, p)
        spans[p] = None if d is None else {"steps": d.get("steps"), "wall_min": round(d.get("wall_s", 0) / 60, 1)}
    append_log_entry(root, L, res, spans)
    (root / "DONE").write_text(f"final gate passed={res['passed']}\n")
    log("[m2v6 complete]")
    return 0


if __name__ == "__main__":
    sys.exit(main())
