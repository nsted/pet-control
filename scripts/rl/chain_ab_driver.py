"""A1 -> A2 -> B1 -> B2 navigation-chain driver (Regimen v2 plan sec 4.2). ONE training job at a time (10 envs), resumable from ``<root>/summary.jsonl``.

Body: L = 8 (the owner's URDF), action_mode hold, head-frame delta channel with delta DR, reply DR, bus model + bus penalty, touch_long and capacitive_obs on (observation layout of the hug stages),
asymmetric critic, gamma 0.997 / n_steps 1024 / gae 0.97, ent 0.005, reward success_bonus 150 / speed_gain 3 / time_penalty 0.5, training DR 0.3 ramped to 0.6 over the first half of each run.
Stages (``petctl.rl.curriculum.stages``; gates ``petctl.rl.curriculum.chain_ab``):
  A1 flat_nav      goals 0.8-2.0 m any bearing, random start pose, 40 s episodes;   gate success >= 0.80 (median t_goal reported)
  A2 flat_omni     goals 0.3-0.8 m any bearing, 15 s limit;                          gate success >= 0.70 AND median lateral displacement >= 0.4 m in 8 s for +-90 deg goals (second measurement)
  B1 terrain_easy  A2 + heightfield level 0.3-0.6 per episode + friction tiles, 20 s; gate success >= 0.75 at level 0.6
  B2 terrain_full  level 0.6-1.0, goals 1.5-2.5 m, 40 s;                             gate success >= 0.70 at level 1.0 AND imu_off paired drop (CI lower bound) >= 0.15
Each stage = ONE run (steps = 0.75 x 45 min x measured steps/s over the first 2 min, cap 4M) warm-started from the previous stage's passing run (A1 from data/rl/m2v7_L8/L8/P1), then the n = 100 gate
(dr 0.6, Wilson lower bound vs the frozen threshold; bus current <= 4 A median / <= 5 % episodes over 10 A and glitch 0 are hard fails). Pass -> skill models/skills/nav_chain/<stage>, next stage.
Fail -> ONE identical continuation run (same settings, warm from the failed run, reward stats kept) and a re-gate; fail again -> ``STOP_<stage>`` for the manager (the driver exits).
Every phase appends a line to summary.jsonl; markers: DONE (B2 passed), STOP_<stage>. A STOP marker must be removed by the manager before the driver continues.

    PETCTL_PROGRESS_EXPORT_S=900 nohup python scripts/rl/chain_ab_driver.py > data/rl/chain_ab/driver.log 2>&1 &
    python scripts/rl/chain_ab_driver.py --root data/rl/chain_ab_smoke --scale 0.01 --gate-n 4 --lateral-n 4 --force-go      # smoke test
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
import sys
import time
import traceback
from pathlib import Path
from typing import Any

sys.path.insert(0, str(Path(__file__).resolve().parent))

import m2v6_driver as d6  # noqa: E402
from m2_length_sweep import RECIPE_ENV, PoolEvaluator  # noqa: E402

from petctl.rl.curriculum import chain_ab as ca  # noqa: E402
from petctl.rl.curriculum import m2v6  # noqa: E402
from petctl.rl.curriculum.early_stop import HealthStopCallback  # noqa: E402
from petctl.rl.curriculum.gate2 import run_gate2  # noqa: E402
from petctl.rl.curriculum.runner import build_config  # noqa: E402
from petctl.rl.curriculum.stages import CHAIN_STAGES, get_stage  # noqa: E402

L = 8
WARM_START = Path("data/rl/m2v7_L8/L8/P1")
LOG = Path("docs/plans/CURRICULUM_LOG.md")
log, finished, lines, record = d6.log, d6.finished, d6.lines, d6.record


def run_dir(root: Path, stage: str, attempt: int) -> Path:
    """``<root>/<stage>/a<attempt>``."""
    return root / stage / f"a{attempt}"


def aside_unfinished(out: Path) -> None:
    """An interrupted run (no meta.json / policy.npz) is moved aside; its episode log is appended to otherwise."""
    if out.exists() and not finished(out):
        aside = out.with_name(f"{out.name}.aborted_{time.strftime('%Y%m%d_%H%M%S')}")
        out.rename(aside)
        log(f"[resume] unfinished run moved to {aside}")


def warm_source(root: Path, stage: str, attempt: int, ls: list[dict[str, Any]]) -> Path:
    """Warm-start run of an attempt: attempt 2 = the failed attempt 1 of the same stage; attempt 1 = the passing run of the previous stage (A1: the L=8 flat hold policy P1)."""
    if attempt == 2:
        return run_dir(root, stage, 1)
    i = CHAIN_STAGES.index(stage)
    if i == 0:
        return WARM_START
    prev = CHAIN_STAGES[i - 1]
    a = ca.passing_attempt(ls, prev)
    assert a is not None, f"previous stage {prev} has no passing gate"
    return run_dir(root, prev, a)


def train_run(root: Path, stage_name: str, attempt: int, init: Path, n_envs: int, scale: float, seed: int) -> dict[str, Any]:
    """Train one run of ``stage_name`` (resumable) under the wall-budget rule; returns the phase info."""
    from petctl.rl.train import train

    out = run_dir(root, stage_name, attempt)
    info_f = out / "phase.json"
    if finished(out) and info_f.exists():
        return json.loads(info_f.read_text())
    aside_unfinished(out)
    stage = get_stage(stage_name)
    cfg = build_config(stage, {"n_modules": float(L)})
    os.environ.update(RECIPE_ENV)
    budget = d6.BudgetCallbackFor(ca.BUDGET_MIN * 60.0 * scale, ca.CAP_STEPS, scale)
    ramp, health = ca.DrRampCallback(budget), HealthStopCallback()
    t0 = time.time()
    train(cfg, ca.CAP_STEPS, n_envs, out, seed, "ppo", None, stage.ent_coef, init, video=False, asymmetric=True, extra_callbacks=[health, budget, ramp],
          warm_copy_ret=attempt == 2, log_std_init=stage.log_std_init)
    wall = time.time() - t0
    steps = int(budget.stopped_at if budget.stopped_at is not None else json.loads((out / "status.json").read_text())["timesteps"])
    m = json.loads((out / "meta.json").read_text())
    m.update({"steps": steps, "wall_s": wall, "init_from": str(init), "n_envs": n_envs, "stage": stage_name, "attempt": attempt})
    (out / "meta.json").write_text(json.dumps(m, indent=1))
    if health.reason:
        (out / "early_stop.txt").write_text(health.reason)
    info = {"steps": steps, "wall_s": wall, "steps_per_s": steps / max(wall, 1e-9), "budget": budget.info, "early_stop": health.reason, "n_envs": n_envs, "dr_final": ramp.last,
            "init_from": str(init)}
    info_f.write_text(json.dumps(info, indent=1))
    return info


def gate_run(root: Path, stage_name: str, attempt: int, gate_n: int, lateral_n: int, workers: int) -> dict[str, Any]:
    """The n = 100 gate of a trained run (+ the lateral second measurement for A2); returns the gate2 result (also written as ``gate2_<stage>.json`` in the run dir)."""
    out = run_dir(root, stage_name, attempt)
    stage = get_stage(stage_name)
    spec = ca.gate_spec(stage_name, gate_n)
    params = dict(stage.gate.task_params)
    params.setdefault("bus_model", 1.0)
    res = run_gate2(out, stage, spec, evaluator=PoolEvaluator(out, "policy.npz", params, spec.dr_scale, stage.gate.episode_s, workers), write=False)
    if stage_name == "flat_omni":
        lp = {**ca.lateral_params(stage_name), "bus_model": 1.0}
        lat = ca.lateral_gate(PoolEvaluator(out, "policy.npz", lp, 0.6, ca.LATERAL_EPISODE_S, workers), lateral_n)
        res = ca.apply_second_measurement(res, stage_name, lat)
    (out / f"gate2_{stage_name}.json").write_text(json.dumps(res, indent=1, default=lambda o: None if o is None else str(o)))
    return res


def gate_line(stage: str, attempt: int, res: dict[str, Any], forced: bool) -> dict[str, Any]:
    """The summary line of a gate."""
    rb, cur, mm = res["robust"], res["current"], res["robust"]["metric_median"]
    abl = {k: {"drop": v["drop"]["drop"], "ci_lo": v["drop"]["ci_lo"], "ci_hi": v["drop"]["ci_hi"], "success": v["success"]["rate"], "passed": v["passed"]} for k, v in res.get("ablations", {}).items()}
    return {"phase": "gate", "stage": stage, "attempt": attempt, "passed": bool(res["passed"]) or forced, "status": res["status"], "n": rb["n"], "success": rb["rate"], "wilson_lo": rb["wilson_lo"],
            "wilson_hi": rb["wilson_hi"], "threshold": res["threshold"], "t_goal_median_s": mm.get("time_to_goal_s"), "lateral": res.get("lateral"), "ablations": abl,
            "bus_mean_a_median": cur.get("bus_mean_a_median"), "bus_peak_a_median": cur.get("bus_peak_a_median"), "frac_episodes_over_10a": cur.get("frac_episodes_over_10a"),
            "glitch_rate": res["glitch_rate"], "fail_reasons": res["fail_reasons"], "forced": forced, "gate2_json": str(Path(res["run"]) / f"gate2_{stage}.json")}


def bank_skill(stage_name: str, run: Path, line: dict[str, Any]) -> str | None:
    """Add the passing policy as ``models/skills/nav_chain/<stage>`` (``petctl rl skill-add`` code path, then moved under ``nav_chain``) with a note of its gate numbers; returns the path or None."""
    from petctl.rl import skills

    dest = skills.skills_root() / "nav_chain" / stage_name
    if dest.exists():
        return str(dest)
    note = (f"{stage_name} (A/B chain, L=8, hold channel): gate n={line['n']} dr 0.6 success {line['success']:.3f} (Wilson lo {line['wilson_lo']:.3f} vs {line['threshold']:.2f}), "
            f"t_goal median {line['t_goal_median_s']}, bus mean {line['bus_mean_a_median']} A, glitch {line['glitch_rate']}"
            + (f", lateral median {line['lateral']['median_m']:.3f} m" if line.get("lateral") else "")
            + (f", imu_off drop {line['ablations']['imu_off']['drop']:.3f} (CI lo {line['ablations']['imu_off']['ci_lo']:.3f})" if "imu_off" in line.get("ablations", {}) else ""))
    try:
        tmp = skills.add_skill(run, f"nav_chain_{stage_name}", note=note, render=False)
        dest.parent.mkdir(parents=True, exist_ok=True)
        shutil.move(str(tmp), str(dest))
        sj = json.loads((dest / "skill.json").read_text())
        sj.update({"id": f"nav_chain/{stage_name}", "name": stage_name, "gate": {k: line[k] for k in ("n", "success", "wilson_lo", "threshold", "t_goal_median_s", "lateral", "ablations", "bus_mean_a_median", "glitch_rate")}})
        (dest / "skill.json").write_text(json.dumps(sj, indent=1, default=str))
        skills.render_demo(dest)
        return str(dest)
    except Exception as e:  # noqa: BLE001
        traceback.print_exc()
        log(f"[skill] {stage_name}: skill-add failed: {type(e).__name__}: {e}")
        return None


def append_log_entry(root: Path, ls: list[dict[str, Any]]) -> None:
    """Factual curriculum-log entry of the finished chain."""
    rows = [f"  - {d['stage']} a{d['attempt']}: success {d['success']:.3f} (Wilson lo {d['wilson_lo']:.3f} vs {d['threshold']}), {d['status']}, t_goal median {d['t_goal_median_s']}" for d in ls if d.get("phase") == "gate"]
    with open(LOG, "a") as f:
        f.write(f"\n## {time.strftime('%Y-%m-%d')} A1-A2-B1-B2 chain finished (L=8)\n\n" + "\n".join(rows) + f"\n- Data: {root}\n")


def main() -> int:
    """CLI entry."""
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--root", type=Path, default=Path("data/rl/chain_ab"))
    ap.add_argument("--n-envs", type=int, default=10)
    ap.add_argument("--scale", type=float, default=1.0, help="multiplies the wall budget (smoke tests)")
    ap.add_argument("--gate-n", type=int, default=100)
    ap.add_argument("--lateral-n", type=int, default=ca.LATERAL_N)
    ap.add_argument("--eval-workers", type=int, default=4)
    ap.add_argument("--warm-from", type=Path, default=None, help="warm-start run for the FIRST attempt of the first stage that still needs training (instead of the previous stage's passing run); used to continue a failed stage under changed physics in a fresh --root")
    ap.add_argument("--force-go", action="store_true", help="smoke tests only: record gate numbers but treat every gate as passed (also skips the skill bank and the log entry)")
    a = ap.parse_args()
    root = a.root
    root.mkdir(parents=True, exist_ok=True)
    for m in root.glob("STOP_*"):
        log(f"[stop] {m} exists: remove it to continue")
        return 0
    if (root / "DONE").exists():
        log("[done] DONE exists")
        return 0
    os.environ.setdefault("PETCTL_PROGRESS_EXPORT_S", "900")
    log(f"chain_ab L={L} n_envs={a.n_envs} root={root} warm start {WARM_START}")
    while True:
        ls = lines(root)
        nxt = ca.next_action(ls)
        log(f"[next] {json.dumps(nxt)}")
        if nxt["action"] == "done":
            break
        st, att = nxt["stage"], nxt.get("attempt", 1)
        if nxt["action"] == "stop":
            fails = [d for d in ls if d.get("phase") == "gate" and d["stage"] == st]
            record(root, {"phase": "stop", "stage": st, "go": False, "why": f"{st} failed its gate twice", "fail_reasons": [d["fail_reasons"] for d in fails]})
            return d6.stop(root, st, f"{st} failed its n=100 gate on both attempts")
        if nxt["action"] == "train":
            init = a.warm_from if (a.warm_from is not None and att == 1 and not run_dir(root, st, 1).exists()) else warm_source(root, st, att, ls)
            log(f"train {st} a{att} warm from {init}")
            info = train_run(root, st, att, init, a.n_envs, a.scale, seed=1000 * (CHAIN_STAGES.index(st) + 1) + att)
            record(root, {"phase": "train", "stage": st, "attempt": att, "go": True, **info})
            continue
        res = gate_run(root, st, att, a.gate_n, a.lateral_n, a.eval_workers)
        line = gate_line(st, att, res, a.force_go)
        record(root, line)
        if line["passed"]:
            sk = None if a.force_go else bank_skill(st, run_dir(root, st, att), line)
            record(root, {"phase": "skill", "stage": st, "attempt": att, "skill": sk})
    if not a.force_go:
        append_log_entry(root, lines(root))
    (root / "DONE").write_text("A1-A2-B1-B2 chain: all gates passed, see summary.jsonl\n")
    log("[chain_ab complete: DONE]")
    return 0


if __name__ == "__main__":
    sys.exit(main())
