"""M2 v7 pilot driver (Regimen v2): the learned hold channel (action_mode ``hold``). ONE training job at a time, the best n_envs of the benchmark, resumable from ``<root>/summary.jsonl``.

Delta actions failed flat takeoff (m2v6), so every stage is back to absolute EMA targets and the policy gets one extra output ``h`` (EMA coefficient 0.3 + 0.7 h; h = 0 the mapping that walked,
h = 1 a frozen pose). Pilots, in order, each with a 12-min wall budget (steps = 0.75 x budget x steps/s measured over the first 2 min, capped) and an automatic go / no-go:

  P0 control   m2_flat4 action_mode abs, <= 2.0M steps, dr 0.3, bearing 45, goals 0.6-1.2 m, ent 0.01. GO if at 1.5M steps the rolling success (last 100 episodes) >= 0.15 AND the median
               final_dist / dist0 <= 0.7. NO-GO: STOP_P0 and data/rl/m2v7/P0_config_diff.txt (m2_flat4 config vs the working m2_flat seed0 config of the n = 8 sweep).
  P1 hold      identical plus action_mode hold. GO if the same signals at 2.0M steps AND time-to-takeoff (first 100-episode window with success >= 0.10) <= 1.5 x P0's. NO-GO: STOP_P1_hold_unused.
  P2 hug       m2_hug7 (action_mode hold), warm start from P1 (``--warm-from p0`` to start from the P0 control instead), <= 2.5M steps, dr 0.3, hold_weight 3 (``--hold-weight 6`` re-pilots).
               Over the last 300 training episodes: GO(approach) >= 5 % reach run >= 3 AND normal span >= 90 deg, else STOP_P2_approach (term sums + 2 contact sheets, no video).
               GO(holdability): among those >= 30 % keep the state for >= 2 s AND the mean h over wrap ticks > 0.7; else STOP_P2_hold_unused (h not rising) or STOP_P2_holdability.
  After a successful P2 the driver writes DONE_P2 and stops (the manager launches the long runs). No videos. A STOP_* marker must be removed by the manager before the driver continues.

    nohup python scripts/rl/m2v7_driver.py --length 10 > data/rl/m2v7/driver.log 2>&1 &
    python scripts/rl/m2v7_driver.py --root data/rl/m2v7_smoke --scale 0.01      # smoke test (wall budgets x scale)
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from typing import Any

sys.path.insert(0, str(Path(__file__).resolve().parent))

import m2v6_driver as d6  # noqa: E402
from m2_length_sweep import RECIPE_ENV  # noqa: E402

from petctl.rl.curriculum import m2v6, m2v7  # noqa: E402
from petctl.rl.curriculum.stages import FLAT4_PILOT, FLAT4_PILOT_ENT, HUG7_PILOT, get_stage  # noqa: E402
from petctl.rl.hug_buffer import BUFFER_ENV  # noqa: E402,F401

REFERENCE_CFG = Path("data/rl/m2_sweep/8/m2_flat/seed0/config.json")      # the working n = 8 flat run (absolute EMA targets, recipe env)
log, finished, lines, record, stop = d6.log, d6.finished, d6.lines, d6.record, d6.stop


def go_line(root: Path, phase: str, **match: Any) -> dict[str, Any] | None:
    """The recorded go line of ``phase`` (all ``match`` items equal) or None."""
    for d in lines(root):
        if d["phase"] == phase and d.get("go") and all(d.get(k) == v for k, v in match.items()):
            return d
    return None


def any_line(root: Path, phase: str, **match: Any) -> dict[str, Any] | None:
    """The last recorded line (go or no-go) of ``phase`` with the matching items, or None."""
    found = [d for d in lines(root) if d["phase"] == phase and all(d.get(k) == v for k, v in match.items())]
    return found[-1] if found else None


def flat_phase(root: Path, base: Path, phase: str, mode: str, n_envs: int, scale: float, cap: int, check: int, force: bool, t_p0: int | None = None) -> tuple[bool, dict[str, Any]]:
    """Run (or reuse) a flat pilot and decide go / no-go; returns ``(go, line)``. ``t_p0``: P0's time-to-takeoff (P1 only)."""
    out = base / phase
    info = d6.train_phase("m2_flat4", out, int(base.name[1:]), FLAT4_PILOT, FLAT4_PILOT_ENT, 0, None, d6.PILOT_BUDGET_MIN, cap, n_envs, scale, action_mode=mode)
    rows = m2v6.read_episodes(out)
    chk = min(check, info["steps"])                                    # a budget-cut run is judged at its end
    sig = m2v7.flat_signals(rows, chk)
    t_take = m2v7.time_to_takeoff(rows)
    go = m2v7.flat_go(sig) if phase == "P0" else m2v7.p1_go(sig, t_p0, t_take)
    line = {"phase": phase, "action_mode": mode, "go": go or force, "check_steps": chk, "signals": sig, "time_to_takeoff": t_take, "takeoff_p0": t_p0,
            "h_mean_last100": _tail_mean(rows, "h_mean"), **info}
    record(root, line)
    return go or force, line


def _tail_mean(rows: list[dict[str, Any]], key: str, n: int = 100) -> float | None:
    """Mean of ``key`` over the last ``n`` episodes that have it (None if none do)."""
    v = [r[key] for r in rows if key in r][-n:]
    return float(sum(v) / len(v)) if v else None


def main() -> int:
    """CLI entry."""
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--root", type=Path, default=Path("data/rl/m2v7"))
    ap.add_argument("--length", type=int, default=10)
    ap.add_argument("--n-envs", type=int, default=0, help="0: the best n_envs of data/rl/m2v5/benchmark.json")
    ap.add_argument("--scale", type=float, default=1.0, help="multiplies every wall budget (smoke tests)")
    ap.add_argument("--warm-from", choices=("p0", "p1"), default="p1", help="P2 warm start: the hold-channel flat pilot (default) or the abs control")
    ap.add_argument("--hold-weight", type=float, default=3.0, help="P2 hold reward per wrap tick (6 for a re-pilot)")
    ap.add_argument("--eval-workers", type=int, default=4)
    ap.add_argument("--force-go", action="store_true", help="smoke tests only: record the go / no-go values but never stop on them")
    a = ap.parse_args()
    root, L, sc = a.root, a.length, a.scale
    root.mkdir(parents=True, exist_ok=True)
    for m in root.glob("STOP_*"):
        log(f"[stop] {m} exists: remove it to continue")
        return 0
    if (root / "DONE_P2").exists():
        log("[done] DONE_P2 exists")
        return 0
    n_envs = a.n_envs or m2v6.best_n_envs(json.loads(d6.BENCH.read_text()))
    log(f"m2v7 L={L} n_envs={n_envs} root={root} warm_from={a.warm_from} hold_weight={a.hold_weight}")
    base = root / f"L{L}"

    # ---- P0 control (abs)
    p0 = go_line(root, "P0")
    if p0 is None:
        go, line = flat_phase(root, base, "P0", "abs", n_envs, sc, m2v7.P0_CAP, m2v7.P0_CHECK_STEPS, a.force_go)
        if not go:
            try:
                m2v7.write_config_diff(base / "P0" / "config.json", REFERENCE_CFG, root / "P0_config_diff.txt",
                                       [f"recipe env (ours): {json.dumps(RECIPE_ENV)}", f"ours: stage m2_flat4 pilot overrides {json.dumps(FLAT4_PILOT)} ent {FLAT4_PILOT_ENT}"])
            except Exception as e:  # noqa: BLE001
                log(f"[stop] config diff failed: {type(e).__name__}: {e}")
            return stop(root, "P0", f"P0 control (abs) failed: rolling success {line['signals']['success']:.3f} (>= 0.15), median final_dist/dist0 {line['signals']['median_ratio']:.3f} (<= 0.7) at {line['check_steps']} steps")
        p0 = line
    t_p0 = m2v7.time_to_takeoff(m2v6.read_episodes(base / "P0"))

    # ---- P1 hold channel (flat)
    if a.warm_from == "p1" and go_line(root, "P1") is None:
        go, line = flat_phase(root, base, "P1", "hold", n_envs, sc, m2v7.P1_CAP, m2v7.P1_CHECK_STEPS, a.force_go, t_p0)
        if not go:
            return stop(root, "P1_hold_unused", f"P1 (hold channel) failed: success {line['signals']['success']:.3f}, median ratio {line['signals']['median_ratio']:.3f} at {line['check_steps']} steps, "
                                                f"takeoff {line['time_to_takeoff']} vs P0 {t_p0} (needs <= 1.5x)")
    warm = base / ("P1" if a.warm_from == "p1" else "P0")

    # ---- P2 hug pilot
    tag = "P2" if a.hold_weight == 3.0 else f"P2_hw{a.hold_weight:g}"
    out = base / tag
    if go_line(root, "P2", hold_weight=a.hold_weight) is None:
        buf = base / f"hug_buffer_{tag}.json"
        info = d6.train_phase("m2_hug7", out, L, {**HUG7_PILOT, "hold_weight": a.hold_weight}, get_stage("m2_hug7").ent_coef, 200, warm, d6.PILOT_BUDGET_MIN, m2v7.P2_CAP, n_envs, sc,
                              buffer=buf, action_mode="hold")
        sig = m2v7.p2_signals(m2v6.read_episodes(out))
        decision = "go" if a.force_go else m2v7.p2_decision(sig)
        record(root, {"phase": "P2", "go": decision == "go", "decision": decision, "hold_weight": a.hold_weight, "warm_from": a.warm_from, "signals": sig,
                      "buffer_states": len(json.loads(buf.read_text())) if buf.exists() else 0, **info})
        if decision == "approach":
            d6.dump_stop(out, root, tag, a.eval_workers, "m2_hug7")
            return stop(root, "P2_approach", f"P2: only {100 * sig['frac_reach']:.1f} % of the last {sig['n']} episodes reached run >= 3 and span >= 90 deg (< 5 %)")
        if decision == "hold_unused":
            return stop(root, "P2_hold_unused", f"P2: approach ok ({100 * sig['frac_reach']:.1f} %) but mean h over wrap ticks = {sig['h_wrap']} (<= 0.7): the hold channel is not rising; re-pilot with --hold-weight 6")
        if decision == "holdability":
            return stop(root, "P2_holdability", f"P2: h over wrap ticks {sig['h_wrap']:.2f} > 0.7 but only {100 * sig['frac_hold']:.0f} % of reaching episodes keep the state >= 2 s (< 30 %)")
    (root / "DONE_P2").write_text(f"P2 go ({tag}, warm from {a.warm_from}): see summary.jsonl\n")
    log("[m2v7 pilots complete: DONE_P2]")
    return 0


if __name__ == "__main__":
    sys.exit(main())
