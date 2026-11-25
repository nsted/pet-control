"""Single-stage driver for B3 terrain_rolling / C0 sparse_field / C2 dense_field / C1 barrier_touch (Regimen v2 plan sec 4.2, revised 2026-11-23); ``--stage`` picks it. Reuses the chain_ab machinery: one training job, resumable from ``<root>/summary.jsonl``.

Stage ``barrier_touch`` (``petctl.rl.curriculum.stages``; gate ``chain_ab.gate_spec_c1``): warm start from the B2 terrain_full run, up to two attempts (the second an identical continuation),
then the n = 100 gate: success >= 0.60 (Wilson lo), touch_off drop CI lo >= 0.20, blind baseline (A1 straight walker) <= 0.30, median contact_frac >= 0.02, bus budget, glitch 0.
Pass -> skill models/skills/nav_chain/barrier_touch + DONE. Fail twice -> STOP_barrier_touch.

    PETCTL_PROGRESS_EXPORT_S=900 nohup python scripts/rl/chain_c_driver.py > data/rl/chain_c/driver.log 2>&1 &
    python scripts/rl/chain_c_driver.py --root data/rl/chain_c_smoke --scale 0.01 --gate-n 4 --force-go
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from pathlib import Path
from typing import Any

sys.path.insert(0, str(Path(__file__).resolve().parent))

import chain_ab_driver as ab  # noqa: E402
import m2v6_driver as d6  # noqa: E402
from m2_length_sweep import PoolEvaluator  # noqa: E402

from petctl.rl.curriculum import chain_ab as ca  # noqa: E402
from petctl.rl.curriculum.gate2 import run_gate2  # noqa: E402
from petctl.rl.curriculum.stages import get_stage  # noqa: E402

STAGE = ca.C1_STAGE                                   # set from --stage in main()
WARM = Path("data/rl/chain_ab_soft/terrain_full/a1")   # first attempt warm start (--warm overrides; required after B3)
BLIND = Path("data/rl/chain_ab/flat_nav/a1")
SCRATCH = False                                       # --scratch: attempt 1 from a random init
log, lines, record = d6.log, d6.lines, d6.record


def next_action(ls: list[dict[str, Any]]) -> dict[str, Any]:
    """done / stop / train / gate for the single stage with two attempts."""
    gates = {int(d["attempt"]): d for d in ls if d.get("phase") == "gate"}
    trained = {int(d["attempt"]) for d in ls if d.get("phase") == "train"}
    if any(g["passed"] for g in gates.values()):
        return {"action": "done"}
    for att in (1, 2):
        if att in gates:
            continue
        return {"action": "gate" if att in trained else "train", "stage": STAGE, "attempt": att}
    return {"action": "stop", "stage": STAGE}


def gate_run(root: Path, attempt: int, gate_n: int, workers: int) -> dict[str, Any]:
    """The C1 gate of a trained run; written as ``gate2_barrier_touch.json`` in the run dir."""
    out = ab.run_dir(root, STAGE, attempt)
    stage = get_stage(STAGE)
    spec = ca.gate_spec_stage(STAGE, gate_n, blind_run=BLIND)
    params = {**stage.gate.task_params, "bus_model": 1.0}
    res = run_gate2(out, stage, spec, evaluator=PoolEvaluator(out, "policy.npz", params, spec.dr_scale, stage.gate.episode_s, workers), write=False)
    (out / f"gate2_{STAGE}.json").write_text(json.dumps(res, indent=1, default=lambda o: None if o is None else str(o)))
    return res


def main() -> int:
    global STAGE, WARM, SCRATCH
    """CLI entry."""
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--stage", default=ca.C1_STAGE, choices=sorted(ca.STAGE_GATES))
    ap.add_argument("--warm", type=Path, default=None, help="run dir to warm-start attempt 1 from (default: the B2 terrain_full run)")
    ap.add_argument("--scratch", action="store_true", help="attempt 1 trains from a random init (no warm start); attempt 2 still continues attempt 1")
    ap.add_argument("--root", type=Path, default=None, help="default data/rl/chain_c_<stage>")
    ap.add_argument("--n-envs", type=int, default=10)
    ap.add_argument("--scale", type=float, default=1.0)
    ap.add_argument("--gate-n", type=int, default=100)
    ap.add_argument("--eval-workers", type=int, default=4)
    ap.add_argument("--force-go", action="store_true")
    a = ap.parse_args()
    STAGE = a.stage
    WARM = a.warm or WARM
    SCRATCH = a.scratch
    root = a.root or Path(f"data/rl/chain_c_{STAGE}")
    root.mkdir(parents=True, exist_ok=True)
    if list(root.glob("STOP_*")):
        log("[stop] marker exists")
        return 0
    if (root / "DONE").exists():
        log("[done]")
        return 0
    os.environ.setdefault("PETCTL_PROGRESS_EXPORT_S", "900")
    while True:
        ls = lines(root)
        nxt = next_action(ls)
        log(f"[next] {json.dumps(nxt)}")
        if nxt["action"] == "done":
            break
        att = nxt.get("attempt", 1)
        if nxt["action"] == "stop":
            fails = [d for d in ls if d.get("phase") == "gate"]
            record(root, {"phase": "stop", "stage": STAGE, "go": False, "why": f"{STAGE} failed its gate twice", "fail_reasons": [d["fail_reasons"] for d in fails]})
            return d6.stop(root, STAGE, f"{STAGE} failed its n=100 gate on both attempts")
        if nxt["action"] == "train":
            init = (None if SCRATCH else WARM) if att == 1 else ab.run_dir(root, STAGE, 1)
            log(f"train {STAGE} a{att} " + (f"warm from {init}" if init is not None else "from scratch"))
            info = ab.train_run(root, STAGE, att, init, a.n_envs, a.scale, seed=7000 + att)
            record(root, {"phase": "train", "stage": STAGE, "attempt": att, "go": True, **info})
            continue
        res = gate_run(root, att, a.gate_n, a.eval_workers)
        line = ab.gate_line(STAGE, att, res, a.force_go)
        line["blind"] = res.get("blind")
        record(root, line)
        if line["passed"] and not a.force_go:
            sk = ab.bank_skill(STAGE, ab.run_dir(root, STAGE, att), line)
            record(root, {"phase": "skill", "stage": STAGE, "attempt": att, "skill": sk})
    (root / "DONE").write_text(f"{STAGE} gate passed\n")
    log("[chain_c complete: DONE]")
    return 0


if __name__ == "__main__":
    sys.exit(main())
