"""Bit-identity guard for the hug env speedups: 5 fixed-seed gate episodes (seeds 10000-10004, DR 0.3, 20 s) replayed with the hug_long_01 policy must reproduce the stored
return, step count, episode metrics and per-term reward sums exactly. A second case replays training-style episodes (start buffer on, 100 % buffer starts from a fixed state file).

Golden file ``tests/golden/hug_perf_identity.json`` was generated from the pre-optimisation code (``python -m tests.test_hug_perf_identity --gen``); regenerate only on an intentional behaviour change.
"""
from __future__ import annotations

import json
import math
import os
import sys
from pathlib import Path

import pytest

pytest.importorskip("mujoco")

from petctl.rl.config import DEFAULT_URDF_DIR  # noqa: E402
from petctl.rl.curriculum.gate2 import PolicyEvaluator  # noqa: E402
from petctl.rl.curriculum.stages import get_stage  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
RUN = ROOT / "data/rl/hug_long/hug_long_01"
GOLDEN = Path(__file__).parent / "golden" / "hug_perf_identity.json"
SEEDS = [10000, 10001, 10002, 10003, 10004]
BUF_STATES = Path(__file__).parent / "golden" / "hug_start_states.json"
needs = pytest.mark.skipif(not os.path.isdir(DEFAULT_URDF_DIR) or not (RUN / "policy.npz").exists() or not GOLDEN.exists(), reason="URDF / policy / golden missing")


def _params(case: str) -> dict[str, float]:
    stage = get_stage("hug_long_01")
    p = dict(stage.gate.task_params)
    p.setdefault("bus_model", 1.0)
    if case == "buffer":
        p.update({"hug_buffer": 1.0, "target_heldout": 0.0, "hug_buffer_p": 1.0, "hug_buffer_min": 1.0, "hug_buffer_min_run": 99.0})
    return p


def run_case(case: str) -> list[dict]:
    """Replay the 5 seeds of ``case`` ('gate' or 'buffer') and return the rows."""
    if case == "buffer":
        os.environ["PETCTL_HUG_BUFFER"] = str(BUF_STATES)
    try:
        return PolicyEvaluator(RUN, "policy.npz", _params(case), 0.3, 20.0)({}, SEEDS)
    finally:
        os.environ.pop("PETCTL_HUG_BUFFER", None)


def _same(a: object, b: object) -> bool:
    if isinstance(a, float) and isinstance(b, float) and math.isnan(a) and math.isnan(b):
        return True
    if isinstance(a, dict) and isinstance(b, dict):
        return a.keys() == b.keys() and all(_same(a[k], b[k]) for k in a)
    if isinstance(a, list) and isinstance(b, list):
        return len(a) == len(b) and all(_same(x, y) for x, y in zip(a, b))
    return a == b


@needs
@pytest.mark.parametrize("case", ["gate", "buffer"])
def test_hug_episodes_bit_identical(case: str) -> None:
    if case == "buffer" and not BUF_STATES.exists():
        pytest.skip("buffer states fixture missing")
    ref = json.loads(GOLDEN.read_text())[case]
    got = json.loads(json.dumps(run_case(case)))
    assert _same(got, ref)


if __name__ == "__main__" and "--gen" in sys.argv:
    out = {c: run_case(c) for c in ("gate", "buffer") if c != "buffer" or BUF_STATES.exists()}
    GOLDEN.write_text(json.dumps(out))
    print("wrote", GOLDEN, {c: len(v) for c, v in out.items()})
