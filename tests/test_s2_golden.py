"""STAGE_1_GATES 1.10 (golden): scripted PD policies on frozen S2 scenarios reproduce the stored per-scenario numbers (the evaluator's read-back assertions run inside ``run_scenario``)."""
from __future__ import annotations

import json
from pathlib import Path

import pytest

from petctl.rl.s2 import suite
from petctl.rl.s2.evaluator import pd_policy, run_scenario

GOLD = Path("tests/golden/s2_golden.json")
POLICIES = {"pd_kp1.6_z1": (1.6, 1.0), "pd_kp0.4_z1": (0.4, 1.0), "limp": (1e-4, 1.0)}
pytestmark = pytest.mark.skipif(not GOLD.exists() or not (suite.SUITE_DIR / suite.MANIFEST).exists(), reason="no golden file / frozen suite")


def test_golden_numbers_reproduce() -> None:
    gold = json.loads(GOLD.read_text())
    by_id = {s.id: s for s in suite.load("decision", "L2", "nonyield")}
    assert len(gold) >= 4
    for sid, pols in gold.items():
        for name, want in pols.items():
            kp, z = POLICIES[name]
            r = run_scenario(by_id[sid], pd_policy, fixed_kp=kp, zeta=z)
            for k, v in want.items():
                assert r[k] == pytest.approx(v, rel=1e-6, abs=1e-9), (sid, name, k)


def test_policies_differ_across_scenarios_and_configs() -> None:
    """The earlier evaluation ran the same gait for every command: stiffer vs limp must score differently, and different families must not all give identical numbers."""
    gold = json.loads(GOLD.read_text())
    es = {sid: {n: p["E"] for n, p in pols.items()} for sid, pols in gold.items()}
    assert all(e["limp"] > e["pd_kp1.6_z1"] for e in es.values())
    assert len({round(e["pd_kp0.4_z1"], 6) for e in es.values()}) > 1
