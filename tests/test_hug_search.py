"""M1 static hug-feasibility search harness: runs for n=8 on a tiny budget and the record schema is right."""
from __future__ import annotations

import os

import numpy as np
import pytest

pytest.importorskip("mujoco")
from petctl.rl.config import DEFAULT_URDF_DIR  # noqa: E402
from petctl.rl.hug_search import HugSearchSim, bounds, search, seeded_population  # noqa: E402

needs_urdf = pytest.mark.skipif(not os.path.isdir(DEFAULT_URDF_DIR), reason="Fusion URDF export not present")

KEYS = {"n_modules", "H", "held_modules", "h_reached", "feasible", "bus_mean_a", "bus_applied_peak_a", "max_joint_rad", "geometry", "wall_s", "q_rad", "post_xy", "n_evals",
        "robust_held_min"}


@needs_urdf
def test_evaluate_runs_and_bounds() -> None:
    hs = HugSearchSim(8)
    assert len(bounds(8)) == 7 + 3
    r = hs.evaluate(np.concatenate([1.0 * np.ones(7), [0.0, 0.0, 0.7]]), settle_s=0.5, hold_s=0.5)
    assert 0 <= r.held_modules <= 8 and np.isfinite(r.score) and r.bus_mean_a >= 0.0
    pop = seeded_population(8, 24, 0)
    assert pop.shape == (24, 10) and np.abs(pop[:12, :7]).max() <= 1.5


@needs_urdf
def test_search_tiny_budget_schema() -> None:
    rec = search(8, budget=40, restarts=1, workers=1, log=lambda *_: None)
    assert KEYS <= set(rec) and rec["n_modules"] == 8 and rec["H"] == 5
    assert len(rec["q_rad"]) == 7 and len(rec["post_xy"]) == 2
    assert rec["geometry"] in ("planar", "helical", "n/a")
