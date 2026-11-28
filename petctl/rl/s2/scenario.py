"""S2 scenarios (STAGE_2_GATES, Families): a pure, JSON-serialisable spec per scenario; the evaluator replays it exactly. Yield draws are generated here but kept in their own list so they
hash into SHA set A; only the press magnitude and the loaded joints (section B) are added at T14."""

from __future__ import annotations

import hashlib
from dataclasses import asdict, dataclass, field
from typing import Any

import numpy as np

from petctl.rl.s2.cells import LIFTED
from petctl.rl.s2.ladder import LEVELS, sample_plant

SEED_BASE = {"decision": 3_000_000, "test": 8_000_000}
FAMILY_COUNTS = {"push": 4, "shove": 2, "load": 2, "tilt": 2, "derate": 1, "none": 1}          # per cell, non-yield
YIELD_MULTS = (1.2, 1.6, 2.0)                                                                     # per lifted-part cell
BODIES = ("head", "mid", "tail-1")
AXES = ("+x", "-x", "+y", "-y", "+z", "-z")
ONSET_S = (3.0, 7.0)
DUR_S = 3.0
SHOVE_S = 0.1
TRAIN_MIX = {"push": 0.20, "shove": 0.15, "load": 0.15, "tilt": 0.15, "derate": 0.10, "yield": 0.15, "none": 0.10}


@dataclass
class S2Scenario:
    """One hold scenario. ``mag`` is a fraction in (0, 1] of the level's family maximum (push / shove / load N, tilt deg); yield uses ``yield_mult`` and the cell's press (section B)."""

    id: str
    split: str
    level: str
    pose: str
    orient: str
    family: str
    seed: int
    onset_s: float
    duration_s: float
    body: str = "head"
    axis: str = "+y"
    mag: float = 1.0
    tilt_axis_deg: float = 0.0
    load_kind: str = "force"              # load family: 'force' (downward, N) or 'head_mass' (extra head mass x level head_mass)
    derate_x: float = 1.0
    yield_mult: float = 0.0
    plant: dict[str, float] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        """Plain dict (for JSON)."""
        return asdict(self)


def make_scenario(split: str, level: str, pose: str, orient: str, family: str, k: int, rng: np.random.Generator, cell_idx: int, yield_mult: float = 0.0) -> S2Scenario:
    """Draw one scenario of ``family`` (``k`` = index within the cell / family)."""
    seed = SEED_BASE[split] + cell_idx * 1000 + hash_int(family) % 50 * 20 + k
    sid = f"{split}-{level}-{pose.replace(' ', '_')}-{orient}-{family}-{k}"
    onset = float(rng.uniform(*ONSET_S))
    sc = S2Scenario(sid, split, level, pose, orient, family, seed, onset, SHOVE_S if family == "shove" else DUR_S, plant=sample_plant(level, rng))
    if family in ("push", "shove"):
        sc.body, sc.axis, sc.mag = str(rng.choice(BODIES)), str(rng.choice(AXES)), float(rng.uniform(0.5, 1.0))
    elif family == "load":
        sc.load_kind = "head_mass" if k % 2 else "force"
        sc.mag = float(rng.uniform(0.5, 1.0))
        sc.body = "head" if sc.load_kind == "head_mass" else str(rng.choice(("mid", "head")))
        sc.axis = "-z"
    elif family == "tilt":
        sc.tilt_axis_deg, sc.mag = float(rng.uniform(0, 360)), float(rng.uniform(0.5, 1.0))
    elif family == "derate":
        sc.derate_x = float(rng.uniform(LEVELS[level].derate[0], 1.0)) if LEVELS[level].derate[0] < 1.0 else 1.0
    elif family == "yield":
        sc.body, sc.axis, sc.yield_mult = "head", "-z", yield_mult
    return sc


def hash_int(s: str) -> int:
    """Stable small hash of a string (Python's ``hash`` is salted per process)."""
    return int(hashlib.sha256(s.encode()).hexdigest()[:8], 16)


def build_suite(split: str, level: str, cells: list[tuple[str, str]], master_seed: int = 20261128) -> tuple[list[S2Scenario], list[S2Scenario]]:
    """(non-yield scenarios, yield draws) for ``split`` at ``level`` over the valid cells ``[(pose, orient), ...]``. Fully determined by the arguments."""
    non_yield: list[S2Scenario] = []
    yields: list[S2Scenario] = []
    for ci, (pose, orient) in enumerate(cells):
        rng = np.random.default_rng([master_seed, hash_int(split), hash_int(level), ci])
        for fam, n in FAMILY_COUNTS.items():
            for k in range(n):
                non_yield.append(make_scenario(split, level, pose, orient, fam, k, rng, ci))
        if pose in LIFTED:
            for k, mult in enumerate(YIELD_MULTS):
                yields.append(make_scenario(split, level, pose, orient, "yield", k, rng, ci, yield_mult=mult))
    return non_yield, yields
