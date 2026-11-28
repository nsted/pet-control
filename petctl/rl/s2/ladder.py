"""S2 robustness ladder (STAGE_2_GATES, Ladder): domain randomisation levels, separate from disturbance magnitudes. Fixed ranges per level; changing them needs the owner's approval.

``friction`` ranges are the intended ones. MuJoCo uses the LARGER of two contacting geoms' friction and the floor is 1.0 until the owner approves the friction fix (T12), so draws below 1.0
are NOT REALISED: ``realised_friction`` reports what actually applies, and the suite manifest marks the levels accordingly.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass

import numpy as np


@dataclass(frozen=True)
class Level:
    """One ladder level. Ranges are (lo, hi) uniform draws; scalars with ``*_max`` are magnitude bounds of the disturbance families."""

    name: str
    friction: tuple[float, float]
    mass: tuple[float, float]
    derate: tuple[float, float]
    delay_ticks: tuple[int, int]
    noise_mult: float
    zeta_m: tuple[float, float]
    start_t_coil: tuple[float, float]
    push_max_n: float
    shove_max_n: float
    load_max_n: float
    tilt_max_deg: float
    head_mass: tuple[float, float]


LEVELS: dict[str, Level] = {
    "L0": Level("L0", (1.0, 1.0), (1.0, 1.0), (1.0, 1.0), (1, 1), 1.0, (1.0, 1.0), (30.0, 30.0), 3.0, 8.0, 3.0, 5.0, (1.5, 1.5)),
    "L1": Level("L1", (0.7, 1.4), (0.9, 1.1), (0.9, 1.0), (1, 2), 1.0, (0.7, 1.4), (25.0, 50.0), 3.0, 8.0, 3.0, 5.0, (1.0, 1.5)),
    "L2": Level("L2", (0.5, 1.8), (0.8, 1.2), (0.8, 1.0), (1, 2), 1.0, (0.7, 1.4), (25.0, 75.0), 5.0, 12.0, 4.0, 10.0, (1.0, 2.0)),
    "L3": Level("L3", (0.4, 2.0), (0.7, 1.3), (0.7, 1.0), (1, 3), 2.0, (0.7, 1.4), (25.0, 85.0), 8.0, 16.0, 6.0, 20.0, (1.0, 3.0)),
}
FLOOR_FRICTION = 1.0            # default floor friction (owner-approved parameter; T12 would change the contact model)
EVAL_ZETA_M = 1.0               # evaluation always uses zeta_m = 1


def level_dict(name: str) -> dict:
    """JSON-serialisable level definition (written into the suite manifest)."""
    return asdict(LEVELS[name])


def realised_friction(f: float) -> float:
    """Effective contact friction while the floor is 1.0 and MuJoCo takes the larger value: draws below the floor's do nothing."""
    return max(float(f), FLOOR_FRICTION)


def sample_plant(level: str, rng: np.random.Generator) -> dict[str, float]:
    """Per-scenario plant draw at ``level`` (stored in the frozen scenario so the evaluator replays it exactly)."""
    lv = LEVELS[level]
    u = rng.uniform
    return {"friction": float(u(*lv.friction)), "mass": float(u(*lv.mass)), "derate": float(u(*lv.derate)), "delay_ticks": int(rng.integers(lv.delay_ticks[0], lv.delay_ticks[1] + 1)),
            "noise_mult": lv.noise_mult, "zeta_m": EVAL_ZETA_M, "start_t_coil": float(u(*lv.start_t_coil)), "head_mass": float(u(*lv.head_mass))}
