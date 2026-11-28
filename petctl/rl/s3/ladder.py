"""S3 robustness ladder (STAGE_3_GATES_PROPOSAL, Ladder; DEV until the owner approves). Domain randomisation is separate from the disturbance magnitudes; L0 = nominal randomisation with the L1 disturbances.
DR ranges are WIDENED until the real-motor data (T13) arrives. Friction below the floor's 1.0 is NOT REALISED until the owner approves T12."""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass

import numpy as np


@dataclass(frozen=True)
class Level:
    """One S3 ladder level (ranges are uniform draws)."""

    name: str
    friction: tuple[float, float]
    mass: tuple[float, float]
    derate: tuple[float, float]
    delay_ticks: tuple[int, int]
    noise_mult: float
    zeta_m: tuple[float, float]
    start_t_coil: tuple[float, float]
    joint_friction_nm: tuple[float, float]
    armature_x: tuple[float, float]
    slope_deg: float
    ridge_level: float            # 0 flat; > 0 = terrain_level of the ridge generator (small ridges 0.4, ridges 0.8)
    push_max_n: float
    head_mass_x: tuple[float, float]


LEVELS: dict[str, Level] = {
    "L0": Level("L0", (1.0, 1.0), (1.0, 1.0), (1.0, 1.0), (1, 1), 1.0, (1.0, 1.0), (30.0, 30.0), (0.0, 0.0), (1.0, 1.0), 5.0, 0.0, 3.0, (1.0, 1.5)),     # nominal DR, L1 disturbances
    "L1": Level("L1", (0.7, 1.4), (0.9, 1.1), (0.9, 1.0), (1, 2), 1.0, (0.7, 1.4), (25.0, 50.0), (0.0, 0.02), (0.7, 1.4), 5.0, 0.0, 3.0, (1.0, 1.5)),
    "L2": Level("L2", (0.5, 1.8), (0.8, 1.2), (0.8, 1.0), (1, 3), 1.0, (0.7, 1.4), (25.0, 75.0), (0.0, 0.04), (0.5, 2.0), 10.0, 0.4, 5.0, (1.0, 2.0)),
    "L3": Level("L3", (0.4, 2.0), (0.7, 1.3), (0.7, 1.0), (1, 3), 2.0, (0.7, 1.4), (25.0, 85.0), (0.0, 0.05), (0.5, 2.0), 15.0, 0.8, 8.0, (1.0, 3.0)),
}
EVAL_ZETA_M = 1.0


def level_dict(name: str) -> dict:
    """JSON-serialisable level definition."""
    return asdict(LEVELS[name])


def sample(level: str, rng: np.random.Generator) -> dict[str, float]:
    """One scenario's plant + disturbance draw at ``level`` (stored in the scenario so the evaluator replays it exactly)."""
    lv, u = LEVELS[level], rng.uniform
    return {"friction": float(u(*lv.friction)), "mass": float(u(*lv.mass)), "derate": float(u(*lv.derate)), "delay_ticks": int(rng.integers(lv.delay_ticks[0], lv.delay_ticks[1] + 1)),
            "noise_mult": lv.noise_mult, "zeta_m": EVAL_ZETA_M, "start_t_coil": float(u(*lv.start_t_coil)), "joint_friction_nm": float(u(*lv.joint_friction_nm)), "armature_x": float(u(*lv.armature_x)),
            "slope_deg": float(u(0.0, lv.slope_deg)), "slope_az_rad": float(u(0.0, 2 * math.pi)), "ridge_level": lv.ridge_level, "push_n": float(u(0.5, 1.0) * lv.push_max_n),
            "head_mass_x": float(u(*lv.head_mass_x))}
