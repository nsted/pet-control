"""Shape-agnostic target family (Regimen v2, hug v2): training sampler, size curriculum and the frozen held-out gate set.

Family (``target_family`` task knob = 1): welded 0.4 m tall objects with a random yaw: cylinders r 2-12 cm (50 %), boxes 4-16 cm wide/deep with aspect <= 3:1 (35 %),
rounded boxes (same sizes, corner radius 1-3 cm; 15 %). Size curriculum ``shape_phase`` 1: cylinders r 4-8 cm, boxes / rounded boxes 6-10 cm; phase 2: the full ranges.

Held-out gate set (frozen; cyl r 11 cm was replaced by r 9 cm on 2026-11-14 after the feasibility data showed no tested length holds r11 with slack - a gate mis-calibration, see docs/plans/M1C_SHAPE_FEASIBILITY.md): cyl r 3 cm; cyl r 9 cm; square 8x8 cm; rectangle 5x14 cm; rounded 12x12 cm r 2 cm; ``PER_SHAPE`` = 20 episodes each on seeds
``GATE_SEED0`` + 0..99 (seed ``GATE_SEED0 + 20 i + j`` is shape ``i``, yaw drawn from that seed). Training never samples within ``MARGIN_M`` (1 cm) of a held-out size.

The held-out set depends on the body length (``held_out_set(n_modules)``): the L = 10 set above, and for L = 8 (2026-11-14, same feasibility table: L = 8 cannot wrap r 9 / r 11 / 12x12 with >= 1 module of slack,
so those shapes would test the body, not the policy; rule: ``gate_r_max`` = largest r with >= 1 module of slack in the table for the length) cyl r 3 cm, cyl r 7 cm, square 8x8, rectangle 5x14, rounded 8x8 r 2 cm.
Every function that depends on the set takes ``n_modules`` (default 10, the unchanged L = 10 behaviour). ``shape_phase`` may be fractional: 1 < p < 2 draws each shape from the phase-2 (full) ranges with probability p - 1, else phase 1.
"""

from __future__ import annotations

import math

import numpy as np

from petctl.sim.shapes import TargetShape

GATE_SEED0 = 10_000
PER_SHAPE = 20
MARGIN_M = 0.01
P_CYL, P_BOX = 0.50, 0.35                       # the rest (15 %) are rounded boxes
CYL_R = {1: (0.04, 0.08), 2: (0.02, 0.12)}      # radius range (m) per phase
BOX_W = {1: (0.06, 0.10), 2: (0.04, 0.16)}      # full width / depth range (m) per phase
MAX_ASPECT = 3.0
RC_RANGE = (0.01, 0.03)
# hug_curr_02 training distribution (task knob ``shape_mix`` > 0; training only, gate sets unchanged): cylinders 50 %, boxes 20 %, rounded boxes 30 %; cylinder radius 1.5-9 cm for every phase
# (r3 of the L=8 gate lies INSIDE the range now: with the 1 cm exclusion margin the thinnest trained cylinder was r4); exclusion margin 0.5 cm around the held-out sizes.
P_CYL_V2, P_BOX_V2 = 0.50, 0.20
CYL_R_V2 = (0.015, 0.09)
MARGIN_V2_M = 0.005
V2_SAMPLER = {"mix": (P_CYL_V2, P_BOX_V2), "cyl_r": CYL_R_V2, "margin": MARGIN_V2_M}

# (name, kind, full dims (m) or radius, corner radius)
HELD_OUT: tuple[tuple[str, str, tuple[float, ...], float], ...] = (
    ("cyl_r3", "cyl", (0.03,), 0.0),
    ("cyl_r9", "cyl", (0.09,), 0.0),
    ("square_8x8", "box", (0.08, 0.08), 0.0),
    ("rect_5x14", "box", (0.05, 0.14), 0.0),
    ("rounded_12x12_r2", "rbox", (0.12, 0.12), 0.02),
)
HELD_OUT_L10 = HELD_OUT
HELD_OUT_L8: tuple[tuple[str, str, tuple[float, ...], float], ...] = (
    ("cyl_r3", "cyl", (0.03,), 0.0),
    ("cyl_r7", "cyl", (0.07,), 0.0),
    ("square_8x8", "box", (0.08, 0.08), 0.0),
    ("rect_5x14", "box", (0.05, 0.14), 0.0),
    ("rounded_8x8_r2", "rbox", (0.08, 0.08), 0.02),
)
HELD_OUT_NAMES = tuple(h[0] for h in HELD_OUT)


def held_out_set(n_modules: int = 10) -> tuple[tuple[str, str, tuple[float, ...], float], ...]:
    """The frozen held-out gate set of a body length: the L = 8 set for 8 modules, the L = 10 set otherwise."""
    return HELD_OUT_L8 if int(n_modules) == 8 else HELD_OUT_L10


def held_out_names(n_modules: int = 10) -> tuple[str, ...]:
    """Names of the held-out set of a body length (gate seed block ``i`` is shape ``i``)."""
    return tuple(h[0] for h in held_out_set(n_modules))


EXTREMES: tuple[tuple[str, str, tuple[float, ...], float], ...] = (            # feasibility check only (not gate shapes)
    ("cyl_r2", "cyl", (0.02,), 0.0),
    ("cyl_r12", "cyl", (0.12,), 0.0),
    ("box_16x16", "box", (0.16, 0.16), 0.0),
    ("cyl_r6", "cyl", (0.06,), 0.0),                                           # fallback L = 8 gate cylinder if r7 is not held with slack
)


def make_shape(kind: str, dims: tuple[float, ...], rc: float = 0.0, yaw: float = 0.0) -> TargetShape:
    """Shape from full dimensions: cylinder radius, or box width, depth (m)."""
    if kind == "cyl":
        return TargetShape("cyl", float(dims[0]), yaw=yaw)
    return TargetShape(kind, dims[0] / 2, dims[1] / 2, rc, yaw)


def heldout_shape(index: int, seed: int, n_modules: int = 10) -> TargetShape:
    """Held-out shape ``index`` (0-4) of the length's set with the yaw drawn deterministically from ``seed``."""
    _, kind, dims, rc = held_out_set(n_modules)[index]
    return make_shape(kind, dims, rc, float(np.random.default_rng([int(seed), 7]).uniform(0.0, math.pi)))


def heldout_index(seed: int) -> int | None:
    """Held-out shape index of a gate seed (``GATE_SEED0`` .. +99), None outside that range."""
    j = int(seed) - GATE_SEED0
    return j // PER_SHAPE if 0 <= j < PER_SHAPE * len(HELD_OUT) else None


def heldout_group(seed: int, n_modules: int = 10) -> str:
    """Held-out shape name of a gate seed in the length's set (``other`` outside the range)."""
    i = heldout_index(seed)
    return "other" if i is None else held_out_names(n_modules)[i]


def near_heldout(shape: TargetShape, n_modules: int = 10, margin: float = MARGIN_M) -> bool:
    """True if ``shape`` is within ``margin`` (default ``MARGIN_M``) of a held-out shape of the length's set, same kind, in every size dimension (such shapes are never trained on)."""
    for _, kind, dims, rc in held_out_set(n_modules):
        if kind != shape.kind:
            continue
        if kind == "cyl":
            if abs(shape.a - dims[0]) < margin:
                return True
            continue
        mine, theirs = sorted((2 * shape.a, 2 * shape.b)), sorted(dims)
        if all(abs(x - y) < margin for x, y in zip(mine, theirs)) and (kind == "box" or abs(shape.rc - rc) < margin):
            return True
    return False


def sample_training_shape(rng: np.random.Generator, phase: float = 2, n_modules: int = 10, *, mix: tuple[float, float] | None = None,
                          cyl_r: tuple[float, float] | None = None, margin: float = MARGIN_M) -> TargetShape:
    """One training shape of size-curriculum ``phase`` (1 | 2; 1 < phase < 2: the full ranges with probability phase - 1, else phase 1), never within the margins of the length's held-out set.
    Keyword overrides (hug_curr_02, ``V2_SAMPLER``): ``mix`` = (P cylinder, P box) (rest rounded boxes), ``cyl_r`` = cylinder radius range replacing the phase ranges, ``margin`` = exclusion margin."""
    p_cyl, p_box = mix if mix is not None else (P_CYL, P_BOX)
    u = float(rng.random())                                  # the kind is drawn once (held-out rejection only redraws size / yaw, so the 50/35/15 mix is exact)
    if phase <= 1:
        ph = 1
    elif phase >= 2:
        ph = 2
    else:
        ph = 2 if float(rng.random()) < phase - 1.0 else 1  # one draw per shape: the size range
    while True:
        yaw = float(rng.uniform(0.0, math.pi))
        if u < p_cyl:
            shape = TargetShape("cyl", float(rng.uniform(*(cyl_r or CYL_R[ph]))), yaw=yaw)
        else:
            while True:
                w, d = (float(rng.uniform(*BOX_W[ph])) for _ in range(2))
                if max(w, d) / min(w, d) <= MAX_ASPECT:
                    break
            if u < p_cyl + p_box:
                shape = TargetShape("box", w / 2, d / 2, 0.0, yaw)
            else:
                rc = min(float(rng.uniform(*RC_RANGE)), 0.9 * min(w, d) / 2)
                shape = TargetShape("rbox", w / 2, d / 2, rc, yaw)
        if not near_heldout(shape, n_modules, margin):
            return shape
