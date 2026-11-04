"""Sample per-episode plant parameters from ``DomainRand`` ranges."""

from __future__ import annotations

import numpy as np

from petctl.rl.config import DomainRand
from petctl.sim.model_builder import BuildParams


def _u(rng: np.random.Generator, lo: float, hi: float, scale: float, mid: float | None = None) -> float:
    m = 0.5 * (lo + hi) if mid is None else mid
    return float(rng.uniform(m + scale * (lo - m), m + scale * (hi - m)))


def sample_params(dr: DomainRand, rng: np.random.Generator, base: BuildParams | None = None) -> tuple[BuildParams, dict[str, float]]:
    """Return randomised ``BuildParams`` and the sampled values (the privileged info for an asymmetric critic)."""
    b = base or BuildParams()
    s = dr.scale
    kp = b.kp * _u(rng, *dr.kp_scale, s, 1.0)
    cs = _u(rng, *dr.coulomb_scale, s, 1.0)
    ms = [_u(rng, *dr.mass_scale, s, 1.0) for _ in b.masses]
    cap = _u(rng, *dr.cap_nm, s)
    p = BuildParams(**{**b.__dict__, "kp": kp, "delay_s": _u(rng, *dr.delay_s, s), "armature": _u(rng, *dr.armature, s),
                       "coulomb": tuple(c * cs for c in b.coulomb), "masses": tuple(m * f for m, f in zip(b.masses, ms)),
                       "cap_nm": (cap,) * len(b.cap_nm), "floor_friction": _u(rng, *dr.floor_friction, s)})
    return p, {"kp": kp, "delay_s": p.delay_s, "armature": p.armature, "coulomb_scale": cs, "cap_nm": cap, "floor_friction": p.floor_friction}
