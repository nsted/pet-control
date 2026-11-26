"""Anisotropic skin friction (petctl/sim/skin.py): slide distances along / across the chain axis give the requested ratio; off = unchanged; stable."""

from __future__ import annotations

import numpy as np
import pytest

from petctl.sim.model_builder import BuildParams, PetSim


def slide(n: int, aniso: float, direction: str, v0: float = 0.3) -> float:
    """Distance (m) the rigid, zero-command body slides from speed ``v0`` along the chain axis ('fwd') or across it ('lat') before stopping."""
    sim = PetSim(BuildParams(n_modules=n, skin_aniso=aniso, dt=0.004, floor_friction=0.5))
    sim.reset()
    sim.step(np.zeros(n - 1), 0.5)                                   # settle
    d = sim.data
    chain = d.xipos[sim.skin.bodies[-1]] - d.xipos[sim.skin.bodies[0]]
    chain[2] = 0.0
    chain /= np.linalg.norm(chain)
    u = chain if direction == "fwd" else np.array([-chain[1], chain[0], 0.0])
    x0 = d.xipos[sim.skin.bodies].mean(axis=0).copy()
    d.qvel[:3] = v0 * u
    d.qvel[3:] = 0.0
    sim.step(np.zeros(n - 1), 3.0)
    assert np.all(np.isfinite(d.qpos))
    return float(np.linalg.norm(d.xipos[sim.skin.bodies].mean(axis=0)[:2] - x0[:2]))


def test_lateral_slide_shortens_by_ratio_and_forward_unchanged() -> None:
    """n = 10 at 0.3 m/s: isotropic slide ~ v^2 / (2 mu g) = 0.009 m; with r = 2.5 the lateral slide is ~1 / 2.5 of it (measured 0.35-0.37 at 0.2-0.4 m/s), forward unchanged."""
    iso_f, iso_l = slide(10, 1.0, "fwd"), slide(10, 1.0, "lat")
    fwd, lat = slide(10, 2.5, "fwd"), slide(10, 2.5, "lat")
    assert iso_f > 0.005 and iso_f == pytest.approx(iso_l, rel=0.15)
    assert fwd == pytest.approx(iso_f, rel=0.05)
    assert 0.25 < lat / iso_l < 0.5, (lat, iso_l)


@pytest.mark.parametrize("n", [11, 12, 13])
def test_stable_and_forward_unchanged_any_length(n: int) -> None:
    """Odd lengths tumble when pushed sideways (not Coulomb sliding), so only forward sliding and finiteness are asserted there."""
    assert slide(n, 2.5, "fwd") == pytest.approx(slide(n, 1.0, "fwd"), rel=0.1)
    assert slide(n, 2.5, "lat") < 0.5


def test_off_is_free_and_leaves_no_force() -> None:
    sim = PetSim(BuildParams(n_modules=10, skin_aniso=1.0, dt=0.004))
    sim.step(np.zeros(9), 0.2)
    assert not sim.data.xfrc_applied.any()
