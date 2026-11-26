"""CPG in joint-pair coordinates: exact equivalence with the serpenoid baseline, continuity under parameter jumps, odd/even joint counts, bounds."""

from __future__ import annotations

import math

import numpy as np
import pytest

from petctl.rl.cpg import CPG, PARAM_BOUNDS, PARAM_NAMES, CPGParams, from_serpenoid, joint_targets, n_pairs, pair_waves


def baseline_targets(amp: float, freq: float, phi: float, alt: bool, n_joints: int, t: float) -> np.ndarray:
    """The baseline of scripts/rl/serpenoid_baseline.py."""
    sign = np.array([(-1.0) ** k if alt else 1.0 for k in range(n_joints)])
    return sign * amp * np.sin(2 * np.pi * freq * t - phi * np.arange(1, n_joints + 1))


@pytest.mark.parametrize("n_joints", [7, 9, 10, 11, 12])
@pytest.mark.parametrize("alt", [False, True])
@pytest.mark.parametrize("phi", [-2.1, -1.0, 0.3, 1.0, 2.1, math.pi])
def test_equals_serpenoid_baseline(n_joints: int, alt: bool, phi: float) -> None:
    params, phase0 = from_serpenoid(0.4, 0.7, phi, alt, n_joints)
    cpg = CPG(n_joints, smooth_hz=0.0)
    cpg.reset(params, phase0)
    dt, t = 0.05, 0.0
    for _ in range(100):
        t += dt
        q = cpg.step(dt, params)
        assert np.allclose(q, baseline_targets(0.4, 0.7, phi, alt, n_joints, t), atol=1e-9), (t, q, baseline_targets(0.4, 0.7, phi, alt, n_joints, t))


def test_parameter_count_independent_of_length() -> None:
    assert len(PARAM_NAMES) == 11 == len(CPGParams().to_vector()) == len(PARAM_BOUNDS)
    for n in (7, 9, 10, 11, 12):
        assert joint_targets(CPGParams().to_vector(), 0.3, n).shape == (n,)


def test_odd_joint_count_drives_last_joint_as_odd_member() -> None:
    p = CPGParams(a_yaw=0.5, a_twist=0.3, psi=0.4, b_twist0=0.2).to_vector()
    q_odd, q_even = joint_targets(p, 1.0, 9), joint_targets(p, 1.0, 10)
    assert n_pairs(9) == n_pairs(10) == 5
    assert np.allclose(q_odd, q_even[:9])                       # same pair waves, the 10th joint just does not exist
    yaw, twist = pair_waves(p, 1.0, 9)
    assert q_odd[8] == pytest.approx(0.5 * (yaw[4] - twist[4]))


def test_pure_biases_give_static_pose_and_twist_bias_signs() -> None:
    p = CPGParams(a_yaw=0.0, a_twist=0.0, b_yaw0=0.2, b_twist0=0.6).to_vector()
    q = joint_targets(p, 5.0, 10)
    assert np.allclose(q[0::2], 0.5 * (0.2 - 0.6)) and np.allclose(q[1::2], 0.5 * (0.2 + 0.6))


def _worst_tick_change(dt: float) -> tuple[float, CPG]:
    """Largest per-tick joint-target change while the parameters are stepped from ``start`` to a very different ``final`` at t = 1 s (simulated 5 s)."""
    start = CPGParams(f=0.5, a_yaw=0.3, a_twist=0.1)
    final = CPGParams(f=1.5, dphi=-1.2, psi=2.0, a_yaw=1.2, a_twist=0.9, b_twist0=2.5, b_yaw1=-3.0)
    cpg = CPG(10, smooth_hz=1.0)
    cpg.reset(start)
    prev, worst = cpg.step(dt, start), 0.0
    for k in range(int(5.0 / dt)):
        q = cpg.step(dt, final if k * dt >= 1.0 else start)
        worst = max(worst, float(np.abs(q - prev).max()))
        prev = q
    return worst, cpg


def test_phase_integrates_and_parameter_jumps_never_jump_joint_targets() -> None:
    """Continuity: the per-tick change scales with dt (a discontinuity would not), and the parameters converge to the commanded ones."""
    w_coarse, cpg = _worst_tick_change(0.05)
    w_fine, _ = _worst_tick_change(0.005)
    assert 7.0 < w_coarse / w_fine < 11.0, (w_coarse, w_fine)
    final = CPGParams(f=1.5, dphi=-1.2, psi=2.0, a_yaw=1.2, a_twist=0.9, b_twist0=2.5, b_yaw1=-3.0)
    assert cpg.x == pytest.approx(final.to_vector(), abs=1e-2)
    assert cpg.phase > 2 * math.pi * 0.5 * 5 * 0.9               # integrated (never reset), faster once f rises


def test_exact_filter_is_critically_damped_without_overshoot() -> None:
    cpg = CPG(8, smooth_hz=1.0)
    cpg.reset(CPGParams(a_yaw=0.0))
    xs = []
    for _ in range(100):
        cpg.step(0.01, CPGParams(a_yaw=1.0))
        xs.append(cpg.x[3])
    assert max(xs) <= 1.0 + 1e-9 and np.all(np.diff(xs) >= -1e-12)


@pytest.mark.parametrize("n_modules,alt,phi", [(10, False, 1.0), (11, True, -2.1)])
def test_sim_trajectory_identical_to_serpenoid_baseline(n_modules: int, alt: bool, phi: float) -> None:
    """The CPG and the baseline command identical targets, so the simulated bodies end in identical states (the evaluator inherits the validation)."""
    from petctl.sim.model_builder import BuildParams, PetSim

    nj = n_modules - 1
    params, phase0 = from_serpenoid(0.5, 1.0, phi, alt, nj)
    sims = [PetSim(BuildParams(n_modules=n_modules, cap_nm=(0.5,) * nj, dt=0.004)) for _ in range(2)]
    cpg = CPG(nj, smooth_hz=0.0)
    cpg.reset(params, phase0)
    for i in range(1, 61):
        sims[0].step(baseline_targets(0.5, 1.0, phi, alt, nj, i * 0.05), 0.05)
        sims[1].step(cpg.step(0.05, params), 0.05)
    assert np.allclose(sims[0].data.qpos, sims[1].data.qpos, atol=1e-8)
