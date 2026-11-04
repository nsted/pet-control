"""
Analysis for the hanging-robot test (``scripts/hang_test.py``).

With the robot held by one module and the rest hanging, each joint holds the chain below it against
gravity. Holding one joint at a series of angles (approached from both sides) gives, per joint:

* the **gravity stiffness** ``K`` (N·m/rad): ``tau = K*theta`` for a chain of mass ``m`` whose centre of mass is
  ``r`` along the chain from the joint axis. For a chain hanging straight down and a joint axis at angle ``beta`` to
  the chain, ``K = m*g*r*sin(beta)^2`` (beta = 90 deg is the ordinary horizontal-axis pendulum, ``K = m*g*r``;
  PET's axes are at 45 deg to the chain, so the factor is 0.5);
* the **Coulomb friction** ``c`` (N·m) from the hysteresis between approaching an angle from below and from
  above: ``tau = K*theta + c*d + b0`` where ``d`` is the direction of the last motion;
* and, because the joint is *not moving*, a clean check of the **bus-current model**:
  ``delta_I = a * tau^2``.

Pure numpy; no I/O.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Sequence

import numpy as np

GRAVITY_M_S2 = 9.81


@dataclass(frozen=True)
class HoldPoint:
    """One static hold: joint angle and motor torque (means over the settled part of the hold)."""

    theta_rad: float        # joint angle relative to the hanging start pose
    tau_nm: float           # measured motor torque
    direction: float        # +1 / -1: sign of the last motion in theta that led here
    delta_bus_a: float = 0.0  # bus current above the idle baseline
    abs_vel_rad_s: float = 0.0


@dataclass
class GravityFriction:
    stiffness_nm_per_rad: float
    coulomb_nm: float
    offset_nm: float
    stiffness_se: float
    coulomb_se: float
    resid_rms_nm: float
    n_points: int


def modules_below(servo_id: int, held_module: int, n_modules: int = 8) -> int:
    """Number of modules the joint of ``servo_id`` supports when the robot is held by ``held_module``.

    Servo k joins modules k-1 and k. If the robot is held on the k-1 side or beyond (``held_module < k``)
    the joint supports modules k..n-1; otherwise it supports modules 0..k-1.
    """
    return n_modules - servo_id if held_module < servo_id else servo_id


def axis_factor(axis_angle_deg: float) -> float:
    """``sin(beta)^2``: fraction of ``m*g*r`` that acts as restoring stiffness for an axis at ``beta`` to the chain."""
    return math.sin(math.radians(axis_angle_deg)) ** 2


def predicted_stiffness(
    n_below: int, total_mass_kg: float, pitch_m: float, *, n_modules: int = 8, axis_angle_deg: float = 45.0
) -> float:
    """Predicted gravity stiffness (N·m/rad) assuming uniform modules of equal mass and pitch.

    ``K = g*sin(beta)^2 * m_module * pitch * n^2 / 2`` (centre of mass of n modules is n*pitch/2 below the joint).
    """
    m = total_mass_kg / n_modules
    return GRAVITY_M_S2 * axis_factor(axis_angle_deg) * m * pitch_m * n_below ** 2 / 2.0


def predicted_stiffness_point_masses(
    servo_id: int, held_module: int, module_masses_kg: Sequence[float], pitch_m: float, *, axis_angle_deg: float = 45.0
) -> float:
    """Predicted gravity stiffness (N·m/rad) with each module's mass concentrated at its motor axis.

    Bench information (Nick, 2026-11-03): the bulk of each module's mass is at the middle of its motor, and
    servo k's motor axis is joint k's axis (confirmed in the CAD: each motor's centre of mass is on its axis,
    adjacent motors 5.9 cm apart along the chain). So module j sits ``|j - k| * pitch_m`` from joint k's axis and a
    module's own mass has no lever arm about its own joint. ``K = g*sin(beta)^2 * sum_j m_j * |j - k| * pitch``
    over the modules on the moving side of the joint (modules >= k if the robot is held by a module < k,
    otherwise modules < k). Servo 7 held by the head therefore has K = 0 (only its friction can be measured).
    """
    n = len(module_masses_kg)
    moving = range(servo_id, n) if held_module < servo_id else range(0, servo_id)
    lever = sum(module_masses_kg[j] * abs(j - servo_id) * pitch_m for j in moving)
    return GRAVITY_M_S2 * axis_factor(axis_angle_deg) * lever


def fit_gravity_friction(points: Sequence[HoldPoint], *, with_offset: bool = True) -> Optional[GravityFriction]:
    """Least squares ``tau = K*theta + c*direction (+ b0)``. None if under-determined."""
    if len(points) < (4 if with_offset else 3):
        return None
    th = np.array([p.theta_rad for p in points])
    tau = np.array([p.tau_nm for p in points])
    d = np.array([p.direction for p in points])
    cols = [th, d] + ([np.ones_like(th)] if with_offset else [])
    A = np.column_stack(cols)
    if np.linalg.matrix_rank(A) < A.shape[1]:
        return None
    sol, *_ = np.linalg.lstsq(A, tau, rcond=None)
    resid = tau - A @ sol
    dof = max(1, len(tau) - A.shape[1])
    s2 = float(resid @ resid) / dof
    cov = s2 * np.linalg.inv(A.T @ A)
    se = np.sqrt(np.diag(cov))
    return GravityFriction(
        stiffness_nm_per_rad=float(sol[0]), coulomb_nm=float(sol[1]),
        offset_nm=float(sol[2]) if with_offset else 0.0,
        stiffness_se=float(se[0]), coulomb_se=float(se[1]),
        resid_rms_nm=float(math.sqrt(float(resid @ resid) / len(tau))), n_points=len(tau),
    )


def fit_current_coefficient(points: Sequence[HoldPoint], *, min_tau_nm: float = 0.02) -> Optional[tuple[float, float]]:
    """Fit ``delta_bus = a * tau^2`` through the origin over *static* holds.

    Returns ``(a, relative_standard_error)`` in A/(N·m)^2, or None if there is no usable point. Points
    that are moving fast are excluded (mechanical power would add to the electrical term).
    """
    use = [p for p in points if abs(p.tau_nm) >= min_tau_nm and p.abs_vel_rad_s < 0.05]
    if len(use) < 2:
        return None
    x = np.array([p.tau_nm ** 2 for p in use])
    y = np.array([p.delta_bus_a for p in use])
    a = float(np.dot(x, y) / np.dot(x, x))
    resid = y - a * x
    se = math.sqrt(float(resid @ resid) / max(1, len(use) - 1) / float(np.dot(x, x)))
    return a, (se / abs(a) if a != 0 else math.inf)
