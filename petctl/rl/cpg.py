"""Central pattern generator in joint-PAIR coordinates (docs/plans/AUTONOMOUS_TRAINER.md, Phase 3). Independent of body length.

Joints are numbered 1..J (J = n_modules - 1). Neighbouring joints form pairs m = 1..M, M = ceil(J / 2): pair m = (o = 2m - 1, e = 2m). With

    yaw_m   = q_e + q_o         twist_m = q_e - q_o          (so q_e = (yaw + twist) / 2, q_o = (yaw - twist) / 2)

each pair is driven by a travelling wave of the integrated phase ``Phi`` (``dPhi/dt = 2 pi f``):

    yaw_m   = A_yaw(m)   sin(Phi - (m - 1) dphi)        + B_yaw(m)
    twist_m = A_twist(m) sin(Phi - (m - 1) dphi + psi)  + B_twist(m)

``dphi`` is the phase lag per pair (the travelling wave), ``psi`` the yaw-twist phase offset, the amplitudes follow a linear envelope along the body
``A(m) = A (1 + taper (x_m - 1/2))`` and the static biases a linear profile ``B(m) = b0 + b1 (x_m - 1/2)`` with ``x_m = (m - 1) / (M - 1)`` in [0, 1] (the twist bias selects
the bending plane, the yaw bias a steady curvature = turning; they are unbounded, so multi-turn behaviour lives here, not in raw joint targets).

Odd J: the last pair has only its odd member ``o``; it is driven as the ``o`` member of a virtual pair M, i.e. q_J = (yaw_M - twist_M) / 2 with the same parameters
(no extra parameter; the missing even joint simply does not exist).

Parameters (11, independent of length): ``f dphi psi a_yaw a_twist taper_yaw taper_twist b_yaw0 b_yaw1 b_twist0 b_twist1``. The phase is integrated and every parameter,
including ``f``, follows critically damped 2nd-order dynamics towards its commanded value (``smooth_hz``), so changing a parameter can never jump a joint target.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, fields

import numpy as np

PARAM_NAMES = ("f", "dphi", "psi", "a_yaw", "a_twist", "taper_yaw", "taper_twist", "b_yaw0", "b_yaw1", "b_twist0", "b_twist1")
# Search / policy bounds (sizes of a search box, not limits of the body: biases are multi-turn capable, see module docstring).
PARAM_BOUNDS = {"f": (0.0, 2.0), "dphi": (-math.pi, math.pi), "psi": (-math.pi, math.pi), "a_yaw": (0.0, 1.5), "a_twist": (0.0, 1.5), "taper_yaw": (-1.5, 1.5),
                "taper_twist": (-1.5, 1.5), "b_yaw0": (-2.0, 2.0), "b_yaw1": (-4.0, 4.0), "b_twist0": (-3.2, 3.2), "b_twist1": (-6.4, 6.4)}


@dataclass
class CPGParams:
    """Commanded CPG parameters (units: Hz, rad)."""

    f: float = 0.5
    dphi: float = 0.9
    psi: float = math.pi / 2
    a_yaw: float = 0.4
    a_twist: float = 0.4
    taper_yaw: float = 0.0
    taper_twist: float = 0.0
    b_yaw0: float = 0.0
    b_yaw1: float = 0.0
    b_twist0: float = 0.0
    b_twist1: float = 0.0

    def to_vector(self) -> np.ndarray:
        """Parameters as an array in ``PARAM_NAMES`` order."""
        return np.array([getattr(self, k) for k in PARAM_NAMES], float)

    @classmethod
    def from_vector(cls, v: np.ndarray) -> CPGParams:
        """Inverse of ``to_vector``."""
        return cls(**{k: float(x) for k, x in zip(PARAM_NAMES, v)})

    def to_dict(self) -> dict[str, float]:
        """Plain dict (JSON)."""
        return {f.name: float(getattr(self, f.name)) for f in fields(self)}


def n_pairs(n_joints: int) -> int:
    """Number of (possibly half) pairs for ``n_joints`` joints."""
    return (n_joints + 1) // 2


def pair_waves(p: np.ndarray, phase: float, n_joints: int) -> tuple[np.ndarray, np.ndarray]:
    """(yaw, twist) of every pair m = 1..M for parameter vector ``p`` (``PARAM_NAMES`` order) at integrated phase ``phase``."""
    f, dphi, psi, ay, at, tay, tat, by0, by1, bt0, bt1 = p
    big_m = n_pairs(n_joints)
    m = np.arange(big_m)
    x = (m / (big_m - 1) - 0.5) if big_m > 1 else np.zeros(1)
    ph = phase - m * dphi
    yaw = ay * (1.0 + tay * x) * np.sin(ph) + by0 + by1 * x
    twist = at * (1.0 + tat * x) * np.sin(ph + psi) + bt0 + bt1 * x
    return yaw, twist


def joint_targets(p: np.ndarray, phase: float, n_joints: int) -> np.ndarray:
    """Joint targets (rad, length ``n_joints``, joint 1 first) for parameters ``p`` at ``phase``."""
    yaw, twist = pair_waves(p, phase, n_joints)
    q = np.empty(2 * len(yaw))
    q[0::2] = 0.5 * (yaw - twist)          # odd joints 1, 3, 5, ... (o members)
    q[1::2] = 0.5 * (yaw + twist)          # even joints 2, 4, 6, ... (e members)
    return q[:n_joints]


class CPG:
    """Stateful generator: integrated phase plus 2nd-order-smoothed parameters. ``step`` returns the joint targets for the next control tick."""

    def __init__(self, n_joints: int, smooth_hz: float = 1.0) -> None:
        """``smooth_hz``: natural frequency of the critically damped parameter filter (settling to ~2 % in about 0.65 s at 1 Hz; 0 disables the filter)."""
        self.n_joints = int(n_joints)
        self.wn = 2.0 * math.pi * float(smooth_hz)
        self.reset(CPGParams())

    def reset(self, params: CPGParams | np.ndarray, phase: float = 0.0) -> None:
        """Start with the parameters already at ``params`` (no transient) and the given phase."""
        self.x = np.asarray(params.to_vector() if isinstance(params, CPGParams) else params, float).copy()
        self.v = np.zeros_like(self.x)
        self.phase = float(phase)

    def step(self, dt: float, target: CPGParams | np.ndarray) -> np.ndarray:
        """Advance ``dt`` seconds towards the commanded ``target`` parameters; returns the joint targets (rad)."""
        tgt = np.asarray(target.to_vector() if isinstance(target, CPGParams) else target, float)
        f_old = self.x[0]
        if self.wn > 0.0:
            e0 = self.x - tgt
            c = self.v + self.wn * e0
            ex = math.exp(-self.wn * dt)
            self.x = tgt + (e0 + c * dt) * ex                       # exact critically damped step (stable for any dt)
            self.v = (self.v - self.wn * c * dt) * ex
        else:
            self.x = tgt.copy()
        self.phase += 2.0 * math.pi * 0.5 * (f_old + self.x[0]) * dt
        return joint_targets(self.x, self.phase, self.n_joints)


def from_serpenoid(amp: float, freq: float, phi: float, alt: bool, n_joints: int) -> tuple[CPGParams, float]:
    """Exact CPG equivalent of the baseline ``q_k = s_k A sin(2 pi f t - k phi)`` (``scripts/rl/serpenoid_baseline.py``; ``s_k = (-1)^(k-1)`` if ``alt`` else 1).

    Returns ``(params, phase0)``: with the CPG phase started at ``phase0`` the targets equal the baseline's at every t (pair phasors: yaw = c_e + c_o, twist = c_e - c_o with
    c_k = s_k A exp(-i k phi); pair m is the pair-1 phasor times exp(-i 2 (m - 1) phi), so ``dphi = 2 phi``).
    """
    s = (lambda k: (-1.0) ** (k - 1)) if alt else (lambda k: 1.0)
    c_o, c_e = s(1) * amp * np.exp(-1j * phi), s(2) * amp * np.exp(-2j * phi)
    yaw, twist = c_e + c_o, c_e - c_o
    th_y = float(np.angle(yaw)) if abs(yaw) > 1e-12 else 0.0
    th_t = float(np.angle(twist)) if abs(twist) > 1e-12 else th_y
    p = CPGParams(f=freq, dphi=2.0 * phi, psi=th_t - th_y, a_yaw=float(abs(yaw)), a_twist=float(abs(twist)))
    return p, th_y
