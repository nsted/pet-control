"""MuJoCo model of the PET chain held by the head, built from the same CAD geometry as ``petctl.gravity_model``.

Seven point-like modules of 0.275 kg sit on their motor axes; joint k (servo k) rotates modules k+1..7 about the CAD axis
through ``Q[k]``. Joints are driven in Python (not by MuJoCo actuators) with the motor's MIT law
``tau = clip(kp*(cmd - q) - kd*qdot, +-cap)``, so a command delay and a time-varying torque cap (the power manager's) can be
applied exactly. Joint friction is MuJoCo ``frictionloss`` (Coulomb); armature stands in for the rotor inertia.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import mujoco  # type: ignore[import-untyped]
import numpy as np

from petctl.config import MOTOR_LIMITS
from petctl.gravity_model import A, G, P, Q, ghat

N_JOINTS = 7
MODULE_MASS_KG = 0.275
MODULE_INERTIA = 1.5e-4          # kg.m^2 about its own centre (~275 g, 6 cm triangular prism); a guess, not measured
PEAK_TORQUE_NM = 0.164           # 2 A peak per motor from the power-manager allocation, used when no logged cap is given
# Coulomb friction per joint (N.m) from the one-run capture ramps (servo 1 not measured: same as the others, ~0.01)
COULOMB_NM = (0.010, 0.013, 0.012, 0.009, 0.006, 0.009, 0.025)


@dataclass
class SimParams:
    """Tunable plant parameters for ``build_model``/``Simulator``."""

    kp: float = MOTOR_LIMITS.kp_default
    kd: float = MOTOR_LIMITS.kd_default
    armature: float = 3e-4                   # rotor inertia reflected to the joint, kg.m^2 (bounded <= ~1e-3, likely 1-5e-4)
    delay_s: float = 0.08                    # command -> motor latency
    coulomb: tuple[float, ...] = COULOMB_NM
    dt: float = 0.001
    module_inertia: float = MODULE_INERTIA
    cap_nm: np.ndarray = field(default_factory=lambda: np.full(N_JOINTS, PEAK_TORQUE_NM))


def _vec(v: np.ndarray) -> str:
    return " ".join(f"{x:.6f}" for x in v)


def mjcf(p: SimParams) -> str:
    """Return the MJCF XML. Body frames coincide with the world at q=0, so geometry is given in world coordinates."""
    lines = [f'<mujoco model="pet"><compiler angle="radian"/><option timestep="{p.dt}" gravity="{_vec(G * ghat)}"/><worldbody>']
    inertia = _vec(np.full(3, p.module_inertia))
    lines.append(f'<body name="m1"><inertial pos="{_vec(P[1])}" mass="{MODULE_MASS_KG}" diaginertia="{inertia}"/>')
    # Module 1 is rigid with the held head. Joint k (k=1..7) carries modules k+1..7 (module 8 = tail end, near-massless).
    for k in range(1, N_JOINTS + 1):
        mass, pos, inert = (MODULE_MASS_KG, P[k + 1], inertia) if k < N_JOINTS else (1e-3, P[7], "1e-7 1e-7 1e-7")
        lines.append(f'<body name="m{k + 1}"><joint name="j{k}" type="hinge" axis="{_vec(A[k])}" pos="{_vec(Q[k])}" '
                     f'armature="{p.armature}" frictionloss="{p.coulomb[k - 1]}" damping="0"/>'
                     f'<inertial pos="{_vec(pos)}" mass="{mass}" diaginertia="{inert}"/>')
    lines.append("</body>" * (N_JOINTS + 1))
    lines.append("</worldbody></mujoco>")
    return "".join(lines)


class Simulator:
    """Steps the model with the MIT PD law, a command delay and a per-joint torque cap."""

    def __init__(self, params: SimParams) -> None:
        """Build the MuJoCo model for ``params``."""
        self.p = params
        self.model = mujoco.MjModel.from_xml_string(mjcf(params))
        self.data = mujoco.MjData(self.model)
        self.last_tau = np.zeros(N_JOINTS)

    def reset(self, q0: np.ndarray) -> None:
        """Start at joint angles ``q0`` (rad) at rest."""
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[:] = q0
        mujoco.mj_forward(self.model, self.data)

    def step_to(self, cmd: np.ndarray, cap: np.ndarray, seconds: float) -> None:
        """Advance ``seconds`` holding ``cmd`` (rad, already delayed) and ``cap`` (N.m, per joint)."""
        for _ in range(max(1, int(round(seconds / self.p.dt)))):
            tau = np.clip(self.p.kp * (cmd - self.data.qpos) - self.p.kd * self.data.qvel, -cap, cap)
            self.data.qfrc_applied[:] = tau
            self.last_tau = tau
            mujoco.mj_step(self.model, self.data)
