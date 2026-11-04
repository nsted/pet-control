"""Free-body (and hanging) MuJoCo model of PET on a ground plane, built from the CAD geometry in ``petctl.gravity_model``.

Bodies B_0..B_7 are the eight modules (B_0 = head, B_7 = tail); joint k (servo k) joins B_{k-1} and B_k. Following the gravity
model, the mass attached to axis k (``P[k]``) belongs to B_{k-1}; the tail's own mass is placed one module pitch beyond ``P[7]``.

Two build modes share one code path:

* ``hanging=True``  - head welded to the world, CAD frame, gravity along the chain (regression anchor against ``gravity_model``).
* ``hanging=False`` - head on a free joint, robot frame (x forward = towards the head, y left, z up) lying on a ground plane in the
  bench posture: modules 0,2,4,6 low, 1,3,5,7 high (the +-0.6 cm zig-zag of the CAD motor centres), gravity -z.

Joints are driven by native MuJoCo ``position`` actuators (``tau = kp*(ctrl - q) - kd*qdot``), which is the MIT law with zero
velocity target; the torque cap is the actuator ``forcerange``. Command delay is applied by ``PetSim``.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field

import mujoco
import numpy as np

from petctl.config import MOTOR_LIMITS
from petctl.gravity_model import A, G, P, Q, ghat
from petctl.sim.mujoco_model import COULOMB_NM, MODULE_MASS_KG, PEAK_TORQUE_NM

N_JOINTS = 7
N_MODULES = 8
MODULE_PITCH_M = 0.058                      # distance between consecutive motor centres along the chain (CAD: 5.8 cm)
BOX_HALF = (0.029, 0.035, 0.0175)           # module collision box half-sizes: along chain, across, thickness (7 cm prism, rough)


@dataclass
class BuildParams:
    """Plant and world parameters for ``build_mjcf`` / ``PetSim``."""

    kp: float = MOTOR_LIMITS.kp_default
    kd: float = MOTOR_LIMITS.kd_default
    armature: float = 3e-4                  # rotor inertia reflected to the joint, kg.m^2 (unidentified: randomise 1e-4..1e-3)
    delay_s: float = 0.08                   # command -> motor latency (unidentified: randomise 0.04..0.14)
    coulomb: tuple[float, ...] = COULOMB_NM
    dt: float = 0.002
    masses: tuple[float, ...] = (MODULE_MASS_KG,) * N_MODULES    # head..tail (kg); to be replaced by weighed values
    cap_nm: tuple[float, ...] = (PEAK_TORQUE_NM,) * N_JOINTS
    floor_friction: float = 1.0             # sliding friction coefficient (carpet: measure)
    torsional_friction: float = 0.005
    rolling_friction: float = 0.0002
    floor_solref: tuple[float, float] = (0.004, 1.0)
    impratio: float = 10.0
    hanging: bool = False
    extra: dict[str, float] = field(default_factory=dict)


def _frame() -> tuple[np.ndarray, np.ndarray]:
    """Rotation ``R`` (CAD world -> robot frame, p_F = R @ p_W) with x forward (towards head), z up, and the bench-posture sign."""
    x = -ghat / np.linalg.norm(ghat)
    e = np.array([1.0, 0.0, 0.0])
    s = e - (e @ x) * x
    s /= np.linalg.norm(s)
    u = np.cross(x, s)
    if np.mean([P[k] @ u for k in (1, 3, 5, 7)]) > np.mean([P[k] @ u for k in (2, 4, 6)]):
        u = -u                                       # so that B_0,B_2,B_4,B_6 (P1,P3,P5,P7) are the LOW modules
    z = u
    y = np.cross(z, x)
    return np.vstack([x, y, z]), np.array([0.0, 0.0, 0.0])


def module_centres() -> list[np.ndarray]:
    """Mass centres of B_0..B_7 in the CAD world frame (B_k holds ``P[k+1]``; the tail is extrapolated)."""
    c = [P[k + 1].copy() for k in range(N_JOINTS)]
    c.append(P[7] + (P[6] - P[5]))                   # tail: one step on, keeping the zig-zag
    return c


def _vec(v: np.ndarray) -> str:
    return " ".join(f"{x:.9g}" for x in v)


def _box_inertia(m: float) -> np.ndarray:
    hx, hy, hz = BOX_HALF
    return m / 3.0 * np.array([hy ** 2 + hz ** 2, hx ** 2 + hz ** 2, hx ** 2 + hy ** 2])


def build_mjcf(p: BuildParams) -> tuple[str, float]:
    """Return ``(xml, z0)``: the MJCF and the root height (m) at which the lying robot just touches the floor."""
    R = np.eye(3) if p.hanging else _frame()[0]
    centres = [R @ c for c in module_centres()]
    axes = {k: R @ A[k] for k in range(1, N_JOINTS + 1)}
    anchors = {k: R @ Q[k] for k in range(1, N_JOINTS + 1)}
    z0 = 0.0 if p.hanging else float(-min(c[2] - BOX_HALF[2] for c in centres) + 1e-3)
    gravity = G * ghat if p.hanging else np.array([0.0, 0.0, -G])
    L: list[str] = [
        f'<mujoco model="pet"><compiler angle="radian"/>'
        f'<option timestep="{p.dt}" gravity="{_vec(gravity)}" integrator="implicitfast" cone="elliptic" impratio="{p.impratio}"/>'
        '<worldbody>']
    if not p.hanging:
        L.append(f'<geom name="floor" type="plane" size="20 20 0.1" friction="{p.floor_friction} {p.torsional_friction} '
                 f'{p.rolling_friction}" condim="4" solref="{p.floor_solref[0]} {p.floor_solref[1]}"/>')
    root_pos = "" if p.hanging else f' pos="0 0 {z0:.6f}"'
    for k in range(N_MODULES):
        joint = ""
        if k == 0:
            joint = "" if p.hanging else '<freejoint name="root"/>'
        else:
            joint = (f'<joint name="j{k}" type="hinge" axis="{_vec(axes[k])}" pos="{_vec(anchors[k])}" '
                     f'armature="{p.armature}" frictionloss="{p.coulomb[k - 1]}" damping="0"/>')
        # Body frames coincide with the root frame at q=0, so geometry/anchors are given in root-frame coordinates.
        pos = root_pos if k == 0 else ""
        L.append(f'<body name="m{k}"{pos}>{joint}')
        m = p.masses[k]
        L.append(f'<inertial pos="{_vec(centres[k])}" mass="{m}" diaginertia="{_vec(_box_inertia(m))}"/>')
        if not p.hanging:
            hx, hy, hz = BOX_HALF
            L.append(f'<geom name="g{k}" type="box" pos="{_vec(centres[k])}" size="{hx} {hy} {hz}" mass="0" '
                     f'friction="{p.floor_friction} {p.torsional_friction} {p.rolling_friction}" condim="4"/>')
        if k == N_MODULES - 1:
            L.append(f'<site name="imu" pos="{_vec(centres[k])}"/>')
        if k == 0:
            L.append(f'<site name="head" pos="{_vec(centres[0])}"/>')
    L.append("</body>" * N_MODULES)
    L.append("</worldbody>")
    L.append("<actuator>" + "".join(
        f'<position name="a{k}" joint="j{k}" kp="{p.kp}" kv="{p.kd}" forcerange="{-p.cap_nm[k - 1]} {p.cap_nm[k - 1]}"/>'
        for k in range(1, N_JOINTS + 1)) + "</actuator>")
    L.append('<sensor><gyro name="gyro" site="imu"/><accelerometer name="accel" site="imu"/><framequat name="quat" objtype="site" objname="imu"/>'
             '<framepos name="head_pos" objtype="site" objname="head"/><framelinvel name="head_vel" objtype="site" objname="head"/></sensor>')
    L.append("</mujoco>")
    return "".join(L), z0


class PetSim:
    """Steps the PET model: delayed position targets, MuJoCo PD actuators with per-joint torque cap, optional free body."""

    def __init__(self, params: BuildParams | None = None) -> None:
        """Build the model; ``reset`` places the robot."""
        self.p = params or BuildParams()
        xml, self.z0 = build_mjcf(self.p)
        self.model = mujoco.MjModel.from_xml_string(xml)
        self.data = mujoco.MjData(self.model)
        self._delay_steps = max(0, int(round(self.p.delay_s / self.p.dt)))
        self._buf: deque[np.ndarray] = deque()
        self.set_cap(np.array(self.p.cap_nm))
        self.reset()

    def set_cap(self, cap_nm: np.ndarray) -> None:
        """Set the per-joint torque cap (N.m), e.g. from the power allocator."""
        self.model.actuator_forcerange[:, 0] = -cap_nm
        self.model.actuator_forcerange[:, 1] = cap_nm

    def reset(self, q0: np.ndarray | None = None) -> None:
        """Reset to the lying (or hanging) pose with joint angles ``q0`` (rad) at rest and an empty command buffer."""
        mujoco.mj_resetData(self.model, self.data)
        if not self.p.hanging:
            self.data.qpos[:7] = [0.0, 0.0, self.z0, 1.0, 0.0, 0.0, 0.0]
        if q0 is not None:
            self.joint_pos()[:] = q0
        self._buf.clear()
        self._last_target = self.joint_pos().copy()
        mujoco.mj_forward(self.model, self.data)

    def joint_pos(self) -> np.ndarray:
        """Joint angles (rad), servo 1..7."""
        a = 7 if not self.p.hanging else 0
        return self.data.qpos[a:a + N_JOINTS]

    def joint_vel(self) -> np.ndarray:
        """True joint velocities (rad/s)."""
        a = 6 if not self.p.hanging else 0
        return self.data.qvel[a:a + N_JOINTS]

    def step(self, target: np.ndarray, seconds: float) -> None:
        """Advance ``seconds`` with the position target ``target`` (rad), delayed by ``delay_s``."""
        for _ in range(max(1, int(round(seconds / self.p.dt)))):
            self._buf.append(np.asarray(target, float).copy())
            if len(self._buf) > self._delay_steps:
                self._last_target = self._buf.popleft()
            self.data.ctrl[:] = self._last_target
            mujoco.mj_step(self.model, self.data)

    def joint_torque(self) -> np.ndarray:
        """Applied actuator torque (N.m) at the last step."""
        return self.data.actuator_force.copy()

    def head_pos(self) -> np.ndarray:
        """Head site position in the world (m) - sim ground truth only."""
        return self.data.sensor("head_pos").data.copy()
