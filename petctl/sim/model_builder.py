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

import mujoco  # type: ignore[import-untyped]
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
    urdf_dir: str | None = None             # PET_description folder from the Fusion export: use its meshes/joints for the lying model
    self_collide: bool = True               # module-vs-module contact (parent/child pairs are excluded by MuJoCo)
    max_hull_verts: int = 200
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


@dataclass
class UrdfGeometry:
    """Geometry read from the Fusion URDF export, in the CAD world frame (m) at q=0."""

    centres: list[np.ndarray]                 # B_0..B_7 hull-volume centroids
    anchors: dict[int, np.ndarray]            # joint k anchor
    axes: dict[int, np.ndarray]               # joint k axis (unit)
    hulls: list[np.ndarray]                   # B_0..B_7 convex-hull vertices (N, 3)


def _read_stl(path: str) -> np.ndarray:
    """Vertices (n_tri*3, 3) of a binary STL, in the file's units."""
    import struct
    b = open(path, "rb").read()
    n = struct.unpack("<I", b[80:84])[0]
    a = np.frombuffer(b[84:84 + 50 * n], dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]))
    return a["v"].reshape(-1, 3).astype(float)


def load_urdf(urdf_dir: str) -> UrdfGeometry:
    """Module shapes from ``meshes/*.stl`` (mm, CAD world frame at q=0) and joints from the validated ``gravity_model``.

    The URDF's own joint frames are not trusted: its joints form a straight chain while the meshes form the real staircase,
    and its axes are 45 deg off the ones validated against the hanging tests. Instead the gravity-model joint axes are rotated
    into the mesh frame (rigid fit of its mass centres to the mesh centroids) and each anchor is placed on its axis line
    nearest the contact between the two neighbouring meshes.
    """
    from pathlib import Path

    from scipy.spatial import ConvexHull, cKDTree  # type: ignore[import-untyped]

    root = Path(urdf_dir)
    names = ["base_link"] + [f"module_{i}_1" for i in range(1, N_MODULES)]
    hulls, centres = [], []
    for n in names:
        v = _read_stl(str(root / "meshes" / f"{n}.stl")) * 1e-3
        h = ConvexHull(v)
        hulls.append(v[h.vertices])
        o = v.mean(axis=0)                                       # volume centroid of the hull (tetrahedra fan from an interior point)
        tet = v[h.simplices] - o
        vol = np.abs(np.einsum("ij,ij->i", tet[:, 0], np.cross(tet[:, 1], tet[:, 2]))) / 6.0
        centres.append((vol[:, None] * (tet.sum(axis=1) / 4.0)).sum(axis=0) / vol.sum() + o)
    gm = np.array([P[k] for k in range(1, N_JOINTS + 1)])
    mc = np.array(centres[:N_JOINTS])
    cg, cm = gm.mean(0), mc.mean(0)
    u, _, vt = np.linalg.svd((gm - cg).T @ (mc - cm))
    rot = vt.T @ np.diag([1, 1, np.sign(np.linalg.det(vt.T @ u.T))]) @ u.T
    axes, anchors = {}, {}
    for k in range(1, N_JOINTS + 1):
        ax_fit = rot @ A[k]                                      # old fitted axis: right line family but 90 deg off about x
        ax_fit /= np.linalg.norm(ax_fit)
        touch = cKDTree(hulls[k]).query(hulls[k - 1])[0] < 4e-3  # patch of B_{k-1} touching B_k (the mating face of the joint)
        patch = np.vstack([hulls[k - 1][touch], hulls[k][cKDTree(hulls[k - 1]).query(hulls[k])[0] < 4e-3]])
        ax = np.linalg.svd(patch - patch.mean(0))[2][-1]         # the joint turns about the mating-face normal (the motor cylinder axis)
        # sign: keep the validated direction convention, i.e. that of the fitted axis rotated 90 deg about x (matches the URDF signs)
        ref = np.array([ax_fit[0], -ax_fit[2], ax_fit[1]])
        ax = ax if ax @ ref >= 0 else -ax
        anchors[k] = patch.mean(0)
        axes[k] = ax
    return UrdfGeometry(centres, anchors, axes, hulls)


def _urdf_frame(g: UrdfGeometry) -> np.ndarray:
    """CAD -> robot rotation for the URDF geometry: x forward (head -> tail reversed so +x = towards the head), z up, modules 0,2,4,6 low."""
    fwd = g.centres[0] - g.centres[-1]
    lat = np.array([1.0, 0.0, 0.0])
    fwd = fwd - (fwd @ lat) * lat
    x = fwd / np.linalg.norm(fwd)
    u = np.cross(x, lat)
    u /= np.linalg.norm(u)                                       # candidate up (+-), in the y-z plane, parallel to the joint axes
    if np.mean([g.centres[k] @ u for k in (0, 2, 4, 6)]) > np.mean([g.centres[k] @ u for k in (1, 3, 5, 7)]):
        u = -u
    y = np.cross(u, x)
    return np.vstack([x, y, u])


def build_mjcf(p: BuildParams) -> tuple[str, float]:
    """Return ``(xml, z0)``: the MJCF and the root height (m) at which the lying robot just touches the floor."""
    geo = load_urdf(p.urdf_dir) if (p.urdf_dir and not p.hanging) else None
    if geo is not None:
        R = _urdf_frame(geo)
        centres = [R @ c for c in geo.centres]
        axes = {k: R @ geo.axes[k] for k in range(1, N_JOINTS + 1)}
        anchors = {k: R @ geo.anchors[k] for k in range(1, N_JOINTS + 1)}
        hulls = [(R @ h.T).T for h in geo.hulls]
        z0 = float(-min(h[:, 2].min() for h in hulls) + 1e-3)
    else:
        R = np.eye(3) if p.hanging else _frame()[0]
        centres = [R @ c for c in module_centres()]
        axes = {k: R @ A[k] for k in range(1, N_JOINTS + 1)}
        anchors = {k: R @ Q[k] for k in range(1, N_JOINTS + 1)}
        hulls = []
        z0 = 0.0 if p.hanging else float(-min(c[2] - BOX_HALF[2] for c in centres) + 1e-3)
    gravity = G * ghat if p.hanging else np.array([0.0, 0.0, -G])
    L: list[str] = [
        f'<mujoco model="pet"><compiler angle="radian"/>'
        f'<option timestep="{p.dt}" gravity="{_vec(gravity)}" integrator="implicitfast" cone="elliptic" impratio="{p.impratio}"/>'
        ]
    L.append('<asset><texture name="grid" type="2d" builtin="checker" rgb1=".25 .3 .35" rgb2=".18 .22 .27" width="256" height="256"/>'
             '<material name="floor" texture="grid" texrepeat="40 40" reflectance="0.05"/></asset>')
    if hulls:
        L.append("<asset>" + "".join(
            f'<mesh name="h{k}" maxhullvert="{p.max_hull_verts}" vertex="{_vec(h.reshape(-1))}"/>' for k, h in enumerate(hulls)) + "</asset>")
    L.append('<worldbody>')
    if not p.hanging:
        L.append(f'<geom name="floor" type="plane" size="20 20 0.1" material="floor" friction="{p.floor_friction} {p.torsional_friction} '
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
        if hulls:
            L.append(f'<geom name="g{k}" type="mesh" mesh="h{k}" mass="0" contype="{3 if p.self_collide else 2}" conaffinity="1" '
                     f'friction="{p.floor_friction} {p.torsional_friction} {p.rolling_friction}" condim="4"/>')
        elif not p.hanging:
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
