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
from functools import lru_cache

import mujoco  # type: ignore[import-untyped]
import numpy as np

from petctl.config import MOTOR_LIMITS
from petctl.gravity_model import A, G, P, Q, ghat
from petctl.sim.mujoco_model import COULOMB_NM, MODULE_MASS_KG, PEAK_TORQUE_NM
from petctl.sim.terrain import BASE_M, CELL_M, ELEV_MAX_M, HALF_M, Terrain, tile_slices

N_JOINTS = 7                                # the CAD robot (8 modules); ``BuildParams.n_modules`` lengthens it in simulation
N_MODULES = 8
MAX_MODULES = 15
MODULE_PITCH_M = 0.058                      # distance between consecutive motor centres along the chain (CAD: 5.8 cm)
BOX_HALF = (0.029, 0.035, 0.0175)           # module collision box half-sizes: along chain, across, thickness (7 cm prism, rough)
POST_RADIUS_M = 0.05                        # the target post: a heavy immovable cylinder (a static world geom), 10 cm diameter
POST_HEIGHT_M = 0.4
FLOOR_BELOW_TERRAIN_M = 0.002               # with a heightfield the safety plane sits just under z = 0 so the terrain tiles carry all the contacts


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
    n_modules: int = N_MODULES              # 8 (the real robot) .. 15: longer bodies repeat the middle-module pair of the CAD chain
    floor_friction: float = 1.0             # sliding friction coefficient (carpet: measure)
    torsional_friction: float = 0.005
    rolling_friction: float = 0.0002
    floor_solref: tuple[float, float] = (0.004, 1.0)
    impratio: float = 10.0
    hanging: bool = False
    urdf_dir: str | None = None             # PET_description folder from the Fusion export: use its meshes/joints for the lying model
    self_collide: bool = True               # module-vs-module contact (parent/child pairs are excluded by MuJoCo)
    max_hull_verts: int = 200
    obstacles: tuple[tuple[str, float, float, float, float, float], ...] = ()    # scattered static obstacles: (kind 'box'|'cyl'|'ell', x, y, yaw, a, b); 12 cm tall. box: half extents a,b; cyl: radius a; ell: ellipsoid half axes a,b
    walls: tuple[tuple[float, float, float, float], ...] = ()    # static vertical walls: (centre x, centre y, yaw rad, half length m); 2 cm thick, 16 cm tall
    terrain: Terrain | None = None          # per-episode heightfield + friction tiles (None = flat plane, byte-identical to before)
    post: tuple[float, float] | None = None  # world xy of the target post (static cylinder POST_RADIUS_M x POST_HEIGHT_M standing on the terrain)
    extra: dict[str, float] = field(default_factory=dict)

    def __post_init__(self) -> None:
        """Validate ``n_modules`` and stretch the per-module / per-joint tuples left at their 8-module defaults to the new length."""
        n = int(self.n_modules)
        if not N_MODULES <= n <= MAX_MODULES:
            raise ValueError(f"n_modules must be in {N_MODULES}..{MAX_MODULES}, got {n}")
        if len(self.masses) != n:
            if len(set(self.masses)) != 1:
                raise ValueError(f"masses has {len(self.masses)} entries for {n} modules")
            self.masses = (self.masses[0],) * n
        if len(self.cap_nm) != n - 1:
            if len(set(self.cap_nm)) != 1:
                raise ValueError(f"cap_nm has {len(self.cap_nm)} entries for {n - 1} joints")
            self.cap_nm = (self.cap_nm[0],) * (n - 1)
        if len(self.coulomb) != n - 1:
            if len(self.coulomb) != N_JOINTS:
                raise ValueError(f"coulomb has {len(self.coulomb)} entries for {n - 1} joints")
            mid = float(np.median(self.coulomb[:N_JOINTS - 1]))          # new middle joints: median of the measured middle joints; the tail joint keeps its own
            self.coulomb = tuple(self.coulomb[:N_JOINTS - 1]) + (mid,) * (n - N_MODULES) + (self.coulomb[N_JOINTS - 1],)


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


@lru_cache(maxsize=4)
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


def _rot_y(a: float) -> np.ndarray:
    """Rotation by ``a`` rad about +y."""
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def _extend_chain(centres: list[np.ndarray], axes: dict[int, np.ndarray], anchors: dict[int, np.ndarray], hulls: list[np.ndarray], n: int
                  ) -> tuple[list[np.ndarray], dict[int, np.ndarray], dict[int, np.ndarray], list[np.ndarray]]:
    """Lengthen the 8-module chain (robot frame) to ``n`` modules by repeating the middle-module pair.

    The CAD chain is periodic with period two (the +-45 deg joint axes and the +-0.6 cm zig-zag alternate), so a module ``j >= 7``
    is a copy of module 5 or 6 (same parity) translated by the measured pair shift, and likewise for the joints. The whole chain is first
    rotated about y so that this pair shift is horizontal (the 8-module frame leaves the lying chain tilted by ~5 deg). The tail is the
    original tail translated to the new end; when ``n`` is odd the parity at the end flips, so the tail is turned 180 deg about the chain
    axis (the same operation that relates module 5 to module 6) before translating. Head and tail hulls are otherwise unchanged.
    """
    pair = np.mean([centres[k + 2] - centres[k] for k in range(1, 5)], axis=0)
    lvl = _rot_y(float(np.arctan(pair[2] / pair[0])))
    centres = [lvl @ c for c in centres]
    axes = {k: lvl @ v for k, v in axes.items()}
    anchors = {k: lvl @ v for k, v in anchors.items()}
    hulls = [(lvl @ h.T).T for h in hulls]
    pair = lvl @ pair
    pair = np.array([pair[0], 0.0, 0.0])
    t_flip = centres[6][0] - centres[5][0]                       # odd -> even parity operator: x += t, y = -y, z = zc - z
    zc = centres[5][2] + centres[6][2]

    def flip(v: np.ndarray) -> np.ndarray:
        return np.array([v[0] + t_flip, -v[1], zc - v[2]])

    def src_of(j: int) -> int:
        return 5 if j % 2 else 6

    c2, h2 = list(centres[:7]), list(hulls[:7]) if hulls else []
    for j in range(7, n - 1):                                   # extra middle modules
        k = src_of(j)
        sh = (j - k) // 2 * pair
        c2.append(centres[k] + sh)
        if hulls:
            h2.append(hulls[k] + sh)
    if n % 2 == 0:
        sh = (n - 8) // 2 * pair
        c2.append(centres[7] + sh)
        if hulls:
            h2.append(hulls[7] + sh)
    else:
        sh = (n - 9) // 2 * pair
        c2.append(flip(centres[7]) + sh)
        if hulls:
            h2.append(np.array([flip(v) for v in hulls[7]]) + sh)
    a2, q2 = dict((k, axes[k]) for k in range(1, 7)), dict((k, anchors[k]) for k in range(1, 7))
    for k in range(7, n):
        if k == n - 1 and n % 2 == 0:
            a2[k], q2[k] = axes[7], anchors[7] + (n - 8) // 2 * pair
        else:
            src = src_of(k)
            a2[k], q2[k] = axes[src], anchors[src] + (k - src) // 2 * pair
    return c2, a2, q2, h2


def build_mjcf(p: BuildParams) -> tuple[str, float]:
    """Return ``(xml, z0)``: the MJCF and the root height (m) at which the lying robot just touches the floor."""
    nm = int(p.n_modules)
    if nm != N_MODULES and p.hanging:
        raise ValueError("the hanging regression model only exists for the 8-module CAD chain")
    geo = load_urdf(p.urdf_dir) if (p.urdf_dir and not p.hanging) else None
    if geo is not None:
        R = _urdf_frame(geo)
        centres = [R @ c for c in geo.centres]
        axes = {k: R @ geo.axes[k] for k in range(1, N_JOINTS + 1)}
        anchors = {k: R @ geo.anchors[k] for k in range(1, N_JOINTS + 1)}
        hulls = [(R @ h.T).T for h in geo.hulls]
        if nm != N_MODULES:
            centres, axes, anchors, hulls = _extend_chain(centres, axes, anchors, hulls, nm)
        z0 = float(-min(h[:, 2].min() for h in hulls) + 1e-3)
    else:
        R = np.eye(3) if p.hanging else _frame()[0]
        centres = [R @ c for c in module_centres()]
        axes = {k: R @ A[k] for k in range(1, N_JOINTS + 1)}
        anchors = {k: R @ Q[k] for k in range(1, N_JOINTS + 1)}
        hulls = []
        if nm != N_MODULES:
            centres, axes, anchors, hulls = _extend_chain(centres, axes, anchors, [], nm)
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
    ter = p.terrain if not p.hanging else None
    if ter is not None:
        L.append("<asset>" + "".join(
            f'<hfield name="t{ty}{tx}" nrow="{yhi - ylo + 1}" ncol="{xhi - xlo + 1}" size="{(xhi - xlo) * CELL_M / 2} {(yhi - ylo) * CELL_M / 2} {ELEV_MAX_M} {BASE_M}"/>'
            for ty, (ylo, yhi) in enumerate(tile_slices()) for tx, (xlo, xhi) in enumerate(tile_slices())) + "</asset>")
    L.append('<worldbody>')
    if not p.hanging:
        fz = f' pos="0 0 {-FLOOR_BELOW_TERRAIN_M}"' if ter is not None else ""
        L.append(f'<geom name="floor" type="plane"{fz} size="20 20 0.1" material="floor" friction="{p.floor_friction} {p.torsional_friction} '
                 f'{p.rolling_friction}" condim="4" solref="{p.floor_solref[0]} {p.floor_solref[1]}"/>')
    if ter is not None:
        tmu = ter.tile_friction(p.floor_friction)
        for ty, (ylo, yhi) in enumerate(tile_slices()):
            for tx, (xlo, xhi) in enumerate(tile_slices()):
                cx, cy = -HALF_M + (xlo + xhi) / 2 * CELL_M, -HALF_M + (ylo + yhi) / 2 * CELL_M
                L.append(f'<geom name="terr{ty}{tx}" type="hfield" hfield="t{ty}{tx}" pos="{cx} {cy} 0" friction="{tmu[ty, tx]} {p.torsional_friction} '
                         f'{p.rolling_friction}" condim="4" solref="{p.floor_solref[0]} {p.floor_solref[1]}" rgba="{"0.42 0.36 0.28 1" if ter.tile_loose[ty, tx] else "0.30 0.34 0.38 1"}"/>')
    hh = p.extra.get("wall_half_h", 0.08)
    for i, (wx, wy, wyaw, wl) in enumerate(p.walls):
        L.append(f'<geom name="wall{i}" type="box" pos="{wx} {wy} {hh}" euler="0 0 {wyaw}" size="{wl} 0.01 {hh}" '
                 f'friction="{p.floor_friction} {p.torsional_friction} {p.rolling_friction}" condim="4" rgba=".7 .7 .75 1"/>')
    oh = p.extra.get("obstacle_half_h", 0.06)
    for i, (kind, ox, oy, oyaw, oa, ob) in enumerate(p.obstacles):
        shape = {"box": f'type="box" size="{oa} {ob} {oh}"', "cyl": f'type="cylinder" size="{oa} {oh}"', "ell": f'type="ellipsoid" size="{oa} {ob} {oh}"'}[kind]
        oz = oh if ter is None else oh + ter.support_height(ox, oy, 0.0 if kind == "ell" else (oa if kind == "cyl" else float(np.hypot(oa, ob))))
        L.append(f'<geom name="obs{i}" {shape} pos="{ox} {oy} {oz}" euler="0 0 {oyaw}" friction="{p.floor_friction} {p.torsional_friction} {p.rolling_friction}" '
                 f'condim="4" rgba=".85 .55 .25 1"/>')
    if p.post is not None:
        px, py = p.post
        pz = (ter.support_height(px, py, POST_RADIUS_M) if ter is not None else 0.0) + POST_HEIGHT_M / 2
        L.append(f'<geom name="post" type="cylinder" size="{POST_RADIUS_M} {POST_HEIGHT_M / 2}" pos="{px} {py} {pz}" friction="{p.floor_friction} {p.torsional_friction} '
                 f'{p.rolling_friction}" condim="4" rgba=".9 .2 .2 1"/>')
    if ter is not None:                                           # the lying body must not start inside the terrain (the start disk is flat, so normally 0)
        foot = max(float(np.hypot(c[0], c[1])) for c in centres) + 0.15
        z0 += ter.footprint_height(foot)
    root_pos = "" if p.hanging else f' pos="0 0 {z0:.6f}"'
    for k in range(nm):
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
        if k == nm - 1:
            L.append(f'<site name="imu" pos="{_vec(centres[k])}"/>')
        if k == 0:
            L.append(f'<site name="head" pos="{_vec(centres[0])}"/>')
    L.append("</body>" * nm)
    L.append("</worldbody>")
    L.append("<actuator>" + "".join(
        f'<position name="a{k}" joint="j{k}" kp="{p.kp}" kv="{p.kd}" forcerange="{-p.cap_nm[k - 1]} {p.cap_nm[k - 1]}"/>'
        for k in range(1, nm)) + "</actuator>")
    L.append('<sensor><gyro name="gyro" site="imu"/><accelerometer name="accel" site="imu"/><framequat name="quat" objtype="site" objname="imu"/>'
             '<framepos name="head_pos" objtype="site" objname="head"/><framelinvel name="head_vel" objtype="site" objname="head"/></sensor>')
    L.append("</mujoco>")
    return "".join(L), z0


def write_terrain(model: mujoco.MjModel, terrain: Terrain, floor_friction: float | None = None) -> None:
    """Write ``terrain`` into a compiled model's heightfield tiles (and, if ``floor_friction`` is given, its tile friction)."""
    for ty, (ylo, yhi) in enumerate(tile_slices()):
        for tx, (xlo, xhi) in enumerate(tile_slices()):
            hid = model.hfield(f"t{ty}{tx}").id
            adr, n = int(model.hfield_adr[hid]), int(model.hfield_nrow[hid] * model.hfield_ncol[hid])
            model.hfield_data[adr:adr + n] = (terrain.heights[ylo:yhi + 1, xlo:xhi + 1] / ELEV_MAX_M).ravel()
            if floor_friction is not None:
                model.geom_friction[model.geom(f"terr{ty}{tx}").id, 0] = terrain.tile_friction(floor_friction)[ty, tx]


class PetSim:
    """Steps the PET model: delayed position targets, MuJoCo PD actuators with per-joint torque cap, optional free body."""

    def __init__(self, params: BuildParams | None = None) -> None:
        """Build the model; ``reset`` places the robot."""
        self.p = params or BuildParams()
        xml, self.z0 = build_mjcf(self.p)
        self.model = mujoco.MjModel.from_xml_string(xml)
        if self.p.terrain is not None and not self.p.hanging:
            write_terrain(self.model, self.p.terrain)
        self.data = mujoco.MjData(self.model)
        self._delay_steps = max(0, int(round(self.p.delay_s / self.p.dt)))
        self._buf: deque[np.ndarray] = deque()
        self.set_cap(np.array(self.p.cap_nm))
        self.reset()
        self._foot = float(np.max(np.hypot(self.data.xipos[1:, 0], self.data.xipos[1:, 1]))) + 0.15      # radius of the lying body about the origin (+margin)
        self._z0_flat = self.z0 - (0.0 if self.p.terrain is None else self.p.terrain.footprint_height(self._foot))

    def set_terrain(self, terrain: Terrain, floor_friction: float | None = None) -> None:
        """Swap in a new terrain (heights, tile friction, start height) without rebuilding the model, then reset. The model must have been built with a terrain."""
        if self.p.terrain is None:
            raise ValueError("this sim was built without a terrain; build it with BuildParams(terrain=...)")
        fr = self.p.floor_friction if floor_friction is None else floor_friction
        write_terrain(self.model, terrain, fr)
        self.p = BuildParams(**{**self.p.__dict__, "terrain": terrain, "floor_friction": fr})
        self.z0 = self._z0_flat + terrain.footprint_height(self._foot)
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
        """Joint angles (rad), servo 1..n_modules-1."""
        a = 7 if not self.p.hanging else 0
        return self.data.qpos[a:a + self.p.n_modules - 1]

    def joint_vel(self) -> np.ndarray:
        """True joint velocities (rad/s)."""
        a = 6 if not self.p.hanging else 0
        return self.data.qvel[a:a + self.p.n_modules - 1]

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
