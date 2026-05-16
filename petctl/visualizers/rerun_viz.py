"""
RerunVisualizer — real-time visualization via Rerun.io.

Displays in the Rerun viewer:
  1. Motor state time-series (velocity, torque per motor)
  2. Battery telemetry (raw ADC voltage and current)
  3. Capacitive touch and FSR pressure (0–1 per module face) as separate charts
  4. 3D robot pose from actual OBJ mesh files, forward-kinematics driven

3D hierarchy per module:
  robot/module_0              ← Transform3D: offset + R_hpr @ R_joint  (dynamic)
  robot/module_0/mesh         ← Asset3D: raw OBJ geometry               (static)
  robot/module_0/module_1     ← Transform3D: offset + R_hpr @ R_joint  (dynamic)
  robot/module_0/module_1/mesh ← Asset3D                               (static)
  ...

Rotation composition matches Panda3D convention:
  combined = R_hpr * R_joint   (HPR applied after joint angle)
This ensures that modules with rotation=[180,0,180] correctly invert
the apparent servo direction in world-frame.

Rerun runs in its own separate process (spawned automatically).
There are no threading conflicts with asyncio.
"""

from __future__ import annotations

import json
import math
import os
from typing import TYPE_CHECKING, Optional

import numpy as np

from petctl.protocols import Visualizer
from petctl.types import RobotState

if TYPE_CHECKING:
    from petctl.controller import Controller

# petctl-local assembly (source of truth for module count / geometry)
_DEFAULT_ASSEMBLY = os.path.normpath(os.path.join(
    os.path.dirname(__file__),
    "../assets/robot_assembly.json",
))

_MODELS_DIR = os.path.normpath(os.path.join(
    os.path.dirname(__file__),
    "../assets/3d_models",
))

# ------------------------------------------------------------------
# Sensor-overlay geometry (in model-local coordinates, units: cm)
#
# Three prism faces, derived from the OBJ/STL mesh:
#   left   x = -3.5 flat face  (normal: -X)
#   right  x = +3.5 flat face  (normal: +X)
#   middle angled back surface  (normal: approx +0.83Y - 0.56Z)
#          centroid ≈ (0, 2.3, -2.1), slopes from (y=3.5,z=0) toward (y=-3.5,z=-7)
#
# Quaternions orient a box/disc Z-axis onto the face normal (xyzw format):
#   right:   90° around +Y  → Z → +X
#   left:   -90° around +Y  → Z → -X
#   middle: 135° around -X → Z → (0, 0.707, -0.707)  (45° slope)
# ------------------------------------------------------------------
_INV_SQRT2 = 1.0 / math.sqrt(2.0)

_SENSOR_FACES: dict[str, dict] = {
    "middle": {
        # Angled back face. 45° slope: normal = (0, 1/√2, -1/√2).
        # Rotation: 135° around -X from Z=(0,0,1) to (0,0.707,-0.707)
        "center":     [0.0, 0.9, -4.6],
        "quaternion": [-0.924, 0.0, 0.0, 0.383],
    },
    "right": {
        "center":     [3.51, -0.75, -2.75],
        "quaternion": [0.0, _INV_SQRT2, 0.0, _INV_SQRT2],
    },
    "left": {
        "center":     [-3.51, -0.75, -2.75],
        "quaternion": [0.0, -_INV_SQRT2, 0.0, _INV_SQRT2],
    },
}

_PRESSURE_RADIUS    = 0.75  # cm — FSR disc radius
_PRESSURE_THICKNESS = 0.15  # cm

# Base RGB colours; alpha is proportional to sensor level (min 30 so overlays
# are always faintly visible even when sensors read zero)
_TOUCH_COLOR_RGB    = (50, 180, 220)   # cyan-blue
_PRESSURE_COLOR_RGB = (230, 100, 20)   # warm orange
_MIN_ALPHA = 30

_SENSOR_FACE_NAMES = ("left", "middle", "right")

# ------------------------------------------------------------------
# Per-pad box layout (model-local coordinates, cm).
# Order matches nibble order from the sensor packet:
#   right pads  0-3  (nibbles 0-3)
#   left  pads  0-3  (nibbles 4-7)
#   middle pads 0-5  (nibbles 8-13)
#
# Right/left: 2×2 grid on the flat rectangular face.
# Middle: 2-col × 3-row grid on the angled rectangular face.
#
# Pad ordering within each face (top-left→top-right→...) is an assumption
# until verified against live hardware; rows/columns may need swapping.
# ------------------------------------------------------------------
_PAD_CENTERS: list[list[float]] = [
    # Right face (x=+3.51): pad_0 lone corner; pads 1,3,2 along -45° line, 3 in centre
    [3.51,  0.25,  -1.75],   # right_0  top-near  (verified)
    [3.51, -0.42,  -5.08],   # right_1  one step from pad_3 along -45°
    [3.51, -3.08,  -2.42],   # right_2  opposite side of pad_3 from pad_1
    [3.51, -1.75,  -3.75],   # right_3  centre of line (verified)
    # Left face (x=-3.51): pad_0 lone corner; pads 1,3,2 along -45° line, 3 in centre
    [-3.51, -1.75,  -1.75],  # left_0   (verified)
    [-3.51, -1.08,  -5.08],  # left_1   one step from pad_3 along -45°
    [-3.51,  1.58,  -2.42],  # left_2   opposite side of pad_3 from pad_1
    [-3.51,  0.25,  -3.75],  # left_3   centre of line (verified)
    # Middle face (angled), 2-col × 3-row
    [-1.5,  2.67, -2.83],   # middle_0  top-left
    [ 1.5,  2.67, -2.83],   # middle_1  top-right
    [-1.5,  0.90, -4.60],   # middle_2  mid-left
    [ 1.5,  0.90, -4.60],   # middle_3  mid-right
    [-1.5, -0.87, -6.37],   # middle_4  bot-left
    [ 1.5, -0.87, -6.37],   # middle_5  bot-right
]

# half_sizes in box-local frame (local-Z = face normal, so local-Z is "thickness")
_PAD_HALF_SIZES: list[list[float]] = (
    [[0.75, 0.75, 0.05]] * 4   # right: 1.5 cm square
    + [[0.75, 0.75, 0.05]] * 4  # left:  1.5 cm square
    + [[0.60, 0.75, 0.05]] * 6  # middle: 1.2 × 1.5 cm
)

# Per-pad quaternions (same orientation as the containing face)
_PAD_QUAT_RIGHT  = [0.0,  _INV_SQRT2, 0.0, _INV_SQRT2]
_PAD_QUAT_LEFT   = [0.0, -_INV_SQRT2, 0.0, _INV_SQRT2]
_PAD_QUAT_MIDDLE = [-0.924, 0.0, 0.0, 0.383]
_PAD_QUATS_XYZW: list[list[float]] = (
    [_PAD_QUAT_RIGHT]  * 4
    + [_PAD_QUAT_LEFT]   * 4
    + [_PAD_QUAT_MIDDLE] * 6
)

# Temporary pad ID labels — to be removed once pad layout is verified on hardware
_PAD_LABELS: list[str] = (
    ["R0", "R1", "R2", "R3"]
    + ["L0", "L1", "L2", "L3"]
    + ["M0", "M1", "M2", "M3", "M4", "M5"]
)


class RerunVisualizer(Visualizer):
    """
    Visualizes robot pose and motor state in Rerun.

    Args:
        app_name:       Rerun application name (shown in viewer title bar)
        assembly_file:  Path to robot_assembly.json.  Defaults to petctl/assets/robot_assembly.json.
        models_dir:     Directory containing the .obj files.  Defaults to petctl/assets/3d_models/.
        show_3d:        Log 3D robot pose (default: True)
    """

    name = "rerun"

    def __init__(
        self,
        app_name: str = "petctl",
        assembly_file: Optional[str] = None,
        models_dir: Optional[str] = None,
        show_3d: bool = True,
    ) -> None:
        self.app_name = app_name
        self.assembly_file = assembly_file or _DEFAULT_ASSEMBLY
        self.models_dir = models_dir or _MODELS_DIR
        self.show_3d = show_3d

        self._module_meta: list[dict] = []
        # Pre-computed HPR rotation matrices keyed by module_id
        self._hpr_mats: dict[int, np.ndarray] = {}
        # Pre-normalized joint axes — computed once in _load_assembly()
        self._joint_axes: dict[int, tuple[float, float, float]] = {}
        # Cached parent map and entity paths — built once in _load_assembly()
        self._parent_map: dict[int, Optional[int]] = {}
        self._entity_path_cache: dict[int, str] = {}
        # Pre-computed module offsets (avoids per-tick dict.get on module dicts)
        self._module_offsets: dict[int, list] = {}
        # Per-module swap_lr flag for sensor overlay (viz X-flip vs backend swap)
        self._overlay_swap_lr: dict[int, bool] = {}
        # Static batched arrays for sensor overlay (set in on_start)
        # FSR pressure: 3 face-level ellipsoids per module
        self._overlay_centers: Optional[np.ndarray] = None
        self._overlay_pressure_hs: Optional[np.ndarray] = None
        self._overlay_quats: list = []
        # Capacitive touch: 14 per-pad boxes per module
        self._pad_centers_np: Optional[np.ndarray] = None
        self._pad_half_sizes_np: Optional[np.ndarray] = None
        self._pad_quats: list = []
        self._rr = None

    # ------------------------------------------------------------------
    # Visualizer interface
    # ------------------------------------------------------------------

    def on_start(self, controller: "Controller") -> None:
        try:
            import rerun as rr
            self._rr = rr
        except ImportError:
            print("[RerunVisualizer] rerun-sdk not installed. Run: pip install rerun-sdk")
            return

        rr.init(self.app_name, spawn=True)
        rr.log("robot", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
        self._load_assembly()
        self._setup_overlay_geometry(rr)
        self._log_pad_labels_static(rr)
        self._send_blueprint()
        self._setup_motor_series()
        self._setup_battery_series()
        self._setup_sensor_face_series()
        if self.show_3d:
            self._setup_3d_geometry()

    def update(self, state: RobotState) -> None:
        if self._rr is None:
            return
        rr = self._rr
        rr.set_time("time", duration=state.timestamp)
        self._log_motor_state(rr, state)
        self._log_battery_series(rr, state)
        self._log_sensor_face_series(rr, state)
        if self.show_3d:
            self._log_3d_pose(rr, state)
            self._log_sensor_overlays(rr, state)

    def on_stop(self) -> None:
        pass  # Rerun viewer stays open after the script exits (by design)

    # ------------------------------------------------------------------
    # Sensor logging
    # ------------------------------------------------------------------

    def _send_blueprint(self) -> None:
        """Send a blueprint with 3D view and time-series charts (motors, battery, sensors)."""
        import rerun.blueprint as rrb
        rr = self._rr
        views = []
        if self.show_3d:
            views.append(rrb.Spatial3DView(origin="robot", name="Robot"))
        views.append(rrb.Vertical(
            rrb.TimeSeriesView(origin="motors/velocity", name="Velocity (rad/s)"),
            rrb.TimeSeriesView(origin="motors/torque", name="Torque (Nm)"),
            rrb.TimeSeriesView(origin="telemetry/voltage_raw", name="Battery voltage (raw ADC)"),
            rrb.TimeSeriesView(origin="telemetry/current_raw", name="Battery current (raw ADC)"),
            rrb.TimeSeriesView(origin="sensors/capacitive", name="Capacitive touch (0–1)"),
            rrb.TimeSeriesView(origin="sensors/fsr", name="FSR pressure (0–1)"),
        ))
        rr.send_blueprint(rrb.Blueprint(rrb.Horizontal(*views)))

    def _setup_motor_series(self) -> None:
        """Declare SeriesLines for each motor telemetry metric."""
        rr = self._rr
        servo_ids = [int(mod["id"]) for mod in self._module_meta if int(mod["id"]) > 0]
        for sid in servo_ids:
            label = f"motor {sid}"
            rr.log(f"motors/velocity/motor_{sid}", rr.SeriesLines(names=label), static=True)
            rr.log(f"motors/torque/motor_{sid}", rr.SeriesLines(names=label), static=True)

    def _log_motor_state(self, rr, state: RobotState) -> None:
        """Log velocity and torque for each motor."""
        for sid, val in state.motor_velocities.items():
            rr.log(f"motors/velocity/motor_{sid}", rr.Scalars(float(val)))
        for sid, val in state.motor_torques.items():
            rr.log(f"motors/torque/motor_{sid}", rr.Scalars(float(val)))

    def _setup_battery_series(self) -> None:
        """Declare SeriesLines for head battery raw ADC streams."""
        rr = self._rr
        rr.log(
            "telemetry/voltage_raw",
            rr.SeriesLines(names="voltage (raw ADC)"),
            static=True,
        )
        rr.log(
            "telemetry/current_raw",
            rr.SeriesLines(names="current (raw ADC)"),
            static=True,
        )

    def _log_battery_series(self, rr, state: RobotState) -> None:
        rr.log("telemetry/voltage_raw", rr.Scalars(float(state.battery_voltage_raw)))
        rr.log("telemetry/current_raw", rr.Scalars(float(state.battery_current_raw)))

    def _setup_sensor_face_series(self) -> None:
        """Declare SeriesLines for each capacitive pad and per-face FSR."""
        rr = self._rr
        if not self._module_meta:
            return
        n_pads = {"left": 4, "right": 4, "middle": 6}
        for mod in self._module_meta:
            mod_id = int(mod["id"])
            for face in _SENSOR_FACE_NAMES:
                for i in range(n_pads[face]):
                    path = f"sensors/capacitive/{mod_id}_{face}_{i}"
                    rr.log(path, rr.SeriesLines(names=f"mod{mod_id} {face}[{i}]"), static=True)
                fsr_path = f"sensors/fsr/{mod_id}_{face}"
                rr.log(fsr_path, rr.SeriesLines(names=f"mod{mod_id} {face} FSR"), static=True)

    def _log_sensor_face_series(self, rr, state: RobotState) -> None:
        """Log per-pad capacitive touch and per-face FSR for every assembly module."""
        if not self._module_meta:
            return
        for mod in self._module_meta:
            mod_id = int(mod["id"])
            sens = state.sensors.get(mod_id)
            for face in _SENSOR_FACE_NAMES:
                pads: tuple[float, ...] = getattr(sens, f"touch_{face}_pads", ()) if sens else ()
                for i, v in enumerate(pads):
                    rr.log(f"sensors/capacitive/{mod_id}_{face}_{i}", rr.Scalars(float(v)))
                pv = float(getattr(sens, f"pressure_{face}", 0.0)) if sens else 0.0
                rr.log(f"sensors/fsr/{mod_id}_{face}", rr.Scalars(pv))

    def _setup_overlay_geometry(self, rr) -> None:
        """Pre-compute static arrays used every tick by _log_sensor_overlays."""
        # FSR pressure: one ellipsoid per face, left/middle/right order
        self._overlay_centers = np.array(
            [_SENSOR_FACES[f]["center"] for f in _SENSOR_FACE_NAMES], dtype=np.float32
        )
        self._overlay_pressure_hs = np.array(
            [[_PRESSURE_RADIUS, _PRESSURE_RADIUS, _PRESSURE_THICKNESS]] * 3, dtype=np.float32
        )
        self._overlay_quats = [
            rr.Quaternion(xyzw=_SENSOR_FACES[f]["quaternion"]) for f in _SENSOR_FACE_NAMES
        ]
        # Capacitive touch: one box per pad (14 total)
        self._pad_centers_np = np.array(_PAD_CENTERS, dtype=np.float32)
        self._pad_half_sizes_np = np.array(_PAD_HALF_SIZES, dtype=np.float32)
        self._pad_quats = [rr.Quaternion(xyzw=q) for q in _PAD_QUATS_XYZW]
        # Per-module: True if left/right sensor reads should be swapped for correct display.
        # Modules whose HPR rotation flips local X (R_hpr[0,0] < 0) need a swap unless
        # the backend has already applied its own left/right inversion (odd module IDs).
        self._overlay_swap_lr = {
            int(mod["id"]): (
                float(self._hpr_mats[int(mod["id"])][0, 0]) < 0
                and (int(mod["id"]) % 2) == 0
            )
            for mod in self._module_meta
        }

    def _log_pad_labels_static(self, rr) -> None:
        """Log temporary pad ID labels as black text at each pad center (static, one-time)."""
        if self._pad_centers_np is None:
            return
        black = [0, 0, 0, 255]
        for mod in self._module_meta:
            mod_id = int(mod["id"])
            entity_base = self._entity_path_cache[mod_id]
            rr.log(
                f"{entity_base}/sensor_overlay/pad_labels",
                rr.Points3D(
                    positions=self._pad_centers_np,
                    labels=_PAD_LABELS,
                    colors=[black] * len(_PAD_LABELS),
                    radii=[0.01] * len(_PAD_LABELS),
                ),
                static=True,
            )

    def _log_sensor_overlays(self, rr, state: RobotState) -> None:
        """
        Log per-pad capacitive touch boxes and per-face FSR pressure discs for
        every module. Touch = 14 cyan boxes (one per cap pad); pressure = 3 orange
        discs (one per face). Alpha is proportional to sensor value.

        Pad boxes are logged as children of the module's joint entity so they ride
        along automatically as the robot articulates.
        """
        if self._pad_centers_np is None or self._overlay_centers is None:
            return
        for mod in self._module_meta:
            mod_id = int(mod["id"])
            sensors = state.sensors.get(mod_id)
            if sensors is None:
                continue

            entity_base = self._entity_path_cache[mod_id]
            swap_lr = self._overlay_swap_lr.get(mod_id, False)

            # Build 14-element pad value list: [right_0..3, left_0..3, middle_0..5]
            if swap_lr:
                right_pads = sensors.touch_left_pads
                left_pads  = sensors.touch_right_pads
            else:
                right_pads = sensors.touch_right_pads
                left_pads  = sensors.touch_left_pads
            all_pad_vals = (*right_pads, *left_pads, *sensors.touch_middle_pads)

            pad_colors = [
                (*_TOUCH_COLOR_RGB, _MIN_ALPHA + int(min(1.0, max(0.0, v)) * (255 - _MIN_ALPHA)))
                for v in all_pad_vals
            ]
            rr.log(f"{entity_base}/sensor_overlay/touch", rr.Boxes3D(
                centers=self._pad_centers_np,
                half_sizes=self._pad_half_sizes_np,
                quaternions=self._pad_quats,
                colors=pad_colors,
                fill_mode="solid",
            ))

            # FSR: one face-level disc per face (left/middle/right order)
            if swap_lr:
                pressure_vals = [sensors.pressure_right, sensors.pressure_middle, sensors.pressure_left]
            else:
                pressure_vals = [sensors.pressure_left, sensors.pressure_middle, sensors.pressure_right]
            pressure_colors = [
                (*_PRESSURE_COLOR_RGB, _MIN_ALPHA + int(min(1.0, max(0.0, p)) * (255 - _MIN_ALPHA)))
                for p in pressure_vals
            ]
            rr.log(f"{entity_base}/sensor_overlay/pressure", rr.Ellipsoids3D(
                centers=self._overlay_centers,
                half_sizes=self._overlay_pressure_hs,
                quaternions=self._overlay_quats,
                colors=pressure_colors,
                fill_mode="solid",
            ))

    # ------------------------------------------------------------------
    # 3D setup and pose logging
    # ------------------------------------------------------------------

    def _load_assembly(self) -> None:
        if not os.path.isfile(self.assembly_file):
            print(f"[RerunVisualizer] Assembly file not found: {self.assembly_file}")
            return
        with open(self.assembly_file) as f:
            data = json.load(f)
        self._module_meta = data.get("assembly", {}).get("modules", [])

        # Pre-compute HPR rotation matrices and normalized joint axes (used each tick)
        for mod in self._module_meta:
            mod_id = int(mod["id"])
            euler = mod.get("rotation", [0.0, 0.0, 0.0])
            self._hpr_mats[mod_id] = _hpr_to_mat3(*euler)

            raw = mod.get("joint", {}).get("axis", [0, 0, 1])
            ax, ay, az = float(raw[0]), float(raw[1]), float(raw[2])
            norm = math.sqrt(ax*ax + ay*ay + az*az)
            if norm > 1e-9:
                ax, ay, az = ax/norm, ay/norm, az/norm
            self._joint_axes[mod_id] = (ax, ay, az)

        # Cache parent map once — avoids rebuilding it per tick per module
        self._parent_map = {
            int(mod["id"]): (int(mod["parent"]) if mod.get("parent") is not None else None)
            for mod in self._module_meta
        }

        # Pre-compute entity path strings (avoids string joins at 50 Hz)
        self._entity_path_cache = {
            int(mod["id"]): self._entity_path(int(mod["id"]))
            for mod in self._module_meta
        }

        # Pre-compute per-module link offsets (avoids dict.get on every tick)
        self._module_offsets = {
            int(mod["id"]): mod.get("offset", [0.0, 0.0, 0.0])
            for mod in self._module_meta
        }

        print(f"[RerunVisualizer] Loaded assembly with {len(self._module_meta)} modules")

    def _setup_3d_geometry(self) -> None:
        """Log static OBJ mesh for each module (once, time-independent)."""
        rr = self._rr
        if not self._module_meta:
            print("[RerunVisualizer] No assembly data — skipping 3D geometry")
            return

        for mod in self._module_meta:
            mod_id = int(mod["id"])
            mesh_path = self._entity_path(mod_id) + "/mesh"

            model_filename = mod.get("model", "")
            obj_path = os.path.join(self.models_dir, model_filename)
            if not os.path.isfile(obj_path):
                print(f"[RerunVisualizer] Model not found: {obj_path}")
                rr.log(mesh_path, rr.Boxes3D(
                    half_sizes=[[3.5, 3.5, 3.59]],
                    colors=[[250, 245, 217, 220]],
                ), static=True)
            else:
                rr.log(mesh_path, rr.Asset3D(path=obj_path), static=True)

    def _log_3d_pose(self, rr, state: RobotState) -> None:
        """
        Log each module's joint transform each tick.

        Transform = link offset (translation) + R_hpr @ R_joint (rotation).

        R_hpr is the module's static mesh orientation from the assembly JSON.
        R_joint is the dynamic rotation from the servo angle around the joint axis.
        Composing R_hpr @ R_joint (HPR applied after joint) matches Panda3D's
        `initial_quat * joint_quat` convention, so modules with rotation=[180,0,180]
        correctly invert the apparent servo direction.

        Rerun composes transforms along the entity hierarchy automatically.
        """
        if not self._module_meta:
            return

        for mod in self._module_meta:
            mod_id = int(mod["id"])
            joint_path = self._entity_path_cache[mod_id]
            offset = self._module_offsets[mod_id]

            # Negate: hardware joint positive sense vs mesh joint axis in assembly
            # right-hand-rule Z rotation, so the viz was a mirror of the real robot.
            angle_rad = -self._servo_angle_rad(mod_id, state)

            R = self._hpr_mats[mod_id] @ _axis_angle_to_mat3(self._joint_axes[mod_id], angle_rad)

            rr.log(joint_path, rr.Transform3D(
                translation=offset,
                mat3x3=R,
            ))

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _entity_path(self, mod_id: int) -> str:
        """Hierarchical entity path: robot/module_0/module_1/module_2/..."""
        if not self._module_meta:
            return f"robot/module_{mod_id}"
        chain = self._ancestor_chain(mod_id)
        return "/".join(["robot"] + [f"module_{m}" for m in chain])

    def _ancestor_chain(self, mod_id: int) -> list[int]:
        """Return module IDs from root to mod_id (inclusive)."""
        chain: list[int] = []
        current: Optional[int] = mod_id
        while current is not None:
            chain.append(current)
            current = self._parent_map.get(current)
        chain.reverse()
        return chain

    @staticmethod
    def _servo_angle_rad(servo_id: int, state: RobotState) -> float:
        return state.servo_positions.get(servo_id, 0.0)


# ------------------------------------------------------------------
# Rotation math helpers
# ------------------------------------------------------------------

def _hpr_to_mat3(h_deg: float, p_deg: float, r_deg: float) -> np.ndarray:
    """
    Panda3D HPR → 3x3 rotation matrix.
    Convention: Heading=Z, Pitch=X, Roll=Y, composed as Rz(H) @ Rx(P) @ Ry(R).
    This matches how robot_assembly.json 'rotation' fields were authored.
    """
    h = math.radians(h_deg)
    p = math.radians(p_deg)
    r = math.radians(r_deg)

    ch, sh = math.cos(h), math.sin(h)
    cp, sp = math.cos(p), math.sin(p)
    cr, sr = math.cos(r), math.sin(r)

    Rz = np.array([[ch, -sh, 0.],
                   [sh,  ch, 0.],
                   [0.,  0., 1.]], dtype=np.float32)
    Rx = np.array([[1., 0.,   0. ],
                   [0., cp,  -sp ],
                   [0., sp,   cp ]], dtype=np.float32)
    Ry = np.array([[ cr, 0., sr],
                   [0.,  1., 0.],
                   [-sr, 0., cr]], dtype=np.float32)

    return Rz @ Rx @ Ry


def _axis_angle_to_mat3(axis: tuple[float, float, float], angle_rad: float) -> np.ndarray:
    """Rodrigues' rotation formula: rotation by angle_rad around a pre-normalized unit axis."""
    ax, ay, az = axis
    c = math.cos(angle_rad)
    s = math.sin(angle_rad)
    t = 1.0 - c

    return np.array([
        [t*ax*ax + c,    t*ax*ay - s*az, t*ax*az + s*ay],
        [t*ax*ay + s*az, t*ay*ay + c,    t*ay*az - s*ax],
        [t*ax*az - s*ay, t*ay*az + s*ax, t*az*az + c   ],
    ], dtype=np.float32)
