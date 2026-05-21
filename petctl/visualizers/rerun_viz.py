"""
RerunVisualizer — real-time visualization via Rerun.io.

Displays in the Rerun viewer:
  1. Motor state time-series (velocity, torque per motor)
  2. Battery telemetry (raw ADC voltage, current in Amps)
  3. 3D robot pose from actual OBJ mesh files, forward-kinematics driven

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
import logging
import math
import os
from typing import TYPE_CHECKING, Optional

import numpy as np

from petctl.perception.stroke import HoldDetector, StrokeDetector
from petctl.protocols import Visualizer
from petctl.types import RobotState

logger = logging.getLogger(__name__)

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

# Base RGB colours; alpha is proportional to sensor level
_TOUCH_COLOR_RGB    = (255, 215, 0)    # gold
_PRESSURE_COLOR_RGB = (50, 120, 220)   # royal blue
_MIN_ALPHA = 38   # 15% opacity at zero activation
_MAX_ALPHA = 204  # 80% opacity at full activation

_SENSOR_FACE_NAMES = ("left", "middle", "right")

# ------------------------------------------------------------------
# Per-pad box layout (model-local coordinates, cm).
# Canonical pad order (applied in robot.py before reaching here):
#   right pads  0-3  (canonical: top_head, top_mid, top_rear, bottom_solo)
#   left  pads  0-3  (canonical: top_head, top_mid, top_rear, bottom_solo)
#   middle pads 0-5
#
# Both faces share the same semantic ordering so behavior code can treat them uniformly.
# Middle: 2-col × 3-row grid on the angled rectangular face.
# ------------------------------------------------------------------
_PAD_CENTERS: list[list[float]] = [
    # Right face (x=+3.51): canonical 0=top_head, 1=top_mid, 2=top_rear, 3=bottom_solo
    [3.51,  1.58,  -2.42],   # right_0  (top_head)
    [3.51,  0.25,  -3.75],   # right_1  (top_mid)
    [3.51, -1.08,  -5.08],   # right_2  (top_rear)
    [3.51, -1.75,  -1.75],   # right_3  (bottom_solo)
    # Left face (x=-3.51): canonical 0=top_head, 1=top_mid, 2=top_rear, 3=bottom_solo
    [-3.51,  1.58,  -2.42],  # left_0   (top_head)
    [-3.51,  0.25,  -3.75],  # left_1   (top_mid)
    [-3.51, -1.08,  -5.08],  # left_2   (top_rear)
    [-3.51, -1.75,  -1.75],  # left_3   (bottom_solo)
    # Middle face (angled), 2-col × 3-row
    [-1.5,  2.67, -2.83],   # middle_0  top-left
    [ 1.5,  2.67, -2.83],   # middle_1  top-right
    [-1.5,  0.90, -4.60],   # middle_2  mid-left
    [ 1.5,  0.90, -4.60],   # middle_3  mid-right
    [-1.5, -0.87, -6.37],   # middle_4  bot-left
    [ 1.5, -0.87, -6.37],   # middle_5  bot-right
]

# half_sizes in face-local frame (local-Z = face normal = thickness)
# Equal x/y → circular disc when rendered as Ellipsoids3D
_PAD_HALF_SIZES: list[list[float]] = [[0.75, 0.75, 0.05]] * 14

# How far (cm) to push sensor centres along the face normal so they sit on the surface
_FSR_SURFACE_OFFSET = 0.01   # FSR sits just on the face, under the cap pads
_PAD_SURFACE_OFFSET = 0.03   # cap pads sit slightly proud of the FSR

# Face-normal vectors (world-space, per pad group and per FSR face)
_NORMAL_RIGHT  = np.array([ 1.0,  0.0,       0.0      ], dtype=np.float32)
_NORMAL_LEFT   = np.array([-1.0,  0.0,       0.0      ], dtype=np.float32)
_NORMAL_MIDDLE = np.array([ 0.0,  _INV_SQRT2, -_INV_SQRT2], dtype=np.float32)

# Touch blob sphere: face-surface anchor points and display constants.
# Each entry is the face centre offset outward by _BLOB_SPHERE_OFFSET cm,
# in module-local coordinates.  The sphere lives here so it sits outside
# the body surface.
_BLOB_SPHERE_OFFSET: float = 1.5          # cm beyond face surface
_BLOB_ACTIVATION_THRESHOLD: float = 0.35  # minimum per-pad value to contribute
_BLOB_RADIUS_CM: float = 0.8              # sphere radius in cm

# Per-pad positions and unit normals in _PAD_CENTERS order: right[0-3], left[4-7], middle[8-13]
_PAD_CENTERS_NP: np.ndarray = np.array(_PAD_CENTERS, dtype=np.float64)
_PAD_NORMALS_UNIT: np.ndarray = np.array(
    [_NORMAL_RIGHT]  * 4
    + [_NORMAL_LEFT]   * 4
    + [_NORMAL_MIDDLE] * 6,
    dtype=np.float64,
)
# Prism face planes (outward_normal_f64, signed_distance) used to project the pad
# centroid onto the prism surface before offsetting the sphere outward.
_PRISM_FACE_PLANES: tuple[tuple[np.ndarray, float], ...] = (
    (_NORMAL_RIGHT.astype(np.float64),
     float(np.dot(_NORMAL_RIGHT,  np.array(_SENSOR_FACES["right"]["center"])))),
    (_NORMAL_LEFT.astype(np.float64),
     float(np.dot(_NORMAL_LEFT,   np.array(_SENSOR_FACES["left"]["center"])))),
    (_NORMAL_MIDDLE.astype(np.float64),
     float(np.dot(_NORMAL_MIDDLE, np.array(_SENSOR_FACES["middle"]["center"])))),
)

# Per-pad surface-offset vectors in the same order as _PAD_CENTERS
_PAD_NORMAL_OFFSETS: np.ndarray = np.array(
    [_NORMAL_RIGHT]  * 4
    + [_NORMAL_LEFT]   * 4
    + [_NORMAL_MIDDLE] * 6,
    dtype=np.float32,
) * _PAD_SURFACE_OFFSET

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

# ------------------------------------------------------------------
# IMU overlay — BNO085 on the rear (no-sensor) face of module 7
#
# In module 7's model-local frame (before HPR rotation):
#   The no-sensor face is the flat rectangular face with normal (0, -1, 0).
#   It sits at Y = -3.5 and spans Z = 0.175 to Z ≈ -5.59 along the tail.
#   Face centre (midpoint of Z extent): (0, -3.5, -2.7)
#   Offset 0.15 cm outward along normal (-Y) for PCB surface placement.
#
# Quaternion: 90° around +X → rotates local-Z to face normal (0, -1, 0).
#
# The PCB rectangle and axes live as a static child entity that
# Rerun automatically carries with the tail as it articulates.
# ------------------------------------------------------------------
_IMU_MODULE_ID: int = 7

# Centre of the PCB on the no-sensor face, in module 7's local frame (cm).
# Face centroid (0, -3.5, -2.7) offset 0.15 cm outward along the -Y normal.
_IMU_CENTER: tuple[float, float, float] = (0.0, -3.65, -2.7)

# World-space position where module 7's joint origin must sit so the IMU lands at (0,0,0).
# At rest, module 7's accumulated world rotation = I, so target = -I @ _IMU_CENTER = -_IMU_CENTER.
_IMU_WORLD_TARGET: tuple[float, float, float] = (
    -_IMU_CENTER[0], -_IMU_CENTER[1], -_IMU_CENTER[2]
)

# Quaternion (xyzw) that orients the IMU slab flat on the -Y face.
# 90° around +X: maps local-Z → (0, -1, 0).
_IMU_QUATERNION_XYZW: tuple[float, float, float, float] = (0.707, 0.0, 0.0, 0.707)

# BNO085 PCB half-extents in the IMU's local frame (cm):
#   X: width along module X (face width direction)
#   Y: height along the angled face (face height direction)
#   Z: thickness (along face normal, perpendicular to board surface)
_IMU_PCB_HALF_SIZES: tuple[float, float, float] = (1.5, 1.0, 0.15)

# Length of each axis arrow drawn at the IMU centre (cm)
_IMU_AXIS_LENGTH: float = 1.5

_IMU_COLOR: tuple[int, int, int, int] = (180, 220, 255, 200)   # light blue, semi-opaque


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
        self._show_pad_labels: bool = False
        self._stroke_detector = StrokeDetector()
        self._stroke_active: bool = False
        self._stroke_color_until: float = 0.0
        self._hold_detector = HoldDetector()
        self._hold_active: bool = False
        self._hold_color_until: float = 0.0
        self._last_viz_time: float = -1.0
        self._prev_touch_colors: dict[int, tuple] = {}
        self._prev_pressure_colors: dict[int, tuple] = {}

    # ------------------------------------------------------------------
    # Visualizer interface
    # ------------------------------------------------------------------

    def on_start(self, controller: "Controller") -> None:
        try:
            import rerun as rr
            self._rr = rr
        except ImportError:
            logger.error("[RerunVisualizer] rerun-sdk not installed. Run: pip install rerun-sdk")
            return

        rr.init(self.app_name)
        rr.spawn(memory_limit="2GiB")
        rr.log("robot", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
        self._load_assembly()
        self._setup_overlay_geometry(rr)
        self._log_pad_labels_static(rr)
        self._send_blueprint()
        self._setup_motor_series()
        self._setup_battery_series()
        if self.show_3d:
            self._setup_3d_geometry()

    _VIZ_PERIOD = 1.0 / 20.0

    def update(self, state: RobotState) -> None:
        if self._rr is None:
            return
        if (self._last_viz_time >= 0
                and state.timestamp - self._last_viz_time < self._VIZ_PERIOD):
            return
        self._last_viz_time = state.timestamp
        rr = self._rr
        rr.set_time("time", duration=state.timestamp)
        now = state.timestamp
        _COLOR_HOLD_S = 0.15   # minimum sphere color hold time to prevent flicker
        stroke_now = self._stroke_detector.update(state) is not None
        hold_reading = self._hold_detector.update(state)
        hold_now = hold_reading is not None
        if stroke_now:
            self._stroke_color_until = now + _COLOR_HOLD_S
            self._hold_color_until = 0.0   # stroke cancels any pending hold display
        if hold_now and not stroke_now:
            self._hold_color_until = now + _COLOR_HOLD_S
        self._stroke_active = stroke_now or now < self._stroke_color_until
        self._hold_active = (hold_now or now < self._hold_color_until) and not self._stroke_active
        self._log_motor_state(rr, state)
        self._log_battery_series(rr, state)
        self._log_power_telemetry(rr, state)
        if self.show_3d:
            self._log_3d_pose(rr, state)
            self._log_sensor_overlays(rr, state)
            self._log_touch_blob_spheres(rr, state)

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
            rrb.TimeSeriesView(origin="motors/temperature", name="Temperature (°C)"),
            rrb.TimeSeriesView(origin="telemetry/voltage_v", name="Battery voltage (V)"),
            rrb.TimeSeriesView(origin="telemetry/current_amps", name="Battery current (A)"),
        ))
        rr.send_blueprint(rrb.Blueprint(
            rrb.Horizontal(*views),
            rrb.SelectionPanel(state="hidden"),
        ))

    def _setup_motor_series(self) -> None:
        """Declare SeriesLines for each motor telemetry metric."""
        rr = self._rr
        servo_ids = [int(mod["id"]) for mod in self._module_meta if int(mod["id"]) > 0]
        for sid in servo_ids:
            label = f"motor {sid}"
            rr.log(f"motors/velocity/motor_{sid}", rr.SeriesLines(names=label), static=True)
            rr.log(f"motors/torque/motor_{sid}", rr.SeriesLines(names=label), static=True)
            rr.log(f"motors/temperature/motor_{sid}", rr.SeriesLines(names=label), static=True)

    def _log_motor_state(self, rr, state: RobotState) -> None:
        """Log velocity, torque, and temperature for each motor."""
        for sid, val in state.motor_velocities.items():
            rr.log(f"motors/velocity/motor_{sid}", rr.Scalars(float(val)))
        for sid, val in state.motor_torques.items():
            rr.log(f"motors/torque/motor_{sid}", rr.Scalars(float(val)))
        for sid, val in state.motor_temperatures.items():
            rr.log(f"motors/temperature/motor_{sid}", rr.Scalars(float(val)))
        for sid, val in state.motor_winding_temperatures.items():
            rr.log(f"motors/winding_temperature/motor_{sid}", rr.Scalars(float(val)))

    def _log_power_telemetry(self, rr, state: RobotState) -> None:
        pt = state.power_telemetry
        if pt is None:
            return
        rr.log("power/voltage/raw", rr.Scalars(pt.voltage_raw_v))
        if pt.voltage_filtered_v is not None:
            rr.log("power/voltage/filtered", rr.Scalars(pt.voltage_filtered_v))
        rr.log("power/voltage/spike_count", rr.Scalars(float(pt.voltage_spike_count)))
        rr.log("power/voltage/spike_rate_per_min", rr.Scalars(float(pt.voltage_spike_rate_per_min)))
        rr.log("power/voltage/state", rr.TextLog(pt.voltage_state))
        rr.log("power/global_state", rr.TextLog(pt.system_state))
        for mid, state_str in pt.motor_states.items():
            rr.log(f"power/motors/{mid}/state", rr.TextLog(state_str))
            scale = pt.motor_compliance_scales.get(mid, 1.0)
            rr.log(f"power/motors/{mid}/compliance_scale", rr.Scalars(scale))
        for event in pt.events:
            rr.log("power/events", rr.TextLog(event))

    def _setup_battery_series(self) -> None:
        """Declare SeriesLines for head battery telemetry."""
        rr = self._rr
        rr.log(
            "telemetry/voltage_v",
            rr.SeriesLines(names="voltage (V)"),
            static=True,
        )
        rr.log(
            "telemetry/current_amps",
            rr.SeriesLines(names="current (A)"),
            static=True,
        )

    def _log_battery_series(self, rr, state: RobotState) -> None:
        if not state.battery_current_raw and not state.battery_voltage_raw:
            return
        rr.log("telemetry/voltage_v", rr.Scalars(state.battery_voltage_v))
        rr.log("telemetry/current_amps", rr.Scalars(state.battery_current_amps))

    def _setup_overlay_geometry(self, rr) -> None:
        """Pre-compute static arrays used every tick by _log_sensor_overlays."""
        # FSR pressure: one ellipsoid per face (left/middle/right), offset onto face surface
        _fsr_normals = np.array(
            [_NORMAL_LEFT, _NORMAL_MIDDLE, _NORMAL_RIGHT], dtype=np.float32
        )
        self._overlay_centers = (
            np.array([_SENSOR_FACES[f]["center"] for f in _SENSOR_FACE_NAMES], dtype=np.float32)
            + _fsr_normals * _FSR_SURFACE_OFFSET
        )
        self._overlay_pressure_hs = np.array(
            [[_PRESSURE_RADIUS, _PRESSURE_RADIUS, _PRESSURE_THICKNESS]] * 3, dtype=np.float32
        )
        self._overlay_quats = [
            rr.Quaternion(xyzw=_SENSOR_FACES[f]["quaternion"]) for f in _SENSOR_FACE_NAMES
        ]
        # Capacitive touch: one disc per pad (14 total)
        # Shift each pad toward its face's FSR centre by one pad radius, then
        # push outward along the face normal to sit on the surface.
        raw_centers = np.array(_PAD_CENTERS, dtype=np.float32)
        face_centers_per_pad = np.array(
            [_SENSOR_FACES["right"]["center"]] * 4
            + [_SENSOR_FACES["left"]["center"]]  * 4
            + [_SENSOR_FACES["middle"]["center"]] * 6,
            dtype=np.float32,
        )
        to_center = face_centers_per_pad - raw_centers
        dist = np.linalg.norm(to_center, axis=1, keepdims=True)
        direction = np.where(dist > 1e-6, to_center / dist, np.float32(0.0))
        pad_radius = _PAD_HALF_SIZES[0][0]
        centers = raw_centers + direction * pad_radius
        # M2 (idx 10) and M3 (idx 11) are at the FSR height, so their shift is
        # purely in X and overshoots the column. Pin their X to match M0/M4 and M1/M5.
        centers[10, 0] = centers[8, 0]   # m2.x ← m0.x
        centers[11, 0] = centers[9, 0]   # m3.x ← m1.x
        # Pad 2 is the midpoint of pads 1 and 3.
        # Pad 0 is the reflection of pad 2 through the FSR centre.
        # Pads 1 and 3 are spaced the same distance from pad 2 as pad 0 is.
        fsr_r = np.array(_SENSOR_FACES["right"]["center"], dtype=np.float32)
        fsr_l = np.array(_SENSOR_FACES["left"]["center"],  dtype=np.float32)
        for i0, i1, i2, i3, fsr in ((3, 0, 1, 2, fsr_r), (7, 4, 5, 6, fsr_l)):
            centers[i2] = 0.5 * (centers[i1] + centers[i3])
            centers[i0] = 2.0 * fsr - centers[i2]
            spacing = float(np.linalg.norm(centers[i0] - centers[i2]))
            axis = centers[i3] - centers[i1]
            axis /= np.linalg.norm(axis)
            centers[i3] = centers[i2] + axis * spacing
            centers[i1] = centers[i2] - axis * spacing
        self._pad_centers_np = centers + _PAD_NORMAL_OFFSETS
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
        if self._pad_centers_np is None or not self._show_pad_labels:
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

    def toggle_pad_labels(self) -> None:
        """Toggle visibility of the pad-ID label overlays (bound to Shift+K)."""
        rr = self._rr
        if rr is None or self._pad_centers_np is None:
            return
        self._show_pad_labels = not self._show_pad_labels
        black = [0, 0, 0, 255]
        for mod in self._module_meta:
            mod_id = int(mod["id"])
            entity_base = self._entity_path_cache[mod_id]
            path = f"{entity_base}/sensor_overlay/pad_labels"
            if self._show_pad_labels:
                rr.log(path, rr.Points3D(
                    positions=self._pad_centers_np,
                    labels=_PAD_LABELS,
                    colors=[black] * len(_PAD_LABELS),
                    radii=[0.01] * len(_PAD_LABELS),
                ))
            else:
                rr.log(path, rr.Clear(recursive=False))

    def _log_touch_blob_spheres(self, rr, state: RobotState) -> None:
        """Log up to 2 spheres at the world-space centroid of each active touch blob.

        Modules are grouped into contiguous blobs; each blob produces one sphere.
        A blob is shown when the group contains at least 2 active cap pads in total
        (covering same-module and neighbouring-module adjacency).  Colour is red
        during a stroke, black otherwise.
        Logged at robot/touch_centroid/{0,1} so Rerun applies the robot entity
        transform and the spheres sit correctly in the 3D scene.
        """
        _MAX_BLOBS = 2

        # Per-module: active pad count and face-weighted world position
        module_data: dict[int, tuple[float, np.ndarray]] = {}
        module_pad_count: dict[int, int] = {}
        for mod in self._module_meta:
            mod_id = int(mod["id"])
            sens = state.sensors.get(mod_id)
            if sens is None:
                continue

            swap = self._overlay_swap_lr.get(mod_id, False)
            left_pads  = sens.touch_right_pads if swap else sens.touch_left_pads
            right_pads = sens.touch_left_pads  if swap else sens.touch_right_pads
            middle_pads = sens.touch_middle_pads

            # Pad weights in _PAD_CENTERS order: right[0-3], left[4-7], middle[8-13]
            w_pads = np.array(
                [v if v >= _BLOB_ACTIVATION_THRESHOLD else 0.0
                 for v in (*right_pads, *left_pads, *middle_pads)],
                dtype=np.float64,
            )
            n_active = int((w_pads > 0).sum())
            if n_active == 0:
                continue
            module_pad_count[mod_id] = n_active

            w_total = float(w_pads.sum())
            # Weighted centroid of active pad positions — may be inside the prism
            # when pads from multiple faces contribute.
            p_local = (_PAD_CENTERS_NP * w_pads[:, None]).sum(axis=0) / w_total
            # Blended outward normal direction
            n_blend = (_PAD_NORMALS_UNIT * w_pads[:, None]).sum(axis=0) / w_total
            n_norm = float(np.linalg.norm(n_blend))
            if n_norm < 1e-9:
                continue
            n_hat = n_blend / n_norm

            # Find how far p_local is below the prism surface in direction n_hat,
            # then offset by that amount plus _BLOB_SPHERE_OFFSET so the sphere
            # always sits outside the prism regardless of which faces are active.
            t_exit = 0.0
            for n_face, d_face in _PRISM_FACE_PLANES:
                denom = float(n_face @ n_hat)
                if denom > 1e-6:
                    t = (d_face - float(n_face @ p_local)) / denom
                    if t > t_exit:
                        t_exit = t

            local_pt = p_local + (t_exit + _BLOB_SPHERE_OFFSET) * n_hat

            pos_m, R_m = self._fk_to_module(mod_id, state)
            p3d = pos_m + R_m @ local_pt
            module_data[mod_id] = (w_total, p3d)

        # Group active modules into contiguous blobs; require >= 2 total active pads
        blobs: list[list[int]] = []
        current: list[int] = []
        for mod_id in sorted(module_data):
            if current and mod_id != current[-1] + 1:
                blobs.append(current)
                current = []
            current.append(mod_id)
        if current:
            blobs.append(current)
        blobs = [b for b in blobs if sum(module_pad_count.get(m, 0) for m in b) >= 2]

        if self._stroke_active:
            color = [220, 30, 30, 220]       # red
        elif self._hold_active:
            color = [137, 207, 240, 220]     # baby blue
        else:
            color = [0, 0, 0, 220]           # black
        for i in range(_MAX_BLOBS):
            path = f"robot/touch_centroid/{i}"
            if i < len(blobs):
                w_sum = np.zeros(3, dtype=np.float64)
                w_total = 0.0
                for mid in blobs[i]:
                    w, p3d = module_data[mid]
                    w_sum += w * p3d
                    w_total += w
                centroid = w_sum / w_total
                rr.log(path, rr.Points3D(
                    positions=[centroid.tolist()],
                    radii=[_BLOB_RADIUS_CM],
                    colors=[color],
                ))
            else:
                rr.log(path, rr.Clear(recursive=False))

    def _log_sensor_overlays(self, rr, state: RobotState) -> None:
        """
        Log per-pad capacitive touch discs and per-face FSR pressure discs for
        every module. Touch = 14 gold ellipsoids (one per cap pad); pressure = 3 olive
        ellipsoids (one per face). Alpha is proportional to sensor value.

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

            touch_colors = tuple(
                (*_TOUCH_COLOR_RGB, _MIN_ALPHA + int(min(1.0, max(0.0, v)) * (_MAX_ALPHA - _MIN_ALPHA)))
                for v in all_pad_vals
            )
            if self._prev_touch_colors.get(mod_id) != touch_colors:
                self._prev_touch_colors[mod_id] = touch_colors
                rr.log(f"{entity_base}/sensor_overlay/touch", rr.Ellipsoids3D(
                    centers=self._pad_centers_np,
                    half_sizes=self._pad_half_sizes_np,
                    quaternions=self._pad_quats,
                    colors=list(touch_colors),
                    fill_mode="solid",
                ))

            # FSR: one face-level disc per face (left/middle/right order)
            if swap_lr:
                pressure_vals = (sensors.pressure_right, sensors.pressure_middle, sensors.pressure_left)
            else:
                pressure_vals = (sensors.pressure_left, sensors.pressure_middle, sensors.pressure_right)
            pressure_colors = tuple(
                (*_PRESSURE_COLOR_RGB, _MIN_ALPHA + int(min(1.0, max(0.0, p)) * (_MAX_ALPHA - _MIN_ALPHA)))
                for p in pressure_vals
            )
            if self._prev_pressure_colors.get(mod_id) != pressure_colors:
                self._prev_pressure_colors[mod_id] = pressure_colors
                rr.log(f"{entity_base}/sensor_overlay/pressure", rr.Ellipsoids3D(
                    centers=self._overlay_centers,
                    half_sizes=self._overlay_pressure_hs,
                    quaternions=self._overlay_quats,
                    colors=list(pressure_colors),
                    fill_mode="solid",
                ))

    # ------------------------------------------------------------------
    # 3D setup and pose logging
    # ------------------------------------------------------------------

    def _load_assembly(self) -> None:
        if not os.path.isfile(self.assembly_file):
            logger.warning("[RerunVisualizer] Assembly file not found: %s", self.assembly_file)
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

        logger.info("[RerunVisualizer] Loaded assembly with %d modules", len(self._module_meta))

    def _fk_to_module(self, mod_id: int, state: RobotState) -> tuple[np.ndarray, np.ndarray]:
        """Cumulative FK position and rotation of mod_id in robot-local space."""
        pos = np.zeros(3, dtype=np.float64)
        R = np.eye(3, dtype=np.float64)
        for mid in self._ancestor_chain(mod_id):
            pos = pos + R @ np.array(self._module_offsets[mid], dtype=np.float64)
            angle_rad = -self._servo_angle_rad(mid, state)
            R = R @ (self._hpr_mats[mid].astype(np.float64)
                     @ _axis_angle_to_mat3(self._joint_axes[mid], angle_rad).astype(np.float64))
        return pos, R

    def _setup_3d_geometry(self) -> None:
        """Log static OBJ mesh for each module (once, time-independent)."""
        rr = self._rr
        if not self._module_meta:
            logger.warning("[RerunVisualizer] No assembly data — skipping 3D geometry")
            return

        for mod in self._module_meta:
            mod_id = int(mod["id"])
            mesh_path = self._entity_path(mod_id) + "/mesh"

            model_filename = mod.get("model", "")
            obj_path = os.path.join(self.models_dir, model_filename)
            if not os.path.isfile(obj_path):
                logger.warning("[RerunVisualizer] Model not found: %s", obj_path)
                rr.log(mesh_path, rr.Boxes3D(
                    half_sizes=[[3.5, 3.5, 3.59]],
                    colors=[[250, 245, 217, 220]],
                ), static=True)
            else:
                rr.log(mesh_path, rr.Asset3D(path=obj_path), static=True)

        self._log_imu_static(rr)

    def _log_imu_static(self, rr) -> None:
        """Log a static PCB slab and axes indicator for the BNO085 on module 7's back face."""
        if _IMU_MODULE_ID not in self._entity_path_cache:
            return
        base = self._entity_path_cache[_IMU_MODULE_ID]
        imu_path = f"{base}/imu"
        al = _IMU_AXIS_LENGTH

        # Child frame: translate to face centre + rotate so local-Z = face normal
        rr.log(imu_path, rr.Transform3D(
            translation=list(_IMU_CENTER),
            quaternion=rr.Quaternion(xyzw=list(_IMU_QUATERNION_XYZW)),
        ), static=True)

        # Thin PCB slab centred at the IMU child origin (flat in local XY plane)
        rr.log(f"{imu_path}/pcb", rr.Boxes3D(
            centers=[[0.0, 0.0, 0.0]],
            half_sizes=[list(_IMU_PCB_HALF_SIZES)],
            colors=[list(_IMU_COLOR)],
        ), static=True)

        # Coordinate axes: red=X, green=Y, blue=Z (in IMU local frame)
        rr.log(f"{imu_path}/axes", rr.Arrows3D(
            origins=[[0.0, 0.0, 0.0]] * 3,
            vectors=[[al, 0.0, 0.0], [0.0, al, 0.0], [0.0, 0.0, al]],
            colors=[[220, 50, 50, 255], [50, 200, 50, 255], [50, 100, 220, 255]],
            radii=[0.08] * 3,
        ), static=True)

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

        # Recompute the robot entity transform every tick so module 7 stays fixed
        # at the world origin (IMU at 0,0,0) regardless of joint angles.
        # Inverse of module 7's current world transform: R_robot = R7^T, t_robot = target - R7^T @ p7
        p7, R7 = self._fk_to_module(_IMU_MODULE_ID, state)
        target = np.array(_IMU_WORLD_TARGET, dtype=np.float64)
        rr.log("robot", rr.Transform3D(
            translation=(target - R7.T @ p7).tolist(),
            mat3x3=R7.T.tolist(),
        ))

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
