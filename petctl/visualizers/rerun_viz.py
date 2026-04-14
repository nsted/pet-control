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
# Quaternions orient the disc Z-axis onto the face normal (xyzw format):
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

_TOUCH_RADIUS       = 1.3   # cm — larger circle for touch
_PRESSURE_RADIUS    = 0.75  # cm — smaller circle for pressure
_DISC_THICKNESS     = 0.05  # cm
_PRESSURE_THICKNESS = 0.15  # cm — thicker so pressure surface sits in front of touch

# Base RGB colours; alpha is proportional to sensor level (min 30 so discs are
# always faintly visible even when sensors read zero)
_TOUCH_COLOR_RGB    = (50, 180, 220)   # cyan-blue
_PRESSURE_COLOR_RGB = (230, 100, 20)   # warm orange
_MIN_ALPHA = 30

_SENSOR_FACE_NAMES = ("left", "middle", "right")


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
        """Declare SeriesLines for capacitive (touch) and FSR (pressure) per module face."""
        rr = self._rr
        if not self._module_meta:
            return
        for mod in self._module_meta:
            mod_id = int(mod["id"])
            for face in _SENSOR_FACE_NAMES:
                cap_path = f"sensors/capacitive/{mod_id}_{face}"
                fsr_path = f"sensors/fsr/{mod_id}_{face}"
                label = f"mod{mod_id} {face}"
                rr.log(cap_path, rr.SeriesLines(names=f"touch {label}"), static=True)
                rr.log(fsr_path, rr.SeriesLines(names=f"pressure {label}"), static=True)

    def _log_sensor_face_series(self, rr, state: RobotState) -> None:
        """Log normalized touch (capacitive) and pressure (FSR) for every assembly module."""
        if not self._module_meta:
            return
        for mod in self._module_meta:
            mod_id = int(mod["id"])
            sens = state.sensors.get(mod_id)
            for face in _SENSOR_FACE_NAMES:
                if sens is not None:
                    tv = float(getattr(sens, f"touch_{face}", 0.0))
                    pv = float(getattr(sens, f"pressure_{face}", 0.0))
                else:
                    tv, pv = 0.0, 0.0
                rr.log(f"sensors/capacitive/{mod_id}_{face}", rr.Scalars(tv))
                rr.log(f"sensors/fsr/{mod_id}_{face}", rr.Scalars(pv))

    def _log_sensor_overlays(self, rr, state: RobotState) -> None:
        """
        Log two diffuse-coloured discs on each prism face (left / right / middle)
        for every module.  Touch = larger cyan disc; pressure = smaller orange disc on top.
        Alpha is proportional to the sensor value (0 → transparent, 1 → opaque).
        Discs are logged as children of the module's joint entity so they ride
        along automatically as the robot articulates.
        """
        for mod in self._module_meta:
            mod_id = int(mod["id"])
            sensors = state.sensors.get(mod_id)
            if sensors is None:
                continue

            entity_base = self._entity_path_cache.get(mod_id) or self._entity_path(mod_id)

            # Modules whose HPR rotation flips local X (R_hpr[0,0] < 0) display the
            # "left" disc on the visual right and vice-versa.  Odd modules are already
            # corrected by the backend swap, so they cancel out.  Even modules 2/4/6
            # have only the visual flip and need a left↔right read swap here.
            viz_x_flipped  = float(self._hpr_mats[mod_id][0, 0]) < 0
            backend_swapped = (mod_id % 2) == 1
            swap_lr = viz_x_flipped and not backend_swapped

            for face_name, face in _SENSOR_FACES.items():
                read_face = face_name
                if swap_lr and face_name in ("left", "right"):
                    read_face = "right" if face_name == "left" else "left"
                touch_val    = float(getattr(sensors, f"touch_{read_face}",    0.0))
                pressure_val = float(getattr(sensors, f"pressure_{read_face}", 0.0))

                center = face["center"]
                quat   = face["quaternion"]
                face_path = f"{entity_base}/sensor_overlay/{face_name}"

                touch_alpha    = _MIN_ALPHA + int(min(1.0, max(0.0, touch_val))    * (255 - _MIN_ALPHA))
                pressure_alpha = _MIN_ALPHA + int(min(1.0, max(0.0, pressure_val)) * (255 - _MIN_ALPHA))

                rr.log(f"{face_path}/touch", rr.Ellipsoids3D(
                    centers=[center],
                    half_sizes=[[_TOUCH_RADIUS, _TOUCH_RADIUS, _DISC_THICKNESS]],
                    quaternions=[rr.Quaternion(xyzw=quat)],
                    colors=[(*_TOUCH_COLOR_RGB, touch_alpha)],
                    fill_mode="solid",
                ))
                rr.log(f"{face_path}/pressure", rr.Ellipsoids3D(
                    centers=[center],
                    half_sizes=[[_PRESSURE_RADIUS, _PRESSURE_RADIUS, _PRESSURE_THICKNESS]],
                    quaternions=[rr.Quaternion(xyzw=quat)],
                    colors=[(*_PRESSURE_COLOR_RGB, pressure_alpha)],
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

        # Pre-compute entity path strings (avoids string joins at 20 Hz)
        self._entity_path_cache = {
            int(mod["id"]): self._entity_path(int(mod["id"]))
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
            joint_path = self._entity_path_cache.get(mod_id) or self._entity_path(mod_id)
            offset = mod.get("offset", [0.0, 0.0, 0.0])

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
