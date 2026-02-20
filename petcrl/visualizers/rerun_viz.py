"""
RerunVisualizer — real-time visualization via Rerun.io.

Displays two things simultaneously in the Rerun viewer:
  1. Sensor time-series (touch + pressure per module) as live charts
  2. 3D robot pose from actual OBJ mesh files, forward-kinematics driven

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
from typing import Dict, List, Optional, TYPE_CHECKING

import numpy as np

from petcrl.protocols import Visualizer
from petcrl.types import RobotState, ServoCommand

if TYPE_CHECKING:
    from petcrl.controller import Controller

# petcrl-local assembly (source of truth for module count / geometry)
_DEFAULT_ASSEMBLY = os.path.normpath(os.path.join(
    os.path.dirname(__file__),
    "../assets/robot_assembly.json",
))

# 3D model files live in grapple_ai (not duplicated into petcrl)
_MODELS_DIR = os.path.normpath(os.path.join(
    os.path.dirname(__file__),
    "../../../python/grapple_ai/3d_models/prisms",
))


class RerunVisualizer(Visualizer):
    """
    Visualizes sensor data and robot pose in Rerun.

    Args:
        app_name:       Rerun application name (shown in viewer title bar)
        assembly_file:  Path to robot_assembly.json.  Defaults to petcrl/assets one.
        models_dir:     Directory containing the .obj files.
        show_sensors:   Log sensor time-series charts (default: True)
        show_3d:        Log 3D robot pose (default: True)
    """

    name = "rerun"

    def __init__(
        self,
        app_name: str = "petcrl",
        assembly_file: Optional[str] = None,
        models_dir: Optional[str] = None,
        show_sensors: bool = True,
        show_3d: bool = True,
    ) -> None:
        self.app_name = app_name
        self.assembly_file = assembly_file or _DEFAULT_ASSEMBLY
        self.models_dir = models_dir or _MODELS_DIR
        self.show_sensors = show_sensors
        self.show_3d = show_3d

        self._module_meta: List[dict] = []
        # Pre-computed HPR rotation matrices keyed by module_id
        self._hpr_mats: Dict[int, np.ndarray] = {}
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
        if self.show_3d:
            self._setup_3d_geometry()

    def update(self, state: RobotState) -> None:
        if self._rr is None:
            return
        rr = self._rr
        rr.set_time("time", timestamp=state.timestamp)
        if self.show_sensors:
            self._log_sensors(rr, state)
        if self.show_3d:
            self._log_3d_pose(rr, state)

    def on_stop(self) -> None:
        pass  # Rerun viewer stays open after the script exits (by design)

    # ------------------------------------------------------------------
    # Sensor logging
    # ------------------------------------------------------------------

    def _log_sensors(self, rr, state: RobotState) -> None:
        for mod_id, sensors in state.sensors.items():
            base = f"sensors/module_{mod_id}"
            rr.log(f"{base}/touch/middle",    rr.Scalars(sensors.touch_middle))
            rr.log(f"{base}/touch/left",      rr.Scalars(sensors.touch_left))
            rr.log(f"{base}/touch/right",     rr.Scalars(sensors.touch_right))
            rr.log(f"{base}/pressure/middle", rr.Scalars(sensors.pressure_middle))
            rr.log(f"{base}/pressure/left",   rr.Scalars(sensors.pressure_left))
            rr.log(f"{base}/pressure/right",  rr.Scalars(sensors.pressure_right))
            rr.log(f"{base}/touch/total",     rr.Scalars(sensors.touch_total))
            rr.log(f"{base}/pressure/total",  rr.Scalars(sensors.pressure_total))

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

        # Pre-compute HPR rotation matrices (used each tick in _log_3d_pose)
        for mod in self._module_meta:
            mod_id = int(mod["id"])
            euler = mod.get("rotation", [0.0, 0.0, 0.0])
            self._hpr_mats[mod_id] = _hpr_to_mat3(*euler)

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
            joint_path = self._entity_path(mod_id)
            offset = mod.get("offset", [0.0, 0.0, 0.0])

            joint_axis = mod.get("joint", {}).get("axis", [0, 0, 1])
            servo_id = mod_id + 1
            angle_rad = self._servo_angle_rad(servo_id, state)

            R = self._hpr_mats[mod_id] @ _axis_angle_to_mat3(joint_axis, angle_rad)

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

    def _ancestor_chain(self, mod_id: int) -> List[int]:
        """Return module IDs from root to mod_id (inclusive)."""
        parent_map: Dict[int, Optional[int]] = {}
        for mod in self._module_meta:
            mid = int(mod["id"])
            parent = mod.get("parent")
            parent_map[mid] = int(parent) if parent is not None else None

        chain: List[int] = []
        current: Optional[int] = mod_id
        while current is not None:
            chain.append(current)
            current = parent_map.get(current)
        chain.reverse()
        return chain

    @staticmethod
    def _servo_angle_rad(servo_id: int, state: RobotState) -> float:
        raw = state.servo_positions.get(servo_id, 2048)
        return ServoCommand.position_to_radians(raw)


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


def _axis_angle_to_mat3(axis, angle_rad: float) -> np.ndarray:
    """Rodrigues' rotation formula: rotation by angle_rad around unit axis."""
    ax, ay, az = [float(v) for v in axis]
    norm = math.sqrt(ax*ax + ay*ay + az*az)
    if norm < 1e-9:
        return np.eye(3, dtype=np.float32)
    ax, ay, az = ax/norm, ay/norm, az/norm

    c = math.cos(angle_rad)
    s = math.sin(angle_rad)
    t = 1.0 - c

    return np.array([
        [t*ax*ax + c,    t*ax*ay - s*az, t*ax*az + s*ay],
        [t*ax*ay + s*az, t*ay*ay + c,    t*ay*az - s*ax],
        [t*ax*az - s*ay, t*ay*az + s*ax, t*az*az + c   ],
    ], dtype=np.float32)
