"""
RerunVisualizer — real-time visualization via Rerun.io.

Displays two things simultaneously in the Rerun viewer:
  1. Sensor time-series (touch + pressure per module) as live charts
  2. 3D robot pose from actual OBJ mesh files, forward-kinematics driven

3D hierarchy per module:
  robot/module_0                ← Transform3D: link offset + joint angle (dynamic)
  robot/module_0/mesh           ← Asset3D + Transform3D: initial mesh rotation (static)
  robot/module_0/module_1       ← Transform3D: link offset + joint angle (dynamic)
  robot/module_0/module_1/mesh  ← Asset3D + Transform3D: initial mesh rotation (static)
  ...

Rerun runs in its own separate process (spawned automatically).
There are no threading conflicts with asyncio.

Install: pip install rerun-sdk

Usage:
  viz = RerunVisualizer()
  ctrl = Controller(backend=..., scheme=..., visualizers=[viz])
  await ctrl.run()
"""

from __future__ import annotations

import json
import math
import os
from typing import Dict, List, Optional, Tuple, TYPE_CHECKING

import numpy as np
from rerun.datatypes import RotationAxisAngle, Angle

from petcrl.protocols import Visualizer
from petcrl.types import RobotState, ServoCommand

if TYPE_CHECKING:
    from petcrl.controller import Controller

# Paths to robot_assembly.json and 3D model files from grapple_ai
_GRAPPLE_AI = os.path.normpath(os.path.join(
    os.path.dirname(__file__),
    "../../../python/grapple_ai",
))
_DEFAULT_ASSEMBLY = os.path.join(_GRAPPLE_AI, "robot_assembly.json")
_MODELS_DIR = os.path.join(_GRAPPLE_AI, "3d_models/prisms")


class RerunVisualizer(Visualizer):
    """
    Visualizes sensor data and robot pose in Rerun.

    Shows the actual OBJ prism mesh files driven by the assembly JSON,
    with joint angles mapped from servo positions.

    Args:
        app_name:       Rerun application name (shown in viewer title bar)
        assembly_file:  Path to robot_assembly.json.  Defaults to grapple_ai one.
        models_dir:     Directory containing the .obj files.  Defaults to grapple_ai one.
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

        self._module_meta: List[dict] = []   # ordered list of module configs from JSON
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
        # Right-hand Z-up coordinate frame (matches assembly JSON)
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
        print(f"[RerunVisualizer] Loaded assembly with {len(self._module_meta)} modules")

    def _setup_3d_geometry(self) -> None:
        """
        Log static mesh geometry for each module (once, time-independent).

        Each module gets a two-level entity:
          robot/.../module_N        ← joint node (updated per-tick with Transform3D)
          robot/.../module_N/mesh   ← Asset3D + initial mesh rotation (static)
        """
        rr = self._rr
        if not self._module_meta:
            print("[RerunVisualizer] No assembly data — skipping 3D geometry")
            return

        for mod in self._module_meta:
            mod_id = int(mod["id"])
            joint_path = self._entity_path(mod_id)
            mesh_path = joint_path + "/mesh"

            # Resolve OBJ file path
            model_filename = mod.get("model", "")
            obj_path = os.path.join(self.models_dir, model_filename)
            if not os.path.isfile(obj_path):
                print(f"[RerunVisualizer] Model not found: {obj_path}")
                # Fall back to a box placeholder
                rr.log(mesh_path, rr.Boxes3D(
                    half_sizes=[[3.5, 3.5, 3.59]],
                    colors=[[250, 245, 217, 220]],
                ), static=True)
            else:
                rr.log(mesh_path, rr.Asset3D(path=obj_path), static=True)

            # Log initial mesh orientation from the JSON "rotation" field [H, P, R] degrees
            euler = mod.get("rotation", [0.0, 0.0, 0.0])
            initial_rot = _hpr_to_mat3(*euler)
            rr.log(mesh_path, rr.Transform3D(mat3x3=initial_rot), static=True)

    def _log_3d_pose(self, rr, state: RobotState) -> None:
        """
        Log each module's joint transform: link offset + joint angle rotation.

        Each module's Transform3D is relative to its parent in the entity
        hierarchy — Rerun composes them automatically for correct FK.
        """
        if not self._module_meta:
            return

        for mod in self._module_meta:
            mod_id = int(mod["id"])
            joint_path = self._entity_path(mod_id)

            # Link offset: translation from parent joint to this joint (cm)
            offset = mod.get("offset", [0.0, 0.0, 0.0])

            # Joint axis and current angle
            joint_info = mod.get("joint", {})
            joint_axis = joint_info.get("axis", [0, 0, 1])

            # servo_id = mod_id + 1 (servo 1 drives module 0's joint, etc.)
            servo_id = mod_id + 1
            angle_rad = self._servo_angle_rad(servo_id, state)

            rr.log(joint_path, rr.Transform3D(
                translation=offset,
                rotation_axis_angle=RotationAxisAngle(
                    axis=joint_axis,
                    angle=Angle(rad=angle_rad),
                ),
            ))

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _entity_path(self, mod_id: int) -> str:
        """
        Build the Rerun entity path for a module, preserving parent-child
        relationships from the assembly so Rerun composes transforms correctly.
        """
        if not self._module_meta:
            return f"robot/module_{mod_id}"
        chain = self._ancestor_chain(mod_id)
        parts = ["robot"] + [f"module_{m}" for m in chain]
        return "/".join(parts)

    def _ancestor_chain(self, mod_id: int) -> List[int]:
        """Return the list of module IDs from root to mod_id (inclusive)."""
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
        """Convert servo_id's raw position in state to radians."""
        raw = state.servo_positions.get(servo_id, 2048)
        return ServoCommand.position_to_radians(raw)


# ------------------------------------------------------------------
# Rotation math helpers
# ------------------------------------------------------------------

def _hpr_to_mat3(h_deg: float, p_deg: float, r_deg: float) -> np.ndarray:
    """
    Convert Panda3D HPR angles (degrees) to a 3x3 rotation matrix.

    Panda3D convention: Heading=Z, Pitch=X, Roll=Y, applied in H*P*R order.
    This matches how robot_assembly.json 'rotation' values were authored.
    """
    h = math.radians(h_deg)
    p = math.radians(p_deg)
    r = math.radians(r_deg)

    ch, sh = math.cos(h), math.sin(h)
    cp, sp = math.cos(p), math.sin(p)
    cr, sr = math.cos(r), math.sin(r)

    # Rz(h)
    Rz = np.array([[ch, -sh, 0],
                   [sh,  ch, 0],
                   [0,   0,  1]], dtype=np.float32)
    # Rx(p)
    Rx = np.array([[1,  0,   0 ],
                   [0,  cp, -sp],
                   [0,  sp,  cp]], dtype=np.float32)
    # Ry(r)
    Ry = np.array([[ cr, 0, sr],
                   [0,   1, 0 ],
                   [-sr, 0, cr]], dtype=np.float32)

    return Rz @ Rx @ Ry
