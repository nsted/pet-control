"""Visual-only scene markers (e.g. the goal ball) for rendered videos and the interactive viewer. Markers never affect the physics."""

from __future__ import annotations

from typing import Any

import mujoco
import numpy as np

Marker = tuple[np.ndarray, float, tuple[float, float, float, float]]     # (world position, radius m, rgba)


def draw_markers(scene: Any, markers: list[Marker], reset: bool = False) -> None:
    """Append sphere markers to an ``mjvScene`` (``reset=True`` first clears it, for the viewer's persistent ``user_scn``)."""
    if reset:
        scene.ngeom = 0
    for pos, radius, rgba in markers:
        if scene.ngeom >= scene.maxgeom:
            break
        mujoco.mjv_initGeom(scene.geoms[scene.ngeom], mujoco.mjtGeom.mjGEOM_SPHERE, np.array([radius, 0.0, 0.0]), np.asarray(pos, float),
                            np.eye(3).flatten(), np.asarray(rgba, np.float32))
        scene.ngeom += 1
