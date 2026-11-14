"""Sensor-face geometry of the PET modules (Regimen v2 plan, sec 5.4).

Every module hull is a triangular prism. Its two slanted facets (normals ~45 deg from the chain axis x) are the faces the neighbouring modules join
through; the three remaining facets, whose normals are perpendicular to the chain axis, carry the capacitive pads:
``middle`` = the large flat facet (normal along +-z), ``left`` / ``right`` = the two triangular caps (normals +y / -y). Labels are in the
body frame as ``RobotBackend`` presents it (the odd-module left/right swap and pad reversals are hardware-side, NOT applied here).

The returned normals are fixed in each module's body frame (outward); rotate with ``data.xmat[body]`` for the world frame.
"""

from __future__ import annotations

import hashlib

import mujoco
import numpy as np
from scipy.spatial import ConvexHull

FACE_NAMES = ("left", "right", "middle")
LEFT, RIGHT, MIDDLE = 0, 1, 2
PERP_TOL = 0.3                 # |n . chain_axis| below this = "perpendicular to the chain axis"
GROUP_COS = np.cos(np.deg2rad(5.0))

_CACHE: dict[str, np.ndarray] = {}


def _hull_vertices_body(model: mujoco.MjModel, k: int) -> np.ndarray:
    """Vertices of module ``k``'s collision hull in its body frame, shape (m, 3)."""
    g = model.geom(f"g{k}")
    ms = int(g.dataid[0])
    v = model.mesh_vert[model.mesh_vertadr[ms]:model.mesh_vertadr[ms] + model.mesh_vertnum[ms]].astype(float)
    r = np.zeros(9)
    mujoco.mju_quat2Mat(r, np.asarray(g.quat, float))
    return np.asarray(g.pos, float) + v @ r.reshape(3, 3).T


def _facets(v: np.ndarray) -> list[tuple[np.ndarray, float]]:
    """Planar facets of the convex hull of ``v`` as (outward unit normal, area), coplanar triangles merged (5 deg)."""
    h = ConvexHull(v)
    m = len(h.simplices)
    gsum, garea, gnorm = np.zeros((m, 3)), np.zeros(m), np.ones(m)             # running normal sum, area and |normal sum| of each group (first-match, in simplex order)
    ng = 0
    for s, e in zip(h.simplices, h.equations):
        area = 0.5 * float(np.linalg.norm(np.cross(v[s[1]] - v[s[0]], v[s[2]] - v[s[0]])))
        n = e[:3] / np.linalg.norm(e[:3])
        if ng:
            hit = np.flatnonzero(gsum[:ng] @ n / gnorm[:ng] > GROUP_COS)
            if hit.size:
                i = int(hit[0])
                gsum[i] = gsum[i] + n * area
                garea[i] += area
                gnorm[i] = np.linalg.norm(gsum[i])
                continue
        gsum[ng], garea[ng], gnorm[ng] = n * area, area, np.linalg.norm(n * area)
        ng += 1
    return [(gsum[i] / np.linalg.norm(gsum[i]), float(garea[i])) for i in range(ng)]


def module_faces(model: mujoco.MjModel, n_modules: int | None = None) -> np.ndarray:
    """Outward face normals in each module's body frame, shape (n_modules, 3, 3), face order (left, right, middle).

    Mesh hulls: the three largest facets whose normals are perpendicular to the chain axis (x). Box collision models (no URDF): the box faces.
    """
    n = int(n_modules if n_modules is not None else model.nbody - 1)
    ids = [model.geom(f"g{k}") for k in range(n)]
    h = hashlib.md5(np.ascontiguousarray(model.mesh_vert).tobytes() + np.concatenate([np.concatenate([g.pos, g.quat]) for g in ids]).tobytes() + bytes([n])).hexdigest()
    if h in _CACHE:
        return _CACHE[h].copy()
    out = np.zeros((n, 3, 3))
    for k in range(n):
        if model.geom(f"g{k}").type[0] != mujoco.mjtGeom.mjGEOM_MESH:
            out[k] = [[0, 1, 0], [0, -1, 0], [0, 0, -1 if k % 2 == 0 else 1]]
            continue
        cand = [(nv, a) for nv, a in _facets(_hull_vertices_body(model, k)) if abs(nv[0]) < PERP_TOL]
        cand = sorted(cand, key=lambda t: -t[1])[:3]
        if len(cand) != 3:
            raise ValueError(f"module {k}: found {len(cand)} facets perpendicular to the chain axis, expected 3")
        mid = max(cand, key=lambda t: abs(t[0][2]))[0]
        sides = [nv for nv, _ in cand if nv is not mid]
        left = [s for s in sides if s[1] > 0]
        right = [s for s in sides if s[1] < 0]
        if len(left) != 1 or len(right) != 1:
            raise ValueError(f"module {k}: could not tell left from right faces")
        out[k] = [left[0], right[0], mid]
    _CACHE[h] = out
    return out.copy()
