"""Target shapes (Regimen v2 shape-agnostic hug): welded vertical cylinders, boxes and rounded boxes of ``TARGET_HEIGHT_M``, with analytic signed distance and outward normal.

A shape is a set of static MuJoCo primitives (cylinder; box; rounded box = 2 boxes + 4 corner cylinders) laid out round the target centre and rotated by ``yaw``. The first
primitive is always centred on the target centre. ``sdf_normal`` gives the exact signed distance of a floor-plane point and the angle of the outward normal at the nearest
surface point (used by the hug metric's normal span and the approach reward).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

TARGET_HEIGHT_M = 0.4
POST_R_M = 0.05                     # the legacy 10 cm post


@dataclass(frozen=True)
class TargetShape:
    """``kind`` 'cyl' (``a`` = radius), 'box' (half extents ``a``, ``b`` along the local x, y) or 'rbox' (as box, corner radius ``rc``); ``yaw`` rotates it about the vertical axis."""

    kind: str
    a: float
    b: float = 0.0
    rc: float = 0.0
    yaw: float = 0.0

    @property
    def label(self) -> str:
        """Short description in centimetres."""
        if self.kind == "cyl":
            return f"cyl r{100 * self.a:.1f}"
        if self.kind == "box":
            return f"box {200 * self.a:.0f}x{200 * self.b:.0f}"
        return f"rbox {200 * self.a:.0f}x{200 * self.b:.0f} r{100 * self.rc:.1f}"

    @property
    def bound_radius(self) -> float:
        """Radius of the circle about the centre that contains the shape."""
        if self.kind == "cyl":
            return self.a
        if self.kind == "box":
            return math.hypot(self.a, self.b)
        return math.hypot(self.a - self.rc, self.b - self.rc) + self.rc

    def _rot(self, dx: float, dy: float) -> tuple[float, float]:
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return c * dx - s * dy, s * dx + c * dy

    def prims(self) -> list[tuple[str, float, float, tuple[float, ...]]]:
        """MuJoCo primitives ``(geom type, dx, dy, size)`` (offset from the centre in the world frame; box sizes are half extents incl. the half height; the box yaw is ``self.yaw``)."""
        h = TARGET_HEIGHT_M / 2
        if self.kind == "cyl":
            return [("cylinder", 0.0, 0.0, (self.a, h))]
        if self.kind == "box":
            return [("box", 0.0, 0.0, (self.a, self.b, h))]
        a, b, r = self.a, self.b, self.rc
        out: list[tuple[str, float, float, tuple[float, ...]]] = [("box", 0.0, 0.0, (a, b - r, h)), ("box", 0.0, 0.0, (a - r, b, h))]
        for sx in (-1.0, 1.0):
            for sy in (-1.0, 1.0):
                dx, dy = self._rot(sx * (a - r), sy * (b - r))
                out.append(("cylinder", dx, dy, (r, h)))
        return out

    def sdf_normal(self, p_rel: np.ndarray) -> tuple[float, float]:
        """Signed distance (m, negative inside) of the floor-plane point ``p_rel`` (relative to the centre) and the world angle of the outward normal there."""
        x, y = float(p_rel[0]), float(p_rel[1])
        if self.kind == "cyl":
            return math.hypot(x, y) - self.a, math.atan2(y, x)
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        lx, ly = c * x + s * y, -s * x + c * y                       # local frame
        hx, hy = self.a - self.rc, self.b - self.rc                    # core box half extents
        qx, qy = abs(lx) - hx, abs(ly) - hy
        ox, oy = max(qx, 0.0), max(qy, 0.0)
        outer = math.hypot(ox, oy)
        d = outer + min(max(qx, qy), 0.0) - self.rc
        if outer > 1e-12:
            nx, ny = ox / outer, oy / outer
        elif qx > qy:
            nx, ny = 1.0, 0.0
        else:
            nx, ny = 0.0, 1.0
        nx, ny = math.copysign(nx, lx) if nx else 0.0, math.copysign(ny, ly) if ny else 0.0
        return d, math.atan2(s * nx + c * ny, c * nx - s * ny)

    def dist_xy(self, pts_xy: np.ndarray, centre: np.ndarray) -> np.ndarray:
        """Distance (>= 0) of each floor-plane point to the shape surface (vectorised)."""
        rel = np.asarray(pts_xy, float) - np.asarray(centre, float)
        if self.kind == "cyl":
            return np.maximum(np.linalg.norm(rel, axis=1) - self.a, 0.0)
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        lx, ly = c * rel[:, 0] + s * rel[:, 1], -s * rel[:, 0] + c * rel[:, 1]
        qx, qy = np.abs(lx) - (self.a - self.rc), np.abs(ly) - (self.b - self.rc)
        d = np.hypot(np.maximum(qx, 0.0), np.maximum(qy, 0.0)) + np.minimum(np.maximum(qx, qy), 0.0) - self.rc
        return np.maximum(d, 0.0)


def shape_of(post_shape: TargetShape | None) -> TargetShape:
    """The shape of an episode: its ``TargetShape`` or the legacy 5 cm-radius post."""
    return post_shape if post_shape is not None else TargetShape("cyl", POST_R_M)


def span_deg(angles: np.ndarray) -> float:
    """Unwrapped angular span (degrees, max - min) of a chain-ordered sequence of angles (< 2 values: 0)."""
    a = np.asarray(angles, float)
    if a.size < 2:
        return 0.0
    u = np.unwrap(a)
    return float(np.rad2deg(u.max() - u.min()))
