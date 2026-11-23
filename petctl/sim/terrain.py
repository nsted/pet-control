"""Per-episode rough terrain for the MuJoCo model: a heightfield plus a 3 x 3 grid of friction tiles (Regimen v2 plan, section 5.1).

The terrain is an 8 x 8 m patch sampled on a 2.5 cm grid (321 x 321 points) centred on the world origin, where the robot starts. Heights are
non-negative: smooth Gaussian bumps (<= 6 cm), 1-3 planar slope patches (<= 15 deg, tanh-blended), and <= 1 cm roughness on "loose" friction tiles,
all scaled by a single ramp knob ``level`` in [0, 1] (0 = flat). A flat disk around the start (and a taper at the border) keeps the start and the edge at
height 0. Generation is deterministic in ``(level, seed, start_radius)``.

The height array is the source of truth: ``Terrain.height_at`` looks heights up for placing obstacles and the post, and ``build_mjcf`` writes the
same array into ``model.hfield_data`` (one hfield asset per friction tile, sharing edge samples).
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

SIZE_M = 8.0                    # side of the square patch (centred on the origin)
CELL_M = 0.025
N_PTS = int(round(SIZE_M / CELL_M)) + 1          # 321
HALF_M = SIZE_M / 2.0
ELEV_MAX_M = 0.5                # hfield elevation range [0, ELEV_MAX_M] (data are normalised by it)
BASE_M = 0.05                   # hfield base depth below z = 0
BUMP_AMP_MAX_M = 0.06
BUMP_RADIUS_M = (0.08, 0.30)
SLOPE_MAX_DEG = 15.0
ROUGH_MAX_M = 0.01
START_RADIUS_M = 0.45
START_BLEND_M = 0.20            # smooth-step width from the flat disk to full terrain
EDGE_TAPER_M = 0.30
ROLL_AMP_MAX_M = 0.12           # rolling terrain: peak |height| about the baseline at level 1 (hills up, valleys down)
ROLL_BASE_M = 0.15              # baseline height of rolling terrain (the hfield cannot go below 0, so valleys are carved down from here); the start disk sits on it
ROLL_SIGMA_M = (0.15, 0.35, 0.70)   # smoothing scales of the three octaves (about 0.3 / 0.7 / 1.5 m features)
ROLL_WEIGHTS = (0.25, 1.0, 1.2)
ROLL_SLOPE_MAX_DEG = 25.0       # 99.5th percentile of the gradient (isolated steeper cells up to ~1.3x)
TILE_MU = (0.3, 1.5)
LOOSE_MU = (0.2, 0.4)
LOOSE_PROB = 0.3                # probability a tile is "loose" at level 1 (scaled by level)
TILE_BOUNDS = (0, 107, 214, N_PTS - 1)           # point indices where the 3 tiles meet (neighbouring tiles share the edge row/column)


@dataclass
class Terrain:
    """One episode's terrain. ``heights[iy, ix]`` (m) at ``x = -4 + ix * 2.5 cm``, ``y = -4 + iy * 2.5 cm``."""

    level: float
    seed: int
    start_radius: float
    heights: np.ndarray
    tile_mu: np.ndarray                          # (3, 3) [ty, tx] sampled friction coefficients (before blending with the floor friction)
    tile_loose: np.ndarray                       # (3, 3) bool
    parts: dict[str, np.ndarray] = field(default_factory=dict, repr=False)      # 'bump', 'slope', 'rough' height components (privileged / for tests)

    @property
    def max_height(self) -> float:
        """Highest point of the terrain (m)."""
        return float(self.heights.max())

    def height_at(self, x: float | np.ndarray, y: float | np.ndarray) -> np.ndarray:
        """Bilinear terrain height (m) at world ``(x, y)``; 0 outside the patch."""
        x, y = np.asarray(x, float), np.asarray(y, float)
        fx, fy = (x + HALF_M) / CELL_M, (y + HALF_M) / CELL_M
        inside = (fx >= 0) & (fx <= N_PTS - 1) & (fy >= 0) & (fy <= N_PTS - 1)
        fx, fy = np.clip(fx, 0, N_PTS - 1 - 1e-9), np.clip(fy, 0, N_PTS - 1 - 1e-9)
        ix, iy = fx.astype(int), fy.astype(int)
        tx, ty = fx - ix, fy - iy
        h = self.heights
        v = (h[iy, ix] * (1 - tx) * (1 - ty) + h[iy, ix + 1] * tx * (1 - ty) + h[iy + 1, ix] * (1 - tx) * ty + h[iy + 1, ix + 1] * tx * ty)
        return np.where(inside, v, 0.0)

    def support_height(self, x: float, y: float, radius: float = 0.0) -> float:
        """Lowest terrain height under the disc of ``radius`` about ``(x, y)`` (centre + 8 ring points): where an object's base can rest without floating."""
        if radius <= 0.0:
            return float(self.height_at(x, y))
        ang = np.linspace(0.0, 2.0 * np.pi, 8, endpoint=False)
        px = np.concatenate([[x], x + radius * np.cos(ang)])
        py = np.concatenate([[y], y + radius * np.sin(ang)])
        return float(np.min(self.height_at(px, py)))

    def footprint_height(self, radius: float) -> float:
        """Highest terrain point within ``radius`` of the origin (m): the lift needed so a body lying there starts above the surface."""
        xs = -HALF_M + CELL_M * np.arange(N_PTS)
        return float(self.heights[np.hypot(*np.meshgrid(xs, xs)) <= radius].max())

    def tile_friction(self, floor_friction: float) -> np.ndarray:
        """(3, 3) tile friction: the floor friction blended towards the sampled tile value by ``level`` (level 0 = uniform floor friction)."""
        return floor_friction + self.level * (self.tile_mu - floor_friction)

    def max_slope_deg(self, part: str = "slope") -> float:
        """Steepest gradient (degrees) of one height component (default: the planar slope patches)."""
        gy, gx = np.gradient(self.parts[part], CELL_M)
        return float(np.degrees(np.arctan(np.hypot(gx, gy).max())))


def tile_slices() -> list[tuple[int, int]]:
    """Point index ranges ``(lo, hi)`` (inclusive of both) of the 3 tiles along one axis."""
    return [(TILE_BOUNDS[i], TILE_BOUNDS[i + 1]) for i in range(3)]


def _smoothstep(t: np.ndarray) -> np.ndarray:
    t = np.clip(t, 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)


def _rolling_heights(level: float, rng: np.random.Generator, X: np.ndarray, mask: np.ndarray) -> np.ndarray:
    """Signed hills-and-valleys relief about 0: weighted octaves of smoothed noise, peak ``ROLL_AMP_MAX_M * level``, gradient capped at ``ROLL_SLOPE_MAX_DEG * level``, zero on the start disk."""
    from scipy import ndimage

    field = np.zeros_like(X)
    for sig, w in zip(ROLL_SIGMA_M, ROLL_WEIGHTS):
        n = ndimage.gaussian_filter(rng.normal(size=X.shape), sigma=sig / CELL_M, mode="reflect")
        field += w * n / max(float(n.std()), 1e-9)
    field *= mask
    typ = float(np.percentile(np.abs(field[mask > 0.99]), 98)) if (mask > 0.99).any() else 0.0     # typical peak: scaling by the single maximum left most hills small
    if typ <= 0.0 or level <= 0.0:
        return np.zeros_like(X)
    field = np.clip(field * ROLL_AMP_MAX_M * level / typ, -0.95 * ROLL_BASE_M, 0.95 * ROLL_BASE_M)      # 98th percentile of |height| = amplitude; never below the hfield floor
    gy, gx = np.gradient(field, CELL_M)
    g, lim = float(np.percentile(np.hypot(gx, gy), 99.5)), float(np.tan(np.radians(ROLL_SLOPE_MAX_DEG * level)))
    if g > lim:
        field *= lim / g
    return field


def generate_terrain(level: float, seed: int, start_radius: float = START_RADIUS_M, rolling: bool = False) -> Terrain:
    """Sample a terrain. ``level`` in [0, 1] scales bump height, slope angle, roughness, bump count and loose-tile probability (0 = flat).

    ``rolling=True`` samples hills and valleys instead (``_rolling_heights``): signed, three-octave smooth relief over the whole patch about a baseline of ``ROLL_BASE_M``,
    peak +-``ROLL_AMP_MAX_M`` x level, slopes up to ``ROLL_SLOPE_MAX_DEG`` x level, the start disk flat on the baseline; friction tiles and loose-patch roughness as before.
    """
    from scipy import ndimage

    level = float(np.clip(level, 0.0, 1.0))
    rng = np.random.default_rng(int(seed))
    xs = -HALF_M + CELL_M * np.arange(N_PTS)
    X, Y = np.meshgrid(xs, xs)                                   # [iy, ix]
    r0 = np.hypot(X, Y)
    edge = np.minimum(HALF_M - np.abs(X), HALF_M - np.abs(Y))
    mask = _smoothstep((r0 - start_radius) / START_BLEND_M) * _smoothstep(edge / EDGE_TAPER_M)

    # friction tiles (always drawn so the rng stream does not depend on level)
    mu = rng.uniform(*TILE_MU, (3, 3))
    loose = rng.random((3, 3)) < LOOSE_PROB * level
    mu = np.where(loose, rng.uniform(*LOOSE_MU, (3, 3)), mu)

    bump = np.zeros_like(X)
    for _ in range(int(round(25 * level))):
        cx, cy = rng.uniform(-3.5, 3.5, 2)
        amp, rad = rng.uniform(0.3, 1.0) * BUMP_AMP_MAX_M * level, rng.uniform(*BUMP_RADIUS_M)
        bump += amp * np.exp(-((X - cx) ** 2 + (Y - cy) ** 2) / (2.0 * rad ** 2))
    bump *= mask
    if bump.max() > BUMP_AMP_MAX_M * level:
        bump *= BUMP_AMP_MAX_M * level / bump.max()

    slope = np.zeros_like(X)
    n_slopes = 1 + int(level * 2.999) if level > 0 else 0
    for _ in range(n_slopes):
        c = rng.uniform(-2.5, 2.5, 2)
        yaw, rad = rng.uniform(0.0, 2.0 * np.pi), rng.uniform(0.6, 1.0)
        tan_a = np.tan(np.radians(rng.uniform(0.4, 1.0) * SLOPE_MAX_DEG * level))
        t = (X - c[0]) * np.cos(yaw) + (Y - c[1]) * np.sin(yaw)
        plane = tan_a * (np.clip(t, -rad / 2, rad / 2) + rad / 2)         # ramp of length ``rad`` then a plateau
        w = 0.5 * (1.0 - np.tanh((np.hypot(X - c[0], Y - c[1]) - rad) / (0.3 * rad)))
        slope += w * plane
    slope *= mask
    if n_slopes:
        gy, gx = np.gradient(slope, CELL_M)
        g = float(np.hypot(gx, gy).max())
        lim = float(np.tan(np.radians(SLOPE_MAX_DEG * level)))
        if g > lim:                                                       # blending and overlaps can steepen the slope: rescale to the limit
            slope *= lim / g

    noise = ndimage.gaussian_filter(rng.normal(size=X.shape), sigma=2.0)
    noise = 0.5 + 0.5 * noise / max(float(np.abs(noise).max()), 1e-9)       # in [0, 1]
    loose_cells = np.zeros_like(X)
    for ty, (ylo, yhi) in enumerate(tile_slices()):
        for tx, (xlo, xhi) in enumerate(tile_slices()):
            if loose[ty, tx]:
                loose_cells[ylo:yhi + 1, xlo:xhi + 1] = 1.0
    rough = ROUGH_MAX_M * level * noise * ndimage.gaussian_filter(loose_cells, sigma=4.0) * mask

    if rolling:
        roll = _rolling_heights(level, rng, X, mask)
        heights = ROLL_BASE_M * (level > 0.0) + roll + rough
        return Terrain(level, int(seed), float(start_radius), heights, mu, loose, {"bump": np.zeros_like(X), "slope": roll, "rough": rough, "roll": roll})
    heights = bump + slope + rough
    return Terrain(level, int(seed), float(start_radius), heights, mu, loose, {"bump": bump, "slope": slope, "rough": rough})
