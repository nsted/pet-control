"""Per-episode rough terrain for the MuJoCo model: a heightfield plus a 3 x 3 grid of friction tiles (Regimen v2 plan, section 5.1).

The terrain is an 8 x 8 m patch sampled on a 2.5 cm grid (321 x 321 points) centred on the world origin, where the robot starts. Heights are
non-negative: smooth Gaussian bumps (<= 6 cm), 1-3 planar slope patches (<= 15 deg, tanh-blended), and <= 1 cm roughness on "loose" friction tiles,
all scaled by a single ramp knob ``level`` in [0, 1] (0 = flat). A flat disk around the start (and a taper at the border) keeps the start and the edge at
height 0. Generation is deterministic in ``(level, seed, start_radius)``.

The height array is the source of truth: ``Terrain.height_at`` looks heights up for placing obstacles and the post, and ``build_mjcf`` writes the
same array into ``model.hfield_data`` (one hfield asset per friction tile, sharing edge samples).
"""

from __future__ import annotations

import os

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
ROLL_AMP_MAX_M = 0.09           # rolling terrain: peak |height| about the baseline at level 1 (hills up, valleys down)
ROLL_BASE_M = 0.15              # baseline height of rolling terrain (the hfield cannot go below 0, so valleys are carved down from here); the start disk sits on it
ROLL_SIGMA_M = (0.15, 0.35, 0.70)   # smoothing scales of the three octaves (about 0.3 / 0.7 / 1.5 m features)
ROLL_WEIGHTS = (0.25, 1.0, 1.2)
ROLL_SLOPE_MAX_DEG = 20.0       # 99.5th percentile of the gradient (isolated steeper cells up to ~1.3x)
RIDGE_N = (4, 7)                # sawtooth ridges per patch at level 1 (scaled by level)
RIDGE_H_M = (0.06, 0.12)        # crest height above the surrounding ground, x level
RIDGE_UP_DEG = (30.0, 50.0)     # climb angle of the rising face, x level (the steep face the robot must climb)
RIDGE_DROP_M = (0.025, 0.075)   # horizontal run of the falling face (1-3 grid cells: a near-vertical step down a few cm after the crest)
RIDGE_CREST_M = 0.04
RIDGE_HALF_LEN_M = (0.9, 1.6)   # half-length along the ridge; the ends taper off so a ridge can be skirted but is cheaper to cross
RING_RADII_M = (0.95, 1.45, 1.95, 2.45)   # closed ridge rings around the start (all goals are 1.5-2.5 m out, so every route crosses at least two)
RING_JITTER_M = 0.08
RING_WOBBLE_M = 0.12            # angular wobble of each ring radius (two harmonics), so the rings are not perfect circles
DUNE_N = (5, 8)                 # dune components (close-packed asymmetric ripples at random orientations around a "wind" direction)
DUNE_WAVELENGTH_M = (0.28, 0.60)
DUNE_P2P_M = 0.09               # peak-to-peak dune height at level 1 (same scale as the ridge rings: 6-12 cm x level); scaled by level
DUNE_SKEW = (0.62, 0.80)        # fraction of each wavelength spent on the gentle windward face; the rest is the steep lee face
DUNE_LOOSE_PROB = 0.7           # sand: most friction tiles are loose (low friction) at level 1
DUNE_LOOSE_MU = (0.3, 0.5) if os.environ.get("PETCTL_DUNE_SOFT_FRICTION", "0") == "1" else (0.2, 0.4)     # default: the original approved sand friction; env 1 = the softer variant that did not help      # loose-sand friction (the generic LOOSE_MU 0.2-0.4 left the robot without traction: 2026-11-24 L0.3 start-stalls had centre-tile mu 0.40 vs 0.63)
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


def _ridge_heights(level: float, rng: np.random.Generator, X: np.ndarray, Y: np.ndarray, mask: np.ndarray) -> np.ndarray:
    """Sawtooth ridges: a steep rising face (30-50 deg x level), a short crest, then a near-vertical drop within a few cm (2.5-7.5 cm run). Zero on the start disk."""
    out = np.zeros_like(X)
    if level <= 0.0:
        return out
    for _ in range(max(1, int(round(rng.integers(RIDGE_N[0], RIDGE_N[1] + 1) * level)))):
        cx, cy = rng.uniform(-2.5, 2.5, 2)
        yaw, half = rng.uniform(0.0, 2.0 * np.pi), rng.uniform(*RIDGE_HALF_LEN_M)
        h = rng.uniform(*RIDGE_H_M) * level
        run = h / np.tan(np.radians(rng.uniform(RIDGE_UP_DEG[0], RIDGE_UP_DEG[1]) * level + (1.0 - level) * 10.0))
        drop = rng.uniform(*RIDGE_DROP_M)
        t = (X - cx) * np.cos(yaw) + (Y - cy) * np.sin(yaw)               # across the ridge (climb direction is +t)
        s_ = -(X - cx) * np.sin(yaw) + (Y - cy) * np.cos(yaw)             # along the ridge
        prof = np.interp(t, [-run, 0.0, RIDGE_CREST_M, RIDGE_CREST_M + drop], [0.0, h, h, 0.0], left=0.0, right=0.0)
        out = np.maximum(out, prof * 0.5 * (1.0 - np.tanh((np.abs(s_) - half) / 0.15)))
    return out * mask


def _ring_heights(level: float, rng: np.random.Generator, X: np.ndarray, Y: np.ndarray, mask: np.ndarray) -> np.ndarray:
    """Closed sawtooth ridge rings around the origin (``RING_RADII_M``, wobbly): moving outward the robot climbs a steep face (30-50 deg x level), crosses a short crest, then drops
    within a few cm. Unavoidable: every route to a goal beyond the first rings crosses them. Zero on the start disk."""
    out = np.zeros_like(X)
    if level <= 0.0:
        return out
    r, th = np.hypot(X, Y), np.arctan2(Y, X)
    for r0 in RING_RADII_M:
        wob = sum(rng.uniform(0.3, 1.0) * RING_WOBBLE_M * np.sin(k * th + rng.uniform(0.0, 2.0 * np.pi)) for k in (2, 3)) / 2.0
        rr = r0 + rng.uniform(-RING_JITTER_M, RING_JITTER_M) + wob
        h = rng.uniform(*RIDGE_H_M) * level
        run = h / np.tan(np.radians(rng.uniform(RIDGE_UP_DEG[0], RIDGE_UP_DEG[1]) * level + (1.0 - level) * 10.0))
        drop = rng.uniform(*RIDGE_DROP_M)
        out = np.maximum(out, np.interp(r - rr, [-run, 0.0, RIDGE_CREST_M, RIDGE_CREST_M + drop], [0.0, h, h, 0.0], left=0.0, right=0.0))
    return out * mask


def _dune_heights(level: float, rng: np.random.Generator, X: np.ndarray, Y: np.ndarray, mask: np.ndarray) -> np.ndarray:
    """Sandy-beach dune field: 5-8 close-packed asymmetric ripple trains (gentle windward face, steep lee face, 28-60 cm wavelength) at orientations within about +-40 deg of a wind
    direction plus a cross train, crests warped by smooth noise so they wander. Covers the whole patch (no flat ground) except the start disk; zero-mean, peak-to-peak ``DUNE_P2P_M * level``."""
    from scipy import ndimage

    if level <= 0.0:
        return np.zeros_like(X)
    wind = rng.uniform(0.0, np.pi)
    field = np.zeros_like(X)
    for i in range(int(rng.integers(DUNE_N[0], DUNE_N[1] + 1))):
        phi = wind + (rng.uniform(-0.7, 0.7) if i else np.pi / 2 + rng.uniform(-0.3, 0.3))
        lam, a = rng.uniform(*DUNE_WAVELENGTH_M), rng.uniform(*DUNE_SKEW)
        warp = ndimage.gaussian_filter(rng.normal(size=X.shape), sigma=0.25 / CELL_M)
        warp *= 0.18 / max(float(np.abs(warp).max()), 1e-9)                              # crest wander: up to ~18 cm
        w = ((X * np.cos(phi) + Y * np.sin(phi)) / lam + warp / lam + rng.uniform()) % 1.0
        field += rng.uniform(0.5, 1.0) * np.where(w < a, w / a, (1.0 - w) / (1.0 - a))   # asymmetric sawtooth in [0, 1]
    field = ndimage.gaussian_filter(field, sigma=1.2)                                    # soften the grid-scale kinks (3 cm)
    inner = mask > 0.99
    lo, hi = (float(np.percentile(field[inner], 1)), float(np.percentile(field[inner], 99))) if inner.any() else (0.0, 1.0)
    field = (field - 0.5 * (lo + hi)) / max(hi - lo, 1e-9) * DUNE_P2P_M * level
    return np.clip(field, -0.45 * ROLL_BASE_M, 0.45 * ROLL_BASE_M) * mask


def generate_terrain(level: float, seed: int, start_radius: float = START_RADIUS_M, rolling: bool = False, ridges: float | bool = False) -> Terrain:
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

    dunes = float(ridges) >= 3.0
    if dunes:                                                                                 # sand: mostly loose, low-friction tiles
        loose = rng.random((3, 3)) < DUNE_LOOSE_PROB * min(1.0, 0.3 + level)
        mu = np.where(loose, rng.uniform(*DUNE_LOOSE_MU, (3, 3)), mu)
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

    if dunes:
        d = _dune_heights(level, rng, X, Y, mask)
        heights = ROLL_BASE_M * (level > 0.0) + d + rough
        return Terrain(level, int(seed), float(start_radius), heights, mu, loose, {"bump": np.zeros_like(X), "slope": d, "rough": rough, "roll": d, "ridge": d})
    if rolling:
        roll = _rolling_heights(level, rng, X, mask)
        if float(ridges) >= 2.0:                                                                  # 2: closed rings around the start (unavoidable); 1: finite line ridges
            ridge = _ring_heights(level, rng, X, Y, mask)
        else:
            ridge = _ridge_heights(level, rng, X, Y, mask) if ridges else np.zeros_like(X)      # drawn after the rolling field so rolling terrain keeps its rng stream
        heights = ROLL_BASE_M * (level > 0.0) + roll + ridge + rough
        if ridges:
            heights = np.minimum(heights, ELEV_MAX_M - 0.01)
        return Terrain(level, int(seed), float(start_radius), heights, mu, loose, {"bump": np.zeros_like(X), "slope": roll, "rough": rough, "roll": roll, "ridge": ridge})
    heights = bump + slope + rough
    return Terrain(level, int(seed), float(start_radius), heights, mu, loose, {"bump": bump, "slope": slope, "rough": rough})
