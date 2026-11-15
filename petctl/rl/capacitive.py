"""Post contact probe (ground truth) and the capacitive face-sensor model (Regimen v2 plan, sec 5.3/5.4, assumptions A1 and A4).

``PostProbe`` reads MuJoCo contacts between the post geom and the module hulls and assigns each to a module face (35 deg cone round the face normal).
It is privileged ground truth, used by the hug metric/reward and as the input of the sensor model. ``CapacitiveSensor`` turns it into what the
robot's face pads would report: touch bit, pressure, with a sample rate, latency, dropouts, false negatives/positives and pressure noise.
ONLY the post geom triggers either of them: the floor, obstacles, walls and the robot's own body never do (A1).
"""

from __future__ import annotations

from dataclasses import dataclass, field

import mujoco
import numpy as np

from petctl.rl.config import DomainRand
from petctl.rl.domain_rand import _u
from petctl.sim.faces import module_faces
from petctl.sim.model_builder import PetSim, post_geom_ids
from petctl.sim.shapes import shape_of, vnorm

FACE_CONE_DEG = 35.0
D_PROX_M = 0.015                 # ground-truth proximity: a face "touches" the post when its distance to the post surface is <= this (plan sec 5.3)
D_DETECT_M = 0.02                # distances are measured up to this (>= the largest per-episode sensor threshold)
PRESSURE_FULL_N = 2.0            # normal force (summed over a face) that reads as pressure 1.0
PRESSURE_LEVELS = 255            # 8-bit pressure
PRESSURE_ZERO_P = 0.3            # fraction of training episodes with the pressure block zeroed (FSRs may be deprecated)
EMA_TAU_S = 0.5                  # contact-memory time constant of the touch EMA (computed in ObsBuilder)


@dataclass
class PostRead:
    """Ground-truth post contacts of one tick (privileged)."""

    touch: np.ndarray            # (n, 3) bool, face contact (left, right, middle)
    force: np.ndarray            # (n, 3) normal force on each face from the post (N)
    any_contact: np.ndarray      # (n,) bool, any contact between the module hull and the post (including edges/end caps that map to no face)
    penetration: float           # deepest penetration into the post this tick (m, >= 0)
    face_dist: np.ndarray | None = None   # (n, 3) distance from each face to the post surface (m, 0 in contact, D_DETECT_M if farther); None = derived from ``touch``
    normal_ang: np.ndarray | None = None  # (n,) world angle (rad, about the vertical) of the outward object normal at the nearest surface point of each module's nearest face; NaN = none

    def __post_init__(self) -> None:
        """Derive ``face_dist`` from ``touch`` when not given (synthetic reads in tests)."""
        if self.face_dist is None:
            self.face_dist = np.where(self.touch, 0.0, D_DETECT_M)


class PostProbe:
    """Per-sim reader of post contacts; the result of one tick is computed once and shared (hug metric, capacitive sensor)."""

    def __init__(self, sim: PetSim) -> None:
        """Locate the post geom and the module hulls and identify the module faces."""
        m = sim.model
        self.sim = sim
        self.n = int(sim.p.n_modules)
        self.post_gids = post_geom_ids(m)                                     # all primitives of the target (cylinder / box / rounded box)
        self.post_gid = self.post_gids[0]                                     # the centred primitive
        self.shape = shape_of(sim.p.post_shape)
        self._reach = 0.25 + self.shape.bound_radius                          # modules farther than this (centre to target centre) cannot touch it
        self.faces = module_faces(m, self.n)                                   # (n, 3, 3) body frame
        self.hull_gids = [int(m.geom(f"g{k}").id) for k in range(self.n)]
        self.gid_to_k = {g: k for k, g in enumerate(self.hull_gids)}
        self.body_ids = [int(m.body(f"m{k}").id) for k in range(self.n)]
        self._cos = float(np.cos(np.deg2rad(FACE_CONE_DEG)))
        self._key: tuple[float, int, float, float, float] | None = None
        self._read: PostRead | None = None
        self._f6 = np.zeros(6)
        self._ft = np.zeros(6)
        self._body_arr = np.array(self.body_ids)
        self._facesT = [f.T for f in self.faces]                  # (3 x 3) per module, columns = face normals
        self._is_post = np.zeros(m.ngeom, bool)
        self._is_post[self.post_gids] = True

    @classmethod
    def of(cls, sim: PetSim) -> PostProbe | None:
        """The probe of ``sim`` (created on first use), or None if the sim has no post."""
        if sim.p.post is None:
            return None
        probe = getattr(sim, "_post_probe", None)
        if probe is None:
            probe = cls(sim)
            sim._post_probe = probe  # type: ignore[attr-defined]
        return probe  # type: ignore[no-any-return]

    def read(self) -> PostRead:
        """Face contacts with the post now (cached while the contact set is unchanged)."""
        d, m = self.sim.data, self.sim.model
        nc = int(d.ncon)
        gp = m.geom_pos[self.post_gid]
        key = (float(d.time), nc, float(d.contact.dist[:nc].sum()) if nc else 0.0, float(gp[0]), float(gp[1]))
        if key == self._key and self._read is not None:
            return self._read
        fdist = np.full((self.n, 3), D_DETECT_M)
        nang = np.full(self.n, np.nan)
        centre = d.geom_xpos[self.post_gid, :2]
        force = np.zeros((self.n, 3))
        anyc = np.zeros(self.n, bool)
        pen = 0.0
        ft = self._ft
        rel = d.xipos[self._body_arr, :2] - centre
        near = np.flatnonzero(~(np.hypot(rel[:, 0], rel[:, 1]) > self._reach))                # modules within reach of the target (centre to target centre)
        for k in near.tolist():                                                               # proximity: nearest hull point -> post surface, mapped to a face
            gid = self.hull_gids[k]
            dist, best = D_DETECT_M, None
            for pg in self.post_gids:                                                         # nearest primitive of the target
                dd = float(mujoco.mj_geomDistance(m, d, gid, pg, D_DETECT_M, ft))
                if 0.0 <= dd < dist:
                    dist, best = dd, ft.copy()
            if best is not None:
                v = best[3:] - best[:3]
                nv = vnorm(v)
                if nv > 1e-9:
                    R = d.xmat[self.body_ids[k]].reshape(3, 3)
                    dots = (R @ self._facesT[k]).T @ (v / nv)
                    f = int(np.argmax(dots))
                    if dots[f] >= self._cos:
                        if dist < fdist[k, f]:
                            nang[k] = self.shape.sdf_normal(best[:2] - centre)[1]
                        fdist[k, f] = min(fdist[k, f], dist)
        if nc:
            cg1, cg2 = d.contact.geom1[:nc], d.contact.geom2[:nc]
            for i in np.flatnonzero(self._is_post[cg1] | self._is_post[cg2]).tolist():       # contacts with the target only
                c = d.contact[i]
                g1, g2 = int(cg1[i]), int(cg2[i])
                p1, p2 = self._is_post[g1], self._is_post[g2]
                other = g2 if p1 else g1
                k = self.gid_to_k.get(other)
                if k is None:
                    continue
                anyc[k] = True
                pen = max(pen, -float(c.dist))
                n_to_post = np.asarray(c.frame[:3]) * (1.0 if p2 else -1.0)      # contact normal points geom1 -> geom2
                R = d.xmat[self.body_ids[k]].reshape(3, 3)
                dots = (R @ self._facesT[k]).T @ n_to_post                                    # cosine to each face's world outward normal
                f = int(np.argmax(dots))
                if dots[f] >= self._cos:
                    mujoco.mj_contactForce(m, d, i, self._f6)
                    fdist[k, f] = 0.0
                    nang[k] = self.shape.sdf_normal(np.asarray(c.pos[:2]) - centre)[1]
                    force[k, f] += max(float(self._f6[0]), 0.0)
        self._key, self._read = key, PostRead(fdist <= D_PROX_M, force, anyc, pen, fdist, nang)
        return self._read


@dataclass
class CapParams:
    """Per-episode capacitive sensor settings (A4: 10-20 Hz, 50-150 ms, 2-5 % dropout)."""

    rate_hz: float = 15.0
    latency_s: float = 0.10
    p_drop: float = 0.035            # whole sample lost (hold last)
    p_fn: float = 0.035              # per-face false negative
    p_fp: float = 0.005              # per-face false positive
    pressure_noise: float = 0.10     # relative (1 sigma)
    pressure_zero: bool = False      # pressure block reads zero this episode
    thresh_m: float = D_PROX_M       # proximity threshold of this episode's pads (the ground-truth metric stays at D_PROX_M)


def sample_cap_params(dr: DomainRand, rng: np.random.Generator, training: bool) -> CapParams:
    """Draw one episode's sensor settings from the ``DomainRand.cap_*`` ranges (shrunk by ``dr.scale``); pressure zeroed in 30 % of training episodes."""
    s = dr.scale
    p = CapParams(rate_hz=_u(rng, *dr.cap_rate_hz, s), latency_s=_u(rng, *dr.cap_latency_s, s), p_drop=_u(rng, *dr.cap_dropout, s),
                  p_fn=_u(rng, *dr.cap_fn, s), p_fp=_u(rng, *dr.cap_fp, s), pressure_noise=_u(rng, *dr.cap_pressure_noise, s))
    p.thresh_m = _u(rng, *dr.cap_thresh_m, s, D_PROX_M)
    p.pressure_zero = bool(training and s > 0.0 and rng.random() < PRESSURE_ZERO_P)
    return p


@dataclass
class CapacitiveSensor:
    """Touch/pressure pads of every module face, fed by ground-truth post contacts once per control tick.

    ``step(touch_gt, force)`` returns ``(touch (n,3) float 0/1, pressure (n,3) in [0,1])`` as the policy would see them at this tick: the newest sample
    that has arrived (sampled at ``rate_hz``, delivered ``latency_s`` later, whole samples dropped with ``p_drop`` and then the previous value is held).
    """

    n_modules: int
    dt: float
    params: CapParams
    rng: np.random.Generator
    enabled: bool = True
    _acc: float = field(default=0.0, init=False)
    _tick: int = field(default=0, init=False)
    _pending: list = field(default_factory=list, init=False)
    _out: tuple = field(init=False)
    n_samples: int = field(default=0, init=False)
    n_dropped: int = field(default=0, init=False)

    def __post_init__(self) -> None:
        """Start with an all-zero output and a random sampling phase."""
        self._acc = float(self.rng.random())
        self._out = (np.zeros((self.n_modules, 3)), np.zeros((self.n_modules, 3)))

    def step(self, touch_gt: np.ndarray, force: np.ndarray, face_dist: np.ndarray | None = None) -> tuple[np.ndarray, np.ndarray]:
        """Advance one control tick with the ground-truth post contacts; returns the sensor output.

        With ``face_dist`` (n, 3 distances to the post, m) the pads respond to proximity within this episode's ``params.thresh_m`` instead of ``touch_gt``.
        """
        if face_dist is not None:
            touch_gt = np.asarray(face_dist) <= self.params.thresh_m
        self._tick += 1
        p = self.params
        if not self.enabled:
            return self._out
        self._acc += p.rate_hz * self.dt
        if self._acc >= 1.0 - 1e-9:
            self._acc -= 1.0
            self.n_samples += 1
            if self.rng.random() < p.p_drop:
                self.n_dropped += 1
            else:
                t = np.asarray(touch_gt, bool) & (self.rng.random(touch_gt.shape) >= p.p_fn)
                t = t | (~np.asarray(touch_gt, bool) & (self.rng.random(touch_gt.shape) < p.p_fp))
                pr = np.clip(np.asarray(force, float) / PRESSURE_FULL_N, 0.0, 1.0) * np.asarray(touch_gt, bool)
                pr = np.clip(pr * (1.0 + p.pressure_noise * self.rng.normal(size=pr.shape)), 0.0, 1.0)
                pr = np.round(pr * PRESSURE_LEVELS) / PRESSURE_LEVELS * t
                if p.pressure_zero:
                    pr = np.zeros_like(pr)
                self._pending.append((self._tick + int(round(p.latency_s / self.dt)), t.astype(float), pr))
        while self._pending and self._pending[0][0] <= self._tick:
            _, t, pr = self._pending.pop(0)
            self._out = (t, pr)
        return self._out
