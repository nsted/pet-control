"""Static hug-feasibility search for arbitrary target shapes (Regimen v2, M1c; sim only). Generalises ``petctl.rl.hug_search`` (M1) to the shape-agnostic metric.

Candidate ``x = (q_1..q_{n-1}, dx, dy, alpha[, yaw])`` as in M1, plus the object's yaw for non-cylinders (its orientation relative to the body). Same protocol (lay the body down at
alpha*q, put the object at the fitted-circle centre + (dx, dy), ramp to q, settle 1 s, hold 3 s under the real PD actuators, torque cap and bus model), but with hug metric v2: H = 4 and
the wrap needs normal span >= 180 deg. Objective: maximise the HELD normal span (min over the 3 s hold of the span of the longest run, counted only while the run >= 4) subject to
run >= 4 held, penetration < 5 mm and the bus budget (soft penalties in the score). Absolute-target PD only (the delta action path comes later).
"""

from __future__ import annotations

import dataclasses
import math
import os
import time
from typing import Any

import mujoco  # type: ignore[import-untyped]
import numpy as np

from petctl.config import REGIMEN_BUS_MEAN_A, REGIMEN_BUS_PEAK_A
from petctl.rl.capacitive import PostProbe
from petctl.rl.config import BUS_A_PER_NM2, DEFAULT_URDF_DIR
from petctl.rl.curriculum.stages import REAL_CAP_NM
from petctl.rl.hug import HOLD_S, HugTracker
from petctl.rl.hug_search import ALPHA_BOUNDS, CONTROL_HZ, FREE_S, LIFT_M, POST_OFFSET_M, Q_BOUND_RAD, RAMP_S, SETTLE_S, SIM_DT, START_PEN_M, circle_centre
from petctl.sim.bus import BusClamp
from petctl.sim.model_builder import BuildParams, PetSim, post_geom_ids
from petctl.sim.shapes import TargetShape

H_RUN = 4
SPAN_DEG = 180.0
PEN_LIMIT_M = 0.005
Z_WEIGHT_TOUCH = 0.5


class ShapeSearchSim:
    """One reusable sim per (module count, shape): the target's pose is changed per candidate."""

    def __init__(self, n_modules: int, shape: TargetShape, urdf_dir: str = DEFAULT_URDF_DIR, cap_nm: float = REAL_CAP_NM) -> None:
        """Build the model with the shape (yaw 0) and the real cap / bus model."""
        self.n, self.shape0 = n_modules, dataclasses.replace(shape, yaw=0.0)
        self.sim = PetSim(BuildParams(n_modules=n_modules, urdf_dir=urdf_dir, dt=SIM_DT, cap_nm=(cap_nm,) * (n_modules - 1), post=(5.0, 5.0), post_shape=self.shape0))
        if os.environ.get("PETCTL_BUS_MOTOR", "gl40") == "gim3505":                   # SteadyWin GIM3505-8 current model (owner data, docs/plans/MOTOR_GIM3505.md); default: legacy GL40 II
            from petctl.config import GIM3505_HOLD_A_PER_NM2, GIM3505_IDLE_PER_MOTOR_A, GIM3505_MOVE_A_PER_NM2, GIM3505_MOVE_OMEGA0_RAD_S
            from petctl.sim.bus import IDLE_ELECTRONICS_A
            self.sim.bus = BusClamp(self.sim.model, GIM3505_HOLD_A_PER_NM2, REGIMEN_BUS_PEAK_A, clamp=True, move_coeff=GIM3505_MOVE_A_PER_NM2, move_omega0=GIM3505_MOVE_OMEGA0_RAD_S,
                                    idle_a=IDLE_ELECTRONICS_A + GIM3505_IDLE_PER_MOTOR_A * (n_modules - 1))
        else:
            self.sim.bus = BusClamp(self.sim.model, BUS_A_PER_NM2, REGIMEN_BUS_PEAK_A, clamp=True)
        self.probe = PostProbe.of(self.sim)
        assert self.probe is not None
        self.ids = post_geom_ids(self.sim.model)
        self.has_yaw = shape.kind != "cyl"

    def place(self, xy: np.ndarray | tuple[float, float], yaw: float) -> None:
        """Put the target's centre at ``xy`` with ``yaw`` (model edit; call ``mj_forward`` afterwards)."""
        sh = dataclasses.replace(self.shape0, yaw=float(yaw))
        for gid, (gtype, dx, dy, _) in zip(self.ids, sh.prims()):
            self.sim.model.geom_pos[gid, :2] = (xy[0] + dx, xy[1] + dy)
            if gtype == "box":
                self.sim.model.geom_quat[gid] = (math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2))
        self.probe.shape = sh
        self.sim.p.post = (float(xy[0]), float(xy[1]))

    def _centres(self) -> np.ndarray:
        return self.sim.data.xipos[1:self.n + 1].copy()

    def _settle_free(self, q: np.ndarray, yaw: float) -> np.ndarray:
        sim = self.sim
        self.place((5.0, 5.0), yaw)
        sim.reset(q)
        sim.data.qpos[2] += LIFT_M
        mujoco.mj_forward(sim.model, sim.data)
        sim.step(q, FREE_S)
        big = np.abs(q) > 0.4 * max(float(np.max(np.abs(q))), 1e-9)
        sel = np.zeros(self.n, bool)
        for j in np.flatnonzero(big):
            sel[j:j + 2] = True
        return circle_centre(self._centres()[sel, :2])

    def evaluate(self, x: np.ndarray, settle_s: float = SETTLE_S, hold_s: float = HOLD_S, record: list[np.ndarray] | None = None) -> dict[str, Any]:
        """Run the protocol for ``x``; returns a record with the score and the hold-window metrics."""
        sim, n = self.sim, self.n
        q = np.asarray(x[:n - 1], float)
        dx, dy, alpha = float(x[n - 1]), float(x[n]), float(x[n + 1])
        yaw = float(x[n + 2]) if self.has_yaw else 0.0
        post = self._settle_free(alpha * q, yaw) + np.array([dx, dy])
        self.place(post, yaw)
        mujoco.mj_forward(sim.model, sim.data)
        sim.bus.begin_tick()
        bad = {"score": -5.0, "held_run": 0, "held_span": 0.0, "peak_span": 0.0, "wrap_duty": 0.0, "mean_run": 0.0, "bus_mean_a": 0.0, "bus_applied_peak_a": 0.0, "pen_max_mm": 0.0,
               "feasible": False, "hold_ok": False, "x": [float(v) for v in x], "post_xy": [float(post[0]), float(post[1])], "yaw": yaw, "max_joint_rad": float(np.max(np.abs(q)))}
        if self.probe.read().penetration > START_PEN_M:
            return bad
        trk = HugTracker(n, CONTROL_HZ, hold_s, h_min=H_RUN, span_min_deg=SPAN_DEG)
        n_ramp, n_set, n_hold = int(round(RAMP_S * CONTROL_HZ)), int(round(settle_s * CONTROL_HZ)), int(round(hold_s * CONTROL_HZ))
        q0 = alpha * q
        runs, spans, ticks, cur, app, pen, touch = [], [], [], [], [], [], []
        for t in range(n_ramp + n_set + n_hold):
            sim.step(q0 + (q - q0) * min(1.0, (t + 1) / max(n_ramp, 1)), 1.0 / CONTROL_HZ)
            if not np.all(np.isfinite(sim.data.qpos)):
                return bad
            tick = sim.bus.end_tick()
            rd = self.probe.read()
            st = trk.update(rd, self._centres()[:, :2], post)
            if record is not None:
                record.append(sim.data.qpos.copy())
            if t >= n_ramp + n_set:
                runs.append(st.hug_modules)
                spans.append(st.span_deg if st.hug_modules >= H_RUN else 0.0)
                ticks.append(st.hug_tick)
                touch.append(float(rd.any_contact.sum()))
                cur.append(tick["mean"])
                app.append(tick["applied_peak"])
                pen.append(rd.penetration)
        held_run, held_span, mean_run = int(min(runs)), float(min(spans)), float(np.mean(runs))
        bus_mean, app_pk, pen_max = float(np.mean(cur)), float(max(app)), float(max(pen))
        ok_cur = bus_mean <= REGIMEN_BUS_MEAN_A and app_pk <= REGIMEN_BUS_PEAK_A + 0.05
        score = held_run + 0.25 * mean_run + 3.0 * min(held_span, 200.0) / 180.0 + Z_WEIGHT_TOUCH * float(np.mean(touch)) / n - 0.05 * bus_mean
        score -= 3.0 * max(0.0, bus_mean - REGIMEN_BUS_MEAN_A) + (3.0 if app_pk > REGIMEN_BUS_PEAK_A + 0.05 else 0.0)
        score -= min(6.0, 3.0 * max(0.0, pen_max - PEN_LIMIT_M) / PEN_LIMIT_M)
        duty = float(np.mean(ticks))
        return {"score": float(score), "held_run": held_run, "held_span": held_span, "peak_span": float(max(spans)), "median_span": float(np.median(spans)), "wrap_duty": duty,
                "mean_run": mean_run, "bus_mean_a": bus_mean, "bus_applied_peak_a": app_pk, "pen_max_mm": 1000 * pen_max, "feasible": bool(ok_cur and pen_max <= PEN_LIMIT_M),
                "hold_ok": bool(ok_cur and pen_max <= PEN_LIMIT_M and duty >= 0.9), "x": [float(v) for v in x], "post_xy": [float(post[0]), float(post[1])], "yaw": yaw,
                "max_joint_rad": float(np.max(np.abs(q)))}


_SIMS: dict[tuple[int, TargetShape], ShapeSearchSim] = {}


def _sim_for(n: int, shape: TargetShape) -> ShapeSearchSim:
    key = (n, dataclasses.replace(shape, yaw=0.0))
    if key not in _SIMS:
        _SIMS[key] = ShapeSearchSim(n, shape)
    return _SIMS[key]


class Cost:
    """Picklable DE objective (spawned workers): minimise ``-score``."""

    def __init__(self, n: int, shape: TargetShape) -> None:
        """Objective for ``n`` modules and ``shape``."""
        self.n, self.shape = n, shape

    def __call__(self, x: np.ndarray) -> float:
        """Negative score of candidate ``x``."""
        return -float(_sim_for(self.n, self.shape).evaluate(np.asarray(x))["score"])


def bounds(n: int, shape: TargetShape) -> list[tuple[float, float]]:
    """DE bounds: n-1 joint angles, the object offset (dx, dy), the start looseness alpha and (non-cylinders) the yaw."""
    b = [(-Q_BOUND_RAD, Q_BOUND_RAD)] * (n - 1) + [(-POST_OFFSET_M, POST_OFFSET_M)] * 2 + [ALPHA_BOUNDS]
    return b + ([(0.0, math.pi)] if shape.kind != "cyl" else [])


def seeded_population(n: int, shape: TargetShape, size: int, seed: int) -> np.ndarray:
    """As M1 (partial coils, both signs, loose start) but with curl magnitudes down to 0.35 rad per joint (large objects need a wide coil); the rest uniform random."""
    b = np.array(bounds(n, shape))
    rng = np.random.default_rng(seed)
    pop = rng.random((size, len(b))) * (b[:, 1] - b[:, 0]) + b[:, 0]
    cands = []
    for w in sorted({max(3, n - 1 - 3), 6, n - 1}):
        for start in sorted({0, (n - 1 - w) // 2, n - 1 - w}):
            for sg in (1.0, -1.0):
                for s in (0.35, 0.5, 0.7, 0.9, 1.2):
                    for alpha in (0.6, 0.85):
                        q = np.zeros(n - 1)
                        q[start:start + w] = sg * s
                        cands.append(np.concatenate([q, [0.0, 0.0, alpha], [float(rng.uniform(0, math.pi))] if shape.kind != "cyl" else []]))
    for k, i in enumerate(rng.permutation(len(cands))[:size // 2]):
        pop[k] = cands[i]
    return pop


def _score_x(args: tuple[int, TargetShape, np.ndarray]) -> float:
    n, shape, x = args
    return float(_sim_for(n, shape).evaluate(x)["score"])


def polish(n: int, shape: TargetShape, x0: np.ndarray, evals: int, workers: int, seed: int = 0) -> tuple[np.ndarray, int]:
    """(1+lambda) ES around ``x0`` (sigma 0.02 rad on joints, 2 mm on the offset, 0.02 on alpha, 0.05 rad on yaw; annealed)."""
    import multiprocessing as mp

    b = np.array(bounds(n, shape))
    rng = np.random.default_rng(seed)
    sig0 = np.array([0.02] * (n - 1) + [0.002, 0.002, 0.02] + ([0.05] if shape.kind != "cyl" else []))
    lam = max(2, workers * 2)
    best_x, best_f, used = np.array(x0, float), _score_x((n, shape, np.array(x0, float))), 0
    pool = mp.get_context("spawn").Pool(workers) if workers > 1 else None
    try:
        while used < evals:
            shrink = 1.0 - 0.7 * min(1.0, used / max(evals, 1))
            cands = [np.clip(best_x + rng.normal(size=len(best_x)) * sig0 * shrink, b[:, 0], b[:, 1]) for _ in range(lam)]
            sc = pool.map(_score_x, [(n, shape, c) for c in cands]) if pool else [_score_x((n, shape, c)) for c in cands]
            used += lam
            i = int(np.argmax(sc))
            if sc[i] > best_f:
                best_f, best_x = float(sc[i]), cands[i]
    finally:
        if pool:
            pool.close()
            pool.join()
    return best_x, used


def search(n: int, shape: TargetShape, budget: int, workers: int, seed: int = 0, polish_evals: int = 200, log: Any = print) -> dict[str, Any]:
    """DE (``budget`` evaluations) + polish for one (n, shape); returns the best candidate's record."""
    from scipy.optimize import differential_evolution

    b = bounds(n, shape)
    d = len(b)
    pop_mult = max(4, min(10, budget // (6 * d)))
    pop = pop_mult * d
    maxiter = max(1, budget // pop - 1)
    t0 = time.time()
    res = differential_evolution(Cost(n, shape), b, maxiter=maxiter, popsize=pop_mult, seed=seed, tol=1e-12, polish=False, workers=workers, updating="deferred",
                                 init=seeded_population(n, shape, pop, seed))
    n_evals = int(res.nfev)
    sim = _sim_for(n, shape)
    best = sim.evaluate(res.x)
    if polish_evals > 0:
        xp, used = polish(n, shape, res.x, polish_evals, workers, seed)
        n_evals += used
        ev = sim.evaluate(xp)
        if ev["score"] >= best["score"]:
            best = ev
    best.update({"n_modules": n, "shape": shape.label, "n_evals": n_evals, "wall_s": time.time() - t0, "settings": {"settle_s": SETTLE_S, "hold_s": HOLD_S, "cap_nm": REAL_CAP_NM, "pop": pop, "maxiter": maxiter}})
    log(f"  n={n} {shape.label}: held run {best['held_run']} held span {best['held_span']:.0f} (peak {best['peak_span']:.0f}) duty {best['wrap_duty']:.2f} bus {best['bus_mean_a']:.2f}/{best['bus_applied_peak_a']:.1f} A "
        f"pen {best['pen_max_mm']:.1f} mm hold_ok {best['hold_ok']} nfev {n_evals} ({best['wall_s']:.0f}s)")
    return best


def render_tile(n: int, shape: TargetShape, rec: dict[str, Any], elev: float, size: tuple[int, int] = (320, 240)) -> np.ndarray:
    """One frame (end of the hold) of the best pose of ``rec``, captioned."""
    from petctl.rl.video import burn_caption

    hs = _sim_for(n, shape)
    frames: list[np.ndarray] = []
    hs.evaluate(np.array(rec["x"]), record=frames)
    w, h = size
    hs.sim.model.vis.global_.offwidth, hs.sim.model.vis.global_.offheight = max(w, 640), max(h, 480)
    rend = mujoco.Renderer(hs.sim.model, height=h, width=w)
    d = mujoco.MjData(hs.sim.model)
    d.qpos[:] = frames[-1]
    mujoco.mj_forward(hs.sim.model, d)
    cam = mujoco.MjvCamera()
    cam.distance, cam.elevation, cam.azimuth = (0.8 if elev > -80 else 0.6), elev, 130 if elev > -80 else 90
    cam.lookat[:] = np.array([rec["post_xy"][0], rec["post_xy"][1], 0.03])
    rend.update_scene(d, camera=cam)
    img = rend.render().copy()
    rend.close()
    return burn_caption(img, f"n={n} {shape.label} run {rec['held_run']} span {rec['held_span']:.0f}")
