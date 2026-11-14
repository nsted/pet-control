"""Static hug-feasibility search (Regimen v2 plan sec 3, step 4; milestone M1). Sim only, no RL.

For a body of ``n`` modules, find by differential evolution the static joint-angle pose (plus the post position relative to the body) that maximises the
ground-truth hug metric (``petctl.rl.hug``: longest chain-contiguous run of modules with a *face* within 15 mm of the 10 cm post, debounced; M1 used geom contact) while the pose is
HELD for 3 s under the real PD actuators, the real torque cap and the per-robot bus-current model.

Evaluation protocol of one candidate ``x = (q_1..q_{n-1}, dx, dy)``:
  1. Placement: the robot is reset with its joints already at ``q`` (lifted ``LIFT_M``), the post far away, and laid down on the floor for ``FREE_S`` (1.5 s).
     Then the post is put at the centre of the least-squares circle through the module centres plus ``(dx, dy)`` (the start offset / radius is a search variable bounded by
     ``POST_OFFSET_M``). A candidate whose post would penetrate the body by more than ``START_PEN_M`` is rejected (it would be ejected violently).
  2. Settle ``SETTLE_S`` around the post with the pose commanded (delayed targets, kp/kd defaults, cap ``REAL_CAP_NM``, ``BusClamp`` on with the 10 A spike clamp), then hold
     ``HOLD_S`` (3 s) more; ticks are 20 Hz as in the env.
  3. Over the hold window: ``held_modules`` = min over the window of the debounced longest hugging run; mean bus current; applied / demanded peak; max
     post penetration. Score = held run + shaping (mean run, any-contact modules) - 0.05 mean current - penalties (current limits, penetration > 1 cm).
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Any

import mujoco  # type: ignore[import-untyped]
import numpy as np

from petctl.config import REGIMEN_BUS_MEAN_A, REGIMEN_BUS_PEAK_A
from petctl.rl.capacitive import PostProbe
from petctl.rl.config import BUS_A_PER_NM2, DEFAULT_URDF_DIR
from petctl.rl.curriculum.stages import REAL_CAP_NM
from petctl.rl.hug import HOLD_S, HugTracker, hug_threshold
from petctl.sim.bus import BusClamp
from petctl.sim.model_builder import BuildParams, PetSim

CONTROL_HZ = 20.0
SIM_DT = 0.004
FREE_S = 1.5                    # phase 1: body laid down in the pose, post far away
RAMP_S = 2.0                    # phase 2: the command ramps linearly from alpha * q to q while the body wraps round the post
SETTLE_S = 1.0                  # phase 2: settle around the post (post placed at the settled centroid + offset)
LIFT_M = 0.03
ALPHA_BOUNDS = (0.3, 0.95)      # start looseness: phase 1 lays the body down at alpha * q (a wider coil so the post fits inside without penetration)
Q_BOUND_RAD = 1.6               # search bound per joint (the plan estimates 0.5-0.95 rad per joint are needed)
POST_OFFSET_M = 0.05            # bound of the post start offset from the base point (fitted-circle centre of the strongly curled modules of the laid-down body), per axis
START_PEN_M = 0.03
PEN_LIMIT_M = 0.01              # penetration beyond this in the hold window is penalised (glitch)
HELIX_Z_M = 0.05                # hugging modules' centre-height span above this = helical (stacked on the post; the even/odd zig-zag alone gives 3 cm), else planar
Z_WEIGHT_TOUCH = 0.5


@dataclass
class EvalResult:
    """Everything measured for one candidate."""

    score: float
    held_modules: int            # min over the 3 s hold window of the debounced longest contiguous hugging run
    mean_modules: float
    touch_modules: float         # mean number of modules touching the post with any part of the hull
    bus_mean_a: float
    bus_applied_peak_a: float
    bus_demand_peak_a: float
    pen_max_m: float
    feasible: bool               # current limits met and no glitch
    max_joint_rad: float
    wrap_deg: float              # floor-plane wrap angle of the final run (diagnostic)
    helix_span_m: float          # height span of the hugging modules' centres at the end
    run_start: int
    post_xy: tuple[float, float] = (0.0, 0.0)
    q: list[float] = field(default_factory=list)
    x: list[float] = field(default_factory=list)      # the full candidate (q.., dx, dy, alpha)
    duty: float = 0.0            # fraction of ticks of the 3 s hold window that are hug ticks (run >= H), plan sec 5.3 proximity definition
    run_median: float = 0.0      # median over the hold window of the debounced longest run
    hold_ok: bool = False        # duty >= 90 %


def circle_centre(xy: np.ndarray) -> np.ndarray:
    """Centre of the least-squares (Kasa) circle through the module centres ``xy`` (n, 2); the mean if the fit is degenerate."""
    A = np.c_[2.0 * xy, np.ones(len(xy))]
    b = (xy ** 2).sum(axis=1)
    sol, *_ = np.linalg.lstsq(A, b, rcond=None)
    c = sol[:2]
    return c if len(xy) >= 3 and np.all(np.isfinite(c)) and np.linalg.norm(c - xy.mean(axis=0)) < 0.2 else xy.mean(axis=0)  # type: ignore[no-any-return]


class HugSearchSim:
    """One reusable sim per module count (build once per process, re-pose per candidate)."""

    def __init__(self, n_modules: int, urdf_dir: str = DEFAULT_URDF_DIR, cap_nm: float = REAL_CAP_NM) -> None:
        """Build the model with a post (moved per candidate), the real cap and the bus model attached."""
        self.n = n_modules
        self.sim = PetSim(BuildParams(n_modules=n_modules, urdf_dir=urdf_dir, dt=SIM_DT, cap_nm=(cap_nm,) * (n_modules - 1), post=(5.0, 5.0)))
        self.sim.bus = BusClamp(self.sim.model, BUS_A_PER_NM2, REGIMEN_BUS_PEAK_A, clamp=True)
        self.probe = PostProbe.of(self.sim)
        assert self.probe is not None
        self.post_gid = int(self.sim.model.geom("post").id)
        self.h = hug_threshold(n_modules)

    def _centres(self) -> np.ndarray:
        return self.sim.data.xipos[1:self.n + 1].copy()

    def _settle_free(self, q: np.ndarray, free_s: float) -> np.ndarray:
        """Phase 1: lay the body down in the pose ``q`` with the post far away (``free_s`` s); returns the xy centroid of the module centres."""
        sim = self.sim
        sim.model.geom_pos[self.post_gid, :2] = (5.0, 5.0)
        sim.reset(q)
        sim.data.qpos[2] += LIFT_M
        mujoco.mj_forward(sim.model, sim.data)
        sim.step(q, free_s)
        big = np.abs(q) > 0.4 * max(float(np.max(np.abs(q))), 1e-9)               # joints that actually curl; the post goes inside the coil they form
        sel = np.zeros(self.n, bool)
        for j in np.flatnonzero(big):
            sel[j:j + 2] = True
        return circle_centre(self._centres()[sel, :2])

    def evaluate(self, x: np.ndarray, settle_s: float = SETTLE_S, hold_s: float = HOLD_S, record: list[np.ndarray] | None = None, free_s: float = FREE_S) -> EvalResult:
        """Run the protocol above for ``x = (q.., dx, dy)``. ``record`` (if given) receives the full qpos every tick (for rendering)."""
        sim, n = self.sim, self.n
        q = np.asarray(x[:n - 1], float)
        dx, dy, alpha = float(x[n - 1]), float(x[n]), float(x[n + 1])
        post = self._settle_free(alpha * q, free_s) + np.array([dx, dy])
        sim.model.geom_pos[self.post_gid, :2] = post
        mujoco.mj_forward(sim.model, sim.data)
        if sim.bus is not None:
            sim.bus.begin_tick()
        bad = EvalResult(-5.0, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, False, float(np.max(np.abs(q))), 0.0, 0.0, 0, (float(post[0]), float(post[1])), [float(v) for v in q], [float(v) for v in x])
        if self.probe.read().penetration > START_PEN_M:
            return bad
        trk = HugTracker(n, CONTROL_HZ, hold_s)
        n_set, n_hold = int(round(settle_s * CONTROL_HZ)), int(round(hold_s * CONTROL_HZ))
        runs: list[int] = []
        ticks: list[bool] = []
        touch: list[float] = []
        cur: list[float] = []
        app: list[float] = []
        dem: list[float] = []
        pen: list[float] = []
        st = None
        n_ramp = int(round(RAMP_S * CONTROL_HZ))
        q0 = alpha * q
        for t in range(n_ramp + n_set + n_hold):
            sim.step(q0 + (q - q0) * min(1.0, (t + 1) / max(n_ramp, 1)), 1.0 / CONTROL_HZ)
            if not np.all(np.isfinite(sim.data.qpos)):
                return bad
            tick = sim.bus.end_tick() if sim.bus is not None else {"mean": 0.0, "applied_peak": 0.0, "peak": 0.0}
            rd = self.probe.read()
            st = trk.update(rd, self._centres()[:, :2], post)
            if record is not None:
                record.append(sim.data.qpos.copy())
            if t >= n_ramp + n_set:
                runs.append(st.hug_modules)
                ticks.append(st.hug_tick)
                touch.append(float(rd.any_contact.sum()))
                cur.append(tick["mean"])
                app.append(tick["applied_peak"])
                dem.append(tick["peak"])
                pen.append(rd.penetration)
        assert st is not None
        held, mean_run = int(min(runs)), float(np.mean(runs))
        bus_mean, app_pk, dem_pk, pen_max = float(np.mean(cur)), float(max(app)), float(max(dem)), float(max(pen))
        ok_cur = bus_mean <= REGIMEN_BUS_MEAN_A and app_pk <= REGIMEN_BUS_PEAK_A + 0.05
        ok = ok_cur and pen_max <= PEN_LIMIT_M
        score = held + 0.25 * mean_run + Z_WEIGHT_TOUCH * float(np.mean(touch)) / n - 0.05 * bus_mean
        score -= 3.0 * max(0.0, bus_mean - REGIMEN_BUS_MEAN_A) + (3.0 if app_pk > REGIMEN_BUS_PEAK_A + 0.05 else 0.0)
        score -= min(6.0, 3.0 * max(0.0, pen_max - PEN_LIMIT_M) / PEN_LIMIT_M)
        in_run = np.zeros(n, bool)
        in_run[st.run_start:st.run_start + st.hug_modules] = st.hug_modules > 0
        zspan = float(np.ptp(self._centres()[in_run, 2])) if in_run.sum() >= 2 else 0.0
        return EvalResult(score, held, mean_run, float(np.mean(touch)), bus_mean, app_pk, dem_pk, pen_max, ok, float(np.max(np.abs(q))), st.wrap_deg, zspan, st.run_start,
                          (float(post[0]), float(post[1])), [float(v) for v in q], [float(v) for v in x],
                          float(np.mean(ticks)), float(np.median(runs)), bool(np.mean(ticks) >= 0.9))


_SIMS: dict[int, HugSearchSim] = {}


def _sim_for(n: int) -> HugSearchSim:
    if n not in _SIMS:
        _SIMS[n] = HugSearchSim(n)
    return _SIMS[n]


class Cost:
    """Picklable DE objective (worker processes are spawned on macOS): minimise ``-score``."""

    def __init__(self, n_modules: int) -> None:
        """Objective for ``n_modules``."""
        self.n = n_modules

    def __call__(self, x: np.ndarray) -> float:
        """Negative score of candidate ``x``."""
        return -_sim_for(self.n).evaluate(np.asarray(x)).score


def bounds(n: int) -> list[tuple[float, float]]:
    """DE bounds: n-1 joint angles, then the post offset (dx, dy)."""
    return [(-Q_BOUND_RAD, Q_BOUND_RAD)] * (n - 1) + [(-POST_OFFSET_M, POST_OFFSET_M)] * 2 + [ALPHA_BOUNDS]


def seeded_population(n: int, size: int, seed: int) -> np.ndarray:
    """Initial DE population: partial coils (a window of ``w`` consecutive joints at +-s, the rest straight; both signs; the scripted +0.88 pose is the full-window
    case) with the post at the fitted-circle centre and a loose start, first; the rest uniform random."""
    b = np.array(bounds(n))
    rng = np.random.default_rng(seed)
    pop = rng.random((size, n + 2)) * (b[:, 1] - b[:, 0]) + b[:, 0]
    k = 0
    cands = []
    for w in sorted({max(3, n - 1 - 4), 5, 6, n - 1}):
        for start in sorted({0, (n - 1 - w) // 2, n - 1 - w}):
            for sg in (1.0, -1.0):
                for s in (0.9, 1.2, 1.5):
                    for alpha in (0.6, 0.8):
                        q = np.zeros(n - 1)
                        q[start:start + w] = sg * s
                        cands.append(np.concatenate([q, [0.0, 0.0], [alpha]]))
    order = rng.permutation(len(cands))
    for i in order[:size // 2]:
        pop[k] = cands[i]
        k += 1
    return pop


def classify(res: EvalResult) -> str:
    """'planar' or 'helical' from the height span of the hugging modules' centres."""
    if res.held_modules < 2:
        return "n/a"
    return "helical" if res.helix_span_m > HELIX_Z_M else "planar"


def _score_x(args: tuple[int, np.ndarray]) -> float:
    n, x = args
    return _sim_for(n).evaluate(x).score


def polish(n: int, x0: np.ndarray, evals: int, workers: int, seed: int = 0) -> tuple[np.ndarray, int]:
    """(1+lambda) evolution strategy around ``x0`` (sigma 0.02 rad on joints, 2 mm on the post offset, 0.02 on alpha, annealed): the hug window is a
    millimetre-scale target (the coil must be centred on the post), which DE's global moves hit only by luck. Returns the improved candidate and the evaluations used."""
    import multiprocessing as mp

    b = np.array(bounds(n))
    rng = np.random.default_rng(seed)
    sig0 = np.array([0.02] * (n - 1) + [0.002, 0.002, 0.02])
    lam = max(2, workers * 2)
    best_x, best_f, used, gen = np.array(x0, float), _score_x((n, np.array(x0, float))), 0, 0
    if workers <= 1:
        pool = None
    else:
        pool = mp.get_context("spawn").Pool(workers)
    try:
        while used < evals:
            shrink = 1.0 - 0.7 * min(1.0, used / max(evals, 1))
            cands = [np.clip(best_x + rng.normal(size=len(best_x)) * sig0 * shrink * (rng.random() < 0.5 or 1.0), b[:, 0], b[:, 1]) for _ in range(lam)]
            sc = pool.map(_score_x, [(n, c) for c in cands]) if pool else [_score_x((n, c)) for c in cands]
            used += lam
            gen += 1
            i = int(np.argmax(sc))
            if sc[i] > best_f:
                best_f, best_x = float(sc[i]), cands[i]
    finally:
        if pool:
            pool.close()
            pool.join()
    return best_x, used


def search(n: int, budget: int, restarts: int, workers: int, seed: int = 0, log: Any = print, polish_evals: int = 800) -> dict[str, Any]:
    """Run DE for length ``n`` (``budget`` evaluations in total over ``restarts`` independent runs); returns the best candidate's record."""
    from scipy.optimize import differential_evolution

    d = n + 2
    per = max(8, budget // max(restarts, 1))
    pop_mult = max(4, min(12, per // (8 * d)))
    pop = pop_mult * d
    maxiter = max(1, per // pop - 1)
    t0 = time.time()
    best: tuple[float, EvalResult] | None = None
    n_evals = 0
    for r in range(restarts):
        init = seeded_population(n, pop, seed * 100 + r)
        res = differential_evolution(Cost(n), bounds(n), maxiter=maxiter, popsize=pop_mult, seed=seed * 100 + r, tol=1e-12, polish=False, workers=workers,
                                     updating="deferred", init=init)
        n_evals += int(res.nfev)
        ev = _sim_for(n).evaluate(res.x)
        if polish_evals > 0:
            xp, used = polish(n, res.x, polish_evals, workers, seed * 100 + r)
            n_evals += used
            ev = _sim_for(n).evaluate(xp)
        log(f"  n={n} restart {r}: score {ev.score:.3f} held {ev.held_modules} (mean {ev.mean_modules:.1f}) bus {ev.bus_mean_a:.2f}/{ev.bus_applied_peak_a:.1f} A "
            f"pen {1000 * ev.pen_max_m:.1f} mm feasible {ev.feasible} nfev {res.nfev} ({time.time() - t0:.0f}s)")
        if best is None or ev.score > best[0]:
            best = (ev.score, ev)
    assert best is not None
    ev = best[1]
    # robustness: re-evaluate under small perturbations of the post start offset (the dynamics settle differently); report the held run of each
    sim = _sim_for(n)
    rob = []
    for jit in ((0.005, 0.0), (-0.005, 0.0), (0.0, 0.005), (0.0, -0.005)):
        xj = np.array(ev.x)
        xj[n - 1:n + 1] += jit
        rob.append(sim.evaluate(xj).held_modules)
    h = hug_threshold(n)
    return {"n_modules": n, "H": h, "held_modules": ev.held_modules, "h_reached": bool(ev.held_modules >= h and ev.feasible), "feasible": ev.feasible,
            "bus_mean_a": ev.bus_mean_a, "bus_applied_peak_a": ev.bus_applied_peak_a, "bus_demand_peak_a": ev.bus_demand_peak_a, "pen_max_mm": 1000 * ev.pen_max_m,
            "max_joint_rad": ev.max_joint_rad, "wrap_deg": ev.wrap_deg, "helix_span_cm": 100 * ev.helix_span_m, "geometry": classify(ev),
            "mean_modules": ev.mean_modules, "touch_modules": ev.touch_modules, "robust_held_min": int(min(rob + [ev.held_modules])), "robust_held": rob,
            "n_evals": n_evals, "wall_s": time.time() - t0, "q_rad": ev.q, "post_xy": list(ev.post_xy), "x": ev.x, "alpha": ev.x[n + 1], "post_offset_m": ev.x[n - 1:n + 1], "run_start": ev.run_start, "score": ev.score,
            "settings": {"settle_s": SETTLE_S, "hold_s": HOLD_S, "cap_nm": REAL_CAP_NM, "q_bound_rad": Q_BOUND_RAD, "post_offset_m": POST_OFFSET_M, "restarts": restarts,
                         "pop": pop, "maxiter": maxiter}}


def render_sheet(n: int, rec: dict[str, Any], out: Any, size: tuple[int, int] = (360, 270)) -> None:
    """Contact-sheet PNG (no video): oblique and top-down frames of the best pose at t = 0, 2 (settled) and 5 s (end of hold)."""
    from pathlib import Path

    from PIL import Image

    from petctl.rl.video import burn_caption

    hs = _sim_for(n)
    frames: list[np.ndarray] = []
    ev = hs.evaluate(np.array(rec["x"]), record=frames)
    w, h = size
    hs.sim.model.vis.global_.offwidth, hs.sim.model.vis.global_.offheight = max(w, 640), max(h, 480)
    rend = mujoco.Renderer(hs.sim.model, height=h, width=w)
    d = mujoco.MjData(hs.sim.model)
    tiles = []
    idx = [0, int(SETTLE_S * CONTROL_HZ) - 1, len(frames) - 1]
    for elev, az in ((-35, 130), (-89, 90)):
        for i in idx:
            d.qpos[:] = frames[i]
            mujoco.mj_forward(hs.sim.model, d)
            cam = mujoco.MjvCamera()
            cam.distance, cam.elevation, cam.azimuth = (0.9 if elev > -80 else 0.7), elev, az
            cam.lookat[:] = np.array([rec["post_xy"][0], rec["post_xy"][1], 0.03])
            rend.update_scene(d, camera=cam)
            tiles.append(burn_caption(rend.render().copy(), f"n={n} static hug t={(i + 1) / CONTROL_HZ:.1f}s held={ev.held_modules}/H={rec['H']}"))
    grid = np.vstack([np.hstack(tiles[:3]), np.hstack(tiles[3:])])
    Path(out).parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(grid).save(out)



__all__ = ["EvalResult", "HugSearchSim", "Cost", "bounds", "search", "render_sheet", "classify"]
