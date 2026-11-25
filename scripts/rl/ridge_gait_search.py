"""Open-loop gait search over a single wide ridge (sim only): can this body hoist part of itself over a ridge and pull the rest across?

A 36-parameter wave controller (per-joint offset, amplitude, phase, second-harmonic amplitude and phase, plus a shared frequency) is optimised by an antithetic evolution strategy
(OpenAI-ES style rank shaping, no extra dependencies). Fitness = forward progress of the body centre along the start heading across a ridge that spans the whole patch, so it cannot be
skirted. The ridge height is the argument; results (best parameters, progress, crossing flag) go to ``--out`` as JSON.

    python scripts/rl/ridge_gait_search.py --height 0.06 --out data/rl/ridge_gait/h06.json
"""

from __future__ import annotations

import argparse
import json
import time
from dataclasses import replace
from multiprocessing import Pool
from pathlib import Path

import mujoco
import numpy as np

from petctl.rl.curriculum.gate2 import GaitPolicy
from petctl.rl.env import PetEnv
from petctl.sim.terrain import CELL_M, HALF_M, ROLL_BASE_M, generate_terrain

RUN = Path("data/rl/chain_c_dunes5_L0.25/terrain_dunes/a1")        # only used for a ready-made env config (obs layout irrelevant: the controller is open loop)
RIDGE_T0_M = 0.9                      # ridge crest distance ahead of the start (m)
UP_DEG, CREST_M, DROP_M = 40.0, 0.04, 0.05
NJ = 7
EP_S = 40.0

_env: PetEnv | None = None
_fwd: np.ndarray | None = None


def _make_env() -> tuple[PetEnv, np.ndarray]:
    pol = GaitPolicy(RUN, "policy.npz")
    cfg = pol.cfg
    g = json.load(open(RUN / "gate2_terrain_dunes.json"))
    cfg.task_params = {**g["eval_task_params"], "terrain_level": 0.5, "terrain_ridges": 0.0, "stall_s": 0.0, "action_unbounded": 1.0}
    cfg.dr.scale = 0.0
    cfg.episode_s = EP_S
    env = PetEnv(cfg)
    env.reset(seed=1)
    flat = generate_terrain(0.5, 1, 0.45, rolling=True)
    env.sim.set_terrain(replace(flat, heights=np.full_like(flat.heights, ROLL_BASE_M), tile_mu=np.full((3, 3), 1.0), tile_loose=np.zeros((3, 3), bool), parts={}))
    d = env.sim.data                                                                    # set_terrain resets the sim to its clean straight home pose
    mujoco.mj_forward(env.sim.model, d)
    v = np.asarray(d.site_xpos[env.sim.head_id][:2] - d.subtree_com[1][:2], float)      # centre of mass -> head direction in the plane (body frames all sit at the origin)
    return env, v / (np.linalg.norm(v) + 1e-9)


def ridge_terrain(env: PetEnv, fwd: np.ndarray, height: float):
    """A flat patch (rolling baseline) with one ridge across the whole width, perpendicular to ``fwd``, crest ``RIDGE_T0_M`` ahead."""
    base = generate_terrain(0.5, 1, 0.45, rolling=True)
    xs = -HALF_M + CELL_M * np.arange(base.heights.shape[0])
    X, Y = np.meshgrid(xs, xs)
    t = X * fwd[0] + Y * fwd[1] - RIDGE_T0_M
    run = height / np.tan(np.radians(UP_DEG))
    prof = np.interp(t, [-run, 0.0, CREST_M, CREST_M + DROP_M], [0.0, height, height, 0.0], left=0.0, right=0.0)
    return replace(base, heights=ROLL_BASE_M + prof, tile_mu=np.full((3, 3), 1.0), tile_loose=np.zeros((3, 3), bool), parts={})


def controller(theta: np.ndarray, t: float) -> np.ndarray:
    """Joint-angle targets (rad) at time ``t``: per-joint offset, wave and second harmonic (36 parameters), plus an optional continuous spin ``omega_j * t`` (7 more, rad/s) for joints that
    turn round and round. No angle range anywhere: only the per-tick slew limit (a speed limit) bounds how fast a target may move."""
    off, amp, ph = theta[0:NJ], theta[NJ:2 * NJ], theta[2 * NJ:3 * NJ]
    b, psi, f = theta[3 * NJ:4 * NJ], theta[4 * NJ:5 * NJ], theta[5 * NJ]
    spin = theta[5 * NJ + 1:6 * NJ + 1] if len(theta) > 5 * NJ + 1 else 0.0
    w = 2.0 * np.pi * f * t
    return off + amp * np.sin(w + ph) + b * np.sin(2.0 * w + psi) + spin * t


def bounds() -> tuple[np.ndarray, np.ndarray]:
    """Search bounds of the 43 controller parameters. Angles are wide (offsets +-pi, amplitudes up to 4 rad) and the spin rate is +-3 rad/s: these are search-space sizes, not limits on the robot."""
    lo = np.r_[np.full(NJ, -np.pi), np.full(NJ, 0.0), np.full(NJ, -np.pi), np.full(NJ, 0.0), np.full(NJ, -np.pi), 0.2, np.full(NJ, -3.0)]
    hi = np.r_[np.full(NJ, np.pi), np.full(NJ, 4.0), np.full(NJ, 2 * np.pi * 2), np.full(NJ, 2.0), np.full(NJ, np.pi), 2.0, np.full(NJ, 3.0)]
    return lo, hi


def evaluate(args: tuple[np.ndarray, float]) -> tuple[float, float, float]:
    """(fitness, progress_m, crossed 0/1) of one parameter vector on a ridge of the given height."""
    global _env, _fwd
    theta, height = args
    if _env is None:
        _env, _fwd = _make_env()
    env, fwd = _env, _fwd
    assert fwd is not None
    env.reset(seed=1)                                            # env.reset REBUILDS the sim, so the custom terrain must be swapped in afterwards
    env.sim.set_terrain(ridge_terrain(env, fwd, height))
    scale = float(env.cfg.action_scale_rad)
    p0 = np.array(env.sim.data.subtree_com[1][:2], float)
    best, t, done = 0.0, 0.0, False
    while not done:
        q = controller(theta, t)
        a = np.concatenate([q / scale, [-1.0]])
        _, _, te, tr, _ = env.step(a)
        done = te or tr
        t += env.dt
        best = max(best, float(np.dot(np.array(env.sim.data.subtree_com[1][:2], float) - p0, fwd)))
    crossed = float(best > RIDGE_T0_M + 0.45)
    return best + 2.0 * crossed, best, crossed


def init_theta(rng: np.random.Generator) -> np.ndarray:
    th = np.zeros(6 * NJ + 1)
    th[NJ:2 * NJ] = 0.8                                          # amplitude
    th[2 * NJ:3 * NJ] = np.arange(NJ) * 0.9                      # travelling-wave phase lag
    th[5 * NJ] = 0.8                                             # Hz
    return th + rng.normal(0, 0.05, th.shape)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--height", type=float, default=0.06)
    ap.add_argument("--iters", type=int, default=40)
    ap.add_argument("--pairs", type=int, default=12)
    ap.add_argument("--sigma", type=float, default=0.2)
    ap.add_argument("--workers", type=int, default=4)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--out", type=Path, default=Path("data/rl/ridge_gait/result.json"))
    a = ap.parse_args()
    rng = np.random.default_rng(a.seed)
    mean = init_theta(rng)
    lo, hi = bounds()
    std = (hi - lo) * 0.15 * a.sigma / 0.2                                 # initial per-dimension sampling width
    a.out.parent.mkdir(parents=True, exist_ok=True)
    hist, best_all = [], (-1.0, mean.copy(), 0.0, 0.0)
    lam, mu, t0 = 2 * a.pairs, max(3, a.pairs // 2), time.time()
    with Pool(a.workers) as pool:                                          # cross-entropy method: sample, keep the best mu, refit mean and std (smoothed)
        for it in range(a.iters):
            cand = [np.clip(rng.normal(mean, std), lo, hi) for _ in range(lam)]
            cand[0] = mean.copy()
            res = pool.map(evaluate, [(c, a.height) for c in cand])
            fit = np.array([r[0] for r in res])
            for c, r in zip(cand, res):
                if r[0] > best_all[0]:
                    best_all = (r[0], c.copy(), r[1], r[2])
            elite = np.array([cand[i] for i in np.argsort(-fit)[:mu]])
            mean = 0.5 * mean + 0.5 * elite.mean(0)
            std = np.maximum(0.7 * std + 0.3 * elite.std(0), (hi - lo) * 0.01)
            hist.append({"iter": it, "mean_progress": float(np.mean([r[1] for r in res])), "elite_progress": float(np.mean(sorted([r[1] for r in res])[-mu:])), "best_progress": float(max(r[1] for r in res)),
                         "crossed_frac": float(np.mean([r[2] for r in res])), "elapsed_min": (time.time() - t0) / 60})
            print(json.dumps(hist[-1]), flush=True)
            a.out.write_text(json.dumps({"height_m": a.height, "best_fitness": best_all[0], "best_progress_m": best_all[2], "best_crossed": best_all[3], "best_theta": best_all[1].tolist(), "history": hist}, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
