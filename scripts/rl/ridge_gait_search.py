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
    cfg.task_params = {**g["eval_task_params"], "terrain_level": 0.5, "terrain_ridges": 0.0, "stall_s": 0.0}
    cfg.dr.scale = 0.0
    cfg.episode_s = EP_S
    env = PetEnv(cfg)
    env.reset(seed=1)
    flat = generate_terrain(0.5, 1, 0.45, rolling=True)
    env.sim.set_terrain(replace(flat, heights=np.full_like(flat.heights, ROLL_BASE_M), tile_mu=np.full((3, 3), 1.0), tile_loose=np.zeros((3, 3), bool), parts={}))
    d = env.sim.data                                                                    # set_terrain resets the sim to its clean straight home pose
    v = np.asarray(d.xpos[1][:2] - d.xpos[env.sim.model.nbody - 1][:2], float)          # tail -> head direction in the plane
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
    """Joint-angle targets (rad) at time ``t`` from the parameter vector ``theta`` (36 entries)."""
    off, amp, ph = theta[0:NJ], theta[NJ:2 * NJ], theta[2 * NJ:3 * NJ]
    b, psi, f = theta[3 * NJ:4 * NJ], theta[4 * NJ:5 * NJ], theta[5 * NJ]
    w = 2.0 * np.pi * f * t
    return off + amp * np.sin(w + ph) + b * np.sin(2.0 * w + psi)


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
        a = np.concatenate([np.clip(q / scale, -1.0, 1.0), [-1.0]])
        _, _, te, tr, _ = env.step(a)
        done = te or tr
        t += env.dt
        best = max(best, float(np.dot(np.array(env.sim.data.subtree_com[1][:2], float) - p0, fwd)))
    crossed = float(best > RIDGE_T0_M + 0.45)
    return best + 2.0 * crossed, best, crossed


def init_theta(rng: np.random.Generator) -> np.ndarray:
    th = np.zeros(5 * NJ + 1)
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
    theta = init_theta(rng)
    lo = np.r_[np.full(NJ, -1.5), np.full(NJ, 0.0), np.full(NJ, -np.pi), np.full(NJ, 0.0), np.full(NJ, -np.pi), 0.2]
    hi = np.r_[np.full(NJ, 1.5), np.full(NJ, 2.0), np.full(NJ, 2 * np.pi * 2), np.full(NJ, 1.0), np.full(NJ, np.pi), 2.0]
    a.out.parent.mkdir(parents=True, exist_ok=True)
    hist, best_all = [], (-1.0, theta.copy(), 0.0, 0.0)
    t0 = time.time()
    with Pool(a.workers) as pool:
        for it in range(a.iters):
            eps = rng.normal(0, 1, (a.pairs, theta.size))
            cand = [np.clip(theta + s * a.sigma * e * (hi - lo) * 0.25, lo, hi) for e in eps for s in (1, -1)]
            res = pool.map(evaluate, [(c, a.height) for c in cand])
            fit = np.array([r[0] for r in res])
            for c, r in zip(cand, res):
                if r[0] > best_all[0]:
                    best_all = (r[0], c.copy(), r[1], r[2])
            rank = (np.argsort(np.argsort(fit)) / (len(fit) - 1)) - 0.5                      # centred rank shaping
            grad = sum(rank[2 * i] * eps[i] - rank[2 * i + 1] * eps[i] for i in range(a.pairs)) / (2 * a.pairs)
            theta = np.clip(theta + 0.5 * a.sigma * grad * (hi - lo) * 0.25 * 2.0, lo, hi)
            hist.append({"iter": it, "mean_progress": float(np.mean([r[1] for r in res])), "best_progress": float(max(r[1] for r in res)), "crossed_frac": float(np.mean([r[2] for r in res])), "elapsed_min": (time.time() - t0) / 60})
            print(json.dumps(hist[-1]), flush=True)
            a.out.write_text(json.dumps({"height_m": a.height, "best_fitness": best_all[0], "best_progress_m": best_all[2], "best_crossed": best_all[3], "best_theta": best_all[1].tolist(), "history": hist}, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
