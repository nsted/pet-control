"""Search for the tallest static pose PET can hold from the flat start (sim only). Run: python scripts/static_pose_search.py --out data/rl/static_poses.json

Black-box optimisation (differential evolution, parallel) over the 7 joint angles. A pose scores the lowest valid height of the highest
module point over the last HOLD_S seconds of commanding it, so only poses that are reached and sustained count. Invalid poses (module
below the floor guard or penetrating it) score 0.
"""
from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import numpy as np
from scipy.optimize import differential_evolution

from petctl.rl.config import EnvConfig
from petctl.rl.env import PetEnv

URDF = "/Users/nsted/Documents/PET/body/URDF/PET_description"
HOLD_S = 2.0
_ENV: PetEnv | None = None
_CFG: dict = {}


def _env() -> PetEnv:
    global _ENV
    if _ENV is None:
        cfg = EnvConfig(task="reach_still", episode_s=_CFG["settle_s"] + HOLD_S, action_scale_rad=np.pi, urdf_dir=URDF)
        cfg.dr.scale, cfg.dr.cap_nm = 0.0, (_CFG["cap"], _CFG["cap"])
        _ENV = PetEnv(cfg)
    return _ENV


def held_height(q: np.ndarray) -> float:
    """Lowest valid height (m above the flat start) of the top point over the final HOLD_S seconds of holding pose ``q``."""
    env = _env()
    env.reset(seed=0)
    a = np.clip(q / np.pi, -1.0, 1.0)
    n_hold = int(round(HOLD_S * env.cfg.control_hz))
    gs: list[float] = []
    done = False
    while not done:
        _, _, te, tr, _ = env.step(a)
        done = te or tr
        ok, _ = env.task._valid(env.sim)
        gs.append(env.task.gain(env.sim) if ok else 0.0)
    return float(min(gs[-n_hold:]))


def _init(cfg: dict) -> None:
    _CFG.update(cfg)


class Cost:
    """Picklable objective (worker processes are spawned on macOS, so the settings must travel with it)."""

    def __init__(self, cap: float, settle_s: float) -> None:
        self.cfg = {"cap": cap, "settle_s": settle_s}

    def __call__(self, q: np.ndarray) -> float:
        _init(self.cfg)
        return -held_height(q)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", type=Path, default=Path("data/rl/static_poses.json"))
    ap.add_argument("--cap-nm", type=float, default=0.5)
    ap.add_argument("--settle-s", type=float, default=6.0)
    ap.add_argument("--restarts", type=int, default=4)
    ap.add_argument("--maxiter", type=int, default=60)
    ap.add_argument("--popsize", type=int, default=12)
    ap.add_argument("--workers", type=int, default=8)
    a = ap.parse_args()
    _init({"cap": a.cap_nm, "settle_s": a.settle_s})
    t0 = time.time()
    held_height(np.zeros(7))
    print(f"one evaluation: {time.time() - t0:.2f}s (incl. env build)", flush=True)
    found = []
    for seed in range(a.restarts):
        r = differential_evolution(Cost(a.cap_nm, a.settle_s), [(-np.pi, np.pi)] * 7, maxiter=a.maxiter, popsize=a.popsize, seed=seed, tol=1e-9,
                                   polish=False, workers=a.workers, updating="deferred", init="sobol")
        found.append({"seed": seed, "held_height_cm": -100 * float(r.fun), "q_rad": [float(x) for x in r.x]})
        print(f"[restart {seed}] held {-100 * r.fun:.1f} cm  q={np.round(r.x, 2)}  ({time.time() - t0:.0f}s)", flush=True)
        a.out.parent.mkdir(parents=True, exist_ok=True)
        a.out.write_text(json.dumps({"cap_nm": a.cap_nm, "poses": sorted(found, key=lambda p: -p["held_height_cm"])}, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
