"""Ceiling search for the jump task: how high can PET leave the ground with a periodic crouch-then-snap motion? (sim only)

Run: python scripts/rl/jump_search.py --out data/rl/jump_motions.json
Differential evolution over a two-pose square-wave motion (crouch pose, snap pose, period, duty), scored by the task's own record
clearance (lowest point of the lowest module above the floor, airborne only, glitch-guarded). Tells RL what is achievable.
"""
from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import numpy as np
from scipy.optimize import differential_evolution

from petctl.rl.config import DEFAULT_URDF_DIR, EnvConfig
from petctl.rl.env import PetEnv

URDF = DEFAULT_URDF_DIR
_ENV: PetEnv | None = None
_CFG: dict = {}


def _env() -> PetEnv:
    global _ENV
    if _ENV is None:
        cfg = EnvConfig(task="jump", episode_s=_CFG["episode_s"], action_scale_rad=np.pi, urdf_dir=URDF,
                        action_ema=_CFG["ema"], max_target_step_rad=_CFG["slew"])
        cfg.dr.scale, cfg.dr.cap_nm = 0.0, (_CFG["cap"], _CFG["cap"])
        _ENV = PetEnv(cfg)
    return _ENV


def run_motion(x: np.ndarray) -> dict:
    """Play the motion ``x = [qa(7), qb(7), period_s, duty]`` and return the episode's jump metrics."""
    env = _env()
    env.reset(seed=0)
    qa, qb, period, duty = x[:7] / np.pi, x[7:14] / np.pi, float(x[14]), float(x[15])
    info: dict = {}
    for i in range(int(env.cfg.episode_s * env.cfg.control_hz)):
        t = i / env.cfg.control_hz
        a = qa if (t % period) < duty * period else qb
        _, _, te, tr, info = env.step(np.clip(a, -1.0, 1.0))
        if te or tr:
            break
    return info["metrics"]


class Cost:
    """Picklable objective (worker processes are spawned on macOS)."""

    def __init__(self, cfg: dict) -> None:
        self.cfg = cfg

    def __call__(self, x: np.ndarray) -> float:
        _CFG.update(self.cfg)
        return -run_motion(x)["record_clearance_m"]


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", type=Path, default=Path("data/rl/jump_motions.json"))
    ap.add_argument("--cap-nm", type=float, default=0.5)
    ap.add_argument("--ema", type=float, default=0.2)
    ap.add_argument("--slew", type=float, default=1.0)
    ap.add_argument("--episode-s", type=float, default=8.0)
    ap.add_argument("--restarts", type=int, default=3)
    ap.add_argument("--maxiter", type=int, default=60)
    ap.add_argument("--popsize", type=int, default=10)
    ap.add_argument("--workers", type=int, default=8)
    a = ap.parse_args()
    cfg = {"cap": a.cap_nm, "ema": a.ema, "slew": a.slew, "episode_s": a.episode_s}
    _CFG.update(cfg)
    t0 = time.time()
    run_motion(np.r_[np.zeros(14), 1.0, 0.5])
    print(f"one evaluation: {time.time() - t0:.2f}s (incl. env build)", flush=True)
    bounds = [(-np.pi, np.pi)] * 14 + [(0.4, 2.0), (0.2, 0.8)]
    found = []
    for seed in range(a.restarts):
        r = differential_evolution(Cost(cfg), bounds, maxiter=a.maxiter, popsize=a.popsize, seed=seed, tol=1e-9, polish=False,
                                   workers=a.workers, updating="deferred", init="sobol")
        m = run_motion(r.x)
        found.append({"seed": seed, "clearance_cm": -100 * float(r.fun), "metrics": m, "x": [float(v) for v in r.x]})
        print(f"[restart {seed}] clearance {-100 * r.fun:.1f} cm airtime {m['best_air_s']:.2f}s jumps {m['jumps']:.0f}  ({time.time() - t0:.0f}s)", flush=True)
        a.out.parent.mkdir(parents=True, exist_ok=True)
        a.out.write_text(json.dumps({"cfg": cfg, "motions": sorted(found, key=lambda p: -p["clearance_cm"])}, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
