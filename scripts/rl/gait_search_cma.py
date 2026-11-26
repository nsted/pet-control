"""CMA-ES search over the 11 CPG parameters for one target behaviour (Phase 4 tool; validated first at n = 11).

    python scripts/rl/gait_search_cma.py --n 11 --objective roll --evals 1500 --workers 8 --out data/rl/gait_search/roll_n11.json

Objectives (maximised; all measured on the open-loop rollout of ``gait_eval.evaluate_gait``): ``forward`` vx, ``reverse`` -vx, ``left`` vy, ``right`` -vy, ``turn_left`` yaw rate, ``turn_right`` -yaw rate,
``roll`` |net roll rate about the long axis| (rad/s), ``roll_travel`` |roll rate| + 4 speed (rolling that also travels). Optimisation runs in the unit box (parameters normalised by ``PARAM_BOUNDS``).
A torque-saturation penalty (0.5 x fraction of samples at the cap) keeps the search away from pure bang-bang. Writes the best parameters, metrics and the search history.
"""

from __future__ import annotations

import argparse
import json
import time
from multiprocessing import get_context
from pathlib import Path

import cma
import numpy as np

from petctl.rl.cpg import PARAM_BOUNDS, PARAM_NAMES
from petctl.rl.gait_eval import build_sim, evaluate_gait

LO = np.array([PARAM_BOUNDS[k][0] for k in PARAM_NAMES])
HI = np.array([PARAM_BOUNDS[k][1] for k in PARAM_NAMES])
OBJECTIVES = {"forward": lambda r: r.vx, "reverse": lambda r: -r.vx, "left": lambda r: r.vy, "right": lambda r: -r.vy, "turn_left": lambda r: r.yaw_rate, "turn_right": lambda r: -r.yaw_rate,
              "roll": lambda r: abs(r.roll_rate), "roll_travel": lambda r: abs(r.roll_rate) + 4.0 * r.speed}
_SIM: dict = {}


def decode(u: np.ndarray) -> np.ndarray:
    """Unit box -> parameter vector."""
    return LO + np.clip(u, 0.0, 1.0) * (HI - LO)


def encode(p: np.ndarray) -> np.ndarray:
    """Parameter vector -> unit box."""
    return (np.asarray(p, float) - LO) / (HI - LO)


def work(args: tuple[int, str, list[float], float]) -> tuple[float, dict]:
    """Evaluate one candidate (worker): returns (score to maximise, metrics)."""
    n, objective, u, seconds = args
    if n not in _SIM:
        _SIM[n] = build_sim(n)
    r = evaluate_gait(decode(np.array(u)), n, seconds=seconds, sim=_SIM[n])
    if not r.ok:
        return -1.0, r.to_dict()
    return OBJECTIVES[objective](r) - 0.5 * r.sat_frac * 0.1, r.to_dict()


def search(n: int, objective: str, evals: int, workers: int, seed: int = 0, x0: np.ndarray | None = None, sigma0: float = 0.25, seconds: float = 14.0, pool=None, log=print) -> dict:
    """One CMA-ES run; returns the best parameters / metrics and the history. ``x0``: start vector (parameters); default a mid-range travelling wave."""
    start = encode(x0 if x0 is not None else np.array([1.0, 1.0, 1.57, 0.6, 0.6, 0, 0, 0, 0, 0, 0]))
    es = cma.CMAEvolutionStrategy(start.tolist(), sigma0, {"bounds": [0.0, 1.0], "popsize": 16, "seed": seed + 1, "verbose": -9})
    own = pool is None
    pool = pool or get_context("spawn").Pool(workers)
    best, hist, n_ev, t0 = (-1e9, None, None), [], 0, time.time()
    while n_ev < evals and not es.stop():
        cands = es.ask()
        res = pool.map(work, [(n, objective, c.tolist(), seconds) for c in cands])
        es.tell(cands, [-s for s, _ in res])
        n_ev += len(cands)
        i = int(np.argmax([s for s, _ in res]))
        if res[i][0] > best[0]:
            best = (res[i][0], cands[i].tolist(), res[i][1])
        hist.append(best[0])
        if len(hist) % 10 == 0:
            log(f"[{time.time() - t0:5.0f}s] {n_ev} evals, best {objective} score {best[0]:.4f}", flush=True)
    if own:
        pool.close()
    return {"n_modules": n, "objective": objective, "score": best[0], "params": decode(np.array(best[1])).tolist(), "metrics": best[2], "history": hist, "evals": n_ev, "seed": seed}


def main() -> int:
    """CLI entry point."""
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=11)
    ap.add_argument("--objective", choices=list(OBJECTIVES), default="roll")
    ap.add_argument("--evals", type=int, default=1500)
    ap.add_argument("--workers", type=int, default=8)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--out", type=Path, default=Path("data/rl/gait_search/result.json"))
    a = ap.parse_args()
    res = search(a.n, a.objective, a.evals, a.workers, a.seed)
    a.out.parent.mkdir(parents=True, exist_ok=True)
    a.out.write_text(json.dumps(res, indent=1))
    m = res["metrics"]
    print(f"best {a.objective} score {res['score']:.4f}: vx {100 * m['vx']:+.1f} vy {100 * m['vy']:+.1f} cm/s, yaw {m['yaw_rate']:+.2f}, roll {m['roll_rate']:+.2f} rad/s, tau peak {m['tau_peak']:.2f}, "
          f"I peak {m['current_peak_a']:.1f} A (proxy)")
    print(dict(zip(PARAM_NAMES, [round(x, 2) for x in res["params"]])))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
