"""Edge reflex controllers and their evaluation (docs/plans/EDGE_AVOIDANCE.md).

    python scripts/rl/edge_reflex.py --episodes 60 --detector data/rl/edge/detector2_look0.npz
Controllers (all wrap the same Wander locomotion): ``Reflex(detector)``: proprioceptive detector -> RETREAT (reverse gait) -> TURN (away, random sign) -> back to wandering; ``Reflex(oracle=True)`` triggers on
the PRIVILEGED overhang count (upper bound); plus Wander (no reflex) and Stand (never moves). Parameters: detector threshold, ticks above threshold, retreat time, turn time, refractory time.
"""

from __future__ import annotations

import argparse
from collections import deque
from multiprocessing import get_context
from pathlib import Path

import numpy as np

from edge_detector import WIN
from edge_sim import Scene, Stand, Wander, summarize

BACK = np.array([-0.14, 0.0, 0.0])
_S: dict = {}


class Detector:
    """Numpy forward pass of the trained MLP with the online feature pipeline (mirrors edge_detector.features)."""

    def __init__(self, path: Path) -> None:
        z = np.load(path)
        self.mu, self.sd, self.thr = z["mu"], z["sd"], float(z["thr"])
        self.W = [(z[f"{i}.weight"], z[f"{i}.bias"]) for i in ("0", "2", "4")]
        self.reset()

    def reset(self) -> None:
        self.tau, self.err, self.up = deque(maxlen=WIN), deque(maxlen=WIN), deque(maxlen=WIN)

    def score(self, o: dict) -> float:
        self.tau.append(np.abs(o["tau"]))
        self.err.append(o["err"])
        self.up.append(o["up"])
        T, E, U = np.array(self.tau), np.array(self.err), np.array(self.up)
        x = np.concatenate([o["tau"], o["err"], o["qd"] * 0.3, o["up"], o["gyro"] * 0.3, o["phase"], o["params"], T.mean(0), T.std(0), E.mean(0), U.mean(0)])
        h = (x - self.mu) / self.sd
        for i, (w, b) in enumerate(self.W):
            h = h @ w.T + b
            if i < 2:
                h = np.maximum(h, 0.0)
        return float(1.0 / (1.0 + np.exp(-h[0])))


class Reflex:
    """LOCOMOTE (wander) -> edge trigger -> RETREAT -> TURN -> LOCOMOTE."""

    def __init__(self, detector: Detector | None = None, oracle: bool = False, thr: float | None = None, k: int = 2, retreat_s: float = 2.5, turn_s: float = 4.0, refractory_s: float = 1.0,
                 snap: bool = False, halt_s: float = 0.0, oracle_dist: float | None = None) -> None:
        self.det, self.oracle, self.k, self.retreat_s, self.turn_s, self.refractory_s = detector, oracle, k, retreat_s, turn_s, refractory_s
        self.snap, self.halt_s, self.oracle_dist = snap, halt_s, oracle_dist
        self.thr = thr if thr is not None else (detector.thr if detector else 0.0)
        self.wander = Wander()

    def reset(self, rng) -> None:
        self.rng, self.mode, self.t_mode, self.hits, self.sign, self.t_block = rng, "LOCOMOTE", 0.0, 0, 1.0, 0.0
        self.wander.reset(rng)
        if self.det:
            self.det.reset()
        self.triggers = 0

    def step(self, obs: dict, priv: dict) -> np.ndarray:
        t = obs["t"]
        trig = False
        if self.oracle:
            trig = priv["overhang"] >= 1 if self.oracle_dist is None else priv["dist_edge"] < self.oracle_dist
        elif self.det is not None:
            s = self.det.score(obs)
            self.hits = self.hits + 1 if s > self.thr else 0
            trig = self.hits >= self.k
        if self.mode == "LOCOMOTE":
            if trig and t >= self.t_block:
                self.mode, self.t_mode, self.triggers = "RETREAT", t, self.triggers + 1
                self.sign = float(self.rng.choice([-1.0, 1.0]))
            else:
                return self.wander.step(obs, priv)
        if self.mode == "RETREAT":
            if t - self.t_mode < self.halt_s:
                return np.zeros(3)                                                                   # HALT: quiet gait first
            if t - self.t_mode < self.halt_s + self.retreat_s:
                return BACK
            self.mode, self.t_mode = "TURN", t
        if self.mode == "TURN":
            if t - self.t_mode < self.turn_s:
                return np.array([0.0, 0.0, 0.5 * self.sign])
            self.mode, self.t_block = "LOCOMOTE", t + self.refractory_s
            self.wander.t_next = t
        return self.wander.step(obs, priv)


def work(args: tuple[str, int, dict, str | None]) -> dict:
    """One evaluation episode: (policy name, seed, reflex kwargs, detector path)."""
    name, seed, kw, det = args
    sc = _S.setdefault("sc", Scene())
    if name == "wander":
        c = Wander()
    elif name == "stand":
        c = Stand()
    else:
        c = Reflex(Detector(Path(det)) if (det and name == "detector") else None, oracle=(name == "oracle"), **kw)
    lg = sc.run(c, seed, 60.0)
    return {"fell": lg.fell, "path_m": lg.path_m, "seconds": lg.seconds, "cells": lg.cells, "moving_frac": lg.moving_frac, "triggers": getattr(c, "triggers", 0)}


def evaluate(name: str, episodes: int, kw: dict, det: str | None, workers: int = 9, seed0: int = 90000) -> dict:
    """Run ``episodes`` episodes in parallel and summarise."""
    with get_context("spawn").Pool(workers) as pool:
        res = pool.map(work, [(name, seed0 + i, kw, det) for i in range(episodes)], chunksize=2)
    out = {"policy": name, "fall_rate": float(np.mean([r["fell"] for r in res])), "path_m_per_min": float(np.mean([r["path_m"] / max(r["seconds"], 1e-6) * 60 for r in res])),
           "cells": float(np.mean([r["cells"] for r in res])), "moving_frac": float(np.mean([r["moving_frac"] for r in res])), "triggers_per_min": float(np.mean([r["triggers"] / max(r["seconds"], 1e-6) * 60 for r in res])),
           "survive_s": float(np.mean([r["seconds"] for r in res]))}
    print(f"{name:10s} fall rate {out['fall_rate']:.2f}  survive {out['survive_s']:.0f}/60 s  path {out['path_m_per_min']:.1f} m/min  cells {out['cells']:.1f}  moving {out['moving_frac']:.2f}  reflex triggers/min {out['triggers_per_min']:.1f}", flush=True)
    return out


def main() -> int:
    """CLI entry point."""
    ap = argparse.ArgumentParser()
    ap.add_argument("--episodes", type=int, default=60)
    ap.add_argument("--detector", type=Path, default=Path("data/rl/edge/detector2_look0.npz"))
    ap.add_argument("--workers", type=int, default=9)
    a = ap.parse_args()
    for name in ("wander", "stand", "oracle", "detector"):
        evaluate(name, a.episodes, {}, str(a.detector), a.workers)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
