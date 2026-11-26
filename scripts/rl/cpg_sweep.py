"""CPG parameter sweeps on the lying body (Phase 3): which parts of the pair basis move the body, in which direction, at lengths 10-13 (cap 0.5 N.m, isotropic friction 1.0).

    python scripts/rl/cpg_sweep.py --lengths 10 11 12 13 --out data/rl/cpg_sweep

Groups: ``yaw`` (twist wave off), ``twist`` (yaw wave off), ``both`` (psi in 0, +-pi/2, pi), ``bias`` (a static twist / yaw bias on top of a wave). Writes one JSON per length and prints a table
of the best configuration per behaviour (forward, reverse, left, right, turn, roll) plus the fraction of configurations that move the body (> 1 cm/s).
"""

from __future__ import annotations

import argparse
import itertools
import json
import math
import time
from multiprocessing import get_context
from pathlib import Path

import numpy as np

from petctl.rl.cpg import CPGParams
from petctl.rl.gait_eval import build_sim, evaluate_gait

_SIM: dict = {}


def grid() -> list[tuple[str, CPGParams]]:
    """The sweep configurations (group name, parameters)."""
    out: list[tuple[str, CPGParams]] = []
    lags = (-2.5, -1.8, -1.0, -0.4, 0.4, 1.0, 1.8, 2.5)
    for a, lag, f in itertools.product((0.3, 0.6, 0.9), lags, (0.5, 1.0)):
        out.append(("yaw", CPGParams(f=f, dphi=lag, a_yaw=a, a_twist=0.0)))
        out.append(("twist", CPGParams(f=f, dphi=lag, a_yaw=0.0, a_twist=a)))
    for ay, at, lag, psi, f in itertools.product((0.3, 0.6, 0.9), (0.3, 0.6, 0.9), (-2.1, -1.0, 1.0, 2.1), (-math.pi / 2, 0.0, math.pi / 2, math.pi), (0.5, 1.0)):
        out.append(("both", CPGParams(f=f, dphi=lag, psi=psi, a_yaw=ay, a_twist=at)))
    for bt, by, lag in itertools.product((-1.57, -0.8, 0.8, 1.57), (0.0, 0.4), (-1.8, 1.8)):
        out.append(("bias", CPGParams(f=0.5, dphi=lag, psi=math.pi / 2, a_yaw=0.5, a_twist=0.3, b_twist0=bt, b_yaw0=by)))
    return out


def work(args: tuple[int, str, list[float]]) -> dict:
    """Evaluate one configuration (worker process; the sim of a length is built once per worker)."""
    n, group, vec = args
    if n not in _SIM:
        _SIM[n] = build_sim(n)
    r = evaluate_gait(np.array(vec), n, seconds=12.0, sim=_SIM[n])
    return {"group": group, "params": vec, **r.to_dict()}


def best(rows: list[dict], key, name: str) -> str:
    """One table row: the best configuration by ``key`` (max)."""
    ok = [r for r in rows if r["ok"] and not r["flipped"]]
    r = max(ok, key=key)
    p = dict(zip(("f", "dphi", "psi", "a_yaw", "a_twist", "taper_yaw", "taper_twist", "b_yaw0", "b_yaw1", "b_twist0", "b_twist1"), r["params"]))
    desc = f"{r['group']} f{p['f']:.1f} lag{p['dphi']:+.1f} psi{p['psi']:+.2f} Ay{p['a_yaw']:.1f} At{p['a_twist']:.1f} bt{p['b_twist0']:+.1f} by{p['b_yaw0']:+.1f}"
    return (f"| {name} | {r['vx'] * 100:+.1f} | {r['vy'] * 100:+.1f} | {r['yaw_rate']:+.2f} | {r['roll_rate']:+.2f} | {r['speed_bl']:.3f} | {r['current_peak_a']:.1f} | {r['tau_peak']:.2f} | {desc} |")


def main() -> int:
    """CLI entry point."""
    ap = argparse.ArgumentParser()
    ap.add_argument("--lengths", type=int, nargs="+", default=[10, 11, 12, 13])
    ap.add_argument("--workers", type=int, default=8)
    ap.add_argument("--out", type=Path, default=Path("data/rl/cpg_sweep"))
    a = ap.parse_args()
    a.out.mkdir(parents=True, exist_ok=True)
    cfgs = grid()
    lines = []
    with get_context("spawn").Pool(a.workers) as pool:
        for n in a.lengths:
            t0 = time.time()
            rows = pool.map(work, [(n, g, p.to_vector().tolist()) for g, p in cfgs], chunksize=4)
            (a.out / f"L{n}.json").write_text(json.dumps(rows))
            ok = [r for r in rows if r["ok"] and not r["flipped"]]
            moving = np.mean([r["speed"] > 0.01 for r in ok]) if ok else 0.0
            by_group = {g: float(np.mean([r["speed"] > 0.01 for r in ok if r["group"] == g])) for g in ("yaw", "twist", "both", "bias")}
            lines += [f"\n### n_modules = {n} ({len(rows)} configs, {100 * moving:.0f} % move > 1 cm/s; by group " + ", ".join(f"{g} {100 * v:.0f} %" for g, v in by_group.items()) + f"; {time.time() - t0:.0f} s)",
                      "| behaviour | vx cm/s | vy cm/s | yaw rad/s | roll rad/s | body len/s | I peak A (proxy) | tau peak | params |", "|---|---|---|---|---|---|---|---|---|",
                      best(rows, lambda r: r["vx"], "forward"), best(rows, lambda r: -r["vx"], "reverse"), best(rows, lambda r: r["vy"], "left"), best(rows, lambda r: -r["vy"], "right"),
                      best(rows, lambda r: r["yaw_rate"], "turn left"), best(rows, lambda r: -r["yaw_rate"], "turn right"), best(rows, lambda r: abs(r["roll_rate"]), "roll")]
            print("\n".join(lines[-12:]), flush=True)
    (a.out / "summary.md").write_text("\n".join(lines) + "\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
