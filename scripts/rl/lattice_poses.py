"""Phase 5 first pass: held poses on the 90-degree lattice (q_i = R_i x 90 deg, R in 0..3) at n = 11, from the Rubik's-snake principles: repeating motifs (period 1-5, every phase) and mirror-symmetric
sequences, plus random lattice samples. Each pose is ramped in, held 3 s on the floor at the 0.5 N.m cap and scored: held (settled, no drift), peak torque, self-penetration, height of the highest module,
footprint (xy bounding box), number of modules off the floor.

    python scripts/rl/lattice_poses.py --n 11 --random 3000 --workers 9 --out data/rl/lattice_poses_n11.json
"""

from __future__ import annotations

import argparse
import itertools
import json
import math
from multiprocessing import get_context
from pathlib import Path

import mujoco
import numpy as np

from petctl.rl.gait_eval import build_sim

_SIM: dict = {}


def evaluate(args: tuple[int, tuple[int, ...]]) -> dict:
    """Hold one lattice pose; returns its metrics."""
    n, seq = args
    sim = _SIM.setdefault(n, build_sim(n))
    nj = n - 1
    q = np.array(seq, float) * math.pi / 2
    sim.reset()
    d, m = sim.data, sim.model
    sim.step(np.zeros(nj), 0.5)
    for i in range(20):
        sim.step(q * (i + 1) / 20, 0.05)
    taus, drift, qv = [], None, []
    for i in range(60):
        sim.step(q, 0.05)
        if not np.all(np.isfinite(d.qpos)):
            return {"seq": list(seq), "ok": False}
        if i >= 40:
            taus.append(float(np.abs(sim.joint_torque()).max()))
            qv.append(float(np.abs(sim.joint_vel()).max()))
            pos = d.xipos[sim.m0_id:sim.m0_id + n]
            drift = pos.mean(axis=0) if drift is None else drift
    pos = d.xipos[sim.m0_id:sim.m0_id + n]
    floor = [g for g in range(m.ngeom) if m.geom_bodyid[g] == 0]
    pen = max([-c.dist for c in d.contact[:d.ncon] if c.geom1 not in floor and c.geom2 not in floor] + [0.0])
    return {"seq": list(seq), "ok": True, "held": bool(max(qv) < 0.1 and np.linalg.norm(pos.mean(axis=0) - drift) < 0.01), "tau_peak": max(taus), "pen_mm": 1000 * pen,
            "height_cm": 100 * float(pos[:, 2].max() - 0.0), "footprint_cm2": 1e4 * float(np.ptp(pos[:, 0]) * np.ptp(pos[:, 1])), "n_off_floor": int((pos[:, 2] > 0.035).sum())}


def candidates(nj: int, n_random: int, seed: int = 0) -> list[tuple[int, ...]]:
    """Repeating motifs (period 1-5), mirror-symmetric sequences, random lattice samples; deduplicated."""
    out: set[tuple[int, ...]] = set()
    for p in range(1, 6):
        for motif in itertools.product(range(4), repeat=p):
            out.add(tuple(motif[i % p] for i in range(nj)))
    for half in itertools.product(range(4), repeat=(nj + 1) // 2):
        out.add(tuple(half) + tuple(reversed(half[:nj // 2])))
    rng = np.random.default_rng(seed)
    for _ in range(n_random):
        out.add(tuple(int(x) for x in rng.integers(0, 4, nj)))
    return sorted(out)


def main() -> int:
    """CLI entry point."""
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=11)
    ap.add_argument("--random", type=int, default=3000)
    ap.add_argument("--workers", type=int, default=9)
    ap.add_argument("--out", type=Path, default=Path("data/rl/lattice_poses_n11.json"))
    a = ap.parse_args()
    cs = candidates(a.n - 1, a.random)
    print(f"{len(cs)} candidate sequences", flush=True)
    with get_context("spawn").Pool(a.workers) as pool:
        res = pool.map(evaluate, [(a.n, c) for c in cs], chunksize=20)
    a.out.parent.mkdir(parents=True, exist_ok=True)
    a.out.write_text(json.dumps(res))
    ok = [r for r in res if r["ok"] and r["held"] and r["tau_peak"] < 0.5 * 0.98 and r["pen_mm"] < 8]
    print(f"valid held poses: {len(ok)} / {len(res)}; tallest {max((r['height_cm'] for r in ok), default=0):.1f} cm; most compact {min((r['footprint_cm2'] for r in ok), default=0):.0f} cm2")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
