"""Phase 5: can STAGING get the body into a lattice pose without collapsing? For every candidate pose of data/rl/lattice_poses_n11.json, try several joint orderings (head-first, tail-first, middle-out,
ends-in + random orders); joints are ramped ONE AT A TIME (1 s each, the already-folded part supports the rest), then the pose is held 3 s. Valid = held (settled, no drift), peak torque < 0.49 N.m,
self-penetration < 8 mm. Compares with the all-at-once ramp of lattice_poses.py.

    python scripts/rl/pose_stage.py --workers 9 --out data/rl/pose_stage_n11.json
"""

from __future__ import annotations

import argparse
import json
import math
from multiprocessing import get_context
from pathlib import Path

import numpy as np

from petctl.rl.gait_eval import build_sim

_SIM: dict = {}
STEP_S, HOLD_S = 1.0, 3.0


def orders(nj: int, seed: int, n_random: int = 4) -> list[tuple[str, list[int]]]:
    """Canonical and random joint orders."""
    mid = sorted(range(nj), key=lambda j: abs(j - (nj - 1) / 2))
    ends = [j for pair in zip(range(nj), reversed(range(nj))) for j in pair][:nj]
    seen, ends_in = set(), []
    for j in ends:
        if j not in seen:
            seen.add(j)
            ends_in.append(j)
    out = [("head_first", list(range(nj))), ("tail_first", list(reversed(range(nj)))), ("middle_out", mid), ("ends_in", ends_in)]
    rng = np.random.default_rng(seed)
    out += [(f"random{i}", [int(x) for x in rng.permutation(nj)]) for i in range(n_random)]
    return out


def staged(n: int, seq: tuple[int, ...], order: list[int]) -> dict:
    """Run one staged entry; returns hold metrics."""
    sim = _SIM.setdefault(n, build_sim(n))
    nj, d, m = n - 1, sim.data, sim.model
    q_t = np.array(seq, float) * math.pi / 2
    sim.reset()
    sim.step(np.zeros(nj), 0.5)
    cur = np.zeros(nj)
    for j in order:
        a, b = cur[j], q_t[j]
        for i in range(20):
            cur[j] = a + (b - a) * (i + 1) / 20
            sim.step(cur, STEP_S / 20)
        if not np.all(np.isfinite(d.qpos)):
            return {"ok": False}
    taus, qv, ref = [], [], None
    for i in range(int(HOLD_S / 0.05)):
        sim.step(q_t, 0.05)
        if i >= int(HOLD_S / 0.05) - 20:
            taus.append(float(np.abs(sim.joint_torque()).max()))
            qv.append(float(np.abs(sim.joint_vel()).max()))
            pos = d.xipos[sim.m0_id:sim.m0_id + n]
            ref = pos.mean(axis=0) if ref is None else ref
    pos = d.xipos[sim.m0_id:sim.m0_id + n]
    floor = [g for g in range(m.ngeom) if m.geom_bodyid[g] == 0]
    pen = max([-c.dist for c in d.contact[:d.ncon] if c.geom1 not in floor and c.geom2 not in floor] + [0.0])
    held = bool(max(qv) < 0.1 and np.linalg.norm(pos.mean(axis=0) - ref) < 0.01)
    return {"ok": True, "valid": bool(held and max(taus) < 0.49 and pen < 0.008), "tau_peak": max(taus), "pen_mm": 1000 * pen, "held": held, "height_cm": 100 * float(pos[:, 2].max()),
            "footprint_cm2": 1e4 * float(np.ptp(pos[:, 0]) * np.ptp(pos[:, 1])), "n_off_floor": int((pos[:, 2] > 0.035).sum())}


def work(args: tuple[int, tuple[int, ...], int]) -> dict:
    """Try all orders for one pose; return the best (first valid, else lowest torque)."""
    n, seq, seed = args
    best = None
    for name, order in orders(n - 1, seed):
        r = staged(n, seq, order)
        if not r["ok"]:
            continue
        r["order"], r["order_name"] = order, name
        if r["valid"]:
            return {"seq": list(seq), **r}
        if best is None or r["tau_peak"] < best["tau_peak"]:
            best = r
    return {"seq": list(seq), **(best or {"ok": False, "valid": False})}


def main() -> int:
    """CLI entry point."""
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=11)
    ap.add_argument("--workers", type=int, default=9)
    ap.add_argument("--limit", type=int, default=0)
    ap.add_argument("--src", type=Path, default=Path("data/rl/lattice_poses_n11.json"))
    ap.add_argument("--out", type=Path, default=Path("data/rl/pose_stage_n11.json"))
    a = ap.parse_args()
    seqs = [tuple(r["seq"]) for r in json.loads(a.src.read_text())]
    if a.limit:
        seqs = seqs[:a.limit]
    with get_context("spawn").Pool(a.workers) as pool:
        res = pool.map(work, [(a.n, s, i) for i, s in enumerate(seqs)], chunksize=10)
    a.out.write_text(json.dumps(res))
    v = [r for r in res if r.get("valid")]
    print(f"{len(v)} / {len(res)} poses reachable by some staged order (all-at-once ramp: 72)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
