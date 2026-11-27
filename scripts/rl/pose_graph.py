"""Phase 5: grow the set of reachable, holdable 90-degree lattice poses by local search from the staged-entry survivors (pose_stage.py). Mutate 1-2 joints of a valid pose (+-90 deg, or any value), test it with the
staged entry (joint orders: head-first, tail-first, middle-out, ends-in, random) and keep it when it holds below the torque cap without self-penetration. Archive cells: height bin x footprint bin x modules-lifted
bin; the fittest per cell is the tallest.

    python scripts/rl/pose_graph.py --evals 4000 --workers 9 --out data/rl/pose_graph_n11.json
"""

from __future__ import annotations

import argparse
import json
import time
from multiprocessing import get_context
from pathlib import Path

import numpy as np

from pose_stage import work


def cell(r: dict) -> tuple[int, int, int]:
    """Descriptor: height (4 cm bins), footprint (area bins), lifted modules (3 bins)."""
    return int(r["height_cm"] // 4), 0 if r["footprint_cm2"] < 150 else 1 if r["footprint_cm2"] < 400 else 2, 0 if r["n_off_floor"] < 5 else 1 if r["n_off_floor"] < 9 else 2


def main() -> int:
    """CLI entry point."""
    ap = argparse.ArgumentParser()
    ap.add_argument("--evals", type=int, default=4000)
    ap.add_argument("--workers", type=int, default=9)
    ap.add_argument("--batch", type=int, default=90)
    ap.add_argument("--n", type=int, default=11)
    ap.add_argument("--out", type=Path, default=Path("data/rl/pose_graph_n11.json"))
    a = ap.parse_args()
    rng = np.random.default_rng(0)
    arch: dict[tuple, dict] = {}
    seen: set[tuple] = set()
    for r in json.loads(Path("data/rl/pose_stage_n11.json").read_text()):
        if r.get("valid"):
            arch[cell(r)] = r if cell(r) not in arch or r["height_cm"] > arch[cell(r)]["height_cm"] else arch[cell(r)]
            seen.add(tuple(r["seq"]))
    pool, n_ev, t0 = get_context("spawn").Pool(a.workers), 0, time.time()
    print(f"start: {len(arch)} cells", flush=True)
    while n_ev < a.evals:
        keys = list(arch)
        cands = []
        while len(cands) < a.batch:
            s = list(arch[keys[rng.integers(len(keys))]]["seq"])
            for j in rng.choice(a.n - 1, int(rng.integers(1, 3)), replace=False):
                s[j] = int(rng.integers(0, 4))
            t = tuple(s)
            if t not in seen:
                seen.add(t)
                cands.append(t)
        for r in pool.map(work, [(a.n, c, int(rng.integers(1 << 20))) for c in cands]):
            n_ev += 1
            if r.get("valid"):
                k = cell(r)
                if k not in arch or r["height_cm"] > arch[k]["height_cm"]:
                    arch[k] = r
        if (n_ev // a.batch) % 5 == 0:
            print(f"[{time.time() - t0:5.0f}s] {n_ev} evals, {len(arch)} cells, tallest {max(e['height_cm'] for e in arch.values()):.1f} cm", flush=True)
    pool.close()
    a.out.write_text(json.dumps(list(arch.values())))
    print(f"wrote {a.out}: {len(arch)} poses")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
