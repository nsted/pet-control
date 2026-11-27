"""Re-evaluate every cell of MAP-Elites archives with the current metric code (after a metric fix) and re-bin, keeping the best per cell.

    python scripts/rl/reeval_archive.py data/rl/map_elites/n*_s*.json     (writes <name>_v2.json next to each)
"""

from __future__ import annotations

import json
import sys
from multiprocessing import get_context
from pathlib import Path

import numpy as np

from map_elites import HI, LO, cell_of, fitness, work                                   # noqa: E402  (script dir is on sys.path)


def main() -> int:
    """CLI entry point."""
    pool = get_context("spawn").Pool(9)
    for path in map(Path, sys.argv[1:]):
        arc = json.loads(path.read_text())
        n = arc["n_modules"]
        us = [((np.array(c["params"]) - LO) / (HI - LO)).tolist() for c in arc["cells"]]
        new: dict = {}
        moved = 0
        for c, u, m in zip(arc["cells"], us, pool.map(work, [(n, u, 12.0) for u in us])):
            if m is None:
                continue
            k, f = cell_of(m), fitness(m)
            moved += list(k) != c["cell"]
            if k not in new or f > new[k]["fitness"]:
                new[k] = {"cell": list(k), "fitness": f, "params": c["params"], "metrics": m}
        out = {**arc, "cells": list(new.values()), "reeval": True}
        out["curve"] = arc["curve"] + [(arc["evals"], len(new), sum(e["fitness"] for e in new.values()))]
        path.with_name(path.stem + "_v2.json").write_text(json.dumps(out))
        print(f"{path.name}: {len(arc['cells'])} -> {len(new)} cells, {moved} cells changed bin, max |yaw| {max(abs(e['metrics']['yaw_rate']) for e in new.values()):.2f}", flush=True)
    pool.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
