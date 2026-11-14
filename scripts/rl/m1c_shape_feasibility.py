"""M1c: static hug-feasibility search per target shape (sim only). Run: python scripts/rl/m1c_shape_feasibility.py [--lengths 8 10] [--budget 600] [--workers 6]

Writes data/rl/m1c_shape_feasibility/table.json (one record per (shape, n)) and one contact-sheet PNG per n (best pose of each shape, oblique + top-down). Protocol: petctl/rl/shape_search.py.
"""
from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import numpy as np

from petctl.rl.shape_search import render_tile, search
from petctl.rl.target_shapes import EXTREMES, HELD_OUT_L8, HELD_OUT_L10, make_shape

OUT = Path("data/rl/m1c_shape_feasibility")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--lengths", type=int, nargs="+", default=[8, 10])
    ap.add_argument("--shapes", nargs="+", default=["cyl_r3", "cyl_r9", "square_8x8", "rect_5x14"])
    ap.add_argument("--budget", type=int, default=600, help="DE evaluations per (shape, n)")
    ap.add_argument("--polish", type=int, default=120)
    ap.add_argument("--workers", type=int, default=6)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--out", type=Path, default=OUT)
    a = ap.parse_args()
    a.out.mkdir(parents=True, exist_ok=True)
    table_p = a.out / "table.json"
    rows = {(r["shape_name"], r["n_modules"]): r for r in json.loads(table_p.read_text())["rows"]} if table_p.exists() else {}
    catalog = {nm: (kind, dims, rc) for nm, kind, dims, rc in (*HELD_OUT_L10, *HELD_OUT_L8, *EXTREMES)}
    t0 = time.time()
    for n in a.lengths:
        for nm in a.shapes:
            kind, dims, rc = catalog[nm]
            shape = make_shape(kind, dims, rc)
            rec = search(n, shape, a.budget, a.workers, a.seed, a.polish)
            rec["shape_name"] = nm
            rows[(nm, n)] = rec
            table_p.write_text(json.dumps({"rows": [rows[k] for k in sorted(rows)]}, indent=1))
        from PIL import Image

        tiles = [[render_tile(n, make_shape(*catalog[nm][:2], catalog[nm][2]), rows[(nm, n)], el) for nm in a.shapes if (nm, n) in rows] for el in (-35, -89)]
        Image.fromarray(np.vstack([np.hstack(t) for t in tiles])).save(a.out / f"best_poses_n{n}.png")
        print(f"n={n} done, {time.time() - t0:.0f}s elapsed", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
