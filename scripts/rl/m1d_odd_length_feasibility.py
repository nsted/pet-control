"""M1d: static hug-feasibility for odd lengths (n = 9, 11) and the L=8 rounded gate shape (sim only). Protocol: petctl/rl/shape_search.py (as M1c).

Run: python scripts/rl/m1d_odd_length_feasibility.py [--budget 3500] [--polish 500] [--workers 6]
Writes data/rl/m1d_feasibility/table.json (resumable; one record per (shape, n)).
"""
from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

from petctl.rl.shape_search import search
from petctl.rl.target_shapes import make_shape

OUT = Path("data/rl/m1d_feasibility")
CATALOG = {
    "cyl_r3": ("cyl", (0.03,), 0.0), "cyl_r7": ("cyl", (0.07,), 0.0), "cyl_r8": ("cyl", (0.08,), 0.0), "cyl_r9": ("cyl", (0.09,), 0.0),
    "cyl_r11": ("cyl", (0.11,), 0.0), "square_8x8": ("box", (0.08, 0.08), 0.0), "rect_5x14": ("box", (0.05, 0.14), 0.0),
    "rounded_8x8_r2": ("rbox", (0.08, 0.08), 0.02),
}
PLAN = [(8, ["rounded_8x8_r2"]),
        (9, ["cyl_r3", "cyl_r7", "cyl_r8", "square_8x8", "rect_5x14", "rounded_8x8_r2"]),
        (11, ["cyl_r3", "cyl_r7", "cyl_r8", "cyl_r9", "cyl_r11", "square_8x8", "rect_5x14", "rounded_8x8_r2"])]


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--budget", type=int, default=3500)
    ap.add_argument("--polish", type=int, default=500)
    ap.add_argument("--workers", type=int, default=6)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--out", type=Path, default=OUT)
    a = ap.parse_args()
    a.out.mkdir(parents=True, exist_ok=True)
    tp = a.out / "table.json"
    rows = {(r["shape_name"], r["n_modules"]): r for r in json.loads(tp.read_text())["rows"]} if tp.exists() else {}
    t0 = time.time()
    for n, names in PLAN:
        for nm in names:
            if (nm, n) in rows:
                continue
            kind, dims, rc = CATALOG[nm]
            rec = search(n, make_shape(kind, dims, rc), a.budget, a.workers, a.seed, a.polish)
            rec["shape_name"] = nm
            rows[(nm, n)] = rec
            tp.write_text(json.dumps({"rows": [rows[k] for k in sorted(rows, key=lambda k: (k[1], k[0]))]}, indent=1))
            print(f"[{time.time() - t0:.0f}s] done {nm} n={n}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
