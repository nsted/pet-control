"""Cylinder-wrap range per length (Phase 3 statics) with the GIM3505-8 current model: largest cylinder radius around which a held pose wraps >= 180 deg with >= 5 consecutive modules.

    PETCTL_BUS_MOTOR=gim3505 python scripts/rl/wrap_statics.py --lengths 10 11 12 13 --radii 3 5 7 9 11 --budget 2000 --out data/rl/statics/wrap.json
Same protocol as M1c / M1d (petctl/rl/shape_search.py: static DE + polish, absolute-target PD, real cap, bus model, hold 3 s); only the current model differs. A "no" is "not found at this budget".
"""

from __future__ import annotations

import argparse
import json
import os
import time
from pathlib import Path

from petctl.rl.shape_search import search
from petctl.rl.target_shapes import make_shape


def main() -> int:
    """CLI entry point."""
    ap = argparse.ArgumentParser()
    ap.add_argument("--lengths", type=int, nargs="+", default=[10, 11, 12, 13])
    ap.add_argument("--radii", type=int, nargs="+", default=[3, 5, 7, 9, 11], help="cylinder radii in cm")
    ap.add_argument("--budget", type=int, default=2000)
    ap.add_argument("--polish", type=int, default=300)
    ap.add_argument("--workers", type=int, default=8)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--out", type=Path, default=Path("data/rl/statics/wrap.json"))
    a = ap.parse_args()
    assert os.environ.get("PETCTL_BUS_MOTOR") == "gim3505", "set PETCTL_BUS_MOTOR=gim3505 (the GIM3505-8 current model)"
    rows = json.loads(a.out.read_text()) if a.out.exists() else []
    done = {(r["n_modules"], r["radius_cm"]) for r in rows}
    t0 = time.time()
    for n in a.lengths:
        for r_cm in a.radii:
            if (n, r_cm) in done:
                continue
            rec = search(n, make_shape("cyl", (r_cm / 100.0,), 0.0), a.budget, a.workers, a.seed, a.polish)
            rec["radius_cm"], rec["motor_model"] = r_cm, "gim3505"
            rows.append({k: v for k, v in rec.items() if isinstance(v, (int, float, str, bool, list)) or v is None})
            a.out.parent.mkdir(parents=True, exist_ok=True)
            a.out.write_text(json.dumps(rows, indent=1))
            print(f"[{time.time() - t0:5.0f}s] n={n} r={r_cm} cm done", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
