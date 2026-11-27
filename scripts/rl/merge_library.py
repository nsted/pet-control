"""Build the CPGEnv gait library (data/rl/cpg_elite_labels_n11.json, git-ignored) from the robust MAP-Elites archives: keep elites whose dispersion over conditions is < 40% of their speed (and the static ones),
label = median velocity over conditions.

    python scripts/rl/map_elites_robust.py --evals 6000 --seed 0 --out data/rl/map_elites/robust_n11.json
    python scripts/rl/map_elites_robust.py --evals 9000 --seed 1 --out data/rl/map_elites/robust_n11_s1.json
    python scripts/rl/merge_library.py
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np


def main() -> int:
    """CLI entry point."""
    rows = []
    for f in sorted(Path("data/rl/map_elites").glob("robust_n11*.json")):
        for x in json.loads(f.read_text()):
            if x["speed"] <= 0.03 or x["disp"] < 0.4 * x["speed"]:
                rows.append({"params": x["params"], "v": x["med"]})
    Path("data/rl/cpg_elite_labels_n11.json").write_text(json.dumps(rows))
    v = np.array([r["v"] for r in rows])
    print(f"{len(rows)} elites; best per axis: fwd {v[:, 0].max():.2f} back {-v[:, 0].min():.2f} left {v[:, 1].max():.2f} right {-v[:, 1].min():.2f} turn+ {v[:, 2].max():.2f} turn- {-v[:, 2].min():.2f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
