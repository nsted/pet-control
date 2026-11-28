"""T21 bridge: the library selector through PetEnv on the DEV suite with the DEPLOYABLE target slew (0.35 rad/tick) vs a relaxed slew (the old CPG env had no ramp). DEV numbers, not gate numbers.

    PYTHONPATH=. python scripts/rl/s3_selector_bridge.py
"""

from __future__ import annotations

import json
import time

import numpy as np

from petctl.rl.s3.evaluator import dev_suite, run_selector_suite


def main() -> int:
    """Run both slew settings on the upright DEV suite."""
    scs = dev_suite(8)
    out = {}
    for name, slew in (("deployable_slew_0.35", None), ("relaxed_slew_10", 10.0)):
        t = time.time()
        rs = run_selector_suite(scs, procs=8, slew_rad=slew)
        M = [r["M"] for r in rs if r["M"] is not None]
        out[name] = {"M_mean": float(np.mean(M)), "M": M, "flipped": int(sum(r["up_dot_end"] < 0.2 for r in rs)), "I_mean": float(np.mean([r["I_mean"] for r in rs])), "families": [r["families"] for r in rs],
                     "seg_T": [r["seg_T"] for r in rs]}
        print(name, "M mean %.3f" % out[name]["M_mean"], [round(m, 2) for m in M], "flipped", out[name]["flipped"], "I_mean %.2f" % out[name]["I_mean"], "%.0fs" % (time.time() - t), flush=True)
    open("data/rl/s3_selector_bridge.json", "w").write(json.dumps(out))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
