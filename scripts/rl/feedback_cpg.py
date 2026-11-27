"""Hand-written closed-loop controller on top of the library selector (headroom test for Phase 7): the amplitudes (a_yaw, a_twist) are scaled by a proportional correction of the measured speed along the
command (translation commands) or of the measured yaw rate (turn commands), using the env's privileged window-velocity observation.

    python scripts/rl/feedback_cpg.py --kp 0.8 --seeds 24
Reports the per-mode success of (a) the selector alone (zero action) and (b) selector + feedback, same seeds.
"""

from __future__ import annotations

import argparse

import numpy as np

from petctl.rl.cpg import PARAM_NAMES
from petctl.rl.cpg_env import NP, CPGEnv

AMP = [PARAM_NAMES.index("a_yaw"), PARAM_NAMES.index("a_twist")]


def feedback(env: CPGEnv, kp: float) -> np.ndarray:
    """Residual action: raise / lower both amplitudes with the normalised tracking error."""
    a = np.zeros(NP + 2)
    sp, sw = float(np.linalg.norm(env.cmd[:2])), abs(float(env.cmd[2]))
    if sp > 1e-6:
        err = (sp - float(env.meas[:2] @ (env.cmd[:2] / sp))) / sp
    elif sw > 1e-6:
        err = (sw - float(env.meas[2] * np.sign(env.cmd[2]))) / sw
    else:
        err = -1.0
    a[AMP] = np.clip(kp * err, -1.0, 1.0)
    return a


def run(use_fb: bool, kp: float, seeds: int) -> dict[str, float]:
    """Mean per-mode success over seeds."""
    acc: dict[str, list[float]] = {}
    for s in range(seeds):
        e = CPGEnv(randomise=True)
        e.reset(seed=900 + s)
        for _ in range(e.episode_ticks):
            _o, _r, _t, _d, info = e.step(feedback(e, kp) if use_fb else np.zeros(NP + 2))
        for k, v in info["metrics"].items():
            if k.startswith("mode_") or k == "success":
                acc.setdefault(k, []).append(v)
    return {k: round(float(np.mean(v)), 3) for k, v in acc.items()}


def main() -> int:
    """CLI entry point."""
    ap = argparse.ArgumentParser()
    ap.add_argument("--kp", type=float, default=0.8)
    ap.add_argument("--seeds", type=int, default=24)
    a = ap.parse_args()
    print("selector alone     ", run(False, a.kp, a.seeds))
    print(f"selector + feedback", run(True, a.kp, a.seeds), f"(kp {a.kp})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
