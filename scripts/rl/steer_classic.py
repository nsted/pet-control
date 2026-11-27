"""Classical steering on the gait library (no learning): pick the library gait whose measured velocity best matches the command; on a command change, ease the amplitudes down to a quiet pose
(``flatten_s``), switch the gait parameters there, then ramp the new gait up (``ramp_s``). Scores it with the CPGEnv per-mode metric against the plain selector (switch instantly).

    python scripts/rl/steer_classic.py --seeds 24 --flatten 1.0 --ramp 1.0
"""

from __future__ import annotations

import argparse

import numpy as np

from petctl.rl.cpg import PARAM_NAMES
from petctl.rl.cpg_env import NP, CPGEnv

QUIET = [PARAM_NAMES.index(k) for k in ("a_yaw", "a_twist")]


class Steer:
    """State machine over the env's selector base: RUN -> (command change) FLATTEN -> RAMP -> RUN."""

    def __init__(self, env: CPGEnv, flatten_s: float, ramp_s: float) -> None:
        self.env, self.fl, self.rp = env, max(flatten_s, 1e-6), max(ramp_s, 1e-6)
        self.cmd = env.cmd.copy()
        self.gait = env.prior(env.cmd)
        self.state, self.t = "RUN", 0.0

    def step(self) -> None:
        env, dt = self.env, self.env.dt
        if not np.allclose(env.cmd, self.cmd):                                                        # new command: start the transition
            self.cmd, self.state, self.t = env.cmd.copy(), "FLATTEN", 0.0
            self.next = env.prior(env.cmd)
        base = self.gait.copy()
        if self.state == "FLATTEN":
            self.t += dt
            k = min(1.0, self.t / self.fl)
            base[QUIET] = self.gait[QUIET] * (1 - k)
            if k >= 1.0:
                self.gait, self.state, self.t = self.next, "RAMP", 0.0
        elif self.state == "RAMP":
            self.t += dt
            k = min(1.0, self.t / self.rp)
            base[QUIET] = self.gait[QUIET] * k
            if k >= 1.0:
                self.state = "RUN"
        env.base = base


def run(classic: bool, seeds: int, flatten: float, ramp: float, randomise: bool) -> dict[str, float]:
    """Mean per-mode success over seeds, switching commands every 6 s."""
    acc: dict[str, list[float]] = {}
    for s in range(seeds):
        e = CPGEnv(randomise=randomise)
        e.reset(seed=1200 + s)
        ctl = Steer(e, flatten, ramp) if classic else None
        for _ in range(e.episode_ticks):
            if ctl:
                ctl.step()
            _o, _r, _t, _d, info = e.step(np.zeros(NP + 2))
        for k, v in info["metrics"].items():
            if k.startswith("mode_") or k == "success":
                acc.setdefault(k, []).append(v)
    return {k: round(float(np.mean(v)), 3) for k, v in acc.items()}


def main() -> int:
    """CLI entry point."""
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, default=24)
    ap.add_argument("--flatten", type=float, default=1.0)
    ap.add_argument("--ramp", type=float, default=1.0)
    a = ap.parse_args()
    for rnd in (False, True):
        print(f"randomise={rnd}  plain selector :", run(False, a.seeds, a.flatten, a.ramp, rnd))
        print(f"randomise={rnd}  classic steer :", run(True, a.seeds, a.flatten, a.ramp, rnd))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
