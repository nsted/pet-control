"""S3 baseline: the robust library selector (PHASE7_CPG_RL.md) replayed through PetEnv as a scripted joint-target policy. NOT RL: it is the bar the RL policy must beat.

Selection follows ``CPGEnv.prior`` (nearest library gait by measured (vx, vy, yaw), turning commands matched on yaw, f scaled to the commanded speed, stop = amplitudes and frequency zero). The CPG's own parameter
filter gives the ease-down / ease-up transitions. The policy reads the command from the task (privileged to the baseline only) and emits joint-target deltas (kp channels fixed).
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from petctl.rl.config import DELTA_STEP_RAD
from petctl.rl.cpg import CPG, PARAM_BOUNDS, PARAM_NAMES

MAX_SPEED_M_S, MAX_YAW_RAD_S = 0.18, 1.0            # the scales CPGEnv used (max_speed 0.18)
LABELS = "data/rl/cpg_elite_labels_n{n}.json"
F_RATIO = (0.4, 1.1)


class Selector:
    """Library gait selection and the per-tick joint-target generator."""

    def __init__(self, n_modules: int = 11, kp_action: float = 0.0, scale_f: bool = True) -> None:
        """Load the env-measured elite labels of this body length."""
        self.n, self.nj = n_modules, n_modules - 1
        rows = json.loads(Path(LABELS.format(n=n_modules)).read_text())
        self.elites = [np.array(r["params"], float) for r in rows]
        self.v = np.array([r["v"] for r in rows], float)
        self.scale_f = scale_f
        self.kp_a = kp_action
        self.lo = np.array([PARAM_BOUNDS[k][0] for k in PARAM_NAMES])
        self.hi = np.array([PARAM_BOUNDS[k][1] for k in PARAM_NAMES])
        self.cpg = CPG(self.nj)
        self.cpg.reset(np.where(np.arange(len(PARAM_NAMES)) == PARAM_NAMES.index("f"), 0.0, self.elites[0]))
        self._ready = False

    def params_for(self, cmd: np.ndarray) -> np.ndarray:
        """The library gait for ``cmd`` = (vx, vy, wz) (``CPGEnv.prior`` rules)."""
        if float(np.linalg.norm(cmd)) < 1e-6:
            t = self.elites[0].copy()
            t[[PARAM_NAMES.index("a_yaw"), PARAM_NAMES.index("a_twist"), PARAM_NAMES.index("f")]] = 0.0
            return t
        sc = np.array([MAX_SPEED_M_S, MAX_SPEED_M_S, MAX_YAW_RAD_S])
        if abs(cmd[2]) > 0.2 * MAX_YAW_RAD_S:
            i = int(np.argmin(np.abs(self.v[:, 2] - cmd[2]) / MAX_YAW_RAD_S + 0.5 * np.linalg.norm(self.v[:, :2], axis=1) / MAX_SPEED_M_S))
            return self.elites[i].copy()
        i = int(np.argmin(np.linalg.norm((self.v - cmd) / sc, axis=1)))
        t = self.elites[i].copy()
        if self.scale_f:
            es, cs = float(np.linalg.norm(self.v[i][:2])), float(np.linalg.norm(cmd[:2]))
            if cs > 1e-6 and es > 0.05:
                t[0] = float(np.clip(t[0] * float(np.clip(cs / es, *F_RATIO)), self.lo[0], self.hi[0]))
        return t

    def __call__(self, obs: np.ndarray, env, t: int) -> np.ndarray:
        """Action for the S3 env (``delta_kp``): joint-target deltas that follow the CPG target, constant kp channels."""
        cmd = env.task.cmd
        tgt = self.cpg.step(env.dt, self.params_for(cmd))
        a = np.zeros(2 * self.nj)
        a[:self.nj] = np.clip((tgt - env._tgt) / DELTA_STEP_RAD, -1.0, 1.0)
        a[self.nj:] = self.kp_a
        return a
