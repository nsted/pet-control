"""Reach-still task: get the highest point of any module as high as possible and hold that pose statically."""

from __future__ import annotations

from collections import deque
from typing import Any

import numpy as np

from petctl.rl.tasks.reach_up import MAX_PENETRATION_M, MIN_MODULE_Z, ReachUp
from petctl.sim.model_builder import PetSim

STILL_S = 1.0                # how long the body must be nearly motionless for a height to count
JOINT_SPEED_MAX = 0.1        # rad/s: every joint slower than this
COM_DRIFT_MAX = 0.005        # m: centre of mass moved less than this over the window


class ReachStill(ReachUp):
    """Score = the highest valid point of any module (gain over the flat start) reached while the body was held still.

    A height counts only if, over the trailing ``STILL_S`` seconds, every joint speed stayed under ``JOINT_SPEED_MAX`` and the
    centre of mass moved under ``COM_DRIFT_MAX``, so swinging or flinging earns nothing. Reward = growth of that running-max
    still height (telescopes to the episode record), plus small shaping terms (current height, and height while still) for gradient.
    The floor-clipping guard of ``reach_up`` still applies. Only the max matters, not the average.
    """

    name = "reach_still"

    def reset(self, sim: PetSim, rng: np.random.Generator) -> None:
        """Record the flat-start height and clear the stillness windows and record."""
        super().reset(sim, rng)
        n = max(2, int(round(STILL_S * self.cfg.control_hz)))
        self._spd: deque[float] = deque(maxlen=n)
        self._com: deque[np.ndarray] = deque(maxlen=n)
        self.peak_still = 0.0
        self.peak_any = 0.0
        self.best_q = sim.joint_pos().copy()
        self.best_module = -1
        self.best_t = 0.0
        self._step = 0

    def distance(self, sim: PetSim) -> float:
        """Evaluation score: the still-held record gain (m)."""
        return self.peak_still

    def reward(self, sim: PetSim, action: np.ndarray, prev_action: np.ndarray) -> tuple[float, dict[str, float]]:
        """Still-record increment + shaping - clipping/energy/smoothness penalties."""
        w = self.cfg.reward
        self._step += 1
        tops = self._tops(sim)
        g = float(tops.max()) - self.h0
        ok, clip = self._valid(sim)
        spd = float(np.max(np.abs(sim.joint_vel())))
        self._spd.append(spd)
        self._com.append(sim.data.subtree_com[1].copy())
        full = len(self._spd) == self._spd.maxlen
        still = bool(ok and full and max(self._spd) < JOINT_SPEED_MAX and np.linalg.norm(self._com[-1] - self._com[0]) < COM_DRIFT_MAX)
        if ok:
            self.peak_any = max(self.peak_any, g)
        inc = 0.0
        if still and g > self.peak_still:
            inc, self.peak_still = g - self.peak_still, g
            self.best_q, self.best_module, self.best_t = sim.joint_pos().copy(), int(tops.argmax()), self._step / self.cfg.control_hz
        calm = float(np.clip(1.0 - spd / 0.5, 0.0, 1.0))
        tau = sim.joint_torque()
        terms = {
            "still_peak_gain": 10.0 * w.height * inc,
            "height": 0.01 * w.height * g if ok else 0.0,
            "still_height": 0.03 * w.height * g * calm if ok else 0.0,
            "clipping": -w.validity * clip,
            "energy": -w.energy * float(np.sum(tau ** 2)),
            "action_rate": -w.action_rate * float(np.sum((action - prev_action) ** 2)),
        }
        return float(sum(terms.values())), terms

    def metrics(self, sim: PetSim) -> dict[str, float]:
        """Episode numbers (m, s): still-held record, best at any instant, current, when and which module."""
        return {"distance": self.peak_still, "peak_still_m": self.peak_still, "peak_any_m": self.peak_any,
                "current_m": self.gain(sim), "best_t_s": self.best_t, "best_module": float(self.best_module)}

    def pose(self) -> dict[str, Any]:
        """Joint angles (rad) at the still record and the module that was highest."""
        return {"joint_pos_rad": [float(x) for x in self.best_q], "top_module": int(self.best_module), "t_s": float(self.best_t)}
