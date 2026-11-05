"""Reach-still task: get the highest point of any module as high as possible and hold that pose statically."""

from __future__ import annotations

from collections import deque
from typing import Any

import numpy as np

from petctl.rl.tasks.reach_up import MAX_PENETRATION_M, MIN_MODULE_Z, ReachUp
from petctl.sim.model_builder import PetSim

HOLD_S = 2.0                 # reported score: the height the pose sustained (never dropped below) for this long
PTS_PER_M = 1000.0           # reward: 1 point per mm of height change
PEAK_BONUS = 0.25            # extra points per mm of NEW running-peak height (pays for trying higher; holding still pays 4x more)
PENALTY_SCALE = 0.25         # the shared penalty weights were tuned for ~4 points/mm; rescale them to 1 point/mm


class ReachStill(ReachUp):
    """Reach as high as possible and stay there. Reward = change in height of the highest module point, 1 point per mm.

    The per-step reward telescopes to the height at the end of the episode: rise 150 mm -> +150, fall back -> -150, net 0, so
    flinging earns nothing and wobbling while balancing costs nothing. Height of an invalid (floor-clipping) step counts as 0.
    The reported score (``best.json``, "max so far") is the highest height *sustained* for ``HOLD_S`` seconds (the minimum valid
    height over the trailing window), so a brief spike never counts. Only the max matters, not the average.
    """

    name = "reach_still"

    def reset(self, sim: PetSim, rng: np.random.Generator) -> None:
        """Record the flat-start height and clear the stillness windows and record."""
        super().reset(sim, rng)
        self._win: deque[float] = deque(maxlen=max(2, int(round(HOLD_S * self.cfg.control_hz))))
        self.peak_still = 0.0
        self.peak_any = 0.0
        self.best_q = sim.joint_pos().copy()
        self.best_module = -1
        self.best_t = 0.0
        self._step = 0
        self._prev_h = 0.0
        self._peak_h = 0.0

    def distance(self, sim: PetSim) -> float:
        """Evaluation score: the still-held record gain (m)."""
        return self.peak_still

    def reward(self, sim: PetSim, action: np.ndarray, prev_action: np.ndarray) -> tuple[float, dict[str, float]]:
        """Held-record increment + shaping - clipping/energy/smoothness penalties."""
        w = self.cfg.reward
        self._step += 1
        tops = self._tops(sim)
        g = float(tops.max()) - self.h0
        ok, clip = self._valid(sim)
        self._win.append(g if ok else 0.0)
        held = min(self._win) if len(self._win) == self._win.maxlen else 0.0
        if ok:
            self.peak_any = max(self.peak_any, g)
        inc = 0.0
        if held > self.peak_still:
            inc, self.peak_still = held - self.peak_still, held
            self.best_q, self.best_module, self.best_t = sim.joint_pos().copy(), int(tops.argmax()), self._step / self.cfg.control_hz
        h = g if ok else 0.0
        tau = sim.joint_torque()
        terms = {
            "height_change": PTS_PER_M * (h - self._prev_h),
            "peak_bonus": PEAK_BONUS * PTS_PER_M * max(h - self._peak_h, 0.0),
            "clipping": -PENALTY_SCALE * w.validity * clip,
            "energy": -PENALTY_SCALE * w.energy * float(np.sum(tau ** 2)),
            "action_rate": -PENALTY_SCALE * w.action_rate * float(np.sum((action - prev_action) ** 2)),
        }
        self._prev_h = h
        self._peak_h = max(self._peak_h, h)
        return float(sum(terms.values())), terms

    def metrics(self, sim: PetSim) -> dict[str, float]:
        """Episode numbers (m, s): still-held record, best at any instant, current, when and which module."""
        return {"distance": self.peak_still, "peak_still_m": self.peak_still, "peak_any_m": self.peak_any,
                "current_m": self.gain(sim), "best_t_s": self.best_t, "best_module": float(self.best_module)}

    def pose(self) -> dict[str, Any]:
        """Joint angles (rad) at the still record and the module that was highest."""
        return {"joint_pos_rad": [float(x) for x in self.best_q], "top_module": int(self.best_module), "t_s": float(self.best_t)}
