"""Locomotion task: translate as far as possible along the initial heading at low energy and low bus current."""

from __future__ import annotations

from collections import deque

import numpy as np

from petctl.rl.config import BUS_A_PER_NM2, BUS_LIMIT_A
from petctl.rl.tasks.base import Task
from petctl.sim.model_builder import PetSim


class Locomote(Task):
    """Reward = progress along the initial heading over a window (not instantaneous speed), minus energy/smoothness/drift/tilt terms."""

    name = "locomote"
    score_label = "m moved"

    def reset(self, sim: PetSim, rng: np.random.Generator) -> None:
        """Record the heading (head forward axis projected on the floor) and the position window."""
        self.p0 = sim.head_pos().copy()
        fwd = sim.data.site_xmat[sim.model.site("head").id].reshape(3, 3)[:, 0].copy()
        fwd[2] = 0.0
        self.heading = fwd / max(np.linalg.norm(fwd), 1e-9)
        self.side = np.array([-self.heading[1], self.heading[0], 0.0])
        n = max(1, int(round(self.cfg.progress_window_s * self.cfg.control_hz)))
        self._hist: deque[np.ndarray] = deque([self.p0.copy()] * (n + 1), maxlen=n + 1)
        self._prev_qd = np.zeros(7)

    def tracked_pos(self, sim: PetSim) -> np.ndarray:
        """World position whose motion is rewarded (head here; subclasses may use the centre of mass)."""
        return sim.head_pos()

    def distance(self, sim: PetSim) -> float:
        """Total displacement along the initial heading (m); used for evaluation."""
        return float((sim.head_pos() - self.p0) @ self.heading)

    def reward(self, sim: PetSim, action: np.ndarray, prev_action: np.ndarray) -> tuple[float, dict[str, float]]:
        """See class docstring. Terms are returned unweighted-signed (already multiplied by their weights)."""
        w = self.cfg.reward
        dt = 1.0 / self.cfg.control_hz
        pos = self.tracked_pos(sim)
        self._hist.append(pos.copy())
        window = (len(self._hist) - 1) * dt
        d = self._hist[-1] - self._hist[0]
        v_prog = float(d @ self.heading) / window
        v_lat = float(d @ self.side) / window
        tau = sim.joint_torque()
        cap = np.maximum(sim.model.actuator_forcerange[:, 1], 1e-9)
        qd = sim.joint_vel()
        acc = (qd - self._prev_qd) / dt
        self._prev_qd = qd.copy()
        bus = 0.6 + BUS_A_PER_NM2 * float(np.sum(tau ** 2))             # idle ~0.6 A + fitted per-motor quadratic
        up = sim.data.site_xmat[sim.model.site("imu").id].reshape(3, 3)[2, 2]
        terms = {
            "progress": w.progress * float(np.clip(v_prog, -0.5, 0.5)),
            "energy": -w.energy * float(np.sum(tau ** 2)),
            "action_rate": -w.action_rate * float(np.sum((action - prev_action) ** 2)),
            "joint_acc": -w.joint_acc * float(np.sum(acc ** 2)),
            "saturation": -w.saturation * float(np.mean(np.abs(tau) >= 0.98 * cap)),
            "lateral": -w.lateral * abs(v_lat),
            "yaw": -w.yaw * abs(float(sim.data.sensor("gyro").data[2])),
            "tilt": -w.tilt * (1.0 - float(up)),
            "current": -w.current * max(0.0, bus - BUS_LIMIT_A),
            "alive": w.alive,
        }
        return float(sum(terms.values())), terms
