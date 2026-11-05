"""Reach-up task: get the highest point of any module as high as possible at any moment of the episode (no hold requirement)."""

from __future__ import annotations

import numpy as np

from petctl.rl.tasks.base import Task
from petctl.sim.model_builder import PetSim

MIN_MODULE_Z = 0.05          # a module origin below this means the body is clipping through / buried in the floor (rest is 0.08)
MAX_PENETRATION_M = 0.005    # contact penetration beyond this is a physics artefact, not a pose


class ReachUp(Task):
    """Reward = growth of the running-maximum height of the highest point on any module, over the flat start.

    The increments telescope to the episode's peak height, so the policy may reconfigure freely and be rewarded only for the
    highest point it reaches. A peak counts only while the pose is physically valid (no module origin below 5 cm, contact
    penetration under 5 mm); a naive height score was exploited by clipping the tail through the floor. A small shaping term
    on the current height gives a gradient early on.
    """

    name = "reach_up"
    terminate_on_flip = False                     # reaching poses may tilt the tail IMU arbitrarily

    def _tops(self, sim: PetSim) -> np.ndarray:
        """Highest world-z collision-mesh vertex of each of the eight modules (m)."""
        m, d = sim.model, sim.data
        out = np.empty(m.nbody - 1)
        for body in range(1, m.nbody):
            g = int(m.body_geomadr[body])
            adr, num = int(m.mesh_vertadr[m.geom_dataid[g]]), int(m.mesh_vertnum[m.geom_dataid[g]])
            v = m.mesh_vert[adr:adr + num]
            out[body - 1] = float(d.geom_xpos[g][2] + (v @ d.geom_xmat[g].reshape(3, 3).T)[:, 2].max())
        return out

    def _top_z(self, sim: PetSim) -> float:
        """Highest world-z collision-mesh vertex over ALL eight modules (orientation-independent 'how high is any part')."""
        return float(self._tops(sim).max())

    def reset(self, sim: PetSim, rng: np.random.Generator) -> None:
        """Record the flat-start height of the highest point."""
        self.h0 = self._top_z(sim)
        self.peak = 0.0
        self.p0 = sim.head_pos().copy()

    def gain(self, sim: PetSim) -> float:
        """Current height gain of the highest point over the flat start (m)."""
        return self._top_z(sim) - self.h0

    def distance(self, sim: PetSim) -> float:
        """Evaluation score: the peak valid gain of the highest point so far (m)."""
        return self.peak

    def _valid(self, sim: PetSim) -> tuple[bool, float]:
        d = sim.data
        zmin = min(float(d.xpos[i][2]) for i in range(1, sim.model.nbody))
        pen = max([-c.dist for c in d.contact[:d.ncon]] + [0.0])
        return zmin >= MIN_MODULE_Z and pen <= MAX_PENETRATION_M, max(0.0, MIN_MODULE_Z - zmin) + max(0.0, pen - MAX_PENETRATION_M)

    def reward(self, sim: PetSim, action: np.ndarray, prev_action: np.ndarray) -> tuple[float, dict[str, float]]:
        """Peak-increment reward, small height shaping, floor-clipping / energy / smoothness penalties."""
        w = self.cfg.reward
        g = self.gain(sim)
        ok, clip = self._valid(sim)
        new_peak = max(self.peak, g) if ok else self.peak
        inc, self.peak = new_peak - self.peak, new_peak
        tau = sim.joint_torque()
        terms = {
            "peak_gain": 10.0 * w.height * inc,
            "height": 0.02 * w.height * g if ok else 0.0,
            "clipping": -w.validity * clip,
            "energy": -w.energy * float(np.sum(tau ** 2)),
            "action_rate": -w.action_rate * float(np.sum((action - prev_action) ** 2)),
        }
        return float(sum(terms.values())), terms

    def metrics(self, sim: PetSim) -> dict[str, float]:
        """Episode numbers (m): peak valid height gain, and the current gain."""
        return {"distance": self.peak, "peak_m": self.peak, "current_m": self.gain(sim)}
