"""Jump task: leave the ground entirely and get the lowest point of the whole robot as high above the floor as possible."""

from __future__ import annotations

from typing import Any

import mujoco
import numpy as np

from petctl.rl.tasks.reach_up import MAX_PENETRATION_M, ReachUp
from petctl.sim.model_builder import PetSim

PTS_PER_M = 1000.0           # reward: 1 point per mm of NEW record clearance
SHAPE_PTS_PER_M = 100.0      # dense, potential-based hint: 0.1 point per mm of centre-of-mass rise (telescopes; cannot be farmed)
PENALTY_SCALE = 0.05         # shared penalty weights were tuned for ~4 points/mm and walking; jumping needs fast snaps, so go light
FLIGHT_W = 0.5               # each flight's own apex is paid at landing at this fraction of the record rate (rewards repeat/consistent hops)
PRED_W = 0.3                 # dense "predicted apex" record (clearance + vz^2/2g) at this fraction of the record rate
G = 9.81
MAX_SPEED_MS = 3.0           # CoM speed above this is a solver blow-up, not a jump (a 30 cm hop needs 2.4 m/s; motors cannot give 3.5)
MAX_CLR_STEP_M = 0.25        # clearance changing this much in one 0.05 s step (5 m/s) is a blow-up even if the sampled speed looks small
GLITCH_PENALTY = 300.0       # points; also ends the episode, since the state after a blow-up is meaningless


class Jump(ReachUp):
    """Reward = growth of the record *clearance*: the height of the lowest point of the lowest module above the floor.

    Clearance only counts while the robot is fully airborne (no contact with the floor, valid physics: contact penetration
    under 5 mm on the previous steps), so sliding, rolling, tipping and standing tall all score 0. The running-record
    increments telescope to the best clearance of the episode, so only the single highest jump matters (not the average, not
    how long it stays up). A small centre-of-mass-rise term (potential-based, telescopes to final CoM gain) gives an early
    gradient toward crouch-and-extend; the energy and smoothness penalties keep it from thrashing.
    """

    name = "jump"
    score_label = "m clearance"
    terminate_on_flip = False

    def reset(self, sim: PetSim, rng: np.random.Generator) -> None:
        """Clear the clearance record, the airborne timers and the flat-start CoM height."""
        super().reset(sim, rng)
        self._floor = sim.model.geom("floor").id
        self.record = 0.0                         # best airborne clearance (m)
        self.record_t = 0.0
        self.record_q = sim.joint_pos().copy()
        self.air_s = 0.0                          # current continuous airtime (s)
        self.best_air_s = 0.0                     # longest airtime so far (s)
        self.jumps = 0                            # number of separate take-offs
        self._was_air = False
        self._step = 0
        self._com0 = float(sim.data.subtree_com[1][2])
        self._prev_com = 0.0
        self._clr = 0.0
        self._prev_clr = 0.0
        self._flight_apex = 0.0                   # highest clearance of the flight in progress (m)
        self._pred_peak = 0.0
        self.glitch = False                       # the physics blew up (implausible speed): no further records this episode

    def terminated(self, sim: PetSim) -> bool:
        """End the episode when the physics has blown up (CoM speed beyond what the motors can impart)."""
        return self.glitch

    def _lows(self, sim: PetSim) -> np.ndarray:
        """Lowest world-z collision-mesh vertex of each of the eight modules (m)."""
        m, d = sim.model, sim.data
        out = np.empty(m.nbody - 1)
        for body in range(1, m.nbody):
            g = int(m.body_geomadr[body])
            adr, num = int(m.mesh_vertadr[m.geom_dataid[g]]), int(m.mesh_vertnum[m.geom_dataid[g]])
            v = m.mesh_vert[adr:adr + num]
            out[body - 1] = float(d.geom_xpos[g][2] + (v @ d.geom_xmat[g].reshape(3, 3).T)[:, 2].min())
        return out

    def clearance(self, sim: PetSim) -> float:
        """Height of the lowest point of the lowest module above the floor plane (m); can be slightly negative in contact."""
        return float(self._lows(sim).min())

    def _floor_contact(self, sim: PetSim) -> bool:
        """True if any module touches the floor right now."""
        d = sim.data
        for c in d.contact[:d.ncon]:
            if c.dist <= 0.0 and (c.geom1 == self._floor or c.geom2 == self._floor):
                return True
        return False

    def distance(self, sim: PetSim) -> float:
        """Evaluation score: the best airborne clearance of the episode (m)."""
        return self.record

    def reward(self, sim: PetSim, action: np.ndarray, prev_action: np.ndarray) -> tuple[float, dict[str, float]]:
        """Clearance-record increment + CoM-rise shaping - energy / smoothness / penetration penalties."""
        w = self.cfg.reward
        self._step += 1
        dt = 1.0 / self.cfg.control_hz
        clr = self.clearance(sim)
        contact = self._floor_contact(sim)
        pen = max([-c.dist for c in sim.data.contact[:sim.data.ncon]] + [0.0])
        air = (not contact) and clr > 0.0
        if air and not self._was_air:
            self.jumps += 1
        self._was_air = air
        self.air_s = self.air_s + dt if air else 0.0
        self.best_air_s = max(self.best_air_s, self.air_s)
        self._clr = clr if air else 0.0
        mujoco.mj_subtreeVel(sim.model, sim.data)
        vz = float(sim.data.subtree_linvel[1][2])
        gl = 0.0
        if (float(np.linalg.norm(sim.data.subtree_linvel[1])) > MAX_SPEED_MS or abs(clr - self._prev_clr) > MAX_CLR_STEP_M) and not self.glitch:
            self.glitch, gl = True, GLITCH_PENALTY
        inc = 0.0
        if air and not self.glitch and pen <= MAX_PENETRATION_M and clr > self.record:
            inc, self.record = clr - self.record, clr
            self.record_t, self.record_q = self._step * dt, sim.joint_pos().copy()
        self._prev_clr = clr
        com = float(sim.data.subtree_com[1][2]) - self._com0
        pred_inc = 0.0
        if not self.glitch and pen <= MAX_PENETRATION_M:
            pred = max(clr, 0.0) + (vz * vz / (2.0 * G) if vz > 0.0 else 0.0)
            pred_inc, self._pred_peak = max(pred - self._pred_peak, 0.0), max(self._pred_peak, pred)
        flight = 0.0
        if air and not self.glitch:
            self._flight_apex = max(self._flight_apex, clr)
        elif not air:
            flight, self._flight_apex = self._flight_apex, 0.0
        tau = sim.joint_torque()
        terms = {
            "clearance_record": PTS_PER_M * inc,
            "flight_apex": FLIGHT_W * PTS_PER_M * flight,
            "predicted_apex": PRED_W * PTS_PER_M * pred_inc,
            "glitch": -gl,
            "com_rise": SHAPE_PTS_PER_M * (com - self._prev_com),
            "clipping": -PENALTY_SCALE * w.validity * max(0.0, pen - MAX_PENETRATION_M),
            "energy": -PENALTY_SCALE * w.energy * float(np.sum(tau ** 2)),
            "action_rate": -PENALTY_SCALE * w.action_rate * float(np.sum((action - prev_action) ** 2)),
        }
        self._prev_com = com
        return float(sum(terms.values())), terms

    def metrics(self, sim: PetSim) -> dict[str, float]:
        """Episode numbers: record clearance (m), current clearance (m), longest airtime (s), take-offs, and when the record was set."""
        return {"distance": self.record, "record_clearance_m": self.record, "current_clearance_m": self._clr,
                "best_air_s": self.best_air_s, "jumps": float(self.jumps), "best_t_s": self.record_t}

    def pose(self) -> dict[str, Any]:
        """Joint angles (rad) at the moment the clearance record was set."""
        return {"joint_pos_rad": [float(x) for x in self.record_q], "top_module": -1, "t_s": float(self.record_t)}
