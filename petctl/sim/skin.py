"""Anisotropic skin friction for the module chain (the real skin friction is unmeasured; snake skins have lateral friction > forward friction).

MuJoCo friction is isotropic, so the model keeps its cone with mu = the floor friction as the FORWARD (chain-axis) friction and this class adds, per robot-vs-world contact, the
missing lateral share as an extra Coulomb force along the lateral direction ``l = n x f`` (f = the module's chain axis projected onto the contact tangent plane):

    F_extra = -(r - 1) * mu * N * tanh(v_l / V0) * l          r = ``aniso`` = mu_lateral / mu_forward >= 1,  v_l = contact-point slip velocity along l

so pure lateral slip feels r * mu * N, pure forward slip mu * N, and in between roughly an ellipse. The chain axis of module k is fixed in its body frame (taken from the
neighbour centres at the zero pose), so it follows the module when the body rolls or twists: rolling gaits see the anisotropy through the faces that touch the ground.
Off (``aniso`` = 1) it costs nothing; on it costs ~45 % throughput at stride 2 (6.2k -> 3.4k env steps/s at 10 envs, measured). Stride 3 overshoots the lateral stop (0.25 vs 0.40 of isotropic at 0.4 m/s), stride 4 loses the effect. The force is refreshed every ``stride`` physics substeps and held in between (cost / accuracy trade-off); each contact's force is capped so that a body's contacts together cannot reverse its slip within one hold interval (explicit-force stability).
"""

from __future__ import annotations

import mujoco
import numpy as np

V0 = 0.01            # m/s: slip speed over which the extra force ramps to full (smooth sign function)


class SkinFriction:
    """Adds the lateral share of an anisotropic friction to ``data.xfrc_applied`` (call ``apply`` before ``mj_step``)."""

    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData, n_modules: int, stride: int = 2) -> None:
        """Cache body / geom ids and the per-module chain axis (body frame, from the zero-pose centres: ``data`` must be at the zero pose)."""
        self.model, self.data, self.stride, self._k = model, data, max(1, int(stride)), 0
        self.bodies = np.array([int(model.body(f"m{k}").id) for k in range(n_modules)])
        self._is_robot_body = np.zeros(model.nbody, bool)
        self._is_robot_body[self.bodies] = True
        ctr = data.xipos[self.bodies]
        self.axis_local = np.zeros((n_modules, 3))
        for k in range(n_modules):
            a = ctr[min(k + 1, n_modules - 1)] - ctr[max(k - 1, 0)]
            a /= np.linalg.norm(a)
            self.axis_local[k] = data.xmat[self.bodies[k]].reshape(3, 3).T @ a
        self._row_of = np.zeros(model.nbody, int)
        self._row_of[self.bodies] = np.arange(n_modules)

    def apply(self, aniso: float) -> None:
        """Refresh ``xfrc_applied`` from the current contacts (every ``stride``-th call; otherwise the previous force stays applied). Vectorised over contacts (elliptic cone: the normal force is ``efc_force[efc_address]``)."""
        d, m = self.data, self.model
        self._k += 1
        if (self._k - 1) % self.stride:
            return
        d.xfrc_applied[self.bodies] = 0.0
        n = d.ncon
        if n == 0 or aniso <= 1.0:
            return
        c = d.contact
        b1, b2 = m.geom_bodyid[c.geom1[:n]], m.geom_bodyid[c.geom2[:n]]
        r1, r2 = self._is_robot_body[b1], self._is_robot_body[b2]
        idx = np.flatnonzero(r1 != r2)                           # robot vs world only (module-module contacts keep isotropic friction)
        if idx.size == 0:
            return
        rb = np.where(r1[idx], b1[idx], b2[idx])
        adr = c.efc_address[idx]
        N = np.where(adr >= 0, d.efc_force[np.maximum(adr, 0)], 0.0)
        keep = N > 0.0
        if not keep.any():
            return
        idx, rb, N = idx[keep], rb[keep], N[keep]
        nrm = c.frame[idx, 0:3]                                  # first row of the contact frame: the normal
        axis = np.einsum("kij,kj->ki", d.xmat[rb].reshape(-1, 3, 3), self.axis_local[self._row_of[rb]])
        fwd = axis - nrm * np.einsum("ki,ki->k", axis, nrm)[:, None]
        nf = np.linalg.norm(fwd, axis=1)
        ok = nf > 1e-6
        fwd = fwd[ok] / nf[ok, None]
        idx, rb, N, nrm = idx[ok], rb[ok], N[ok], nrm[ok]
        if idx.size == 0:
            return
        lat = np.cross(nrm, fwd)
        p = c.pos[idx]
        cv = d.cvel[rb]                                          # [rot, lin] about the subtree COM, world frame
        v_p = cv[:, 3:] + np.cross(cv[:, :3], p - d.subtree_com[m.body_rootid[rb]])
        v_l = np.einsum("ki,ki->k", v_p, lat)
        count = np.bincount(rb, minlength=m.nbody)
        mag = (aniso - 1.0) * c.friction[idx, 0] * N * np.tanh(np.abs(v_l) / V0)
        mag = np.minimum(mag, 0.5 * m.body_mass[rb] * np.abs(v_l) / (self.stride * float(m.opt.timestep) * count[rb]))      # stability: a body's contacts together never reverse its slip within one hold interval
        force = -(np.sign(v_l) * mag)[:, None] * lat
        torque = np.cross(p - d.xipos[rb], force)
        np.add.at(d.xfrc_applied[:, :3], rb, force)
        np.add.at(d.xfrc_applied[:, 3:], rb, torque)
