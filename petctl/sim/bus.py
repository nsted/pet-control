"""Per-robot bus-current model and spike clamp for the MuJoCo sim (Regimen v2 plan sec 5.5, assumption A2).

Model: ``bus = I_idle(n) + coeff * sum_j tau_j^2`` with ``I_idle(n) = 0.25 + 0.05 * n_joints`` A (0.6 A at 7 joints). The clamp mirrors the real
demand allocator: when the *demanded* current of a physics substep would exceed the peak limit, every actuator's ``forcerange`` is scaled down for
that substep so the prediction equals the limit, then restored. Off unless a ``BusClamp`` is attached to ``PetSim.bus``.
"""

from __future__ import annotations

import mujoco  # type: ignore[import-untyped]
import numpy as np

IDLE_ELECTRONICS_A = 0.25      # assumption A2: electronics share of the idle current
IDLE_PER_MOTOR_A = 0.05        # assumption A2: per-motor idle current


def idle_current_a(n_joints: int) -> float:
    """Idle bus current (A) of a robot with ``n_joints`` motors."""
    return IDLE_ELECTRONICS_A + IDLE_PER_MOTOR_A * n_joints


def bus_current_a(tau: np.ndarray, coeff: float) -> float:
    """Bus current (A) for joint torques ``tau`` (N.m) under the model above."""
    return idle_current_a(len(tau)) + coeff * float(np.sum(np.square(tau)))


class BusClamp:
    """Per-substep current estimate with an optional spike clamp; accumulates per-tick statistics for ``end_tick``."""

    def __init__(self, model: mujoco.MjModel, coeff: float, peak_a: float, clamp: bool = True) -> None:
        """Bind to ``model``'s position actuators; ``coeff`` is A per (N.m)^2, ``peak_a`` the spike ceiling."""
        self.coeff, self.peak_a, self.clamp = float(coeff), float(peak_a), bool(clamp)
        self.n = model.nu
        self.idle = idle_current_a(self.n)
        self._qadr = np.array([model.jnt_qposadr[model.actuator_trnid[k, 0]] for k in range(self.n)])
        self._dadr = np.array([model.jnt_dofadr[model.actuator_trnid[k, 0]] for k in range(self.n)])
        self._gain, self._bias = model.actuator_gainprm[:, 0].copy(), model.actuator_biasprm[:, :3].copy()
        self._cap: np.ndarray | None = None
        self._clamped_this_sub = False
        self.begin_tick()

    def begin_tick(self) -> None:
        """Clear the per-tick accumulators."""
        self._sum_applied, self._n_sub, self._max_demand, self._max_applied, self._any_over = 0.0, 0, 0.0, 0.0, False

    def demand(self, model: mujoco.MjModel, data: mujoco.MjData, cap: np.ndarray) -> np.ndarray:
        """Torque (N.m) the position actuators would produce this substep, capped at ``cap``."""
        f = self._gain * data.ctrl + self._bias[:, 0] + self._bias[:, 1] * data.qpos[self._qadr] + self._bias[:, 2] * data.qvel[self._dadr]
        return np.clip(f, -cap, cap)

    def before(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        """Call after ``data.ctrl`` is set and before ``mj_step``: estimate demand and, if over the peak, scale ``forcerange`` for this substep."""
        cap = model.actuator_forcerange[:, 1].copy()
        self._cap = cap
        tau = self.demand(model, data, cap)
        ss = float(np.sum(tau * tau))
        i_dem = self.idle + self.coeff * ss
        self._max_demand = max(self._max_demand, i_dem)
        self._any_over = self._any_over or i_dem > self.peak_a
        if self.clamp and i_dem > self.peak_a and ss > 0.0:
            s = float(np.sqrt((self.peak_a - self.idle) / (self.coeff * ss))) if self.peak_a > self.idle else 0.0
            lim = np.maximum(s * np.abs(tau), 1e-12)
            model.actuator_forcerange[:, 0] = -lim
            model.actuator_forcerange[:, 1] = lim

    def after(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        """Call after ``mj_step``: record the applied current and restore the original ``forcerange``."""
        tau = data.actuator_force
        i_app = self.idle + self.coeff * float(np.sum(tau * tau))
        self._sum_applied += i_app
        self._n_sub += 1
        self._max_applied = max(self._max_applied, i_app)
        if self._cap is not None:
            model.actuator_forcerange[:, 0] = -self._cap
            model.actuator_forcerange[:, 1] = self._cap

    def end_tick(self) -> dict[str, float]:
        """Statistics of the control tick just simulated: ``mean`` / ``peak`` (demanded, pre-clamp) / ``applied_peak`` (A) and ``over`` (demand exceeded the limit)."""
        out = {"mean": self._sum_applied / max(self._n_sub, 1), "peak": self._max_demand, "applied_peak": self._max_applied, "over": float(self._any_over)}
        self.begin_tick()
        return out
