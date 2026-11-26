"""Per-robot bus-current model and spike clamp for the MuJoCo sim (Regimen v2 plan sec 5.5, assumption A2).

Model: ``bus = I_idle(n) + coeff * sum_j tau_j^2`` with ``I_idle(n) = 0.25 + 0.05 * n_joints`` A (0.6 A at 7 joints). The clamp mirrors the real
demand allocator: when the *demanded* current of a physics substep would exceed the peak limit, every actuator's ``forcerange`` is scaled down for
that substep so the prediction equals the limit, then restored. Off unless a ``BusClamp`` is attached to ``PetSim.bus``.
"""

from __future__ import annotations

import mujoco  # type: ignore[import-untyped]
import numpy as np

_add_reduce = np.add.reduce

IDLE_ELECTRONICS_A = 0.25      # assumption A2: electronics share of the idle current
IDLE_PER_MOTOR_A = 0.05        # assumption A2: per-motor idle current


def idle_current_a(n_joints: int) -> float:
    """Idle bus current (A) of a robot with ``n_joints`` motors."""
    return IDLE_ELECTRONICS_A + IDLE_PER_MOTOR_A * n_joints


def bus_current_a(tau: np.ndarray, coeff: float) -> float:
    """Bus current (A) for joint torques ``tau`` (N.m) under the model above."""
    return idle_current_a(len(tau)) + coeff * float(np.sum(np.square(tau)))


def soft_limit_current(i_dem: float, peak_a: float, knee: float) -> float:
    """Applied bus current (A) for a demanded current ``i_dem``: identity up to ``knee * peak_a``, then a smooth (C1) tanh roll-off that approaches ``peak_a`` asymptotically and never exceeds it.

    ``knee`` in [0, 1): 0 gives the hard cutoff ``min(i_dem, peak_a)``; e.g. 0.7 starts attenuating at 7 A for a 10 A ceiling.
    """
    k = knee * peak_a
    if knee <= 0.0 or i_dem <= k:
        return min(i_dem, peak_a)
    w = peak_a - k
    return k + w * float(np.tanh((i_dem - k) / w))


class BusClamp:
    """Per-substep current estimate with an optional spike limiter (hard cutoff or soft roll-off); accumulates per-tick statistics for ``end_tick``."""

    def __init__(self, model: mujoco.MjModel, coeff: float, peak_a: float, clamp: bool = True, soft_knee: float = 0.0, move_coeff: float | None = None,
                 move_omega0: float = 1.0, idle_a: float | None = None) -> None:
        """Bind to ``model``'s position actuators; ``coeff`` is A per (N.m)^2, ``peak_a`` the spike ceiling, ``soft_knee`` the fraction of ``peak_a`` where attenuation starts (0 = hard cutoff).

        ``move_coeff`` (default None = the legacy scalar model, bit-identical to before): a joint's coefficient rises linearly from ``coeff`` (still) to ``move_coeff`` at joint speed
        ``move_omega0`` rad/s (GIM3505-8 model, ``petctl.config.GIM3505_*``). ``idle_a``: total idle current (default: the legacy ``idle_current_a(n)``).
        """
        self.coeff, self.peak_a, self.clamp = float(coeff), float(peak_a), bool(clamp)
        self.soft_knee = min(max(float(soft_knee), 0.0), 0.99)
        self.n = model.nu
        self.idle = idle_current_a(self.n) if idle_a is None else float(idle_a)
        self.move_coeff = None if move_coeff is None else float(move_coeff)
        self.move_omega0 = float(move_omega0)
        self._qadr = np.array([model.jnt_qposadr[model.actuator_trnid[k, 0]] for k in range(self.n)])
        self._dadr = np.array([model.jnt_dofadr[model.actuator_trnid[k, 0]] for k in range(self.n)])
        for nm in ("_qadr", "_dadr"):                                                  # contiguous joint addresses -> a slice (a view) instead of a fancy-index copy
            ad = getattr(self, nm)
            setattr(self, nm[:2] + "sel", slice(int(ad[0]), int(ad[0]) + self.n) if np.array_equal(ad, np.arange(ad[0], ad[0] + self.n)) else ad)
        self._gain, self._bias = model.actuator_gainprm[:, 0].copy(), model.actuator_biasprm[:, :3].copy()
        self._bias0, self._bias1, self._bias2 = (self._bias[:, k].copy() for k in range(3))
        self._cap: np.ndarray | None = None
        self._clamped_this_sub = False
        self.begin_tick()

    def begin_tick(self) -> None:
        """Clear the per-tick accumulators."""
        self._sum_applied, self._n_sub, self._max_demand, self._max_applied, self._any_over = 0.0, 0, 0.0, 0.0, False
        self._sum_demand, self._n_dem = 0.0, 0

    def _weighted_ss(self, tau: np.ndarray, data: mujoco.MjData) -> float:
        """GIM3505-8 model: sum_j c_j tau_j^2 with c_j = coeff + (move_coeff - coeff) * min(|qdot_j| / move_omega0, 1)."""
        assert self.move_coeff is not None
        w = self.coeff + (self.move_coeff - self.coeff) * np.minimum(np.abs(data.qvel[self._dsel]) / self.move_omega0, 1.0)
        return float(_add_reduce(w * tau * tau))

    def demand(self, model: mujoco.MjModel, data: mujoco.MjData, cap: np.ndarray) -> np.ndarray:
        """Torque (N.m) the position actuators would produce this substep, capped at ``cap``."""
        f = self._gain * data.ctrl
        f += self._bias0
        f += self._bias1 * data.qpos[self._qsel]                                       # same left-to-right float ops as ``g*u + b0 + b1*q + b2*v``
        f += self._bias2 * data.qvel[self._dsel]
        return np.minimum(np.maximum(f, -cap), cap)                                    # == np.clip(f, -cap, cap), without its wrapper overhead

    def before(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        """Call after ``data.ctrl`` is set and before ``mj_step``: estimate demand and, if over the peak, scale ``forcerange`` for this substep."""
        fr = model.actuator_forcerange
        cap = fr[:, 1]
        tau = self.demand(model, data, cap)
        if self.move_coeff is None:
            ss = float(_add_reduce(tau * tau))                                         # add.reduce == np.sum
            wss = self.coeff * ss
        else:
            wss = self._weighted_ss(tau, data)
            ss = wss
        i_dem = self.idle + wss
        self._sum_demand += i_dem
        self._n_dem += 1
        if i_dem > self._max_demand:
            self._max_demand = i_dem
        self._any_over = self._any_over or i_dem > self.peak_a
        if self.clamp and i_dem > (self.soft_knee * self.peak_a if self.soft_knee > 0.0 else self.peak_a) and ss > 0.0:
            i_app = soft_limit_current(i_dem, self.peak_a, self.soft_knee)                 # hard: min(i_dem, peak); soft: smooth roll-off below the peak
            s = float(np.sqrt(max(i_app - self.idle, 0.0) / wss)) if self.peak_a > self.idle else 0.0
            lim = np.maximum(s * np.abs(tau), 1e-12)
            self._cap = cap.copy()
            fr[:, 0] = -lim
            fr[:, 1] = lim

    def after(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        """Call after ``mj_step``: record the applied current and restore the original ``forcerange``."""
        tau = data.actuator_force
        i_app = self.idle + (self.coeff * float(_add_reduce(tau * tau)) if self.move_coeff is None else self._weighted_ss(tau, data))
        self._sum_applied += i_app
        self._n_sub += 1
        if i_app > self._max_applied:
            self._max_applied = i_app
        if self._cap is not None:                                                      # only a clamped substep changed the forcerange
            fr = model.actuator_forcerange
            fr[:, 0] = -self._cap
            fr[:, 1] = self._cap
            self._cap = None

    def end_tick(self) -> dict[str, float]:
        """Statistics of the control tick just simulated: ``mean`` / ``peak`` (demanded, pre-clamp) / ``applied_peak`` (A) and ``over`` (demand exceeded the limit), ``demand_mean`` (mean demanded pre-clamp current over the tick's substeps = the 50 ms window value)."""
        out = {"mean": self._sum_applied / max(self._n_sub, 1), "peak": self._max_demand, "applied_peak": self._max_applied, "over": float(self._any_over),
               "demand_mean": self._sum_demand / max(self._n_dem, 1)}
        self.begin_tick()
        return out
