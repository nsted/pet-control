"""Motor heat model (STAGE_1_GATES 1.4): per-joint first-order thermal RC driven by I^2 R, a quantised T_coil reading, and torque derating near the limit.

``C dT/dt = P - (T - T_amb) / R_th`` with ``P = 0.75 I^2 R_ll`` (three-phase motor, torque current ``I = tau / Kt``). The time constant ``tau_th = R_th C`` and the
starting temperature are randomised per episode. ALL thermal parameters are UNVALIDATED assumptions until the real-motor measurements (TASK_QUEUE T13). Task knobs: ``thermal`` (on),
``thermal_accel`` (time acceleration so a 15 s episode can heat a motor; default 1).
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

KT_NM_PER_A = 0.41                 # output-shaft torque constant (MOTOR_GIM3505.md)
R_LL_OHM = 5.47                    # line-to-line resistance
T_AMB_C = 25.0
R_TH_K_PER_W = 8.0                 # assumed winding-to-ambient thermal resistance (unvalidated)
TAU_TH_S = (60.0, 600.0)           # assumed thermal time constant range (unvalidated)
T_KNOB_C = 80.0                    # derating starts here (the driver's temperature-limit register reads 80 C in the manual's example)
T_LIMIT_C = 100.0                  # torque derated to zero here
T_START_C = (25.0, 85.0)           # randomised starting temperature
T_OBS_STEP_C = 1.0                 # the reply frame reports whole degrees
T_OBS_SCALE_C = 75.0               # observation = (T_coil - T_AMB) / this


def derate(temp_c: np.ndarray | float) -> np.ndarray:
    """Torque derating factor in [0, 1]: 1 up to ``T_KNOB_C``, linear down to 0 at ``T_LIMIT_C`` (monotone non-increasing)."""
    return np.clip((T_LIMIT_C - np.asarray(temp_c, float)) / (T_LIMIT_C - T_KNOB_C), 0.0, 1.0)


def heat_power_w(torque_nm: np.ndarray | float) -> np.ndarray:
    """Copper loss (W) at output torque ``torque_nm``: ``0.75 R_ll (tau / Kt)^2``."""
    return 0.75 * R_LL_OHM * (np.asarray(torque_nm, float) / KT_NM_PER_A) ** 2


@dataclass
class ThermalModel:
    """Per-joint T_coil state; ``step`` advances it with exact first-order integration (stable for any dt)."""

    n: int
    dt: float
    accel: float = 1.0
    tau_th_s: float = 300.0
    r_th: float = R_TH_K_PER_W
    temp: np.ndarray | None = None

    def __post_init__(self) -> None:
        """Start at ambient unless a state was given."""
        if self.temp is None:
            self.temp = np.full(self.n, T_AMB_C)

    @classmethod
    def sample(cls, n: int, dt: float, rng: np.random.Generator, scale: float = 1.0, accel: float = 1.0) -> ThermalModel:
        """Per-episode draw: tau_th log-uniform in ``TAU_TH_S`` and start temperatures uniform in ``T_START_C`` (both shrunk towards ambient / the middle by ``scale`` in [0, 1])."""
        lo, hi = np.log(TAU_TH_S[0]), np.log(TAU_TH_S[1])
        mid = 0.5 * (lo + hi)
        tau = float(np.exp(mid + scale * (rng.uniform(lo, hi) - mid)))
        start = T_AMB_C + scale * (rng.uniform(*T_START_C, size=n) - T_AMB_C)
        return cls(n, dt, accel, tau, R_TH_K_PER_W, start)

    def step(self, torque_nm: np.ndarray) -> np.ndarray:
        """Advance one control tick with the applied torque (N.m); returns the true temperatures."""
        t_ss = T_AMB_C + self.r_th * heat_power_w(torque_nm)
        a = 1.0 - np.exp(-self.dt * self.accel / self.tau_th_s)
        self.temp = self.temp + a * (t_ss - self.temp)
        return self.temp

    def reading(self) -> np.ndarray:
        """T_coil as the reply frame reports it: whole degrees."""
        return np.round(self.temp / T_OBS_STEP_C) * T_OBS_STEP_C

    def derating(self) -> np.ndarray:
        """Torque derating factor per joint from the TRUE temperature (the driver derates on its own measurement)."""
        return derate(self.temp)
