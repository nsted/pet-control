"""Fit the GIM3505-8 bus-current-vs-torque data measured by the owner (2026-11-26) and compare with the physics estimate and the old GL40 II model.

Test: Motor Wizard, torque mode, cantilever at 3 cm lever arm, 13 V, no current limit. Load mass -> output torque tau = m g 0.03 N.m; current read from the supply.
Run: python scripts/rl/fit_motor_current.py
"""

from __future__ import annotations

import numpy as np

G, ARM_M, V_TEST = 9.81, 0.03, 13.0
OLD_A_PER_NM2 = 13.3                                  # petctl.rl.config.BUS_A_PER_NM2 (measured on the GL40 II robot at 14.7 V)
KT, R_LL = 0.41, 5.47                                 # datasheet: torque constant at the output (N.m/A), line resistance (ohm)

IDLE_A = 0.02
HOLD = {0.25: 0.05, 0.5: 0.07, 1.0: 0.2, 1.25: 0.25, 2.0: 0.8, 2.4: 1.25}        # kg -> A (holding still)
MOVE = {0.25: 0.05, 0.5: 0.07, 1.0: 0.28, 1.25: 0.4, 2.0: 1.6}                  # kg -> A (rotating)


def fit(data: dict[float, float]) -> tuple[float, float, np.ndarray]:
    """Least-squares I = I0 + k tau^2; returns (I0, k, residuals)."""
    tau = np.array(list(data)) * G * ARM_M
    cur = np.array(list(data.values()))
    a = np.c_[np.ones_like(tau), tau ** 2]
    sol, *_ = np.linalg.lstsq(a, cur, rcond=None)
    return float(sol[0]), float(sol[1]), a @ sol - cur


def main() -> None:
    """Print the fits and the predictions at the 0.5 N.m cap."""
    for name, d in (("hold", HOLD), ("rotate", MOVE)):
        i0, k, res = fit(d)
        print(f"{name}: I = {i0:+.3f} + {k:.2f} tau^2   (max |resid| {np.abs(res).max():.2f} A)   at 0.5 N.m: {i0 + k * 0.25:.2f} A")
    phys = 0.75 * R_LL / KT ** 2 / V_TEST
    print(f"physics (3/4 R_ll tau^2 / (Kt^2 V)): {phys:.2f} A/(N.m)^2 at {V_TEST:.0f} V;  old GL40 II model: {OLD_A_PER_NM2} A/(N.m)^2 -> ratio {OLD_A_PER_NM2 / fit(HOLD)[1]:.1f}x")
    print(f"10 motors holding at the 0.5 N.m cap: {10 * (fit(HOLD)[0] + fit(HOLD)[1] * 0.25):.1f} A (budget 4 A mean / 10 A peak)")


if __name__ == "__main__":
    main()
