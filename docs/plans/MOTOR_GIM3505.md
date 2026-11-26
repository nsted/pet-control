# Motor: SteadyWin GIM3505-8 (owner data, 2026-11-26)

Replaces the GL40 II as the motor of the rebuilt robot (CLAUDE.md, `petctl/motors/gl40.py`, `docs/plans/POWER_MANAGEMENT.md` still describe the GL40 II: stale for the new robot).
Sources: owner's bench test (below) and the manufacturer manual (`Planetary-Gear-Motor-Series-Product-Manual-(GIM-Series)-GDK-driver.pdf`, updated 2025-07-14, 26 pages; local copy in the owner's Downloads).

## Datasheet (GIM3505-8, the 48 V rated version is the only -8 version the manual lists)
| item | value |
|---|---|
| gear | 8:1 planetary, aluminium gears, backlash 15 arcmin (0.25 deg) |
| rated voltage / range | 48 V / 12-60 V |
| rated torque / current | 0.79 N.m / 1.2 A (57.6 W) |
| stall (peak) torque / current | 2.01 N.m / 3.92 A |
| torque constant (output shaft) | 0.41 N.m/A |
| speed constant (output) | 12.59 rpm/V; rated 365 rpm, max 604.5 rpm (at 48 V) |
| line resistance / inductance | 5.47 ohm / 0.48 mH; 11 pole pairs |
| weight | 84 g motor, 97 g with driver (diameter 43 mm, 23 / 32 mm long) |
| interface | CAN and RS485, 14-bit on-board encoder, no second encoder; max axial / radial load 75 / 300 N |
The manual has NO MIT-mode / kp / kd / delay information (protocol, gain ranges, reply format still unknown), no rotor inertia, no friction figures.

## Owner's bench test
Motor Wizard, torque mode, holding a cantilever load at 3 cm lever arm, 13 V supply, no current limit. tau = m g 0.03.
| load (kg) | tau (N.m) | hold (A) | rotate (A) |
|---|---|---|---|
| idle | 0 | 0.02 | |
| 0.25 | 0.074 | 0.05 | 0.05 |
| 0.5 | 0.147 | 0.07 | 0.07 |
| 1.0 | 0.294 | 0.20 | 0.28 |
| 1.25 | 0.368 | 0.25 | 0.40 |
| 2.0 | 0.589 | 0.80 | 1.60 |
| 2.4 | 0.706 | 1.25 | (holds only) |

## Fit (`scripts/rl/fit_motor_current.py`)
- **Hold:** I = 0.02 + 2.4 tau^2 (A, N.m), error <= 0.07 A. **Rotate:** about 1.9x the hold coefficient (4.6 tau^2), error <= 0.16 A, but the rotation SPEED of the test is not stated, so
  the moving coefficient is only an upper-bound-ish guess.
- Physics check: bus current ~ 0.75 R_ll tau^2 / (Kt^2 V_bus) = 1.9 A/(N.m)^2 at 13 V, within 25 % of the fit, so the data is consistent with the datasheet. Current is quadratic in torque, not linear.
- **The old model (13.3 A per (N.m)^2 per motor, GL40 II) is ~5.5x too high for this motor.** The owner's suspicion was right.

## Consequences
1. At the 0.5 N.m cap one motor draws ~0.6 A holding, ~1.1 A moving at 13 V. 10 joints all holding at the cap: ~6 A (above the 4 A mean budget, below 10 A); all moving at the cap: ~11 A.
   So the 4 A / 10 A budgets still bind, but only when roughly 7+ joints work near the cap at once, instead of 1-2 joints under the old model. Static poses and statics (Phase 3, 5) were
   current-limited under the old model and mostly will not be now; the torque cap (0.5 N.m = 63 % of rated, 25 % of stall) becomes the limiting quantity.
2. The bus current scales with 1/V: the figures hold at 13 V; the robot's real battery voltage (earlier logs: 14.7 V) matters (about -12 % current at 14.7 V).
3. Speed: no-load ~12.59 rpm/V x 13 V = 164 rpm = 17 rad/s at the output (more at higher voltage). The sim's 6 rad/s slew limit is below it.
4. Backlash 0.25 deg is not modelled (small against the 0.5-rad action scale). Reflected rotor inertia (x64) and gear friction are unknown (armature DR 1e-4..1e-3 may not cover it).

## Needs the owner's approval (physical limits / current model, approval 2) before I change the sim
- Replace the current model in the sim: idle 0.02 A per motor + 2.4 A/(N.m)^2 hold + a moving term (4.6 A/(N.m)^2 until the test speed is known), scaled by 13 V / V_bus. The 10 A peak / 4 A
  mean budgets themselves stay as they are until the owner gives the real battery / bus limits. Until approved: new runs use the old coefficient ONLY as an unvalidated proxy (no gates).

## Still unknown (owner data would remove an assumption)
- MIT-mode / gain protocol of the GDK driver: kp, kd ranges, torque-feedforward, reply frame and rate, command delay (needed for dynamic gains and the delay randomisation).
- Rotation speed during the "rotate" runs; supply voltage of the finished robot; whether a 24 V -8 variant is used (the manual lists only 48 V for -8).
- Joint friction / back-drivability (turn a joint by hand with the motor off: torque to move it), rotor + gear inertia.
