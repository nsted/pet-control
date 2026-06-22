#!/usr/bin/env python3
"""
Fit GL40 II per-motor power model coefficients using Module 7.

The model being fitted (against measured bus current):
    I_bus = I_idle + torque_coeff × τ² × (V_nom / V_bus)   [copper loss]
                   + mech_coeff  × |τ × ω| / V_bus          [mechanical power]

where I_idle = I_electronics + base_a is measured during the idle phase and
subtracted so the regression only needs to explain the load-dependent terms.

Robot must be lying flat. All other motors should be unloaded.
Do NOT restrain the motor shaft — it must swing freely so torque and velocity
vary across the sweep and the two regression features are separable.

Joint geometry: the rotation axis is ~45 deg from the ground when the robot
lies flat, so rotating the joint moves the segment CoM up and down. Gravity
contributes an angle-dependent torque (tau_gravity ~ m*g*r*sin(angle)). This
is fine for calibration — the motor produces real torque fighting gravity,
giving a valid copper-loss signal. The regression fits tau_fb^2 regardless of
whether the torque source is gravity or inertia.

Phases:
  1  Sensor validation    — enable motor 7, wait for MIT replies and battery ADC
  2  Idle baseline        — 5 s at rest; measures I_electronics + per-motor base_a
  3  Dynamic sweeps       — sinusoidal position commands at varied amplitude/frequency
  4  Regression           — least-squares fit; cross-checks copper-loss at ω≈0

Raw samples are saved to a timestamped JSONL file for offline re-analysis.

Usage:
    python scripts/calibrate_power_model.py
    python scripts/calibrate_power_model.py --skip-prompts
    python scripts/calibrate_power_model.py --host pet-robot.local
"""

from __future__ import annotations

import argparse
import asyncio
import datetime
import json
import math
import time
from dataclasses import dataclass

import numpy as np

from petctl.backends.robot import ROBOT_DEFAULT_HOST, ROBOT_DEFAULT_PORT, RobotBackend
from petctl.config import MOTOR_LIMITS, POWER_BUDGET
from petctl.types import ServoCommand

MOTOR_ID = 7
TICK_HZ  = 30
DT       = 1.0 / TICK_HZ

# Sweep schedule: (amplitude_deg, frequency_hz, num_cycles).
# kp_default (0.4) used throughout — kp_max (1.5) with kd_max (0.04) is underdamped
# for the tail segment (kd_crit >> kd_max), causing motor oscillation that
# overwhelms the commanded velocity range and ruins the regression.
_SWEEPS: list[tuple[float, float, int]] = [
    (15.0, 0.10,  5),   # slow: many near-zero-velocity points for copper-loss cross-check
    (30.0, 0.20,  6),   # medium: moderate torque and velocity
    (45.0, 0.30,  7),   # larger amplitude: higher inertial torque
    (20.0, 0.50, 10),   # fast: higher velocity, separates mech_coeff
    (10.0, 1.00, 15),   # higher frequency: wider velocity range for mech_coeff fit
]

# Near-zero velocity threshold for copper-loss cross-check.
# 0.15 rad/s captures velocity zero crossings from the slow sweeps without
# including the high-velocity mechanical-power-dominated regime.
_OMEGA_STATIC = 0.15  # rad/s

# Battery ADC sanity: raw=0 gives V_bus≈-0.59V, I_bus≈12.5A from the calibration formula.
# Anything below 5 V indicates the head ADS1015 hasn't populated yet or isn't running.
_V_BUS_MIN_SANE = 5.0


@dataclass
class Sample:
    phase: str
    t: float
    tau: float     # motor torque feedback (Nm)
    omega: float   # motor velocity feedback (rad/s)
    i_bus: float   # total bus current (A)
    v_bus: float   # bus voltage (V)


def _prompt(msg: str, skip: bool) -> None:
    if skip:
        print(f"  [auto] {msg}")
    else:
        input(f"\n  >>> {msg} — press Enter to continue... ")


def _banner(title: str) -> None:
    print(f"\n{'─' * 60}")
    print(f"  {title}")
    print(f"{'─' * 60}")


async def _collect(
    backend: RobotBackend,
    duration_s: float,
    phase: str,
    cmd_fn=None,
) -> list[Sample]:
    """Run at TICK_HZ for duration_s, collecting one Sample per tick.

    cmd_fn(t_elapsed) → ServoCommand — called each tick and sent immediately.
    """
    samples: list[Sample] = []
    start = time.monotonic()
    while True:
        t0 = time.monotonic()
        elapsed = t0 - start
        if elapsed >= duration_s:
            break

        if cmd_fn is not None:
            await backend.send_commands([cmd_fn(elapsed)])

        state = await backend.get_state()
        samples.append(Sample(
            phase=phase,
            t=elapsed,
            tau=state.motor_torques.get(MOTOR_ID, 0.0),
            omega=state.motor_velocities.get(MOTOR_ID, 0.0),
            i_bus=state.battery_current_amps,
            v_bus=state.battery_voltage_v,
        ))

        await asyncio.sleep(max(0.0, DT - (time.monotonic() - t0)))

    return samples


def _hold_cmd(pos: float) -> ServoCommand:
    return ServoCommand(
        servo_id=MOTOR_ID,
        position=pos,
        kp=MOTOR_LIMITS.kp_default,
        kd=MOTOR_LIMITS.kd_default,
        torque_ff=0.0,
    )


async def _validate(backend: RobotBackend) -> float:
    """Enable motor 7, wait for MIT replies and battery ADC, return home position.

    Aborts (raises RuntimeError) if battery ADC isn't reporting or motor
    doesn't appear in position feedback after 4 s.

    NOTE: does NOT call disable_torques(). Disabling all motors sends MIT
    exit-motor-mode to every motor, which stops the TX loop and causes the
    Arduino to go silent — triggering a spurious 10 s reconnect that leaves
    the session in a broken state. Avoid it.
    """
    _banner("Phase 1 — Sensor validation (4 s warm-up)")

    state = await backend.get_state()
    home_pos = state.servo_positions.get(MOTOR_ID, 0.0)

    # Warm up: send hold commands so MIT reply frames start flowing.
    samples = await _collect(backend, 4.0, "warmup", lambda t: _hold_cmd(home_pos))

    # --- Battery ADC check ---
    v_vals = [s.v_bus for s in samples[-30:]]   # last ~1 s
    v_mean = float(np.mean(v_vals))
    i_vals = [s.i_bus for s in samples[-30:]]
    i_mean = float(np.mean(i_vals))
    print(f"  V_bus = {v_mean:.2f} V   I_bus = {i_mean:.3f} A")

    if v_mean < _V_BUS_MIN_SANE:
        raise RuntimeError(
            f"Battery ADC not reporting (V_bus={v_mean:.2f} V — raw=0 gives −0.59 V).\n"
            "  Check that the head module is powered and the ADS1015 is initialised.\n"
            "  Without bus-current telemetry this calibration cannot run."
        )

    # --- Motor feedback check ---
    tau_vals = [s.tau for s in samples[-30:]]
    pos_ok = MOTOR_ID in (await backend.get_state()).servo_positions
    tau_nonzero = any(abs(t) > 1e-4 for t in tau_vals)
    if not pos_ok and not tau_nonzero:
        raise RuntimeError(
            f"Motor {MOTOR_ID} not responding — no position or torque feedback after 4 s.\n"
            "  Verify the motor is powered and the CAN bus is active."
        )

    print(f"  Motor {MOTOR_ID} feedback OK  (home = {math.degrees(home_pos):.1f}°)")
    return home_pos


async def _idle_baseline(backend: RobotBackend, home_pos: float) -> float:
    """5 s idle at home position. Returns mean I_bus (electronics + per-motor base_a)."""
    _banner("Phase 2 — Idle baseline (5 s at rest)")

    samples = await _collect(backend, 5.0, "idle", lambda t: _hold_cmd(home_pos))

    i_idle = float(np.mean([s.i_bus for s in samples]))
    v_mean = float(np.mean([s.v_bus for s in samples]))
    tau_mean = float(np.mean([abs(s.tau) for s in samples]))
    print(f"  I_idle = {i_idle:.4f} A   V_bus = {v_mean:.2f} V   |τ_fb| ≈ {tau_mean:.4f} Nm")
    print(f"  (I_idle includes electronics quiescent + motor-7 idle draw)")
    return i_idle


async def _dynamic_sweeps(backend: RobotBackend, home_pos: float) -> list[Sample]:
    """Sinusoidal position sweeps at kp_default across 5 (amplitude, frequency) pairs."""
    _banner("Phase 3 — Dynamic sweeps")
    all_samples: list[Sample] = []

    for amp_deg, freq_hz, n_cycles in _SWEEPS:
        amp_rad = math.radians(amp_deg)
        duration_s = n_cycles / freq_hz
        omega_peak = amp_rad * 2 * math.pi * freq_hz

        print(
            f"\n  Sweep ±{amp_deg:.0f}°  {freq_hz:.2f} Hz  "
            f"{n_cycles} cycles  ({duration_s:.0f} s)  ω_peak≈{omega_peak:.2f} rad/s"
        )

        def cmd_fn(t: float, _a=amp_rad, _f=freq_hz, _h=home_pos) -> ServoCommand:
            return ServoCommand(
                servo_id=MOTOR_ID,
                position=_h + _a * math.sin(2 * math.pi * _f * t),
                kp=MOTOR_LIMITS.kp_default,   # kp_max causes underdamped resonance
                kd=MOTOR_LIMITS.kd_default,
                torque_ff=0.0,
            )

        samples = await _collect(backend, duration_s, f"sweep_{amp_deg:.0f}deg_{freq_hz:.2f}hz", cmd_fn)
        tau_peak = float(np.max(np.abs([s.tau for s in samples])))
        print(f"  Collected {len(samples)} samples  |τ_fb|_max = {tau_peak:.4f} Nm")
        all_samples.extend(samples)

    return all_samples



def _fit(
    samples: list[Sample],
    i_idle: float,
) -> tuple[float, float]:
    """Least-squares fit of torque_coeff and mech_coeff.

    Fits: I_load = torque_coeff × τ² × (V_nom/V) + mech_coeff × |τω| / V
    where I_load = I_bus - I_idle strips the constant idle offset so the
    regression only explains load-dependent current.
    """
    V_nom = POWER_BUDGET.bus_voltage_nominal_v

    taus   = np.array([s.tau   for s in samples])
    omegas = np.array([s.omega for s in samples])
    i_bus  = np.array([s.i_bus for s in samples])
    v_bus  = np.clip(np.array([s.v_bus for s in samples]), 8.0, 40.0)

    i_load = i_bus - i_idle

    x1 = taus ** 2 * (V_nom / v_bus)       # copper-loss feature
    x2 = np.abs(taus * omegas) / v_bus      # mechanical-power feature

    # Oscillation sanity check: if most samples have velocity far above the commanded
    # sweep maximum (~1.6 rad/s), the motor was oscillating (likely kp too high).
    frac_high_omega = float(np.mean(np.abs(omegas) > 2.0))
    if frac_high_omega > 0.5:
        print(
            f"\n  WARNING: {100*frac_high_omega:.0f}% of samples have |omega| > 2 rad/s "
            f"(commanded max ≈1.6). Motor was likely oscillating — regression will be "
            f"unreliable. Check kp/kd settings in the sweep commands."
        )

    A = np.column_stack([x1, x2])
    coeffs, _, rank, sv = np.linalg.lstsq(A, i_load, rcond=None)

    torque_coeff = float(coeffs[0])
    mech_coeff   = float(coeffs[1])

    i_pred = A @ coeffs
    ss_res = float(np.sum((i_load - i_pred) ** 2))
    ss_tot = float(np.sum((i_load - float(np.mean(i_load))) ** 2))
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else float("nan")
    rms = math.sqrt(ss_res / max(len(samples), 1))

    print(f"\n  R² = {r2:.4f}   residual rms = {rms:.4f} A")
    print(f"  Matrix rank = {rank}   singular values = {sv}")

    # Cross-check: torque_coeff from near-static points only (ω ≈ 0)
    mask = np.abs(omegas) < _OMEGA_STATIC
    n_static = int(np.sum(mask))
    if n_static >= 5 and np.dot(x1[mask], x1[mask]) > 0:
        tc_static = float(np.dot(x1[mask], i_load[mask]) / np.dot(x1[mask], x1[mask]))
        print(f"  torque_coeff (near-static, n={n_static}): {tc_static:.3f}  [cross-check]")
    else:
        print(f"  (< 5 near-static points — copper-loss cross-check skipped)")

    return torque_coeff, mech_coeff


def _save_jsonl(samples: list[Sample], path: str) -> None:
    with open(path, "w") as f:
        for s in samples:
            f.write(json.dumps({
                "phase": s.phase, "t": s.t,
                "tau": s.tau, "omega": s.omega,
                "i_bus": s.i_bus, "v_bus": s.v_bus,
            }) + "\n")
    print(f"  Raw samples → {path}")


async def run(host: str, port: int, skip_prompts: bool) -> None:
    backend = RobotBackend(host=host, port=port, auto_reconnect=False)
    print(f"Connecting to {host}:{port} ...")
    if not await backend.connect():
        print("Connection failed — is the robot reachable?")
        return

    motors = backend.discovered_servos
    print(f"Connected.  Discovered motors: {sorted(motors)}")
    if MOTOR_ID not in motors:
        print(f"Motor {MOTOR_ID} not found — aborting.")
        await backend.disconnect()
        return

    print(
        "\n  Robot must be lying flat. Module 7 joint must be free to rotate.\n"
        "  All other modules should be unloaded."
    )
    _prompt("Confirm robot is flat and clear", skip_prompts)

    home_pos = 0.0
    try:
        # ── Phase 1: validate ──────────────────────────────────────────────────
        home_pos = await _validate(backend)

        # ── Phase 2: idle baseline ─────────────────────────────────────────────
        _prompt("Phase 2 ready (5 s idle at current position)", skip_prompts)
        i_idle = await _idle_baseline(backend, home_pos)

        # ── Phase 3: dynamic sweeps ────────────────────────────────────────────
        _prompt(
            "Phase 3 ready (sinusoidal sweeps — motor 7 will move up to ±45°, ~2.5 min total)",
            skip_prompts,
        )
        sweep_samples = await _dynamic_sweeps(backend, home_pos)

        print(f"\n  Total samples for regression: {len(sweep_samples)}")

        # ── Phase 4: regression ────────────────────────────────────────────────
        _banner("Phase 4 — Regression")
        torque_coeff, mech_coeff = _fit(sweep_samples, i_idle)

        # ── Results ────────────────────────────────────────────────────────────
        _banner("Results")
        print("  Paste into petctl/config.py PowerBudgetConfig:\n")
        print(f"    per_motor_torque_coeff: {torque_coeff:.3f}   # was {POWER_BUDGET.per_motor_torque_coeff:.1f}")
        print(f"    per_motor_mech_coeff:   {mech_coeff:.4f}   # was {POWER_BUDGET.per_motor_mech_coeff:.1f}")
        print(f"\n  Informational (not directly config values):")
        print(f"    i_idle (electronics + motor-7 base): {i_idle:.4f} A")
        print(f"    per_motor_base_a ≈ i_idle - I_electronics")
        print(f"    (measure I_electronics separately with a DC bench supply + ammeter)")

        if torque_coeff < 0:
            print(
                "\n  WARNING: torque_coeff < 0 — τ_fb signal was too weak relative to noise.\n"
                "  Check that τ_fb is non-zero during sweeps (printed above).\n"
                "  If |τ_fb|_max ≈ 0 the motor was not in MIT mode during sweeps."
            )
        if mech_coeff < 0:
            print(
                "\n  WARNING: mech_coeff < 0 — mechanical power term underdetermined.\n"
                "  Try faster sweeps or increase sweep amplitude."
            )

        # ── Save raw data ──────────────────────────────────────────────────────
        ts  = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        log = f"power_calib_{ts}.jsonl"
        _save_jsonl(sweep_samples, log)

    except RuntimeError as exc:
        print(f"\n  ABORTED: {exc}")

    finally:
        # Return to home and limp
        print("\n  Returning motor 7 to home ...")
        try:
            state = await backend.get_state()
            cur = state.servo_positions.get(MOTOR_ID, 0.0)
            steps = max(30, int(abs(cur - home_pos) / 0.02))
            for i in range(steps):
                frac = (i + 1) / steps
                await backend.send_commands([
                    ServoCommand(
                        servo_id=MOTOR_ID,
                        position=cur + (home_pos - cur) * frac,
                        kp=MOTOR_LIMITS.kp_default,
                        kd=MOTOR_LIMITS.kd_default,
                    )
                ])
                await asyncio.sleep(DT)
        except Exception:
            pass
        await backend.disable_torques()
        await backend.disconnect()
        print("Done.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Calibrate GL40 II power model coefficients")
    parser.add_argument("--host", default=ROBOT_DEFAULT_HOST)
    parser.add_argument("--port", type=int, default=ROBOT_DEFAULT_PORT)
    parser.add_argument("--skip-prompts", action="store_true")
    args = parser.parse_args()
    asyncio.run(run(args.host, args.port, args.skip_prompts))
