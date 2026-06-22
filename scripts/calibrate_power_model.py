#!/usr/bin/env python3
"""
Fit GL40 II per-motor power model coefficients using Module 7.

The model being fitted (against measured bus current):
    I_bus = I_idle + torque_coeff × τ² × (V_nom / V_bus)   [copper loss]
                   + mech_coeff  × |τ × ω| / V_bus          [mechanical power]

where I_idle = I_electronics + base_a is measured during the idle phase and
subtracted so the regression only needs to explain the load-dependent terms.

Robot must be lying flat. All other motors should be unloaded.
The module 7 joint must be free to rotate.

Joint notes:
  - Gravity is negligible at this joint (segment is balanced; holds any position).
  - Position control (kp > 0) with kd_max causes underdamped oscillation
    (settling time ~18 s) — not useful for calibration.
  - MIT torque feedback has a large zero offset (~0.3 Nm). Corrected by
    measuring tau_offset in Phase 2 and subtracting before regression.

Calibration approach — torque pulses (kp=0):
  With kp=0, kd=kd_max, torque_ff=T, the motor accelerates smoothly with no
  position-feedback oscillation. tau_fb ≈ T during the transient before omega
  gets large. Alternating +T/-T pulses keep the motor near home.
  Both torque_coeff and mech_coeff are determined from the same dataset because
  tau and omega are separable across pulse magnitudes and within each pulse.

Phases:
  1  Sensor validation    — motor 7 MIT mode, battery ADC check
  2  Idle baseline        — 5 s at rest (kp=0); measures I_idle and tau_offset
  3  Torque pulses        — brief +T/-T pulses at 6 magnitudes, kp=0
  4  Regression           — 2-D least-squares fit of torque_coeff and mech_coeff

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
import sys
import time
from dataclasses import dataclass

import numpy as np

from petctl.backends.robot import ROBOT_DEFAULT_HOST, ROBOT_DEFAULT_PORT, RobotBackend
from petctl.config import MOTOR_LIMITS, POWER_BUDGET
from petctl.types import ServoCommand

MOTOR_ID = 7
TICK_HZ  = 30
DT       = 1.0 / TICK_HZ

# Torque magnitudes to apply as brief pulses. Each magnitude is applied twice:
# once in the + direction and once in the - direction so the motor stays near
# home and we sample both positive and negative tau_fb.
_PULSE_TORQUES_NM: list[float] = [0.1, 0.3, 0.5, 0.7, 1.0]

# How long each torque pulse lasts. Short enough that the motor doesn't drift
# far from home; long enough to get good tau and omega variation within a pulse.
# At T=1.0 Nm and J~0.09: omega_peak ≈ T/J × t = 5.5 rad/s after 0.5 s.
_PULSE_DURATION_S = 0.5

# Velocity threshold used to decide "motor has settled" between pulses.
# The braking phase (kp=0, kd=kd_max, torque_ff=0) applies pure velocity damping
# with time constant J/kd ≈ 2 s; wait until the ringing/drift drops to this level.
_SETTLE_OMEGA_RAD_S = 0.05

# Maximum time to wait for settling between pulses before proceeding anyway.
_SETTLE_TIMEOUT_S = 20.0

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


async def _prompt(msg: str, skip: bool) -> None:
    """Print a prompt and wait for Enter without blocking the asyncio event loop.

    input() is a blocking call that freezes the entire event loop, killing the
    backend TX loop's RX watchdog and leaving the motor unresponsive. Running it
    in a thread executor lets the TX/receive loops keep ticking while we wait.
    """
    if skip:
        print(f"  [auto] {msg}")
    else:
        print(f"\n  >>> {msg} — press Enter to continue... ", end="", flush=True)
        await asyncio.to_thread(sys.stdin.readline)


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


def _idle_cmd(pos: float) -> ServoCommand:
    """kp=0: pure velocity damping, no position spring, no oscillation."""
    return ServoCommand(
        servo_id=MOTOR_ID,
        position=pos,
        kp=0.0,
        kd=MOTOR_LIMITS.kd_max,
        torque_ff=0.0,
    )


def _torque_cmd(pos: float, torque_ff: float) -> ServoCommand:
    """kp=0 with torque feedforward: motor accelerates under torque_ff with damping."""
    return ServoCommand(
        servo_id=MOTOR_ID,
        position=pos,
        kp=0.0,
        kd=MOTOR_LIMITS.kd_max,
        torque_ff=torque_ff,
    )


async def _wait_settled(backend: RobotBackend, pos: float) -> float:
    """Send idle (kp=0, kd_max) commands until |omega| < threshold or timeout.

    Returns elapsed time. Prints a dot each second so the user can see progress.
    """
    start = time.monotonic()
    last_print = start
    while True:
        t0 = time.monotonic()
        elapsed = t0 - start
        if elapsed >= _SETTLE_TIMEOUT_S:
            print(f" (timeout after {elapsed:.0f} s)")
            break

        await backend.send_commands([_idle_cmd(pos)])
        state = await backend.get_state()
        omega = state.motor_velocities.get(MOTOR_ID, 0.0)

        if t0 - last_print >= 1.0:
            print(f"  |ω| = {abs(omega):.3f} rad/s ...", end="\r", flush=True)
            last_print = t0

        if abs(omega) < _SETTLE_OMEGA_RAD_S:
            print(f"  settled in {elapsed:.1f} s (|ω| = {abs(omega):.3f} rad/s)        ")
            break

        await asyncio.sleep(max(0.0, DT - (time.monotonic() - t0)))

    return time.monotonic() - start


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

    # Warm up: send commands so MIT reply frames start flowing.
    # kp=0 avoids any position-spring oscillation during warm-up.
    samples = await _collect(backend, 4.0, "warmup", lambda t: _idle_cmd(home_pos))

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


async def _idle_baseline(backend: RobotBackend, home_pos: float) -> tuple[float, float]:
    """5 s idle at home (kp=0). Returns (I_idle, tau_offset).

    tau_offset is the systematic zero offset in the MIT torque feedback. Since
    gravity is negligible and the motor is at rest with no spring force, any
    non-zero tau_fb is a sensor bias that must be subtracted before regression.
    """
    _banner("Phase 2 — Idle baseline (5 s at rest)")

    # Wait until truly still, then collect.
    await _wait_settled(backend, home_pos)
    samples = await _collect(backend, 5.0, "idle", lambda t: _idle_cmd(home_pos))

    i_idle = float(np.mean([s.i_bus for s in samples]))
    v_mean = float(np.mean([s.v_bus for s in samples]))
    # With kp=0, no gravity, no motion: tau_fb reflects sensor zero offset only.
    tau_offset = float(np.mean([s.tau for s in samples]))
    omega_mean = float(np.mean([abs(s.omega) for s in samples]))
    print(f"  I_idle     = {i_idle:.4f} A")
    print(f"  V_bus      = {v_mean:.2f} V")
    print(f"  tau_offset = {tau_offset:.4f} Nm  (sensor zero bias; subtracted in regression)")
    print(f"  |omega|    = {omega_mean:.4f} rad/s  (should be near 0)")
    if omega_mean > _SETTLE_OMEGA_RAD_S * 2:
        print(f"  WARNING: motor still moving during idle baseline — tau_offset may be noisy.")
    return i_idle, tau_offset


async def _torque_pulses(backend: RobotBackend, home_pos: float) -> list[Sample]:
    """Brief +T/-T torque pulses to generate (tau, omega, I_bus) calibration data.

    kp=0 throughout — no position spring, no oscillation. With kp=0 and
    kd=kd_max, the motor responds to torque_ff smoothly: it accelerates during
    the pulse and decelerates under pure velocity damping between pulses.

    Each magnitude T is applied as a +T pulse then a -T pulse so the motor
    returns approximately to home before the next magnitude.

    The braking between pulses uses kd-only damping (time constant J/kd ≈ 2 s)
    and waits until |omega| < _SETTLE_OMEGA_RAD_S before proceeding.
    """
    _banner("Phase 3 — Torque pulses (kp=0)")
    all_samples: list[Sample] = []

    for T in _PULSE_TORQUES_NM:
        for sign, label in [(+1, "+"), (-1, "−")]:
            t_ff = sign * T
            print(f"\n  Pulse {label}{T:.1f} Nm × {_PULSE_DURATION_S:.1f} s ...")

            def cmd_fn(t: float, _pos=home_pos, _tff=t_ff) -> ServoCommand:
                return _torque_cmd(_pos, _tff)

            samples = await _collect(backend, _PULSE_DURATION_S, f"pulse_{label}{T:.1f}Nm", cmd_fn)

            tau_peak = float(np.max(np.abs([s.tau for s in samples])))
            omega_peak = float(np.max(np.abs([s.omega for s in samples])))
            i_mean = float(np.mean([s.i_bus for s in samples]))
            print(
                f"  |τ_fb|_peak = {tau_peak:.4f} Nm   "
                f"|ω|_peak = {omega_peak:.4f} rad/s   "
                f"I_bus_mean = {i_mean:.4f} A"
            )
            all_samples.extend(samples)

            # Brake and wait for motor to settle before next pulse.
            print(f"  Braking (kd-only) — waiting for |ω| < {_SETTLE_OMEGA_RAD_S} rad/s ...")
            await _wait_settled(backend, home_pos)

    return all_samples



def _fit(
    samples: list[Sample],
    i_idle: float,
    tau_offset: float,
) -> tuple[float, float]:
    """2-D least-squares fit of torque_coeff and mech_coeff from torque pulses.

    tau_fb is corrected for the MIT sensor zero offset measured in Phase 2.
    Fits: I_load = torque_coeff × τ_true² × (V_nom/V) + mech_coeff × |τ_true×ω| / V
    where τ_true = τ_fb - tau_offset.

    The two features are separable because:
    - torque_coeff varies across pulse magnitudes (τ² changes)
    - mech_coeff varies within each pulse as omega builds from zero
    """
    V_nom = POWER_BUDGET.bus_voltage_nominal_v

    taus_raw = np.array([s.tau   for s in samples])
    omegas   = np.array([s.omega for s in samples])
    i_bus    = np.array([s.i_bus for s in samples])
    v_bus    = np.clip(np.array([s.v_bus for s in samples]), 8.0, 40.0)

    taus   = taus_raw - tau_offset          # correct sensor zero offset
    i_load = i_bus - i_idle

    x1 = taus ** 2 * (V_nom / v_bus)       # copper-loss feature
    x2 = np.abs(taus * omegas) / v_bus      # mechanical-power feature

    A = np.column_stack([x1, x2])
    coeffs, _, rank, sv = np.linalg.lstsq(A, i_load, rcond=None)
    torque_coeff = float(coeffs[0])
    mech_coeff   = float(coeffs[1])

    i_pred = A @ coeffs
    ss_res = float(np.sum((i_load - i_pred) ** 2))
    ss_tot = float(np.sum((i_load - float(np.mean(i_load))) ** 2))
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else float("nan")
    rms = math.sqrt(ss_res / max(len(samples), 1))

    print(f"\n  tau_offset correction: {tau_offset:+.4f} Nm")
    print(f"  R² = {r2:.4f}   residual rms = {rms:.4f} A   n = {len(samples)}")
    print(f"  Matrix rank = {rank}   singular values = {sv}")

    # Per-pulse summary
    print(f"\n  Per-pulse (phase, |τ_true| Nm, |ω| rad/s, I_load A, fitted A, residual A):")
    for phase_name in dict.fromkeys(s.phase for s in samples):
        ph = [s for s in samples if s.phase == phase_name]
        tau_h  = float(np.mean([abs(s.tau - tau_offset) for s in ph]))
        om_h   = float(np.mean([abs(s.omega)            for s in ph]))
        il_h   = float(np.mean([s.i_bus                 for s in ph])) - i_idle
        x1_h   = float(np.mean([(s.tau-tau_offset)**2 * V_nom/max(s.v_bus,8.) for s in ph]))
        x2_h   = float(np.mean([abs((s.tau-tau_offset)*s.omega)/max(s.v_bus,8.) for s in ph]))
        fit_h  = torque_coeff * x1_h + mech_coeff * x2_h
        print(
            f"    {phase_name:22s}  τ={tau_h:.3f}  ω={om_h:.3f}  "
            f"I_load={il_h:+.4f}  fit={fit_h:+.4f}  res={il_h-fit_h:+.4f}"
        )

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
    await _prompt("Confirm robot is flat and clear", skip_prompts)

    home_pos = 0.0
    try:
        # ── Phase 1: validate ──────────────────────────────────────────────────
        home_pos = await _validate(backend)

        # ── Phase 2: idle baseline ─────────────────────────────────────────────
        await _prompt("Phase 2 ready (5 s idle at current position)", skip_prompts)
        i_idle, tau_offset = await _idle_baseline(backend, home_pos)

        # ── Phase 3: torque pulses ─────────────────────────────────────────────
        await _prompt(
            "Phase 3 ready (torque pulses — motor 7 will spin briefly, ~3 min total)",
            skip_prompts,
        )
        pulse_samples = await _torque_pulses(backend, home_pos)

        print(f"\n  Total samples for regression: {len(pulse_samples)}")

        # ── Phase 4: regression ────────────────────────────────────────────────
        _banner("Phase 4 — Regression")
        torque_coeff, mech_coeff = _fit(pulse_samples, i_idle, tau_offset)

        # ── Results ────────────────────────────────────────────────────────────
        _banner("Results")
        print("  Paste into petctl/config.py PowerBudgetConfig:\n")
        print(f"    per_motor_torque_coeff: {torque_coeff:.3f}   # was {POWER_BUDGET.per_motor_torque_coeff:.1f}")
        print(f"    per_motor_mech_coeff:   {mech_coeff:.4f}   # was {POWER_BUDGET.per_motor_mech_coeff:.1f}")
        print(f"\n  Informational (not directly config values):")
        print(f"    i_idle (electronics + motor-7 base): {i_idle:.4f} A")
        print(f"    per_motor_base_a ≈ i_idle - I_electronics")
        print(f"    (measure I_electronics separately with a DC bench supply + ammeter)")

        if math.isnan(torque_coeff) or torque_coeff < 0:
            print(
                "\n  WARNING: torque_coeff is invalid — τ_fb² feature was near zero or "
                "anti-correlated with I_load.\n"
                "  Check per-hold breakdown above: each hold should show positive I_load "
                "growing with |τ_fb|.\n"
                "  If all |τ_fb| ≈ 0, the motor was not in MIT mode. "
                "If I_load is negative at high τ, the motor may have been regenerating."
            )

        # ── Save raw data ──────────────────────────────────────────────────────
        ts  = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        log = f"power_calib_{ts}.jsonl"
        _save_jsonl(pulse_samples, log)

    except RuntimeError as exc:
        print(f"\n  ABORTED: {exc}")

    finally:
        # Brake to a stop then disengage.
        print("\n  Braking motor 7 (kd-only) ...")
        try:
            await _wait_settled(backend, home_pos)
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
