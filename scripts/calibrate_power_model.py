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

Calibration approach — torque pulses (kp=0, kd≈0):
  With kp=0, kd=_KD_PULSE (0.002), torque_ff=T, the motor accelerates smoothly.
  Using kd_max (0.04) would drive the motor to velocity equilibrium omega_eq=T/kd
  where tau_fb→0 and I_load→0 — no calibration signal. _KD_PULSE is chosen so
  that kd×omega_abort << T throughout, keeping tau_fb ≈ T and I_load measurable.
  Alternating +T/-T pulses keep the motor near home. Settling between pulses uses
  kd_max (strong regenerative braking) via _idle_cmd.
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
import os
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
_PULSE_TORQUES_NM: list[float] = [0.1, 0.2, 0.3, 0.4, 0.5]

# Pulse duration. Must be short enough that the motor doesn't reach vel_max
# (30 rad/s) before the pulse ends. With J ~ 0.003 kg·m² and T=0.5 Nm:
# t_max = J × vel_max / T = 0.003 × 30 / 0.5 = 0.18 s — so 0.1 s keeps ω low.
_PULSE_DURATION_S = 0.10

# Each (magnitude, direction) pair is repeated this many times for averaging.
# SNR improves as sqrt(N_REPS): 20 reps gives ~4.5× noise reduction.
_N_REPS = 20

# Abort a pulse early if |omega| exceeds this — motor is at saturation and
# regenerating; further samples corrupt the copper-loss measurement.
_OMEGA_MAX_PULSE = 15.0  # rad/s

# kd used during torque pulses. Must be << tau_ff/omega_abort so that
# kd×omega << tau_ff throughout the pulse, keeping tau_fb ≈ tau_ff.
# With kd_max=0.04 the motor equilibrates at omega_eq=tau_ff/kd (e.g. 12.5 rad/s
# at 0.5 Nm), driving tau_fb → 0 — no signal to fit. kd=0.002 gives
# omega_eq=125 rad/s (never reached) and kd×omega < 0.03 Nm at abort=15 rad/s.
# Settling uses kd_max (strong braking) via _idle_cmd; this only affects pulses.
_KD_PULSE = 0.002

# Velocity threshold used to decide "motor has settled" between pulses.
# kd-only braking has time constant J/kd ≈ 0.075 s; settling is fast.
_SETTLE_OMEGA_RAD_S = 0.10

# Maximum time to wait for settling between pulses before proceeding anyway.
_SETTLE_TIMEOUT_S = 5.0

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
    extra_cmds: list[ServoCommand] | None = None,
) -> list[Sample]:
    """Run at TICK_HZ for duration_s, collecting one Sample per tick.

    cmd_fn(t_elapsed) → ServoCommand — called each tick and sent immediately.
    extra_cmds are appended to every send_commands() batch (used to relax other motors).
    """
    samples: list[Sample] = []
    start = time.monotonic()
    while True:
        t0 = time.monotonic()
        elapsed = t0 - start
        if elapsed >= duration_s:
            break

        cmds: list[ServoCommand] = list(extra_cmds) if extra_cmds else []
        if cmd_fn is not None:
            cmds.append(cmd_fn(elapsed))
        if cmds:
            await backend.send_commands(cmds)

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


def _relax_cmd(servo_id: int, pos: float) -> ServoCommand:
    """kp=0: free-wheeling velocity damper for non-calibration motors.

    Sending this to motors 1-6 removes their position spring (kp_default=0.4)
    so they can't fight back when joint 7 is pulsed and excite body resonance.
    kd_max provides enough damping to prevent runaway.
    """
    return ServoCommand(
        servo_id=servo_id,
        position=pos,
        kp=0.0,
        kd=MOTOR_LIMITS.kd_max,
        torque_ff=0.0,
    )


def _torque_cmd(pos: float, torque_ff: float) -> ServoCommand:
    """kp=0 with torque feedforward: motor accelerates under torque_ff with minimal damping.

    Uses _KD_PULSE (not kd_max) so that kd×omega << torque_ff throughout the
    pulse and tau_fb ≈ torque_ff — the quantity we need for power regression.
    Settling between pulses uses _idle_cmd with kd_max for fast braking.
    """
    return ServoCommand(
        servo_id=MOTOR_ID,
        position=pos,
        kp=0.0,
        kd=_KD_PULSE,
        torque_ff=torque_ff,
    )


async def _wait_settled(
    backend: RobotBackend,
    pos: float,
    extra_cmds: list[ServoCommand] | None = None,
) -> float:
    """Send idle (kp=0, kd_max) commands until |omega| < threshold or timeout.

    Returns elapsed time. Prints a dot each second so the user can see progress.
    extra_cmds are sent alongside the idle command each tick (relax other motors).
    """
    start = time.monotonic()
    last_print = start
    while True:
        t0 = time.monotonic()
        elapsed = t0 - start
        if elapsed >= _SETTLE_TIMEOUT_S:
            print(f" (timeout after {elapsed:.0f} s)")
            break

        cmds = [_idle_cmd(pos)]
        if extra_cmds:
            cmds.extend(extra_cmds)
        await backend.send_commands(cmds)
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


async def _idle_baseline(
    backend: RobotBackend,
    home_pos: float,
    extra_cmds: list[ServoCommand] | None = None,
) -> tuple[float, float]:
    """5 s idle at home (kp=0). Returns (I_idle, tau_offset).

    tau_offset is the systematic zero offset in the MIT torque feedback. Since
    gravity is negligible and the motor is at rest with no spring force, any
    non-zero tau_fb is a sensor bias that must be subtracted before regression.
    extra_cmds are sent alongside each idle command (relax other motors).
    """
    _banner("Phase 2 — Idle baseline (5 s at rest)")

    # Wait until truly still, then collect.
    await _wait_settled(backend, home_pos, extra_cmds)
    samples = await _collect(backend, 5.0, "idle", lambda t: _idle_cmd(home_pos), extra_cmds)

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


async def _one_pulse(
    backend: RobotBackend,
    home_pos: float,
    t_ff: float,
    phase_label: str,
    extra_cmds: list[ServoCommand] | None = None,
) -> list[Sample]:
    """Apply one torque pulse, stopping early if |omega| exceeds _OMEGA_MAX_PULSE."""
    samples: list[Sample] = []
    start = time.monotonic()
    while True:
        t0 = time.monotonic()
        if t0 - start >= _PULSE_DURATION_S:
            break
        cmds = [_torque_cmd(home_pos, t_ff)]
        if extra_cmds:
            cmds.extend(extra_cmds)
        await backend.send_commands(cmds)
        state = await backend.get_state()
        omega = state.motor_velocities.get(MOTOR_ID, 0.0)
        samples.append(Sample(
            phase=phase_label,
            t=t0 - start,
            tau=state.motor_torques.get(MOTOR_ID, 0.0),
            omega=omega,
            i_bus=state.battery_current_amps,
            v_bus=state.battery_voltage_v,
        ))
        if abs(omega) >= _OMEGA_MAX_PULSE:
            break
        await asyncio.sleep(max(0.0, DT - (time.monotonic() - t0)))
    return samples


async def _torque_pulses(
    backend: RobotBackend,
    home_pos: float,
    extra_cmds: list[ServoCommand] | None = None,
) -> list[Sample]:
    """Repeated short +T/-T torque pulses to generate (tau, omega, I_bus) data.

    kp=0 throughout — no position spring, no oscillation. Each pulse is cut
    short if |omega| reaches _OMEGA_MAX_PULSE, keeping the motor in the
    resistive (copper-loss) regime and out of the regenerative regime at high ω.

    _N_REPS repetitions per (magnitude, direction) pair improve SNR by sqrt(N).
    extra_cmds are forwarded to each pulse and settle tick (relax other motors).
    """
    _banner("Phase 3 — Torque pulses (kp=0)")
    all_samples: list[Sample] = []

    for T in _PULSE_TORQUES_NM:
        for sign, label in [(+1, "+"), (-1, "−")]:
            t_ff = sign * T
            phase_label = f"pulse_{label}{T:.1f}Nm"
            rep_samples: list[Sample] = []

            for _rep in range(_N_REPS):
                pulse = await _one_pulse(backend, home_pos, t_ff, phase_label, extra_cmds)
                rep_samples.extend(pulse)
                await _wait_settled(backend, home_pos, extra_cmds)

            tau_mean = float(np.mean([abs(s.tau) for s in rep_samples]))
            omega_mean = float(np.mean([abs(s.omega) for s in rep_samples]))
            i_mean = float(np.mean([s.i_bus for s in rep_samples]))
            n_samp = len(rep_samples)
            print(
                f"  {label}{T:.1f} Nm × {_N_REPS} reps ({n_samp} samples):  "
                f"|τ_fb|={tau_mean:.4f} Nm  |ω|={omega_mean:.3f} rad/s  "
                f"I_bus={i_mean:.4f} A"
            )
            all_samples.extend(rep_samples)

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

        # Build relax commands for all other motors at their current positions.
        # kp=0 removes the position spring so they can't excite body resonance
        # when motor 7 is pulsed during calibration.
        state0 = await backend.get_state()
        other_ids = [m for m in motors if m != MOTOR_ID]
        relax_cmds: list[ServoCommand] = [
            _relax_cmd(mid, state0.servo_positions.get(mid, 0.0))
            for mid in other_ids
        ]
        if relax_cmds:
            print(f"  Relaxing motors {other_ids} (kp=0) to suppress body resonance.")
            await backend.send_commands(relax_cmds)

        # ── Phase 2: idle baseline ─────────────────────────────────────────────
        await _prompt("Phase 2 ready (5 s idle at current position)", skip_prompts)
        i_idle, tau_offset = await _idle_baseline(backend, home_pos, relax_cmds)

        # ── Phase 3: torque pulses ─────────────────────────────────────────────
        await _prompt(
            "Phase 3 ready (torque pulses — motor 7 will spin briefly, ~3 min total)",
            skip_prompts,
        )
        pulse_samples = await _torque_pulses(backend, home_pos, relax_cmds)

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
        os.makedirs("data", exist_ok=True)
        log = f"data/power_calib_{ts}.jsonl"
        _save_jsonl(pulse_samples, log)

    except RuntimeError as exc:
        print(f"\n  ABORTED: {exc}")

    finally:
        # Brake to a stop then disengage.
        print("\n  Braking motor 7 (kd-only) ...")
        try:
            await _wait_settled(backend, home_pos)  # no relax_cmds — cleanup only
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
