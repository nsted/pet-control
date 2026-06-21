#!/usr/bin/env python3
"""
Fit GL40 II per-motor power model coefficients using Module 7.

The model being fitted:
    I_bus = I_electronics + base_a
            + torque_coeff × τ² × (V_nom / V_bus)   [copper loss]
            + mech_coeff  × |τ × ω| / V_bus          [mechanical power]

Robot must be lying flat on a surface. The Module 7 joint is Z-axis revolute,
so gravity contributes no torque when the robot is horizontal. All other motors
should be in limp mode (not commanded).

Phases:
  1  Electronics baseline — all motors limp, 5 s of quiescent bus current
  2  Motor idle           — motor 7 enabled at rest, zero commanded torque, 5 s
  3  Dynamic sweep        — sinusoidal position commands at varying amplitude
                            and frequency; collects (τ, ω, I, V) at 30 Hz
  4  Regression           — least-squares fit; prints PowerBudgetConfig snippet

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
from typing import Sequence

import numpy as np

from petctl.backends.robot import ROBOT_DEFAULT_HOST, ROBOT_DEFAULT_PORT, RobotBackend
from petctl.config import MOTOR_LIMITS, POWER_BUDGET
from petctl.types import ServoCommand

MOTOR_ID   = 7
TICK_HZ    = 30
DT         = 1.0 / TICK_HZ

# Sweep schedule: (amplitude_deg, frequency_hz, num_cycles) — lower amps first.
# Four sweeps span a range of (τ, ω) combinations needed for a well-conditioned fit.
_SWEEPS: list[tuple[float, float, int]] = [
    (15.0, 0.10,  5),   # slow + small: low τ, low ω
    (30.0, 0.20,  6),   # medium: moderate τ and ω
    (45.0, 0.30,  7),   # larger amplitude: higher τ during acceleration
    (20.0, 0.50, 10),   # fast + medium: higher ω, separates mech_coeff
]

# Torque_ff steps for an optional stationary copper-loss sweep (Phase 3b).
# Sent with kp=kp_max, kd=kd_max to hold position while injecting torque.
# Motor will deflect slightly but ω stays near zero, isolating copper loss.
_TORQUE_STEPS = [0.0, 0.05, 0.10, 0.20, 0.35, 0.50]  # Nm; stays well below torque_max

# Near-zero velocity threshold for extracting copper-loss-only points
_OMEGA_ZERO_THRESHOLD = 0.05  # rad/s


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
    """Run for duration_s seconds at TICK_HZ, collecting one Sample per tick.

    cmd_fn(t_elapsed) → ServoCommand | None  — called each tick; None means
    no command sent this tick (used for baseline with motor limp).
    """
    samples: list[Sample] = []
    start = time.monotonic()
    while True:
        t0 = time.monotonic()
        elapsed = t0 - start
        if elapsed >= duration_s:
            break

        if cmd_fn is not None:
            cmd = cmd_fn(elapsed)
            if cmd is not None:
                await backend.send_commands([cmd])

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


async def _phase1_baseline(backend: RobotBackend) -> float:
    """Electronics-only quiescent current. All motors limp."""
    _banner("Phase 1 — Electronics baseline (all motors limp, 5 s)")
    await backend.disable_torques()
    await asyncio.sleep(0.5)  # let motor exit MIT mode

    samples = await _collect(backend, 5.0, "baseline")
    i_baseline = float(np.mean([s.i_bus for s in samples]))
    v_mean = float(np.mean([s.v_bus for s in samples]))
    print(f"  I_electronics = {i_baseline:.4f} A  (V_bus = {v_mean:.2f} V)")
    return i_baseline


async def _phase2_idle(
    backend: RobotBackend,
    i_baseline: float,
) -> tuple[float, float]:
    """Motor enabled at rest, zero commanded torque. Returns (base_a, home_pos)."""
    _banner("Phase 2 — Motor idle (5 s at rest)")

    # Read current position before enabling
    state = await backend.get_state()
    home_pos = state.servo_positions.get(MOTOR_ID, 0.0)
    print(f"  Motor 7 home position: {math.degrees(home_pos):.1f}°")

    def cmd_fn(t: float) -> ServoCommand:
        return ServoCommand(
            servo_id=MOTOR_ID,
            position=home_pos,
            kp=MOTOR_LIMITS.kp_default,
            kd=MOTOR_LIMITS.kd_default,
            torque_ff=0.0,
        )

    samples = await _collect(backend, 5.0, "idle", cmd_fn)
    i_idle = float(np.mean([s.i_bus for s in samples]))
    base_a = i_idle - i_baseline
    print(f"  I_motor_idle  = {i_idle:.4f} A")
    print(f"  base_a        = {base_a:.4f} A  (idle - electronics)")
    return base_a, home_pos


async def _phase3_sweeps(
    backend: RobotBackend,
    home_pos: float,
    skip_prompts: bool,
) -> list[Sample]:
    """Dynamic sweeps: sinusoidal position commands at varying amplitude/frequency."""
    _banner("Phase 3a — Dynamic sweeps")
    all_samples: list[Sample] = []

    for amp_deg, freq_hz, n_cycles in _SWEEPS:
        amp_rad = math.radians(amp_deg)
        duration_s = n_cycles / freq_hz
        omega_max = amp_rad * 2 * math.pi * freq_hz

        print(
            f"\n  Sweep: ±{amp_deg:.0f}°  {freq_hz:.2f} Hz  "
            f"{n_cycles} cycles  ({duration_s:.0f} s)  "
            f"ω_peak≈{omega_max:.2f} rad/s"
        )

        def cmd_fn(t: float, _a=amp_rad, _f=freq_hz, _h=home_pos) -> ServoCommand:
            pos = _h + _a * math.sin(2 * math.pi * _f * t)
            return ServoCommand(
                servo_id=MOTOR_ID,
                position=pos,
                kp=MOTOR_LIMITS.kp_default,
                kd=MOTOR_LIMITS.kd_default,
                torque_ff=0.0,
            )

        samples = await _collect(backend, duration_s, f"sweep_{amp_deg:.0f}deg_{freq_hz:.2f}hz", cmd_fn)
        all_samples.extend(samples)
        print(f"  Collected {len(samples)} samples")

    return all_samples


async def _phase3b_torque_steps(
    backend: RobotBackend,
    home_pos: float,
) -> list[Sample]:
    """Stationary torque injection to isolate copper-loss at near-zero velocity.

    Motor holds home position with kp_max while torque_ff is stepped. Because
    the robot is flat (no gravity load), ω stays near zero and copper loss
    dominates the bus current.
    """
    _banner("Phase 3b — Stationary copper-loss sweep (torque_ff steps)")
    all_samples: list[Sample] = []

    for tau_ff in _TORQUE_STEPS:
        hold_s = 2.0  # settle + collect at each torque level

        def cmd_fn(t: float, _h=home_pos, _tau=tau_ff) -> ServoCommand:
            return ServoCommand(
                servo_id=MOTOR_ID,
                position=_h,
                kp=MOTOR_LIMITS.kp_max,
                kd=MOTOR_LIMITS.kd_max,
                torque_ff=_tau,
            )

        # Half a second settle, then collect
        await _collect(backend, 0.5, "settle", cmd_fn)
        samples = await _collect(backend, hold_s, f"torque_step_{tau_ff:.2f}Nm", cmd_fn)

        tau_fb_mean = float(np.mean([abs(s.tau) for s in samples]))
        omega_mean  = float(np.mean([abs(s.omega) for s in samples]))
        i_mean      = float(np.mean([s.i_bus for s in samples]))
        print(
            f"  τ_ff={tau_ff:.2f} Nm  τ_fb≈{tau_fb_mean:.3f} Nm  "
            f"ω≈{omega_mean:.3f} rad/s  I_bus≈{i_mean:.3f} A"
        )
        all_samples.extend(samples)

    return all_samples


def _fit(
    samples: list[Sample],
    i_baseline: float,
    base_a: float,
) -> tuple[float, float]:
    """Least-squares fit of torque_coeff and mech_coeff.

    Fits: I_load = torque_coeff × τ² × (V_nom/V) + mech_coeff × |τ×ω| / V
    where I_load = I_bus - I_electronics - base_a.
    """
    V_nom = POWER_BUDGET.bus_voltage_nominal_v

    taus   = np.array([s.tau   for s in samples])
    omegas = np.array([s.omega for s in samples])
    i_bus  = np.array([s.i_bus for s in samples])
    v_bus  = np.clip(np.array([s.v_bus for s in samples]), 8.0, 40.0)

    i_load = i_bus - i_baseline - base_a

    x1 = taus ** 2 * (V_nom / v_bus)       # copper-loss feature
    x2 = np.abs(taus * omegas) / v_bus      # mechanical-power feature

    A = np.column_stack([x1, x2])
    coeffs, residuals, rank, sv = np.linalg.lstsq(A, i_load, rcond=None)

    torque_coeff = float(coeffs[0])
    mech_coeff   = float(coeffs[1])

    # Diagnostics
    i_pred = A @ coeffs
    ss_res = float(np.sum((i_load - i_pred) ** 2))
    ss_tot = float(np.sum((i_load - np.mean(i_load)) ** 2))
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else float("nan")

    print(f"\n  Fit quality  R²={r2:.4f}  residual_rms={math.sqrt(ss_res / len(samples)):.4f} A")
    print(f"  Matrix rank: {rank}  singular values: {sv}")

    # Separate copper-loss fit on near-zero-velocity points
    mask_static = np.abs(omegas) < _OMEGA_ZERO_THRESHOLD
    n_static = int(np.sum(mask_static))
    if n_static >= 5:
        x1_s = x1[mask_static]
        y_s  = i_load[mask_static]
        tc_static = float(np.dot(x1_s, y_s) / np.dot(x1_s, x1_s))
        print(f"  torque_coeff from {n_static} near-static points: {tc_static:.3f}  (cross-check)")
    else:
        print(f"  (< 5 near-static points — skipping copper-loss cross-check)")

    return torque_coeff, mech_coeff


def _save_jsonl(samples: list[Sample], path: str) -> None:
    with open(path, "w") as f:
        for s in samples:
            f.write(json.dumps({
                "phase": s.phase,
                "t": s.t,
                "tau": s.tau,
                "omega": s.omega,
                "i_bus": s.i_bus,
                "v_bus": s.v_bus,
            }) + "\n")
    print(f"\n  Raw samples saved → {path}")


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
        "\n  Robot must be lying flat on a surface with module 7 free to rotate.\n"
        "  All other modules should be unloaded (no external forces)."
    )
    _prompt("Confirm robot is flat and clear", skip_prompts)

    try:
        # ── Phase 1 ────────────────────────────────────────────────────────────
        i_baseline = await _phase1_baseline(backend)

        # ── Phase 2 ────────────────────────────────────────────────────────────
        _prompt("Phase 2 ready (motor 7 will enable at its current position)", skip_prompts)
        base_a, home_pos = await _phase2_idle(backend, i_baseline)

        # ── Phase 3a ───────────────────────────────────────────────────────────
        _prompt("Phase 3a ready (sinusoidal sweeps — motor 7 will move ±45° max)", skip_prompts)
        sweep_samples = await _phase3_sweeps(backend, home_pos, skip_prompts)

        # ── Phase 3b ───────────────────────────────────────────────────────────
        _prompt("Phase 3b ready (stationary torque steps — motor 7 holds position)", skip_prompts)
        step_samples = await _phase3b_torque_steps(backend, home_pos)

        all_phase3 = sweep_samples + step_samples
        print(f"\n  Total phase-3 samples: {len(all_phase3)}")

        # ── Phase 4: fit ───────────────────────────────────────────────────────
        _banner("Phase 4 — Regression")
        torque_coeff, mech_coeff = _fit(all_phase3, i_baseline, base_a)

        # ── Results ────────────────────────────────────────────────────────────
        _banner("Results")
        print(f"  Measured / fitted values (substitute in petctl/config.py):\n")
        print(f"    per_motor_base_a:       {base_a:.4f}   # was {POWER_BUDGET.per_motor_base_a}")
        print(f"    per_motor_torque_coeff: {torque_coeff:.3f}  # was {POWER_BUDGET.per_motor_torque_coeff}")
        print(f"    per_motor_mech_coeff:   {mech_coeff:.4f}  # was {POWER_BUDGET.per_motor_mech_coeff}")
        print(f"\n  Electronics quiescent (informational, not in config):")
        print(f"    i_electronics: {i_baseline:.4f} A")

        # Warn about suspicious fit results
        if torque_coeff < 0:
            print("\n  WARNING: torque_coeff is negative — data may be noisy or motion too gentle.")
            print("           Increase sweep amplitude or try again with a heavier load.")
        if mech_coeff < 0:
            print("\n  WARNING: mech_coeff is negative — mechanical power term underdetermined.")
            print("           The fast sweep (0.5 Hz) may have insufficient velocity range.")

        # ── Save raw data ──────────────────────────────────────────────────────
        ts  = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        log = f"power_calib_{ts}.jsonl"
        _save_jsonl(all_phase3, log)

    finally:
        # Return motor to home and limp
        print("\n  Returning motor 7 to home (0.0 rad) ...")
        home_state = await backend.get_state()
        cur = home_state.servo_positions.get(MOTOR_ID, 0.0)
        steps = max(30, int(abs(cur) / 0.02))  # ~0.02 rad/step ≈ 1°/step
        for i in range(steps):
            frac = (i + 1) / steps
            await backend.send_commands([
                ServoCommand(servo_id=MOTOR_ID, position=cur * (1 - frac),
                             kp=MOTOR_LIMITS.kp_default, kd=MOTOR_LIMITS.kd_default)
            ])
            await asyncio.sleep(DT)

        await backend.disable_torques()
        await backend.disconnect()
        print("Done.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Calibrate GL40 II power model coefficients")
    parser.add_argument("--host", default=ROBOT_DEFAULT_HOST)
    parser.add_argument("--port", type=int, default=ROBOT_DEFAULT_PORT)
    parser.add_argument(
        "--skip-prompts", action="store_true",
        help="Run all phases without waiting for Enter between them"
    )
    args = parser.parse_args()
    asyncio.run(run(args.host, args.port, args.skip_prompts))
