#!/usr/bin/env python3
"""
Static-hold power model calibration for the GL40 II.

Motor 7 joint must be mechanically clamped before running — the script is
fully autonomous (no user interaction after launch).  With omega=0 enforced
by the clamp, torque_command = torque_ff exactly, and:

    I_load = torque_coeff × tau_fb² × (V_nom / V_bus)

giving a clean 1-parameter fit for torque_coeff.

mech_coeff cannot be calibrated this way (requires omega > 0), but free-spin
calibration is too noisy to resolve it. Use the reactive backstop for that term.

Usage:
    python scripts/calibrate_power_model_static.py
    python scripts/calibrate_power_model_static.py --host pet-robot.local
"""

from __future__ import annotations

import argparse
import asyncio
import datetime
import json
import math
import os
import time
from dataclasses import dataclass

import numpy as np

from petctl.backends.robot import ROBOT_DEFAULT_HOST, ROBOT_DEFAULT_PORT, RobotBackend
from petctl.config import MOTOR_LIMITS, POWER_BUDGET
from petctl.types import ServoCommand

MOTOR_ID = 7
TICK_HZ  = 30
DT       = 1.0 / TICK_HZ

# Torque levels to sweep, in Nm.
# Motor saturates at ~0.132 Nm with current overcurrent setting; sweep below
# that ceiling to get a multi-point fit for torque_coeff.
_TORQUE_LEVELS_NM: list[float] = [0.02, 0.04, 0.06, 0.08, 0.10, 0.12]

# Each level is held for this long after the ramp completes.
_HOLD_DURATION_S = 3.0

# Ramp from 0 → target over this many steps at this interval.
# Gives τ_fb time to settle and avoids stressing the clamp with a sudden impulse.
_RAMP_STEPS = 12
_RAMP_DT    = 0.08   # seconds per step → ~1 s total ramp

# Skip the first N ticks of each hold (τ_fb settling after ramp end).
_HOLD_SKIP_TICKS = 5

_V_BUS_MIN_SANE = 5.0


@dataclass
class Sample:
    phase: str
    t: float
    tau: float
    omega: float
    i_bus: float
    v_bus: float


def _banner(title: str) -> None:
    print(f"\n{'─' * 60}")
    print(f"  {title}")
    print(f"{'─' * 60}")


def _motor_cmd(pos: float, torque_ff: float = 0.0) -> ServoCommand:
    """kp=0, kd_max, torque_ff — motor pushes against clamp."""
    return ServoCommand(
        servo_id=MOTOR_ID,
        position=pos,
        kp=0.0,
        kd=MOTOR_LIMITS.kd_max,
        torque_ff=torque_ff,
    )


def _relax_cmd(servo_id: int, pos: float) -> ServoCommand:
    """kp=0, kd_max, no torque — keeps other motors limp for a stable I_idle."""
    return ServoCommand(servo_id=servo_id, position=pos, kp=0.0,
                        kd=MOTOR_LIMITS.kd_max, torque_ff=0.0)


async def _send(backend: RobotBackend, cmd: ServoCommand,
                extra: list[ServoCommand] | None = None) -> None:
    await backend.send_commands([cmd] + (extra or []))


async def _collect(
    backend: RobotBackend,
    pos: float,
    torque_ff: float,
    duration_s: float,
    phase: str,
    relax_cmds: list[ServoCommand],
) -> list[Sample]:
    samples: list[Sample] = []
    start = time.monotonic()
    while True:
        t0 = time.monotonic()
        elapsed = t0 - start
        if elapsed >= duration_s:
            break
        await _send(backend, _motor_cmd(pos, torque_ff), relax_cmds)
        state = await backend.get_state()
        samples.append(Sample(
            phase=phase, t=elapsed,
            tau=state.motor_torques.get(MOTOR_ID, 0.0),
            omega=state.motor_velocities.get(MOTOR_ID, 0.0),
            i_bus=state.battery_current_amps,
            v_bus=state.battery_voltage_v,
        ))
        await asyncio.sleep(max(0.0, DT - (time.monotonic() - t0)))
    return samples


async def _ramp(
    backend: RobotBackend,
    pos: float,
    t_start: float,
    t_end: float,
    relax_cmds: list[ServoCommand],
) -> None:
    for i in range(_RAMP_STEPS + 1):
        frac = i / _RAMP_STEPS
        await _send(backend, _motor_cmd(pos, t_start + frac * (t_end - t_start)), relax_cmds)
        await asyncio.sleep(_RAMP_DT)


async def _validate(backend: RobotBackend, motors: set[int]) -> tuple[float, list[ServoCommand]]:
    """4 s warm-up. Returns (home_pos, relax_cmds). Aborts on ADC/motor fault."""
    _banner("Phase 1 — Validation (4 s warm-up)")

    state = await backend.get_state()
    home_pos = state.servo_positions.get(MOTOR_ID, 0.0)
    relax_cmds = [
        _relax_cmd(mid, state.servo_positions.get(mid, 0.0))
        for mid in motors if mid != MOTOR_ID
    ]

    samples: list[Sample] = []
    start = time.monotonic()
    while time.monotonic() - start < 4.0:
        t0 = time.monotonic()
        await _send(backend, _motor_cmd(home_pos), relax_cmds)
        state = await backend.get_state()
        samples.append(Sample(
            phase="warmup", t=t0 - start,
            tau=state.motor_torques.get(MOTOR_ID, 0.0),
            omega=state.motor_velocities.get(MOTOR_ID, 0.0),
            i_bus=state.battery_current_amps,
            v_bus=state.battery_voltage_v,
        ))
        await asyncio.sleep(max(0.0, DT - (time.monotonic() - t0)))

    v_mean = float(np.mean([s.v_bus for s in samples[-30:]]))
    i_mean = float(np.mean([s.i_bus for s in samples[-30:]]))
    print(f"  V_bus = {v_mean:.2f} V   I_bus = {i_mean:.3f} A")

    if v_mean < _V_BUS_MIN_SANE:
        raise RuntimeError(
            f"Battery ADC not reporting (V_bus={v_mean:.2f} V). "
            "Check head module power and ADS1015 initialisation."
        )
    if MOTOR_ID not in (await backend.get_state()).servo_positions:
        raise RuntimeError(f"Motor {MOTOR_ID} not responding. Check power and CAN bus.")

    print(f"  Motor {MOTOR_ID} OK  (home = {math.degrees(home_pos):.1f}°)")
    return home_pos, relax_cmds


async def _idle_baseline(
    backend: RobotBackend,
    home_pos: float,
    relax_cmds: list[ServoCommand],
) -> tuple[float, float]:
    """5 s idle; returns (I_idle, tau_offset)."""
    _banner("Phase 2 — Idle baseline (5 s, torque_ff = 0)")
    samples = await _collect(backend, home_pos, 0.0, 5.0, "idle", relax_cmds)
    i_idle     = float(np.mean([s.i_bus for s in samples]))
    tau_offset = float(np.mean([s.tau   for s in samples]))
    omega_mean = float(np.mean([abs(s.omega) for s in samples]))
    print(f"  I_idle     = {i_idle:.4f} A")
    print(f"  tau_offset = {tau_offset:.4f} Nm  (sensor bias at rest)")
    print(f"  |omega|    = {omega_mean:.4f} rad/s  (should be ~0 if clamped)")
    return i_idle, tau_offset


async def _torque_sweep(
    backend: RobotBackend,
    home_pos: float,
    relax_cmds: list[ServoCommand],
) -> list[Sample]:
    """Sweep torque levels in both directions; joint is mechanically clamped."""
    _banner("Phase 3 — Torque sweep (joint clamped, fully autonomous)")
    all_samples: list[Sample] = []

    for T in _TORQUE_LEVELS_NM:
        for sign, label in [(+1, "+"), (-1, "−")]:
            t_ff      = sign * T
            phase_lbl = f"hold_{label}{T:.2f}Nm"

            await _ramp(backend, home_pos, 0.0, t_ff, relax_cmds)
            raw = await _collect(backend, home_pos, t_ff, _HOLD_DURATION_S, phase_lbl, relax_cmds)
            await _ramp(backend, home_pos, t_ff, 0.0, relax_cmds)

            usable = raw[_HOLD_SKIP_TICKS:]
            tau_mean  = float(np.mean([abs(s.tau)  for s in usable]))
            omega_rms = float(np.sqrt(np.mean([s.omega**2 for s in usable])))
            i_mean    = float(np.mean([s.i_bus      for s in usable]))

            print(
                f"  {label}{T:.2f} Nm: |τ_fb|={tau_mean:.4f} Nm  "
                f"ω_rms={omega_rms:.3f} rad/s  I_bus={i_mean:.4f} A"
            )
            all_samples.extend(usable)

    return all_samples


def _fit(samples: list[Sample], i_idle: float, tau_offset: float) -> float:
    """1-D OLS: I_load = torque_coeff × tau² × (V_nom/V_bus). Returns torque_coeff."""
    V_nom = POWER_BUDGET.bus_voltage_nominal_v

    taus  = np.array([s.tau   for s in samples]) - tau_offset
    i_bus = np.array([s.i_bus for s in samples])
    v_bus = np.clip(np.array([s.v_bus for s in samples]), 8.0, 40.0)

    i_load = i_bus - i_idle
    x1 = taus**2 * (V_nom / v_bus)

    torque_coeff = float(np.dot(x1, i_load) / np.dot(x1, x1))

    i_pred = torque_coeff * x1
    ss_res = float(np.sum((i_load - i_pred)**2))
    ss_tot = float(np.sum((i_load - float(np.mean(i_load)))**2))
    r2  = 1.0 - ss_res / ss_tot if ss_tot > 0 else float("nan")
    rms = math.sqrt(ss_res / max(len(samples), 1))

    print(f"\n  tau_offset correction: {tau_offset:+.4f} Nm")
    print(f"  R² = {r2:.4f}   residual rms = {rms:.4f} A   n = {len(samples)}")
    print(f"\n  Per-level summary:")
    for phase_name in dict.fromkeys(s.phase for s in samples):
        ph    = [s for s in samples if s.phase == phase_name]
        tau_h = float(np.mean([abs(s.tau - tau_offset) for s in ph]))
        il_h  = float(np.mean([s.i_bus for s in ph])) - i_idle
        x1_h  = float(np.mean([(s.tau - tau_offset)**2 * V_nom / max(s.v_bus, 8.) for s in ph]))
        fit_h = torque_coeff * x1_h
        print(
            f"    {phase_name:24s}  τ={tau_h:.4f}  "
            f"I_load={il_h:+.4f}  fit={fit_h:+.4f}  res={il_h-fit_h:+.4f}"
        )

    return torque_coeff


def _save_jsonl(samples: list[Sample], path: str) -> None:
    with open(path, "w") as f:
        for s in samples:
            f.write(json.dumps({
                "phase": s.phase, "t": s.t,
                "tau": s.tau, "omega": s.omega,
                "i_bus": s.i_bus, "v_bus": s.v_bus,
            }) + "\n")
    print(f"  Raw samples → {path}")


async def run(host: str, port: int) -> None:
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

    home_pos = 0.0
    relax_cmds: list[ServoCommand] = []
    try:
        home_pos, relax_cmds = await _validate(backend, motors)
        i_idle, tau_offset   = await _idle_baseline(backend, home_pos, relax_cmds)
        hold_samples         = await _torque_sweep(backend, home_pos, relax_cmds)

        if len(hold_samples) < 10:
            print("\n  Too few samples — check motor responses.")
            return

        _banner("Phase 4 — Regression")
        torque_coeff = _fit(hold_samples, i_idle, tau_offset)

        _banner("Results")
        print("  Paste into petctl/config.py PowerBudgetConfig:\n")
        print(f"    per_motor_torque_coeff: {torque_coeff:.3f}   # was {POWER_BUDGET.per_motor_torque_coeff:.2f}")
        print(
            f"\n  Note: mech_coeff not calibrated here (requires omega>0).\n"
            f"  Current value ({POWER_BUDGET.per_motor_mech_coeff}) left unchanged.\n"
            f"  The reactive EMA backstop covers residual modelling error."
        )

        ts  = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        os.makedirs("data", exist_ok=True)
        log = f"data/power_calib_static_{ts}.jsonl"
        _save_jsonl(hold_samples, log)

    except RuntimeError as exc:
        print(f"\n  ABORTED: {exc}")

    finally:
        print("\n  Releasing motor 7 ...")
        try:
            for _ in range(10):
                await _send(backend, _motor_cmd(home_pos, 0.0))
                await asyncio.sleep(DT)
        except Exception:
            pass
        await backend.disable_torques()
        await backend.disconnect()
        print("Done.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="GL40 II torque_coeff calibration — motor 7 must be mechanically clamped"
    )
    parser.add_argument("--host", default=ROBOT_DEFAULT_HOST)
    parser.add_argument("--port", type=int, default=ROBOT_DEFAULT_PORT)
    args = parser.parse_args()
    asyncio.run(run(args.host, args.port))
