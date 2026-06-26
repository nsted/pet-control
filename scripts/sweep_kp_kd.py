#!/usr/bin/env python3
"""
Sweep (kp, kd) pairs on motor 7 to find stable MIT-mode operating points.

Each pair is tested with two phases:
  1. Hold test (3 s): measure torque std-dev at rest — catches kd-driven vibration
     from encoder velocity noise amplification.
  2. Step response (step + settle + return + watch): command ±STEP_DEG, let motor
     settle, command back, count zero-crossings during return transient — catches
     kp-driven oscillation when underdamped.

For each kp, kd is swept low→high. Sweep stops when vibration is detected
(higher kd will only be worse). Results are printed live and saved to JSON.
A lookup table of validated stable pairs is printed at the end — copy it
into config.py as STIFFNESS_TABLE.

Usage:
    python scripts/sweep_kp_kd.py
    python scripts/sweep_kp_kd.py --kp 1 2 5 10 20 --kd 0.05 0.1 0.2 0.5 1.0
    python scripts/sweep_kp_kd.py --step 8 --host pet-robot.local
"""

from __future__ import annotations

import argparse
import asyncio
import json
import math
import os
import statistics
import time
from dataclasses import asdict, dataclass, field

from petctl.backends.robot import ROBOT_DEFAULT_HOST, ROBOT_DEFAULT_PORT, RobotBackend
from petctl.types import ServoCommand

MOTOR_ID = 7
UPDATE_HZ = 50
DT = 1.0 / UPDATE_HZ

# ── Default sweep grid ────────────────────────────────────────────────────────
DEFAULT_KP = [1.0, 2.0, 5.0, 8.0, 10.0, 15.0, 20.0, 30.0]
DEFAULT_KD = [0.001, 0.002, 0.005, 0.010, 0.020, 0.035, 0.050, 0.070, 0.100]

# ── Test protocol ─────────────────────────────────────────────────────────────
STEP_DEG_DEFAULT = 12.0     # position disturbance size
HOLD_S = 1.0                # hold-test duration (torque variance)
SETTLE_S = 0.5              # time at step target before commanding return
RETURN_S = 1.5              # transient window after return command
COOLDOWN_S = 2.0            # zero-torque pause between tests to prevent overheating

# ── Safety ────────────────────────────────────────────────────────────────────
SAFETY_ERR_DEG = 60.0       # abort if position error exceeds this
SAFETY_VEL_RAD_S = 10.0    # abort if velocity exceeds this

# ── Stability thresholds ──────────────────────────────────────────────────────
# Velocity std-dev during hold above this → motor is oscillating at rest
OSCILLATION_VEL_STD_RAD_S = 2.0
# Torque std-dev at rest above this → kd amplifying encoder noise (vibration)
VIBRATION_TAU_STD_NM = 0.10
# Zero-crossing count of position error during return transient above this → oscillating
OSCILLATION_CROSSINGS = 4


# ── Data types ────────────────────────────────────────────────────────────────

@dataclass
class _Sample:
    t: float
    pos_rad: float
    vel_rad_s: float
    torque_nm: float


@dataclass
class TestResult:
    kp: float
    kd: float
    hold_tau_std: float = 0.0       # Nm — torque std at rest
    hold_vel_std: float = 0.0       # rad/s — velocity std during hold (oscillation indicator)
    step_crossings: int = 0         # zero-crossing count during return transient
    final_err_deg: float = 0.0      # residual position error after return window
    aborted: bool = False
    abort_reason: str = ""

    @property
    def vibrating(self) -> bool:
        return not self.aborted and self.hold_tau_std > VIBRATION_TAU_STD_NM

    @property
    def oscillating(self) -> bool:
        return not self.aborted and self.step_crossings > OSCILLATION_CROSSINGS

    @property
    def stable(self) -> bool:
        return not self.aborted and not self.vibrating and not self.oscillating

    @property
    def status(self) -> str:
        if self.aborted:
            return "ABORT"
        if self.vibrating:
            return "VIBRATE"
        if self.oscillating:
            return "OSCILLATE"
        return "STABLE"


# ── Core helpers ──────────────────────────────────────────────────────────────

async def _warmup(backend: RobotBackend, hold_pos: float, duration_s: float = 3.0) -> None:
    """Enable motor and hold position without safety gating so it can settle."""
    t_end = time.monotonic() + duration_s
    while time.monotonic() < t_end:
        t0 = time.monotonic()
        await backend.send_commands([
            ServoCommand(servo_id=MOTOR_ID, position=hold_pos, kp=1.0, kd=0.3, torque_ff=0.0)
        ])
        await backend.get_state()
        await asyncio.sleep(max(0.0, DT - (time.monotonic() - t0)))


async def _collect(
    backend: RobotBackend,
    target_pos: float,
    kp: float,
    kd: float,
    duration_s: float,
    settle_samples: int = 25,
) -> tuple[list[_Sample], str | None]:
    """Command hold at target_pos for duration_s. Returns (samples, abort_reason).

    Skips safety checks for the first settle_samples ticks (~0.5 s at 50 Hz)
    to let the motor shed stale velocity readings right after enable.
    """
    samples: list[_Sample] = []
    t_end = time.monotonic() + duration_s
    n = 0
    while time.monotonic() < t_end:
        t0 = time.monotonic()
        state = await backend.get_state()
        pos = state.servo_positions.get(MOTOR_ID, target_pos)
        vel = state.motor_velocities.get(MOTOR_ID, 0.0)
        tau = state.motor_torques.get(MOTOR_ID, 0.0)
        if n >= settle_samples:
            err_deg = abs(math.degrees(target_pos - pos))
            if err_deg > SAFETY_ERR_DEG:
                return samples, f"err={err_deg:.1f}° > {SAFETY_ERR_DEG}°"
        samples.append(_Sample(t=time.monotonic(), pos_rad=pos, vel_rad_s=vel, torque_nm=tau))
        await backend.send_commands([
            ServoCommand(servo_id=MOTOR_ID, position=target_pos, kp=kp, kd=kd, torque_ff=0.0)
        ])
        n += 1
        elapsed = time.monotonic() - t0
        await asyncio.sleep(max(0.0, DT - elapsed))
    return samples, None


async def _run_test(
    backend: RobotBackend,
    hold_pos: float,
    kp: float,
    kd: float,
    step_rad: float,
) -> TestResult:
    result = TestResult(kp=kp, kd=kd)

    # Phase 1: hold at rest — detect kd-driven vibration
    samples, abort = await _collect(backend, hold_pos, kp, kd, HOLD_S)
    if abort:
        result.aborted = True
        result.abort_reason = f"hold: {abort}"
        return result
    if len(samples) > 1:
        result.hold_tau_std = statistics.stdev(s.torque_nm for s in samples)
        result.hold_vel_std = statistics.stdev(s.vel_rad_s for s in samples)

    # Skip step test if already vibrating — saves motor heat on bad pairs
    if result.vibrating:
        return result

    # Phase 2: move to step target and settle
    step_target = hold_pos + step_rad
    samples, abort = await _collect(backend, step_target, kp, kd, SETTLE_S)
    if abort:
        result.aborted = True
        result.abort_reason = f"step_settle: {abort}"
        return result

    # Phase 3: command return, watch transient
    samples, abort = await _collect(backend, hold_pos, kp, kd, RETURN_S)
    if abort:
        result.aborted = True
        result.abort_reason = f"step_return: {abort}"
        return result

    if samples:
        errors = [hold_pos - s.pos_rad for s in samples]
        crossings = sum(
            1 for i in range(1, len(errors))
            if errors[i - 1] * errors[i] < 0
        )
        result.step_crossings = crossings
        result.final_err_deg = math.degrees(abs(errors[-1]))

    return result


# ── Main ──────────────────────────────────────────────────────────────────────

async def main(
    host: str,
    port: int,
    kp_grid: list[float],
    kd_grid: list[float],
    step_deg: float,
) -> None:
    backend = RobotBackend(host=host, port=port, auto_reconnect=False)
    print(f"Connecting to {host}:{port} ...")
    if not await backend.connect():
        print("Connection failed.")
        return

    if MOTOR_ID not in backend.discovered_servos:
        print(f"Motor {MOTOR_ID} not found — aborting.")
        await backend.disconnect()
        return

    state0 = await backend.get_state()
    hold_pos = state0.servo_positions.get(MOTOR_ID, 0.0)
    step_rad = math.radians(step_deg)

    # Warm up: enable motor and hold for 3 s without safety gating so it settles
    print("Enabling motor and settling (3 s)...")
    await _warmup(backend, hold_pos, duration_s=3.0)
    state0 = await backend.get_state()
    hold_pos = state0.servo_positions.get(MOTOR_ID, hold_pos)

    n_tests = len(kp_grid) * len(kd_grid)
    secs_per_test = HOLD_S + SETTLE_S + RETURN_S + COOLDOWN_S
    est_min = n_tests * secs_per_test / 60
    print(f"Motor {MOTOR_ID} settled at {math.degrees(hold_pos):.1f}°   step={step_deg:.1f}°")
    print(f"Grid: {len(kp_grid)} kp × {len(kd_grid)} kd = up to {n_tests} tests  (~{est_min:.0f} min worst-case)\n")
    print(f"{'kp':>6}  {'kd':>5}  {'τ_std':>6}  {'v_std':>6}  {'cross':>5}  {'err°':>5}  status")
    print("─" * 60)

    all_results: list[TestResult] = []

    try:
        for kp in kp_grid:
            kd_wall_hit = False
            for kd in sorted(kd_grid):
                if kd_wall_hit:
                    r = TestResult(kp=kp, kd=kd, aborted=True,
                                   abort_reason="skipped: above kd vibration wall")
                else:
                    r = await _run_test(backend, hold_pos, kp, kd, step_rad)
                    if r.vibrating:
                        kd_wall_hit = True
                    # Cooldown: zero-gain MIT hold — motor stays in motor mode
                    # (no torque output) so no disable/re-enable cycle is needed.
                    t_cool = time.monotonic() + COOLDOWN_S
                    while time.monotonic() < t_cool:
                        await backend.send_commands([
                            ServoCommand(servo_id=MOTOR_ID, position=hold_pos,
                                         kp=0.0, kd=0.0, torque_ff=0.0)
                        ])
                        await asyncio.sleep(DT)

                all_results.append(r)
                print(
                    f"{kp:>6.1f}  {kd:>5.3f}  {r.hold_tau_std:>6.4f}"
                    f"  {r.hold_vel_std:>6.3f}  {r.step_crossings:>5}  {r.final_err_deg:>5.2f}  {r.status}"
                )

    finally:
        await backend.disable_torques()
        await backend.disconnect()

    # ── Save raw results ──────────────────────────────────────────────────────
    ts = time.strftime("%Y%m%d_%H%M%S")
    os.makedirs("data", exist_ok=True)
    out_path = f"data/sweep_kp_kd_{ts}.json"
    with open(out_path, "w") as f:
        json.dump(
            {
                "motor_id": MOTOR_ID,
                "hold_pos_deg": math.degrees(hold_pos),
                "step_deg": step_deg,
                "results": [asdict(r) for r in all_results],
            },
            f,
            indent=2,
        )
    print(f"\nRaw results saved → {out_path}")

    # ── Find stable window per kp ─────────────────────────────────────────────
    from collections import defaultdict
    stable_kd_by_kp: dict[float, list[float]] = defaultdict(list)
    for r in all_results:
        if r.stable:
            stable_kd_by_kp[r.kp].append(r.kd)

    if not stable_kd_by_kp:
        print("\nNo stable pairs found — try a narrower kp range or smaller step.")
        return

    print("\n\n=== Stable windows ===\n")
    print(f"{'kp':>6}  {'kd_min':>6}  {'kd_max':>6}  {'kd_opt':>6}  (mid of stable window)")
    print("─" * 42)

    optimal: list[tuple[float, float]] = []
    for kp in sorted(stable_kd_by_kp):
        kd_vals = sorted(stable_kd_by_kp[kp])
        kd_opt = kd_vals[len(kd_vals) // 2]
        optimal.append((kp, kd_opt))
        print(f"{kp:>6.1f}  {kd_vals[0]:>6.3f}  {kd_vals[-1]:>6.3f}  {kd_opt:>6.3f}")

    # ── Print lookup table ────────────────────────────────────────────────────
    print("\n\n=== Lookup table — copy into config.py ===\n")
    print("# Validated (kp, kd) stiffness levels, soft → stiff.")
    print("# Use get_stiffness_gains(t) to interpolate for t in [0.0, 1.0].")
    print("STIFFNESS_TABLE: list[tuple[float, float]] = [")
    print("    # (kp_Nm_rad,  kd_Nm_s_rad)")
    print("    (  0.4,  0.035),   # 0.00 — float (current default, untested here)")
    n = len(optimal)
    for i, (kp, kd) in enumerate(optimal):
        t = (i + 1) / (n + 1)
        print(f"    ({kp:>5.1f},  {kd:.3f}),   # {t:.2f}")
    print("]")

    print("\n\n=== Interpolation helper — add to config.py or a utils module ===\n")
    print("""\
def get_stiffness_gains(t: float) -> tuple[float, float]:
    \"\"\"Return (kp, kd) for stiffness t in [0.0, 1.0] via linear interpolation.\"\"\"
    table = STIFFNESS_TABLE
    if len(table) == 1:
        return table[0]
    idx_f = max(0.0, min(1.0, t)) * (len(table) - 1)
    lo = int(idx_f)
    hi = min(lo + 1, len(table) - 1)
    frac = idx_f - lo
    kp = table[lo][0] + frac * (table[hi][0] - table[lo][0])
    kd = table[lo][1] + frac * (table[hi][1] - table[lo][1])
    return kp, kd
""")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Sweep kp/kd pairs on motor 7 to find stable operating points."
    )
    parser.add_argument("--host", default=ROBOT_DEFAULT_HOST)
    parser.add_argument("--port", type=int, default=ROBOT_DEFAULT_PORT)
    parser.add_argument("--kp", type=float, nargs="+", default=DEFAULT_KP,
                        metavar="KP", help="kp values to test (Nm/rad)")
    parser.add_argument("--kd", type=float, nargs="+", default=DEFAULT_KD,
                        metavar="KD", help="kd values to test (Nm·s/rad)")
    parser.add_argument("--step", type=float, default=STEP_DEG_DEFAULT,
                        metavar="DEG", help="step size in degrees (default 12)")
    args = parser.parse_args()
    asyncio.run(main(args.host, args.port, args.kp, args.kd, args.step))
