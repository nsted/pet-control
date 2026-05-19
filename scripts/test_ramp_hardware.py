#!/usr/bin/env python3
"""
Hardware test for the position ramp filter on motor 7.

Three scenarios:
  1. Sudden jump  — command 0 → TARGET_RAD; verify commanded position ramps smoothly.
  2. set_home()   — call set_home() mid-motion; verify first post-home command doesn't snap.
  3. reset_motor_zero() — reset zero at current position; verify first command ramps correctly.

Usage:
    python scripts/test_ramp_hardware.py
    python scripts/test_ramp_hardware.py --target 0.5   # smaller movement
"""

from __future__ import annotations

import argparse
import asyncio
import time

from petctl.backends.robot import RobotBackend
from petctl.config import LOOP_LIMITS
from petctl.types import ServoCommand

MOTOR_ID   = 7
TICK_HZ    = 30
DT         = 1.0 / TICK_HZ
TARGET_RAD = 1.0   # override with --target


def _banner(title: str) -> None:
    print(f"\n{'─' * 52}")
    print(f"  {title}")
    print(f"{'─' * 52}")


async def _settle(backend: RobotBackend, target: float, ticks: int = 90) -> None:
    """Drive motor to target and wait for it to settle (ticks at TICK_HZ)."""
    for _ in range(ticks):
        t0 = time.monotonic()
        await backend.send_commands([ServoCommand(servo_id=MOTOR_ID, position=target)])
        await asyncio.sleep(max(0.0, DT - (time.monotonic() - t0)))


async def run(target_rad: float) -> None:
    backend = RobotBackend(auto_reconnect=False)
    print("Connecting to robot...")
    if not await backend.connect():
        print("Connection failed — is pet-robot.local reachable?")
        return

    motors = backend.discovered_servos
    print(f"Connected. Motors: {motors}")
    if MOTOR_ID not in motors:
        print(f"Motor {MOTOR_ID} not present — aborting.")
        await backend.disconnect()
        return

    max_step_30hz = LOOP_LIMITS.max_speed_rad_s / TICK_HZ

    # ── Test 1: sudden jump ─────────────────────────────────────────────────────
    _banner(f"Test 1 · sudden jump  0 → {target_rad:.2f} rad")
    print(f"  max_speed_rad_s={LOOP_LIMITS.max_speed_rad_s:.1f}  max_step@{TICK_HZ}Hz≈{max_step_30hz:.4f} rad")
    print(f"  {'tick':>4}  {'commanded':>10}  {'actual':>10}  {'delta':>8}")

    prev_cmd = 0.0
    for i in range(TICK_HZ * 3):  # 3 seconds
        t0 = time.monotonic()
        await backend.send_commands([ServoCommand(servo_id=MOTOR_ID, position=target_rad)])

        offset = backend._angle_offsets.get(MOTOR_ID, 0.0)
        raw_cmd = backend._last_mit_abs_pos.get(MOTOR_ID, 0.0)
        commanded = raw_cmd - offset  # back to software coords

        state = await backend.get_state()
        actual = state.servo_positions.get(MOTOR_ID, 0.0)
        step = abs(commanded - prev_cmd)

        flag = "  ← SNAP!" if (i > 0 and step > max_step_30hz * 1.5) else ""
        print(f"  {i:>4}  {commanded:>+10.4f}  {actual:>+10.4f}  {step:>8.4f}{flag}")
        prev_cmd = commanded
        await asyncio.sleep(max(0.0, DT - (time.monotonic() - t0)))

    # ── Test 2: set_home() seeding ──────────────────────────────────────────────
    _banner("Test 2 · set_home() seeding")
    state = await backend.get_state()
    pos_before = state.servo_positions.get(MOTOR_ID, 0.0)
    seeded_abs = backend._last_mit_abs_pos.get(MOTOR_ID)

    await backend.set_home()

    seeded_after = backend._last_mit_abs_pos.get(MOTOR_ID)
    print(f"  actual pos before set_home()        : {pos_before:+.4f} rad")
    print(f"  _last_mit_abs_pos before set_home() : {seeded_abs}")
    print(f"  _last_mit_abs_pos after  set_home() : {seeded_after}")
    print(f"  _last_mit_wall_s seeded             : {MOTOR_ID in backend._last_mit_wall_s}")

    # First command after home — should ramp, not snap
    await backend.send_commands([ServoCommand(servo_id=MOTOR_ID, position=target_rad)])
    raw_first = backend._last_mit_abs_pos.get(MOTOR_ID, 0.0)
    offset_after = backend._angle_offsets.get(MOTOR_ID, 0.0)
    first_cmd = raw_first - offset_after
    delta_first = abs(first_cmd)  # from new home (0.0 software)
    print(f"\n  first cmd after set_home()          : {first_cmd:+.4f} rad")
    print(f"  expected max step (dt floor 1/120s) : {LOOP_LIMITS.max_speed_rad_s / 120:.4f} rad")
    result_str = "PASS ✓" if delta_first <= LOOP_LIMITS.max_speed_rad_s / 120 * 1.5 else "FAIL ✗  (snapped!)"
    print(f"  result                              : {result_str}")

    # ── Test 3: reset_motor_zero() seeding ─────────────────────────────────────
    _banner("Test 3 · reset_motor_zero() seeding")

    # Settle partway to target so there's a meaningful physical offset
    await _settle(backend, target_rad * 0.5, ticks=60)

    state = await backend.get_state()
    phys_pos_sw = state.servo_positions.get(MOTOR_ID, 0.0)
    phys_pos_abs = phys_pos_sw + backend._angle_offsets.get(MOTOR_ID, 0.0)
    print(f"  physical position before reset      : {phys_pos_sw:+.4f} rad (sw)  {phys_pos_abs:+.4f} rad (abs)")

    backend.reset_motor_zero(MOTOR_ID)

    seeded = backend._last_mit_abs_pos.get(MOTOR_ID)
    motor_raw = backend._motor_state.get(MOTOR_ID, {}).get("pos", None)
    print(f"  motor_state['pos'] at reset         : {motor_raw}")
    print(f"  _last_mit_abs_pos after reset       : {seeded}")
    print(f"  _last_mit_wall_s seeded             : {MOTOR_ID in backend._last_mit_wall_s}")

    await backend.send_commands([ServoCommand(servo_id=MOTOR_ID, position=0.5)])
    raw_first = backend._last_mit_abs_pos.get(MOTOR_ID, 0.0)
    new_offset = backend._angle_offsets.get(MOTOR_ID, 0.0)
    first_sw = raw_first - new_offset
    delta = abs(first_sw)
    print(f"\n  first cmd after reset()             : {first_sw:+.4f} rad (sw)")
    print(f"  expected max step (dt floor 1/120s) : {LOOP_LIMITS.max_speed_rad_s / 120:.4f} rad")
    result_str = "PASS ✓" if delta <= LOOP_LIMITS.max_speed_rad_s / 120 * 1.5 else "FAIL ✗  (snapped!)"
    print(f"  result                              : {result_str}")

    # ── Return home ────────────────────────────────────────────────────────────
    _banner("Returning to home (0.0 rad)")
    await _settle(backend, 0.0, ticks=TICK_HZ * 3)
    state = await backend.get_state()
    final = state.servo_positions.get(MOTOR_ID, 0.0)
    print(f"  final position: {final:+.4f} rad")

    await backend.disable_torques()
    await backend.disconnect()
    print("\nDone.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--target", type=float, default=TARGET_RAD, help="Target position in radians")
    args = parser.parse_args()
    asyncio.run(run(args.target))
