#!/usr/bin/env python3
"""
Hold motor 7 at its current position with a given kp/kd. Run until Ctrl+C.

Usage:
    python scripts/tune_kp_kd.py --kp 0.3 --kd 0.030
"""

from __future__ import annotations

import argparse
import asyncio
import math
import time

from petctl.backends.robot import ROBOT_DEFAULT_HOST, ROBOT_DEFAULT_PORT, RobotBackend
from petctl.types import ServoCommand

MOTOR_ID = 7
TICK_HZ = 30
DT = 1.0 / TICK_HZ


async def run(host: str, port: int, kp: float, kd: float) -> None:
    backend = RobotBackend(host=host, port=port, auto_reconnect=False)
    print(f"Connecting to {host}:{port} ...")
    if not await backend.connect():
        print("Connection failed.")
        return

    motors = backend.discovered_servos
    if MOTOR_ID not in motors:
        print(f"Motor {MOTOR_ID} not found — aborting.")
        await backend.disconnect()
        return

    state0 = await backend.get_state()
    hold_pos = state0.servo_positions.get(MOTOR_ID, 0.0)

    print(f"Holding motor {MOTOR_ID} at {math.degrees(hold_pos):.1f}°  kp={kp}  kd={kd}")
    print("Press Ctrl+C to stop.\n")

    try:
        while True:
            t0 = time.monotonic()
            state = await backend.get_state()
            actual = state.servo_positions.get(MOTOR_ID, hold_pos)
            torque = state.motor_torques.get(MOTOR_ID, 0.0)
            current = state.battery_current_amps
            err_deg = math.degrees(hold_pos - actual)
            print(
                f"\r  err={err_deg:+6.2f}°  τ={torque:+.3f}Nm  I={current:.2f}A",
                end="", flush=True,
            )
            await backend.send_commands([
                ServoCommand(servo_id=MOTOR_ID, position=hold_pos, kp=kp, kd=kd, torque_ff=0.0)
            ])
            await asyncio.sleep(max(0.0, DT - (time.monotonic() - t0)))
    finally:
        print("\nStopping.")
        await backend.disable_torques()
        await backend.disconnect()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Hold motor 7 at current position with given kp/kd")
    parser.add_argument("--host", default=ROBOT_DEFAULT_HOST)
    parser.add_argument("--port", type=int, default=ROBOT_DEFAULT_PORT)
    parser.add_argument("--kp", type=float, required=True)
    parser.add_argument("--kd", type=float, required=True)
    args = parser.parse_args()
    asyncio.run(run(args.host, args.port, args.kp, args.kd))
