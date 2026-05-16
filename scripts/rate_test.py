#!/usr/bin/env python3
"""
Rate stress test: incrementally increase ws_tx_hz until WebSocket dropout.

Connects to the real robot backend (no sine control, motors held at zero
torque) and steps through increasing TX rates. Reports the max stable rate
and the rate at which dropout first occurs.

Usage:
    python scripts/rate_test.py
    python scripts/rate_test.py --step 5 --dwell 10 --start 10 --stop 120
    python scripts/rate_test.py --sensor-hz 10
"""

from __future__ import annotations

import argparse
import asyncio
import dataclasses
import time
from typing import Optional

import petctl.backends.robot as robot_module
from petctl.backends.robot import RobotBackend
from petctl.config import ControlLoopLimits, LOOP_LIMITS


# ── Defaults ──────────────────────────────────────────────────────────────────
DEFAULT_START_HZ     = 10
DEFAULT_STOP_HZ      = 120
DEFAULT_STEP_HZ      = 5
DEFAULT_DWELL_S      = 12      # seconds to hold each rate before declaring it stable
DEFAULT_SENSOR_HZ    = LOOP_LIMITS.sensor_poll_hz  # override with --sensor-hz


# ── Per-step result ────────────────────────────────────────────────────────────

class StepResult:
    def __init__(self, hz: float, dwell: float) -> None:
        self.hz = hz
        self.dwell = dwell
        self.elapsed: float = 0.0
        self.dropout: bool = False
        self.sensor_ok: int = 0
        self.sensor_fail: int = 0
        self.send_count: int = 0

    @property
    def ok(self) -> bool:
        return not self.dropout

    def __str__(self) -> str:
        if self.dropout:
            status = f"DROPOUT after {self.elapsed:.1f}s"
        else:
            status = f"OK  ({self.elapsed:.1f}s)"
        sensor_str = f"snsr ok={self.sensor_ok} fail={self.sensor_fail}"
        return f"  {self.hz:>5.0f} Hz  {status}  {sensor_str}"


# ── Instrumented TX loop ───────────────────────────────────────────────────────

async def _instrumented_tx_loop(
    backend: RobotBackend,
    hz: float,
    sensor_poll_hz: float,
    result: StepResult,
    stop_event: asyncio.Event,
) -> None:
    """
    Replacement _ws_tx_loop that uses the given hz and feeds stats into result.

    Mirrors the real loop but:
    - reads hz from the argument instead of LOOP_LIMITS.ws_tx_hz each tick
    - accepts an explicit sensor_poll_hz override
    - counts sensor ok/fail into result
    - exits when stop_event is set
    """
    _RX_SILENCE_TIMEOUT = 5.0
    sensor_poll_hz = min(sensor_poll_hz, hz)
    sensor_interval = max(1, round(hz / sensor_poll_hz))
    tick = 0
    sensor_failures = 0

    while not stop_event.is_set() and backend._connected:
        try:
            t0 = time.monotonic()
            ids = backend._discovered_motors or list(range(1, 8))

            # RX watchdog
            if backend._last_rx_time > 0 and t0 - backend._last_rx_time > _RX_SILENCE_TIMEOUT:
                print(
                    f"[rate_test] RX watchdog triggered at {hz} Hz — "
                    f"no message in {t0 - backend._last_rx_time:.1f}s"
                )
                backend._connected = False
                break

            if sensor_failures >= 5:
                print(f"[rate_test] Sensor timeout ×{sensor_failures} at {hz} Hz — dropout")
                backend._connected = False
                break

            # Motor frames — one WS send per motor (Arduino processes one SLCAN frame per message)
            if backend._ws is not None:
                async with backend._ws_send_lock:
                    for mid in ids:
                        frame = (
                            backend._pending_frames.pop(mid, None)
                            or backend._last_sent_frames.get(mid, robot_module._encode_mit_zero(mid))
                        )
                        backend._last_sent_frames[mid] = frame
                        await backend._ws.send(frame)
                        result.send_count += 1

            # Sensor request
            if tick % sensor_interval == 0:
                data = await backend._send_text("snsr 0 108", timeout=1.0)
                sensors, batt_cur, batt_vol = backend._parse_sensor_response(data)
                if sensors is not None:
                    backend._latest_sensors = sensors
                    backend._latest_battery_current_raw = batt_cur
                    backend._latest_battery_voltage_raw = batt_vol
                    sensor_failures = 0
                    result.sensor_ok += 1
                else:
                    sensor_failures += 1
                    result.sensor_fail += 1

            tick += 1
            period = 1.0 / hz
            elapsed = time.monotonic() - t0
            remaining = period - elapsed
            if remaining > 0:
                await asyncio.sleep(remaining)

        except asyncio.CancelledError:
            raise
        except Exception as e:
            if not backend._connected:
                break
            print(f"[rate_test] TX loop error at {hz} Hz: {e}")
            await asyncio.sleep(0.01)


# ── Single rate step ───────────────────────────────────────────────────────────

async def run_step(backend: RobotBackend, hz: float, sensor_poll_hz: float, dwell: float) -> StepResult:
    result = StepResult(hz, dwell)

    # Cancel any existing TX task
    if backend._ws_tx_task and not backend._ws_tx_task.done():
        backend._ws_tx_task.cancel()
        try:
            await backend._ws_tx_task
        except (asyncio.CancelledError, Exception):
            pass

    backend._latest_sensors = None

    stop_event = asyncio.Event()
    tx_task = asyncio.create_task(_instrumented_tx_loop(backend, hz, sensor_poll_hz, result, stop_event))
    backend._ws_tx_task = tx_task

    t_start = time.monotonic()
    while time.monotonic() - t_start < dwell:
        await asyncio.sleep(0.25)
        if not backend.is_connected or tx_task.done():
            result.dropout = True
            break

    result.elapsed = time.monotonic() - t_start

    # Stop the loop
    stop_event.set()
    if not tx_task.done():
        tx_task.cancel()
    try:
        await tx_task
    except (asyncio.CancelledError, Exception):
        pass

    return result


# ── Main ──────────────────────────────────────────────────────────────────────

async def main(start: float, stop: float, step: float, dwell: float, sensor_hz: float) -> None:
    backend = RobotBackend(auto_reconnect=False)
    print("[rate_test] Connecting to robot...")
    ok = await backend.connect()
    if not ok:
        print("[rate_test] Connection failed — is pet-robot.local reachable?")
        return

    motors = backend.discovered_servos
    print(f"[rate_test] Connected. Motors: {motors}")
    print(f"[rate_test] Sweep: {start}–{stop} Hz, step {step} Hz, {dwell}s per rate, sensor_poll {sensor_hz} Hz\n")

    rates = []
    r = start
    while r <= stop + 0.01:
        rates.append(r)
        r += step

    results: list[StepResult] = []
    last_good: Optional[StepResult] = None
    first_fail: Optional[StepResult] = None

    for hz in rates:
        if not backend.is_connected:
            print(f"[rate_test] Backend disconnected before {hz} Hz step — stopping.")
            break

        print(f"[rate_test] Testing {hz:.0f} Hz ...", end="", flush=True)
        result = await run_step(backend, hz, sensor_hz, dwell)
        results.append(result)

        if result.ok:
            print(f"  OK  (snsr ok={result.sensor_ok} fail={result.sensor_fail})")
            last_good = result
        else:
            print(f"  DROPOUT after {result.elapsed:.1f}s  (snsr ok={result.sensor_ok} fail={result.sensor_fail})")
            first_fail = result
            break

        # Brief pause between steps to let the robot settle
        await asyncio.sleep(0.5)

    # ── Report ─────────────────────────────────────────────────────────────────
    print("\n══════════════════════════════════════════")
    print("  RATE SWEEP RESULTS")
    print("══════════════════════════════════════════")
    for r in results:
        print(str(r))

    print()
    if last_good:
        print(f"  Max stable rate : {last_good.hz:.0f} Hz")
    else:
        print("  Max stable rate : none — dropout at first step")

    if first_fail:
        print(f"  First dropout   : {first_fail.hz:.0f} Hz  (after {first_fail.elapsed:.1f}s)")
    else:
        print("  First dropout   : none — sweep completed without dropout")
    print("══════════════════════════════════════════")

    try:
        await backend.disable_torques()
    except Exception:
        pass
    await backend.disconnect()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="WebSocket rate stress test")
    parser.add_argument("--start", type=float, default=DEFAULT_START_HZ, help="Start rate Hz")
    parser.add_argument("--stop",  type=float, default=DEFAULT_STOP_HZ,  help="Stop rate Hz")
    parser.add_argument("--step",  type=float, default=DEFAULT_STEP_HZ,  help="Step size Hz")
    parser.add_argument("--dwell",      type=float, default=DEFAULT_DWELL_S,   help="Seconds per step")
    parser.add_argument("--sensor-hz",  type=float, default=DEFAULT_SENSOR_HZ, help="Sensor poll rate Hz (default: config value)")
    args = parser.parse_args()

    asyncio.run(main(args.start, args.stop, args.step, args.dwell, args.sensor_hz))
