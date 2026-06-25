#!/usr/bin/env python3
"""
FSR zero-pressure offset calibration for PET robot.

With nothing touching the robot, reads N samples of each module's FSR sensors,
averages them, and saves the results to fsr_offsets.json.  The RobotBackend
automatically subtracts these offsets when the file is present.

Usage:
    python scripts/calibrate_fsr.py
    python scripts/calibrate_fsr.py --host pet-robot.local --samples 100
    python scripts/calibrate_fsr.py --output fsr_offsets.json
"""

from __future__ import annotations

import argparse
import asyncio
import datetime
import json
import sys
import time

from petctl.backends.robot import ROBOT_DEFAULT_HOST, ROBOT_DEFAULT_PORT, RobotBackend

FACES = ("left", "right", "middle")
DEFAULT_SAMPLES = 60
DEFAULT_OUTPUT = "fsr_offsets.json"
SAMPLE_HZ = 20


def _bar(value: float, width: int = 20) -> str:
    filled = round(max(0.0, min(1.0, value)) * width)
    return "[" + "#" * filled + "." * (width - filled) + f"] {value:.3f}"


async def collect_offsets(
    backend: RobotBackend,
    n_samples: int,
) -> dict[int, dict[str, float]]:
    """Collect n_samples FSR readings and return mean per (module, face)."""
    interval = 1.0 / SAMPLE_HZ
    accum: dict[int, dict[str, list[float]]] = {}

    lines_printed = 0
    is_tty = sys.stdout.isatty()

    for i in range(n_samples):
        t0 = time.monotonic()
        state = await backend.get_state()

        for mod_id, ms in state.sensors.items():
            if mod_id not in accum:
                accum[mod_id] = {"left": [], "right": [], "middle": []}
            accum[mod_id]["left"].append(ms.pressure_left)
            accum[mod_id]["right"].append(ms.pressure_right)
            accum[mod_id]["middle"].append(ms.pressure_middle)

        if is_tty and accum:
            lines: list[str] = [f"  Collecting sample {i + 1}/{n_samples}"]
            for mod_id in sorted(accum):
                d = accum[mod_id]
                cur_l = d["left"][-1] if d["left"] else 0.0
                cur_r = d["right"][-1] if d["right"] else 0.0
                cur_m = d["middle"][-1] if d["middle"] else 0.0
                lines.append(
                    f"  M{mod_id}  L={_bar(cur_l)}  R={_bar(cur_r)}  Mid={_bar(cur_m)}"
                )
            if lines_printed:
                sys.stdout.write(f"\033[{lines_printed}A")
            for line in lines:
                sys.stdout.write(line + "\n")
            sys.stdout.flush()
            lines_printed = len(lines)

        elapsed = time.monotonic() - t0
        remaining = interval - elapsed
        if remaining > 0:
            await asyncio.sleep(remaining)

    offsets: dict[int, dict[str, float]] = {}
    for mod_id, data in accum.items():
        offsets[mod_id] = {
            face: (sum(data[face]) / len(data[face]) if data[face] else 0.0)
            for face in FACES
        }
    return offsets


def print_offset_table(offsets: dict[int, dict[str, float]]) -> None:
    print()
    sep = "═" * 60
    print(sep)
    print("  FSR CALIBRATION OFFSETS")
    print(sep)
    print(f"  {'Mod':>3}  {'Left':>8}  {'Right':>8}  {'Middle':>8}")
    print("  " + "─" * 36)
    for mod_id in sorted(offsets):
        d = offsets[mod_id]
        print(
            f"  {mod_id:>3}  {d['left']:>8.4f}  {d['right']:>8.4f}  {d['middle']:>8.4f}"
        )
    print(sep)
    print()


async def main(host: str, port: int, n_samples: int, output: str) -> None:
    backend = RobotBackend(host=host, port=port, calibrate_on_connect=False, auto_reconnect=False)
    print(f"[calibrate_fsr] Connecting to {host}:{port} ...")
    ok = await backend.connect()
    if not ok:
        print("[calibrate_fsr] Connection failed.")
        return

    print("[calibrate_fsr] Connected.")
    print()
    input("  Make sure NOTHING is touching the robot, then press Enter to begin: ")
    print(f"\n  Collecting {n_samples} samples at {SAMPLE_HZ} Hz ...")

    # Let the first few sensor frames settle
    await asyncio.sleep(0.3)

    try:
        offsets = await collect_offsets(backend, n_samples)
    finally:
        await backend.disconnect()

    print_offset_table(offsets)

    data = {
        "generated_at": datetime.datetime.now().isoformat(),
        "samples": n_samples,
        "offsets": {str(mid): v for mid, v in offsets.items()},
    }
    with open(output, "w") as f:
        json.dump(data, f, indent=2)
    print(f"  Saved to: {output}")
    print()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Calibrate FSR zero-pressure offsets")
    parser.add_argument("--host", default=ROBOT_DEFAULT_HOST)
    parser.add_argument("--port", type=int, default=ROBOT_DEFAULT_PORT)
    parser.add_argument("--samples", type=int, default=DEFAULT_SAMPLES, help="Number of samples to average")
    parser.add_argument("--output", default=DEFAULT_OUTPUT, help="Output JSON file path")
    args = parser.parse_args()

    asyncio.run(main(args.host, args.port, args.samples, args.output))
