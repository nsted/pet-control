#!/usr/bin/env python3
"""
Identify the hardware pad index → physical position mapping for the
left and right rectangular faces.

Touch sequence (prompted interactively, same for both faces):
  1. bottom_solo  — the single isolated pad at the bottom row
  2. top_head     — top row, pad closest to the head end
  3. top_mid      — top row, middle pad
  4. top_rear     — top row, pad closest to the rear/tail end

Outputs a mapping table and a remap list for each face so you know
how to reorder _PAD_CENTERS in rerun_viz.py.

Usage:
    python scripts/pad_remap.py                  # real robot, module 6
    python scripts/pad_remap.py --module 1
"""

from __future__ import annotations

import argparse
import asyncio
import time

POSITIONS = [
    ("bottom_solo", "the SINGLE isolated pad at the bottom"),
    ("top_head",    "TOP ROW — pad closest to the HEAD end"),
    ("top_mid",     "TOP ROW — MIDDLE pad"),
    ("top_rear",    "TOP ROW — pad closest to the REAR/tail end"),
]

BASELINE_SAMPLES  = 15
ACTIVATED_SAMPLES = 15
SAMPLE_HZ         = 20
BAR_WIDTH         = 24
MIN_DELTA         = 0.15


def _bar(v: float) -> str:
    filled = round(max(0.0, min(1.0, v)) * BAR_WIDTH)
    return "[" + "#" * filled + "." * (BAR_WIDTH - filled) + f"] {v:.3f}"


async def _collect(backend, module_id: int, face: str, n: int) -> list[list[float]]:
    samples: list[list[float]] = []
    interval = 1.0 / SAMPLE_HZ
    for _ in range(n):
        t0 = time.monotonic()
        state = await backend.get_state()
        sens = state.sensors.get(module_id)
        pads = list(getattr(sens, f"touch_{face}_pads", [])) if sens else []
        pads = (pads + [0.0] * 4)[:4]
        samples.append(pads)
        rem = interval - (time.monotonic() - t0)
        if rem > 0:
            await asyncio.sleep(rem)
    return samples


def _mean_pads(samples: list[list[float]]) -> list[float]:
    n = len(samples)
    if not n:
        return [0.0] * 4
    return [sum(s[i] for s in samples) / n for i in range(4)]


async def _prompt(loop: asyncio.AbstractEventLoop, msg: str) -> str:
    return (await loop.run_in_executor(None, input, msg)).strip().lower()


async def test_face(
    backend, module_id: int, face: str, loop: asyncio.AbstractEventLoop
) -> list[int]:
    """Walk through 4 positions. Returns hardware pad indices in POSITIONS order."""
    print(f"\n{'═'*56}")
    print(f"  Face: {face.upper()}   Module: {module_id}")
    print(f"{'═'*56}")

    await _prompt(loop, "\n  Don't touch the robot. Press Enter to collect baseline... ")
    base_mean = _mean_pads(await _collect(backend, module_id, face, BASELINE_SAMPLES))
    print(f"  Baseline: {[f'{v:.3f}' for v in base_mean]}")

    mapping: list[int] = []

    for pos_name, pos_desc in POSITIONS:
        while True:
            await _prompt(loop, f"\n  Touch: {pos_desc}\n  Hold steady, then press Enter... ")
            act_mean = _mean_pads(await _collect(backend, module_id, face, ACTIVATED_SAMPLES))
            deltas = [a - b for a, b in zip(act_mean, base_mean)]
            best = deltas.index(max(deltas))

            print(f"\n  {face.upper()} pad deltas:")
            for i, d in enumerate(deltas):
                marker = "◀" if i == best else " "
                print(f"    pad {i}: {_bar(d)}  {marker}")

            if max(deltas) < MIN_DELTA:
                print(f"  ⚠  No clear activation (max Δ={max(deltas):.3f}). Try again.")
                continue

            resp = await _prompt(loop, f"\n  Detected pad {best}. Accept? (Enter=yes / r=redo): ")
            if resp != "r":
                mapping.append(best)
                print(f"  ✓  {pos_name} → pad {best}")
                break

    return mapping


async def main(host: str, port: int, module_id: int) -> None:
    from petctl.backends.robot import RobotBackend

    backend = RobotBackend(
        host=host, port=port,
        calibrate_on_connect=False,
        auto_reconnect=False,
    )
    print(f"[pad_remap] Connecting to {host}:{port} ...")
    if not await backend.connect():
        print("[pad_remap] Connection failed.")
        return

    if hasattr(backend, "sensor_poll_hz"):
        backend.sensor_poll_hz = float(SAMPLE_HZ)
    await asyncio.sleep(0.5)

    loop = asyncio.get_running_loop()
    results: dict[str, list[int]] = {}

    try:
        for face in ("left", "right"):
            results[face] = await test_face(backend, module_id, face, loop)
    except KeyboardInterrupt:
        print("\n[pad_remap] Interrupted.")
    finally:
        await backend.disconnect()

    if not results:
        return

    print(f"\n{'═'*56}")
    print("  RESULTS")
    print(f"{'═'*56}")
    for face, mapping in results.items():
        names = [p[0] for p in POSITIONS]
        print(f"\n  {face.upper()} face:")
        for pos_name, hw_idx in zip(names, mapping):
            print(f"    {pos_name:12s} → hardware pad {hw_idx}")
        print(f"\n  Hardware order seen: {mapping}")
        print(f"  (index = position in [bottom_solo, top_head, top_mid, top_rear])")

    print(f"\n{'═'*56}\n")


if __name__ == "__main__":
    from petctl.backends.robot import ROBOT_DEFAULT_HOST, ROBOT_DEFAULT_PORT

    parser = argparse.ArgumentParser(
        description="Map hardware pad indices to physical positions on left/right faces"
    )
    parser.add_argument("--host",   default=ROBOT_DEFAULT_HOST)
    parser.add_argument("--port",   type=int, default=ROBOT_DEFAULT_PORT)
    parser.add_argument("--module", type=int, default=6,
                        help="Module ID to test (default: 6)")
    args = parser.parse_args()

    asyncio.run(main(args.host, args.port, args.module))
