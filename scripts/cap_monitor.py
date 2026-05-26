#!/usr/bin/env python3
"""
Continuous cap/FSR monitor for PET robot.

Streams all sensor values with live range bars showing rolling min/max to
assess whether cap sensors hold a steady level.

Usage:
    python scripts/cap_monitor.py                      # real robot, all modules, face averages
    python scripts/cap_monitor.py --expand 2           # per-pad detail for module 2
    python scripts/cap_monitor.py --modules 0,1,2      # subset of modules
    python scripts/cap_monitor.py --window 3.0         # rolling window in seconds (default 2s)
    python scripts/cap_monitor.py --dry-run            # MockBackend
    python scripts/cap_monitor.py --log                # write JSONL log
"""

from __future__ import annotations

import argparse
import asyncio
import collections
import datetime
import json
import sys
import time
from typing import TYPE_CHECKING

from petctl.config import LOOP_LIMITS

if TYPE_CHECKING:
    from petctl.protocols import Backend as RobotBackendProto
    from petctl.types import ModuleSensors, RobotState


POLL_HZ   = 20
BAR_WIDTH = 24
FACES     = ("left", "right", "middle")
PAD_COUNTS = {"left": 4, "right": 4, "middle": 6}


# ── Bar rendering ──────────────────────────────────────────────────────────────

def _value_bar(value: float) -> str:
    """Filled progress bar for a 0-1 value."""
    filled = round(max(0.0, min(1.0, value)) * BAR_WIDTH)
    return "[" + "#" * filled + "." * (BAR_WIDTH - filled) + f"] {value:.3f}"


def _range_bar(cur: float, lo: float, hi: float) -> str:
    """Show rolling range as a span with current value marker within the bar."""
    lo_pos  = round(max(0.0, min(1.0, lo))  * BAR_WIDTH)
    hi_pos  = round(max(0.0, min(1.0, hi))  * BAR_WIDTH)
    cur_pos = round(max(0.0, min(1.0, cur)) * BAR_WIDTH)

    bar = ["."] * BAR_WIDTH
    for i in range(lo_pos, hi_pos):
        bar[i] = "-"
    if 0 <= cur_pos < BAR_WIDTH:
        bar[cur_pos] = "#"

    rng = hi - lo
    return "[" + "".join(bar) + f"] {cur:.3f} [{lo:.3f}–{hi:.3f}] rng={rng:.3f}"


# ── Rolling window ─────────────────────────────────────────────────────────────

class RollingStats:
    """Keeps a time-windowed deque of (timestamp, value) pairs."""

    def __init__(self, window_s: float) -> None:
        self._window = window_s
        self._buf: collections.deque[tuple[float, float]] = collections.deque()

    def push(self, value: float) -> None:
        now = time.monotonic()
        self._buf.append((now, value))
        cutoff = now - self._window
        while self._buf and self._buf[0][0] < cutoff:
            self._buf.popleft()

    def min_max(self) -> tuple[float, float]:
        if not self._buf:
            return 0.0, 0.0
        vals = [v for _, v in self._buf]
        return min(vals), max(vals)

    def latest(self) -> float:
        return self._buf[-1][1] if self._buf else 0.0


# ── Per-module stats store ─────────────────────────────────────────────────────

class ModuleStats:
    """Rolling stats for every pad and FSR in one module."""

    def __init__(self, module_id: int, window_s: float) -> None:
        self.module_id = module_id
        self.left_pads   = [RollingStats(window_s) for _ in range(PAD_COUNTS["left"])]
        self.right_pads  = [RollingStats(window_s) for _ in range(PAD_COUNTS["right"])]
        self.middle_pads = [RollingStats(window_s) for _ in range(PAD_COUNTS["middle"])]
        self.left_fsr    = RollingStats(window_s)
        self.right_fsr   = RollingStats(window_s)
        self.middle_fsr  = RollingStats(window_s)
        # Face averages
        self.left_avg    = RollingStats(window_s)
        self.right_avg   = RollingStats(window_s)
        self.middle_avg  = RollingStats(window_s)

    def update(self, sensors: "ModuleSensors") -> None:
        for i, v in enumerate(sensors.touch_left_pads):
            self.left_pads[i].push(v)
        for i, v in enumerate(sensors.touch_right_pads):
            self.right_pads[i].push(v)
        for i, v in enumerate(sensors.touch_middle_pads):
            self.middle_pads[i].push(v)
        self.left_fsr.push(sensors.pressure_left)
        self.right_fsr.push(sensors.pressure_right)
        self.middle_fsr.push(sensors.pressure_middle)
        self.left_avg.push(sensors.touch_left)
        self.right_avg.push(sensors.touch_right)
        self.middle_avg.push(sensors.touch_middle)


# ── Display ────────────────────────────────────────────────────────────────────

class Monitor:
    def __init__(
        self,
        module_ids: list[int],
        window_s: float,
        expand_module: int | None,
        log_path: str | None,
    ) -> None:
        self._module_ids    = module_ids
        self._expand        = expand_module
        self._stats: dict[int, ModuleStats] = {
            m: ModuleStats(m, window_s) for m in module_ids
        }
        self._lines_printed = 0
        self._is_tty        = sys.stdout.isatty()
        self._log_file      = open(log_path, "w") if log_path else None
        self._sample_count  = 0

    def update(self, state: "RobotState") -> None:
        for mod_id, stats in self._stats.items():
            sensors = state.sensors.get(mod_id)
            if sensors is not None:
                stats.update(sensors)

        if self._log_file:
            record = {
                "ts": time.time(),
                "modules": {},
            }
            for mod_id in self._module_ids:
                sensors = state.sensors.get(mod_id)
                if sensors is not None:
                    record["modules"][mod_id] = {
                        "left_pads":   list(sensors.touch_left_pads),
                        "right_pads":  list(sensors.touch_right_pads),
                        "middle_pads": list(sensors.touch_middle_pads),
                        "left_fsr":    sensors.pressure_left,
                        "right_fsr":   sensors.pressure_right,
                        "middle_fsr":  sensors.pressure_middle,
                    }
            self._log_file.write(json.dumps(record) + "\n")
            self._log_file.flush()

        self._sample_count += 1
        self._render()

    def _render(self) -> None:
        lines: list[str] = []
        lines.append(
            f"  PET cap monitor  |  samples={self._sample_count}"
            f"  |  q=quit"
        )
        lines.append("")

        for mod_id in self._module_ids:
            stats = self._stats[mod_id]
            lines.append(f"  ── Module {mod_id} ─────────────────────────────────────────────")

            if self._expand == mod_id:
                # Per-pad detail
                for face in FACES:
                    pad_stats = getattr(stats, f"{face}_pads")
                    fsr_stats = getattr(stats, f"{face}_fsr")
                    lines.append(f"     {face.upper()}")
                    for i, ps in enumerate(pad_stats):
                        cur = ps.latest()
                        lo, hi = ps.min_max()
                        lines.append(f"       pad{i}: {_range_bar(cur, lo, hi)}")
                    cur = fsr_stats.latest()
                    lo, hi = fsr_stats.min_max()
                    lines.append(f"       FSR:  {_range_bar(cur, lo, hi)}")
            else:
                # Face averages only
                for face in FACES:
                    avg_stats = getattr(stats, f"{face}_avg")
                    fsr_stats = getattr(stats, f"{face}_fsr")
                    cur = avg_stats.latest()
                    lo, hi = avg_stats.min_max()
                    fsr_cur = fsr_stats.latest()
                    fsr_lo, fsr_hi = fsr_stats.min_max()
                    lines.append(
                        f"     {face:<6} cap: {_range_bar(cur, lo, hi)}"
                    )
                    lines.append(
                        f"            fsr: {_range_bar(fsr_cur, fsr_lo, fsr_hi)}"
                    )
            lines.append("")

        if self._is_tty and self._lines_printed:
            sys.stdout.write(f"\033[{self._lines_printed}A")
        for line in lines:
            sys.stdout.write(line + "\033[K\n")
        sys.stdout.flush()
        self._lines_printed = len(lines)

    def close(self) -> None:
        if self._log_file:
            self._log_file.close()


# ── Main ───────────────────────────────────────────────────────────────────────

async def main(
    host: str,
    port: int,
    modules: list[int],
    window_s: float,
    expand_module: int | None,
    log_path: str | None,
    dry_run: bool,
    dry_run_mode: str,
) -> None:
    if dry_run:
        from petctl.backends.mock import MockBackend
        backend: "RobotBackendProto" = MockBackend(
            mode=dry_run_mode, num_modules=max(modules) + 1
        )
        print("[cap_monitor] Using MockBackend")
    else:
        from petctl.backends.robot import RobotBackend
        backend = RobotBackend(
            host=host, port=port, calibrate_on_connect=False, auto_reconnect=False
        )
        print(f"[cap_monitor] Connecting to {host}:{port} ...")

    ok = await backend.connect()
    if not ok:
        print("[cap_monitor] Connection failed.")
        return

    print(f"[cap_monitor] Connected. Modules: {modules}  window={window_s}s")
    if log_path:
        print(f"[cap_monitor] Logging to: {log_path}")
    print("[cap_monitor] Press Ctrl-C to stop.\n")

    if hasattr(backend, "sensor_poll_hz"):
        backend.sensor_poll_hz = min(POLL_HZ, LOOP_LIMITS.sensor_poll_hz_max)

    await asyncio.sleep(0.3)

    monitor = Monitor(modules, window_s, expand_module, log_path)
    interval = 1.0 / POLL_HZ

    try:
        while True:
            t0 = time.monotonic()
            state: "RobotState" = await backend.get_state()
            monitor.update(state)
            elapsed = time.monotonic() - t0
            remaining = interval - elapsed
            if remaining > 0:
                await asyncio.sleep(remaining)
    except KeyboardInterrupt:
        pass
    finally:
        monitor.close()
        await backend.disconnect()
        print("\n[cap_monitor] Stopped.")
        if log_path:
            print(f"[cap_monitor] Log: {log_path}")


if __name__ == "__main__":
    from petctl.backends.robot import ROBOT_DEFAULT_HOST, ROBOT_DEFAULT_PORT

    parser = argparse.ArgumentParser(
        description="Live cap/FSR monitor with rolling range bars"
    )
    parser.add_argument("--host",    default=ROBOT_DEFAULT_HOST)
    parser.add_argument("--port",    type=int, default=ROBOT_DEFAULT_PORT)
    parser.add_argument(
        "--modules", default="0,1,2,3,4,5,6,7",
        help="Comma-separated module IDs (default: all 8)",
    )
    parser.add_argument(
        "--expand", type=int, default=None, metavar="MODULE_ID",
        help="Show per-pad detail for this module ID",
    )
    parser.add_argument(
        "--window", type=float, default=2.0, metavar="SECONDS",
        help="Rolling window for min/max range (default: 2s)",
    )
    parser.add_argument("--log",  action="store_true", help="Write JSONL log")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--dry-run-mode", default="noise", choices=["noise", "mock-sensor-sine"])
    args = parser.parse_args()

    module_ids = [int(m.strip()) for m in args.modules.split(",") if m.strip()]
    log_path = None
    if args.log:
        ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        log_path = f"cap_monitor_{ts}.jsonl"

    asyncio.run(main(
        host=args.host,
        port=args.port,
        modules=module_ids,
        window_s=args.window,
        expand_module=args.expand,
        log_path=log_path,
        dry_run=args.dry_run,
        dry_run_mode=args.dry_run_mode,
    ))
