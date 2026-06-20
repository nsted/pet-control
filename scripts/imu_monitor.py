#!/usr/bin/env python3
"""
Live IMU monitor for PET robot.

Streams quaternion, linear acceleration, and gyroscope from each active
module, with derived roll/pitch/yaw and a rolling motion magnitude bar.

Usage:
    python scripts/imu_monitor.py                  # real robot
    python scripts/imu_monitor.py --modules 7      # single module
    python scripts/imu_monitor.py --window 3.0     # rolling window in seconds (default 2s)
    python scripts/imu_monitor.py --log            # write JSONL log
"""

from __future__ import annotations

import argparse
import asyncio
import collections
import datetime
import json
import math
import sys
import time
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from petctl.types import ImuReading, RobotState

POLL_HZ   = 30
BAR_WIDTH = 20
MAG_MAX   = 2.0   # m/s² full-scale for accel magnitude bar


# ── Quaternion helpers ─────────────────────────────────────────────────────────

def _euler_deg(qr: float, qi: float, qj: float, qk: float) -> tuple[float, float, float]:
    """Convert BNO085 rotation vector quaternion (w,x,y,z) to roll/pitch/yaw in degrees."""
    roll  = math.degrees(math.atan2(2.0 * (qr*qi + qj*qk), 1.0 - 2.0 * (qi*qi + qj*qj)))
    sin_p = 2.0 * (qr*qj - qk*qi)
    pitch = math.degrees(math.asin(max(-1.0, min(1.0, sin_p))))
    yaw   = math.degrees(math.atan2(2.0 * (qr*qk + qi*qj), 1.0 - 2.0 * (qj*qj + qk*qk)))
    return roll, pitch, yaw


def _mag(x: float, y: float, z: float) -> float:
    return math.sqrt(x*x + y*y + z*z)


# ── Bar rendering ──────────────────────────────────────────────────────────────

def _mag_bar(value: float, full_scale: float) -> str:
    frac = max(0.0, min(1.0, value / full_scale))
    filled = round(frac * BAR_WIDTH)
    return "[" + "#" * filled + "." * (BAR_WIDTH - filled) + f"] {value:.3f}"


def _range_bar(cur: float, lo: float, hi: float, full_scale: float) -> str:
    def pos(v: float) -> int:
        return round(max(0.0, min(1.0, v / full_scale)) * BAR_WIDTH)
    bar = ["."] * BAR_WIDTH
    lo_p, hi_p, cur_p = pos(lo), pos(hi), pos(cur)
    for i in range(lo_p, hi_p):
        bar[i] = "-"
    if 0 <= cur_p < BAR_WIDTH:
        bar[cur_p] = "#"
    rng = hi - lo
    return "[" + "".join(bar) + f"] {cur:.3f} [{lo:.3f}–{hi:.3f}] rng={rng:.3f}"


# ── Rolling window ─────────────────────────────────────────────────────────────

class RollingStats:
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


# ── Per-module state ───────────────────────────────────────────────────────────

class ModuleImuStats:
    def __init__(self, module_id: int, window_s: float) -> None:
        self.module_id = module_id
        self.accel_mag = RollingStats(window_s)
        self.gyro_mag  = RollingStats(window_s)
        self.last: "ImuReading | None" = None

    def update(self, imu: "ImuReading") -> None:
        self.last = imu
        self.accel_mag.push(_mag(imu.ax, imu.ay, imu.az))
        self.gyro_mag.push(_mag(imu.gx, imu.gy, imu.gz))


# ── Display ────────────────────────────────────────────────────────────────────

class Monitor:
    def __init__(
        self,
        module_ids: list[int],
        window_s: float,
        log_path: str | None,
    ) -> None:
        self._module_ids    = module_ids
        self._stats: dict[int, ModuleImuStats] = {
            m: ModuleImuStats(m, window_s) for m in module_ids
        }
        self._lines_printed = 0
        self._is_tty        = sys.stdout.isatty()
        self._log_file      = open(log_path, "w") if log_path else None
        self._sample_count  = 0

    def update(self, state: "RobotState") -> None:
        now = time.monotonic()
        for mid, imu in state.imu.items():
            if mid not in self._stats:
                self._stats[mid] = ModuleImuStats(mid, 2.0)
                if mid not in self._module_ids:
                    self._module_ids.append(mid)
                    self._module_ids.sort()
            self._stats[mid].update(imu)

        if self._log_file and state.imu:
            record: dict = {"ts": time.time(), "modules": {}}
            for mid, imu in state.imu.items():
                record["modules"][mid] = {
                    "qr": imu.qr, "qi": imu.qi, "qj": imu.qj, "qk": imu.qk,
                    "ax": imu.ax, "ay": imu.ay, "az": imu.az,
                    "gx": imu.gx, "gy": imu.gy, "gz": imu.gz,
                }
            self._log_file.write(json.dumps(record) + "\n")
            self._log_file.flush()

        self._sample_count += 1
        self._render(now)

    def _render(self, now: float) -> None:
        lines: list[str] = []
        lines.append(
            f"  PET IMU monitor  |  samples={self._sample_count}  |  Ctrl-C=quit"
        )
        lines.append("")

        active = [mid for mid in self._module_ids if self._stats[mid].last is not None]

        if not active:
            lines.append("  (waiting for IMU data...)")
            lines.append("")
        else:
            for mid in active:
                st = self._stats[mid]
                imu = st.last
                assert imu is not None

                roll, pitch, yaw = _euler_deg(imu.qr, imu.qi, imu.qj, imu.qk)
                age = now - imu.timestamp
                amag = _mag(imu.ax, imu.ay, imu.az)
                gmag = _mag(imu.gx, imu.gy, imu.gz)
                alo, ahi = st.accel_mag.min_max()
                glo, ghi = st.gyro_mag.min_max()

                lines.append(
                    f"  ── Module {mid} ──────────────────────────────────────────── age {age*1000:.0f}ms"
                )
                lines.append(
                    f"     Quat    qr={imu.qr:+.4f}  qi={imu.qi:+.4f}"
                    f"  qj={imu.qj:+.4f}  qk={imu.qk:+.4f}"
                )
                lines.append(
                    f"     Euler   roll={roll:+7.2f}°  pitch={pitch:+7.2f}°  yaw={yaw:+8.2f}°"
                )
                lines.append(
                    f"     Accel   x={imu.ax:+7.3f}  y={imu.ay:+7.3f}  z={imu.az:+7.3f}  m/s²"
                )
                lines.append(
                    f"     |accel| {_range_bar(amag, alo, ahi, MAG_MAX)}"
                )
                lines.append(
                    f"     Gyro    x={imu.gx:+7.3f}  y={imu.gy:+7.3f}  z={imu.gz:+7.3f}  rad/s"
                )
                lines.append(
                    f"     |gyro|  {_range_bar(gmag, glo, ghi, 1.0)}"
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
    modules: list[int] | None,
    window_s: float,
    log_path: str | None,
) -> None:
    from petctl.backends.robot import RobotBackend

    backend = RobotBackend(host=host, port=port, calibrate_on_connect=False, auto_reconnect=False)
    print(f"[imu_monitor] Connecting to {host}:{port} ...")

    ok = await backend.connect()
    if not ok:
        print("[imu_monitor] Connection failed.")
        return

    # Discover module IDs to watch; default to modules 1–7 (head has no IMU).
    watch = modules if modules is not None else list(range(1, 8))
    print(f"[imu_monitor] Connected. Watching modules: {watch}  window={window_s}s")
    if log_path:
        print(f"[imu_monitor] Logging to: {log_path}")
    print("[imu_monitor] Press Ctrl-C to stop.\n")

    await asyncio.sleep(0.3)

    monitor = Monitor(watch, window_s, log_path)
    interval = 1.0 / POLL_HZ

    try:
        while True:
            t0 = time.monotonic()
            state = await backend.get_state()
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
        print("\n[imu_monitor] Stopped.")
        if log_path:
            print(f"[imu_monitor] Log: {log_path}")


if __name__ == "__main__":
    from petctl.backends.robot import ROBOT_DEFAULT_HOST, ROBOT_DEFAULT_PORT

    parser = argparse.ArgumentParser(description="Live IMU monitor for PET robot")
    parser.add_argument("--host",    default=ROBOT_DEFAULT_HOST)
    parser.add_argument("--port",    type=int, default=ROBOT_DEFAULT_PORT)
    parser.add_argument(
        "--modules", default=None, metavar="IDS",
        help="Comma-separated module IDs to watch (default: 1–7, auto-expands on first data)",
    )
    parser.add_argument(
        "--window", type=float, default=2.0, metavar="SECONDS",
        help="Rolling window for min/max range bars (default: 2s)",
    )
    parser.add_argument("--log", action="store_true", help="Write JSONL log")
    args = parser.parse_args()

    module_ids: list[int] | None = None
    if args.modules:
        module_ids = [int(m.strip()) for m in args.modules.split(",") if m.strip()]

    log_path: str | None = None
    if args.log:
        ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        log_path = f"imu_monitor_{ts}.jsonl"

    asyncio.run(main(
        host=args.host,
        port=args.port,
        modules=module_ids,
        window_s=args.window,
        log_path=log_path,
    ))
