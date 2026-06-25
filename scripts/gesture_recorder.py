#!/usr/bin/env python3
"""
Interactive gesture data recorder for PET robot.

Streams sensor data from the robot while you perform labeled touch gestures.
Analyze the output with gesture_analyze.py to compute feature distributions
and tune detector thresholds.

Usage:
    python scripts/gesture_recorder.py               # real robot
    python scripts/gesture_recorder.py --dry-run     # MockBackend for testing
    python scripts/gesture_recorder.py --out out.jsonl

Controls:
    s = stroke    h = hold    p = poke    q = squeeze
    c = cradle    b = baseline    n = none
    Press a label key to START recording that gesture.
    Press the same key again, Enter, or Space to STOP.
    x or Ctrl-C = quit and save.
"""

from __future__ import annotations

import argparse
import asyncio
import datetime
import json
import select
import sys
import termios
import time
import tty
from typing import TYPE_CHECKING

from petctl.config import LOOP_LIMITS
from petctl.perception.stroke import CradleDetector, HoldDetector, StrokeDetector

if TYPE_CHECKING:
    from petctl.types import RobotState

POLL_HZ = 20

DEFAULT_COUNTDOWN_S: float = 2.0

LABEL_KEYS: dict[str, str] = {
    "s": "stroke",
    "h": "hold",
    "p": "poke",
    "q": "squeeze",
    "c": "cradle",
    "b": "baseline",
    "n": "none",
}
STOP_KEYS = {"\r", "\n", " "}
QUIT_KEYS = {"x", "\x03"}  # x or Ctrl-C byte


class Recorder:
    """Manages recording state, JSONL logging, and terminal rendering."""

    def __init__(self, out_path: str, countdown_s: float = DEFAULT_COUNTDOWN_S) -> None:
        self._out_path = out_path
        self._countdown_s = countdown_s
        self._file = open(out_path, "w")
        self._total_frames = 0
        self._label_counts: dict[str, int] = {}
        self._current_label: str | None = None
        self._pending_label: str | None = None   # label waiting for countdown
        self._countdown_end: float | None = None
        self._session_id = 0
        self._session_frames = 0
        self._session_start: float | None = None
        self._lines_printed = 0
        self._is_tty = sys.stdout.isatty()
        self._stroke_det = StrokeDetector()
        self._hold_det = HoldDetector()
        self._cradle_det = CradleDetector()
        self._detector_str = "—"

    @property
    def label_counts(self) -> dict[str, int]:
        return self._label_counts

    def toggle_label(self, label: str) -> None:
        if self._current_label == label or self._pending_label == label:
            self._stop()
        else:
            self._stop()
            if self._countdown_s > 0:
                self._pending_label = label
                self._countdown_end = time.monotonic() + self._countdown_s
            else:
                self._start(label)

    def stop(self) -> None:
        self._stop()

    def _start(self, label: str) -> None:
        self._current_label = label
        self._pending_label = None
        self._countdown_end = None
        self._session_id += 1
        self._session_frames = 0
        self._session_start = time.monotonic()

    def _stop(self) -> None:
        self._current_label = None
        self._pending_label = None
        self._countdown_end = None
        self._session_start = None
        self._session_frames = 0

    def _tick_countdown(self) -> None:
        if self._pending_label and self._countdown_end is not None:
            if time.monotonic() >= self._countdown_end:
                self._start(self._pending_label)

    def update(self, state: "RobotState") -> None:
        self._tick_countdown()
        stroke = self._stroke_det.update(state)
        cradle = self._cradle_det.update(state)
        hold = self._hold_det.update(state)

        if stroke:
            self._detector_str = (
                f"STROKE  vel={stroke.velocity:+.2f}  R²={stroke.confidence:.2f}"
                f"  cen={stroke.centroid:.1f}  {'h→t' if stroke.velocity > 0 else 't→h'}"
            )
        elif cradle:
            self._detector_str = (
                f"CRADLE  {len(cradle.modules)} mods  intensity={cradle.intensity:.2f}"
            )
        elif hold:
            self._detector_str = (
                f"HOLD    blobs={len(hold.q_blobs)}  cen={hold.centroid:.1f}"
                f"  dur={hold.duration:.1f}s"
            )
        else:
            self._detector_str = "—"

        if self._current_label is not None:
            self._log(state, stroke, hold, cradle)
            self._session_frames += 1
            self._label_counts[self._current_label] = (
                self._label_counts.get(self._current_label, 0) + 1
            )

        self._total_frames += 1
        self._render(state)

    def _log(self, state: "RobotState", stroke, hold, cradle) -> None:
        sensors_out: dict[str, dict] = {}
        for mid, sens in state.sensors.items():
            sensors_out[str(mid)] = {
                "left_pads": list(sens.touch_left_pads),
                "right_pads": list(sens.touch_right_pads),
                "middle_pads": list(sens.touch_middle_pads),
                "left_fsr": sens.pressure_left,
                "right_fsr": sens.pressure_right,
                "middle_fsr": sens.pressure_middle,
            }
        record = {
            "ts": time.time(),
            "label": self._current_label,
            "session_id": self._session_id,
            "sensors": sensors_out,
            "motor": {
                "pos": {str(k): v for k, v in state.servo_positions.items()},
                "vel": {str(k): v for k, v in state.motor_velocities.items()},
                "torque": {str(k): v for k, v in state.motor_torques.items()},
            },
            "detector": {
                "stroke": (
                    {"vel": stroke.velocity, "r2": stroke.confidence,
                     "centroid": stroke.centroid}
                    if stroke else None
                ),
                "hold": (
                    {"blobs": len(hold.q_blobs), "centroid": hold.centroid,
                     "duration": hold.duration}
                    if hold else None
                ),
                "cradle": (
                    {"modules": cradle.modules, "intensity": cradle.intensity}
                    if cradle else None
                ),
            },
        }
        self._file.write(json.dumps(record) + "\n")
        self._file.flush()

    def _render(self, state: "RobotState") -> None:
        lines: list[str] = []
        lines.append(
            f"  PET gesture recorder  |  frames={self._total_frames}"
            f"  |  {self._out_path}"
        )
        lines.append("")
        lines.append(f"  Detector:  {self._detector_str}")
        lines.append("")
        lines.append("  ── Sensors ─────────────────────────────────────────────────────")
        for mid in sorted(state.sensors):
            s = state.sensors[mid]
            lines.append(
                f"  M{mid}  "
                f"L:{s.touch_left:.2f}  R:{s.touch_right:.2f}  T:{s.touch_middle:.2f}"
                f"  fsr {s.pressure_left:.2f}/{s.pressure_right:.2f}/{s.pressure_middle:.2f}"
            )
        lines.append("")
        lines.append("  ── Recording ───────────────────────────────────────────────────")
        if self._pending_label and self._countdown_end is not None:
            remaining = max(0.0, self._countdown_end - time.monotonic())
            lines.append(
                f"  ◉ COUNTDOWN  label=\"{self._pending_label}\"  "
                f"starting in {remaining:.1f}s — get into position!"
            )
        elif self._current_label is not None:
            dur = time.monotonic() - (self._session_start or time.monotonic())
            lines.append(
                f"  ● RECORDING  label=\"{self._current_label}\"  "
                f"session={self._session_id}  frames={self._session_frames}  ({dur:.1f}s)"
            )
        else:
            lines.append("  ○ idle  (press a label key to start recording)")
        counts = "  ".join(
            f"{lb}={self._label_counts.get(lb, 0)}"
            for lb in LABEL_KEYS.values()
        )
        lines.append(f"  {counts}")
        lines.append("")
        key_help = "  ".join(f"{k}={v}" for k, v in LABEL_KEYS.items())
        lines.append(f"  Keys: {key_help}  x=quit")
        lines.append("        (press label key to start/stop; Enter or Space to stop)")

        if self._is_tty and self._lines_printed:
            sys.stdout.write(f"\033[{self._lines_printed}A")
        for line in lines:
            sys.stdout.write(line + "\033[K\n")
        sys.stdout.flush()
        self._lines_printed = len(lines)

    def close(self) -> None:
        self._file.close()


async def main(host: str, port: int, out_path: str, dry_run: bool, dry_run_mode: str, countdown_s: float = DEFAULT_COUNTDOWN_S) -> None:
    if dry_run:
        from petctl.backends.mock import MockBackend
        backend = MockBackend(mode=dry_run_mode, num_modules=8)
        print("[gesture_recorder] Using MockBackend")
    else:
        from petctl.backends.robot import RobotBackend
        backend = RobotBackend(
            host=host, port=port, calibrate_on_connect=False, auto_reconnect=False
        )
        print(f"[gesture_recorder] Connecting to {host}:{port} ...")

    ok = await backend.connect()
    if not ok:
        print("[gesture_recorder] Connection failed.")
        return

    print(f"[gesture_recorder] Connected. Writing to: {out_path}")
    print("[gesture_recorder] Press label keys to record. Ctrl-C or x to quit.\n")

    recorder = Recorder(out_path, countdown_s=countdown_s)
    interval = 1.0 / POLL_HZ

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    tty.setcbreak(fd)

    running = True
    try:
        while running:
            t0 = time.monotonic()

            state = await backend.get_state()
            recorder.update(state)

            if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                key = sys.stdin.read(1)
                if key in QUIT_KEYS:
                    running = False
                elif key in STOP_KEYS:
                    recorder.stop()
                elif key in LABEL_KEYS:
                    recorder.toggle_label(LABEL_KEYS[key])

            elapsed = time.monotonic() - t0
            remaining = interval - elapsed
            if remaining > 0:
                await asyncio.sleep(remaining)

    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        recorder.close()
        await backend.disconnect()
        print(f"\n[gesture_recorder] Saved: {out_path}")
        total = sum(recorder.label_counts.values())
        print(f"[gesture_recorder] {total} labeled frames")
        for lb, count in recorder.label_counts.items():
            if count:
                print(f"  {lb}: {count}")


if __name__ == "__main__":
    from petctl.backends.robot import ROBOT_DEFAULT_HOST, ROBOT_DEFAULT_PORT

    parser = argparse.ArgumentParser(
        description="Record labeled gesture sensor data from PET robot"
    )
    parser.add_argument("--host", default=ROBOT_DEFAULT_HOST)
    parser.add_argument("--port", type=int, default=ROBOT_DEFAULT_PORT)
    parser.add_argument(
        "--out", default=None, metavar="PATH",
        help="Output JSONL path (default: gesture_data_<timestamp>.jsonl)",
    )
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument(
        "--dry-run-mode", default="mock-sensor-sine",
        choices=["noise", "mock-sensor-sine"],
    )
    parser.add_argument(
        "--countdown", type=float, default=DEFAULT_COUNTDOWN_S, metavar="SECONDS",
        help=f"Delay before recording starts after pressing a label key (default: {DEFAULT_COUNTDOWN_S}s, 0 to disable)",
    )
    args = parser.parse_args()

    if args.out is None:
        ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        out_path = f"gesture_data_{ts}.jsonl"
    else:
        out_path = args.out

    asyncio.run(main(
        host=args.host,
        port=args.port,
        out_path=out_path,
        dry_run=args.dry_run,
        dry_run_mode=args.dry_run_mode,
        countdown_s=args.countdown,
    ))
