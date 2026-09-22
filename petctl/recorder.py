"""
StateRecorder — writes one JSON line per control-loop tick to a `.jsonl` file.

Feeds `petctl run --record`. Non-blocking: `record()` enqueues onto a bounded
queue drained by a daemon thread, mirroring `Controller._viz_worker` (see
controller.py) so a slow disk never stalls the control loop — frames are
dropped, not blocked on, when the writer falls behind.
"""

from __future__ import annotations

import dataclasses
import json
import logging
import queue
import threading
from typing import Optional

from petctl.types import RobotState, ServoCommand

logger = logging.getLogger(__name__)

_DEFAULT_QUEUE_MAXSIZE = 64


class StateRecorder:
    """Records `(RobotState, commands)` pairs to a JSON-lines file, one line per tick.

    Args:
        path:    Output file path, opened line-buffered so a crash doesn't
                 lose the tail.
        maxsize: Bounded queue depth between `record()` and the writer thread.
    """

    def __init__(self, path: str, maxsize: int = _DEFAULT_QUEUE_MAXSIZE) -> None:
        self._path = path
        self._queue: queue.Queue[Optional[dict]] = queue.Queue(maxsize=maxsize)
        self._dropped = 0
        self._file = open(path, "w", buffering=1)
        self._thread = threading.Thread(target=self._worker, name="state-recorder", daemon=True)
        self._thread.start()

    def record(self, state: RobotState, commands: list[ServoCommand]) -> None:
        """Enqueue one tick's frame. Never blocks; drops the frame if the queue is full."""
        frame = {
            "timestamp": state.timestamp,
            "dt": state.dt,
            "connected": state.connected,
            "sensors": {mid: s.as_dict() for mid, s in state.sensors.items()},
            "servo_positions": state.servo_positions,
            "motor_velocities": state.motor_velocities,
            "motor_torques": state.motor_torques,
            "motor_temperatures": state.motor_temperatures,
            "motor_winding_temperatures": state.motor_winding_temperatures,
            "motor_err_codes": state.motor_err_codes,
            "gesture": dataclasses.asdict(state.gesture) if state.gesture is not None else None,
            "commands": [dataclasses.asdict(cmd) for cmd in commands],
            "power_telemetry": (
                dataclasses.asdict(state.power_telemetry) if state.power_telemetry is not None else None
            ),
        }
        try:
            self._queue.put_nowait(frame)
        except queue.Full:
            self._dropped += 1

    def close(self) -> None:
        """Flush remaining frames, stop the writer thread, and close the file."""
        self._queue.put(None)
        self._thread.join(timeout=2.0)
        self._file.close()
        if self._dropped:
            logger.warning(
                "[StateRecorder] Dropped %d frame(s) (queue full) writing to %s",
                self._dropped, self._path,
            )

    def _worker(self) -> None:
        while True:
            frame = self._queue.get()
            if frame is None:
                break
            try:
                self._file.write(json.dumps(frame, default=str) + "\n")
            except Exception as e:
                logger.error("[StateRecorder] Write error: %s", e)
