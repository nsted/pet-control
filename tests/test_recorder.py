"""Tests for StateRecorder (SHAPE_LAYER Stage 1.2)."""

from __future__ import annotations

import json
import threading

from petctl.config import MOTOR_LIMITS
from petctl.perception.contact import ContactReading, ContactType
from petctl.recorder import StateRecorder
from petctl.types import GestureFrame, ModuleSensors, RobotState, ServoCommand

_EXPECTED_KEYS = (
    "timestamp", "dt", "connected", "sensors", "servo_positions",
    "motor_velocities", "motor_torques", "motor_temperatures",
    "motor_winding_temperatures", "motor_err_codes", "gesture",
    "commands", "power_telemetry",
)


class _NoOpThread:
    """Stand-in for threading.Thread that never drains the queue."""

    def __init__(self, *args, **kwargs) -> None:
        pass

    def start(self) -> None:
        pass

    def join(self, timeout: float | None = None) -> None:
        pass


class TestStateRecorderBasics:
    def test_records_lines_with_expected_keys(self, tmp_path):
        path = tmp_path / "out.jsonl"
        rec = StateRecorder(str(path))

        state = RobotState(timestamp=1.0, dt=0.02, connected=True)
        state.sensors = {0: ModuleSensors(module_id=0)}
        rec.record(state, [ServoCommand(servo_id=1, position=0.5)])
        rec.record(state, [])
        rec.close()

        lines = path.read_text().strip().splitlines()
        assert len(lines) == 2
        for line in lines:
            frame = json.loads(line)
            for key in _EXPECTED_KEYS:
                assert key in frame

        first = json.loads(lines[0])
        assert first["timestamp"] == 1.0
        assert first["dt"] == 0.02
        assert first["connected"] is True
        assert first["servo_positions"] == {}
        assert first["commands"] == [{
            "servo_id": 1,
            "position": 0.5,
            "velocity": None,
            "kp": MOTOR_LIMITS.kp_default,
            "kd": MOTOR_LIMITS.kd_default,
            "torque_ff": 0.0,
        }]

    def test_gesture_and_contact_type_serialize_as_plain_json(self, tmp_path):
        path = tmp_path / "out.jsonl"
        rec = StateRecorder(str(path))

        state = RobotState()
        state.gesture = GestureFrame(
            contact=ContactReading(contact_type=ContactType.POKE, centroid=1.0, side="top")
        )
        rec.record(state, [])
        rec.close()

        frame = json.loads(path.read_text().strip())
        assert frame["gesture"]["contact"]["contact_type"] == "poke"
        assert frame["gesture"]["contact"]["side"] == "top"

    def test_no_gesture_or_power_telemetry_serializes_as_none(self, tmp_path):
        path = tmp_path / "out.jsonl"
        rec = StateRecorder(str(path))

        rec.record(RobotState(), [])
        rec.close()

        frame = json.loads(path.read_text().strip())
        assert frame["gesture"] is None
        assert frame["power_telemetry"] is None


class TestQueueFullHandling:
    def test_full_queue_is_dropped_not_raised_or_blocked(self, tmp_path, monkeypatch):
        """When the writer thread never drains, record() must not raise or block."""
        monkeypatch.setattr(threading, "Thread", _NoOpThread)

        path = tmp_path / "out.jsonl"
        rec = StateRecorder(str(path), maxsize=2)

        state = RobotState()
        for _ in range(5):
            rec.record(state, [])  # must not raise even once the queue is full

        assert rec._dropped == 3
