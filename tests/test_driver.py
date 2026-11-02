"""Driver tests over an in-memory bus (no Zenoh needed)."""

from __future__ import annotations

import asyncio
from typing import Any

import pytest

from petctl import wire
from petctl.backends.mock import MockBackend
from petctl.backends.safety import SafetyBackend
from petctl.driver import Driver
from petctl.transport import LoopbackBus
from petctl.types import ServoCommand


class _SpyMock(MockBackend):
    """MockBackend that records Backend calls."""

    def __init__(self) -> None:
        super().__init__()
        self.calls: list[tuple[str, tuple]] = []
        self.sent: list[list[ServoCommand]] = []

    async def send_commands(self, commands: list[ServoCommand]) -> None:
        self.sent.append(list(commands))
        await super().send_commands(commands)

    async def disable_torques(self) -> None:
        self.calls.append(("disable_torques", ()))

    async def disable_motor(self, motor_id: int) -> None:
        self.calls.append(("disable_motor", (motor_id,)))

    async def enable_motor(self, motor_id: int) -> None:
        self.calls.append(("enable_motor", (motor_id,)))

    async def write_home_offsets(self) -> None:
        self.calls.append(("write_home_offsets", ()))

    def set_stagger(self, motor_id: int, delay_s: float) -> None:
        self.calls.append(("set_stagger", (motor_id, delay_s)))

    async def calibrate_touch(self) -> bool:
        self.calls.append(("calibrate_touch", ()))
        return True

    async def set_touch_threshold(self, touch12, release12, touch3=None, release3=None) -> bool:  # type: ignore[no-untyped-def]
        self.calls.append(("set_touch_threshold", (touch12, release12, touch3, release3)))
        return True

    async def disconnect(self) -> None:
        self.calls.append(("disconnect", ()))
        await super().disconnect()


async def _call_service(bus: LoopbackBus, name: str, **args: Any) -> Any:
    loop = asyncio.get_running_loop()
    raw = await loop.run_in_executor(
        None, bus.request, wire.service_key(name), wire.encode_request(name, **args)
    )
    return wire.decode_reply(raw)


async def _started(inner: _SpyMock) -> tuple[Driver, LoopbackBus, asyncio.Task]:
    bus = LoopbackBus()
    drv = Driver(SafetyBackend(inner), bus)
    task = asyncio.create_task(drv.run())
    for _ in range(200):
        if bus.published:
            break
        await asyncio.sleep(0.01)
    assert bus.published, "driver never published a state"
    return drv, bus, task


async def _stopped(drv: Driver, task: asyncio.Task) -> None:
    drv.stop()
    await asyncio.wait_for(task, timeout=3.0)


@pytest.mark.asyncio
async def test_publishes_decodable_state() -> None:
    inner = _SpyMock()
    drv, bus, task = await _started(inner)
    await asyncio.sleep(0.1)
    await _stopped(drv, task)  # closes the bus
    key, data = bus.published[-1]
    assert key == wire.KEY_STATE
    state = wire.decode_state(data)
    assert state.connected
    assert state.active_servo_ids == {1, 2, 3, 4, 5, 6, 7}
    assert state.power_telemetry is not None
    assert drv.states_published >= 5


@pytest.mark.asyncio
async def test_commands_applied_in_order_through_safety() -> None:
    inner = _SpyMock()
    drv, bus, task = await _started(inner)
    for i in range(5):
        bus.publish(wire.KEY_COMMANDS, wire.encode_commands(
            [ServoCommand(servo_id=1, position=0.1 * i, kp=0.8, kd=0.1)]))
    await asyncio.sleep(0.2)
    await _stopped(drv, task)
    positions = [c.position for batch in inner.sent for c in batch if c.servo_id == 1]
    assert positions == pytest.approx([0.0, 0.1, 0.2, 0.3, 0.4])
    assert drv.commands_applied == 5


@pytest.mark.asyncio
async def test_every_service_reaches_the_backend() -> None:
    inner = _SpyMock()
    drv, bus, task = await _started(inner)
    await _call_service(bus, "enable_motor", motor_id=2)
    await _call_service(bus, "disable_motor", motor_id=3)
    await _call_service(bus, "disable_torques")
    await _call_service(bus, "write_home_offsets")
    await _call_service(bus, "set_stagger", motor_id=4, delay_s=0.01)
    assert await _call_service(bus, "calibrate_touch") is True
    assert await _call_service(bus, "set_touch_threshold", touch12=5, release12=2, touch3=6) is True
    assert isinstance(await _call_service(bus, "power_reset"), bool)
    names = [c[0] for c in inner.calls]
    for expected in ("enable_motor", "disable_motor", "disable_torques", "write_home_offsets",
                     "set_stagger", "calibrate_touch", "set_touch_threshold"):
        assert expected in names, expected
    assert ("set_touch_threshold", (5, 2, 6, None)) in inner.calls
    assert ("set_stagger", (4, 0.01)) in inner.calls
    await _stopped(drv, task)


@pytest.mark.asyncio
async def test_service_errors_come_back_as_errors() -> None:
    inner = _SpyMock()
    drv, bus, task = await _started(inner)
    loop = asyncio.get_running_loop()
    bad = wire.encode_request("enable_motor", motor_id=1).replace(b"enable_motor", b"bogus_name")
    raw = await loop.run_in_executor(None, bus.request, wire.service_key("enable_motor"), bad)
    with pytest.raises(RuntimeError):
        wire.decode_reply(raw)
    await _stopped(drv, task)


@pytest.mark.asyncio
async def test_shutdown_disables_torques_then_disconnects() -> None:
    inner = _SpyMock()
    drv, bus, task = await _started(inner)
    await _stopped(drv, task)
    names = [c[0] for c in inner.calls]
    assert names[-2:] == ["disable_torques", "disconnect"]


@pytest.mark.asyncio
async def test_command_queue_drops_oldest_when_overloaded() -> None:
    inner = _SpyMock()
    drv, bus, task = await _started(inner)
    assert drv._commands is not None
    # Fill without yielding to the consumer, then overflow by 3.
    n = drv._commands.maxsize + 3
    for i in range(n):
        drv._enqueue_command(wire.encode_commands([ServoCommand(servo_id=1, position=float(i))]))
    assert drv.commands_dropped == 3
    await _stopped(drv, task)
