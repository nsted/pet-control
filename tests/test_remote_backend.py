"""RemoteBackend + Driver end-to-end over an in-memory bus."""

from __future__ import annotations

import asyncio

import pytest

from petctl.backends.mock import MockBackend
from petctl.backends.remote import RemoteBackend
from petctl.backends.safety import SafetyBackend
from petctl.controller import Controller
from petctl.driver import Driver
from petctl.schemes.sine import SineMotion
from petctl.transport import LoopbackBus
from petctl.types import GestureFrame, ServoCommand


async def _rig() -> tuple[Driver, asyncio.Task, RemoteBackend, MockBackend]:
    bus = LoopbackBus()
    inner = MockBackend()
    drv = Driver(SafetyBackend(inner), bus)
    task = asyncio.create_task(drv.run())
    remote = RemoteBackend(bus, connect_timeout_s=3.0)
    return drv, task, remote, inner


async def _teardown(drv: Driver, task: asyncio.Task) -> None:
    drv.stop()
    await asyncio.wait_for(task, timeout=3.0)


@pytest.mark.asyncio
async def test_connect_and_state() -> None:
    drv, task, remote, _ = await _rig()
    assert await remote.connect() is True
    assert remote.is_connected
    s = await remote.get_state()
    assert s.connected and s.active_servo_ids == {1, 2, 3, 4, 5, 6, 7}
    assert s.power_telemetry is not None
    await _teardown(drv, task)


@pytest.mark.asyncio
async def test_connect_times_out_without_driver() -> None:
    remote = RemoteBackend(LoopbackBus(), connect_timeout_s=0.2)
    assert await remote.connect() is False
    assert not remote.is_connected
    s = await remote.get_state()
    assert s.connected is False and s.dt == 0.0


@pytest.mark.asyncio
async def test_each_get_state_is_a_fresh_copy_with_per_call_dt() -> None:
    drv, task, remote, _ = await _rig()
    await remote.connect()
    a = await remote.get_state()
    assert a.dt == 0.0                      # first call
    a.gesture = GestureFrame()              # Controller-style mutation
    a.is_motion_active = True
    a.servo_commanded_positions[1] = 1.0
    await asyncio.sleep(0.02)
    b = await remote.get_state()
    assert b is not a
    assert b.gesture is None and b.is_motion_active is False
    assert 0.015 < b.dt < 0.2               # time since previous call, not since last message
    await _teardown(drv, task)


@pytest.mark.asyncio
async def test_commands_move_the_mock_through_the_bus() -> None:
    drv, task, remote, inner = await _rig()
    await remote.connect()
    for _ in range(100):
        await remote.send_commands([ServoCommand(servo_id=1, position=0.8, kp=0.8, kd=0.1)])
        await asyncio.sleep(0.01)
    s = await remote.get_state()
    assert s.servo_positions[1] > 0.1
    await _teardown(drv, task)


@pytest.mark.asyncio
async def test_services() -> None:
    drv, task, remote, _ = await _rig()
    await remote.connect()
    await remote.enable_motor(1)
    await remote.disable_motor(2)
    await remote.disable_torques()
    await remote.write_home_offsets()
    remote.set_stagger(1, 0.01)
    assert await remote.calibrate_touch() is True
    assert await remote.set_touch_threshold(5, 2) is True
    assert isinstance(await remote.power_reset(), bool)
    await asyncio.sleep(0.05)
    await _teardown(drv, task)


@pytest.mark.asyncio
async def test_controller_drives_mock_through_split() -> None:
    """The real Controller, unchanged, over RemoteBackend → Driver → SafetyBackend → Mock."""
    drv, task, remote, _ = await _rig()
    ctrl = Controller(backend=remote, motion=SineMotion(), visualizers=[])
    asyncio.get_running_loop().call_later(2.0, ctrl.stop)
    await asyncio.wait_for(ctrl.run(), timeout=15.0)
    # Controller shutdown disconnects the remote (closes the bus) — the driver stops itself.
    drv.stop()
    await asyncio.wait_for(task, timeout=3.0)
    assert drv.commands_applied > 50
    assert drv.states_published > 50
    assert remote.states_received > 50
