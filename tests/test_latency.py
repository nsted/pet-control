"""Tests for latency statistics and their wiring."""

from __future__ import annotations

import asyncio

import pytest

from petctl import wire
from petctl.backends.mock import MockBackend
from petctl.backends.remote import RemoteBackend
from petctl.driver import Driver
from petctl.latency import SampleStats
from petctl.transport import LoopbackBus
from petctl.types import ServoCommand


def test_empty_stats() -> None:
    s = SampleStats()
    assert s.summary_ms() is None
    assert "no samples" in s.format("x")


def test_percentiles_in_milliseconds() -> None:
    s = SampleStats()
    for i in range(1, 101):
        s.add(i / 1000.0)             # 1..100 ms
    m = s.summary_ms()
    assert m is not None
    assert m["n"] == 100 and m["max"] == pytest.approx(100.0)
    assert m["p50"] == pytest.approx(50.5) and m["p95"] == pytest.approx(95.05)


def test_interarrival() -> None:
    s = SampleStats()
    for t in (0.0, 0.005, 0.010, 0.020):
        s.add_arrival(t)
    m = s.summary_ms()
    assert m is not None and m["n"] == 3 and m["max"] == pytest.approx(10.0)


def test_rolling_buffer_is_bounded() -> None:
    s = SampleStats(maxlen=10)
    for i in range(100):
        s.add(float(i))
    assert len(s) == 10


@pytest.mark.asyncio
async def test_driver_and_remote_collect_latency() -> None:
    bus = LoopbackBus()
    drv = Driver(MockBackend(), bus)
    task = asyncio.create_task(drv.run())
    remote = RemoteBackend(bus, connect_timeout_s=3.0)
    assert await remote.connect()
    for _ in range(20):
        await remote.send_commands([ServoCommand(servo_id=1, position=0.1)])
        await asyncio.sleep(0.01)
    drv.stop()
    await asyncio.wait_for(task, timeout=3.0)
    assert len(drv.cmd_latency) >= 10 and len(drv.cmd_interarrival) >= 10
    assert len(remote.state_latency) >= 10 and len(remote.state_interarrival) >= 10
    m = remote.state_latency.summary_ms()
    assert m is not None and 0.0 <= m["p50"] < 50.0     # in-memory bus: tiny, non-negative
    assert wire.KEY_STATE  # sanity: module imported
