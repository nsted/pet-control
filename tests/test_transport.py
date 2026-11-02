"""Tests for petctl.transport (LoopbackBus always; ZenohBus when eclipse-zenoh is installed)."""

from __future__ import annotations

import socket
import time

import pytest

from petctl.transport import Bus, LoopbackBus


def _free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def _exercise(driver_side: Bus, brain_side: Bus) -> None:
    got: list[bytes] = []
    brain_side.subscribe("pet/state", got.append)
    driver_side.serve("pet/svc/echo", lambda b: b"re:" + b)
    time.sleep(0.5)  # let Zenoh sessions discover each other
    for i in range(20):
        driver_side.publish("pet/state", str(i).encode())
        time.sleep(0.005)
    time.sleep(0.2)
    assert got == [str(i).encode() for i in range(20)]  # delivered, in order
    assert brain_side.request("pet/svc/echo", b"hi", timeout=2.0) == b"re:hi"
    with pytest.raises(TimeoutError):
        brain_side.request("pet/svc/missing", b"", timeout=0.3)


def test_loopback_bus() -> None:
    bus = LoopbackBus()
    _exercise(bus, bus)


def test_zenoh_bus_pub_sub_and_request() -> None:
    pytest.importorskip("zenoh")
    from petctl.transport import ZenohBus

    ep = f"tcp/127.0.0.1:{_free_port()}"
    driver = ZenohBus(listen=ep)
    brain = ZenohBus(connect=ep)
    try:
        _exercise(driver, brain)
    finally:
        brain.close()
        driver.close()


def test_zenoh_bus_requires_exactly_one_endpoint_kind() -> None:
    pytest.importorskip("zenoh")
    from petctl.transport import ZenohBus

    with pytest.raises(ValueError):
        ZenohBus()
    with pytest.raises(ValueError):
        ZenohBus(listen="tcp/127.0.0.1:1", connect="tcp/127.0.0.1:2")
