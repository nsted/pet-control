"""Tests for SafetyBackend: PowerManager protection applied around an inner Backend."""

from __future__ import annotations

import pytest

from petctl.backends.safety import SafetyBackend
from petctl.power_manager import BinPackPolicy, PowerManager, PowerThresholds
from petctl.protocols import Backend
from petctl.types import RobotState, ServoCommand

M1, M2, M3 = 1, 2, 3


def _state(
    *,
    ids: list[int],
    drive_temps: dict[int, int] | None = None,
    voltage_v: float = 12.0,
) -> RobotState:
    rs = RobotState(
        active_servo_ids=set(ids),
        servo_positions={i: 0.25 * i for i in ids},
        motor_temperatures=drive_temps or {},
        connected=True,
    )
    rs.__class__ = type(
        "_TestState",
        (RobotState,),
        {
            "battery_voltage_v": property(lambda self: voltage_v),
            "battery_current_amps": property(lambda self: 0.0),
        },
    )
    return rs


class _StubBackend(Backend):
    """Scripted inner backend that records every call."""

    def __init__(self) -> None:
        self.state = _state(ids=[M1, M2, M3])
        self.sent: list[list[ServoCommand]] = []
        self.calls: list[tuple] = []

    async def connect(self) -> bool:
        return True

    async def disconnect(self) -> None:
        pass

    async def get_state(self) -> RobotState:
        return self.state

    async def send_commands(self, commands: list[ServoCommand]) -> None:
        self.sent.append(list(commands))

    @property
    def is_connected(self) -> bool:
        return True

    async def disable_torques(self) -> None:
        self.calls.append(("disable_torques",))

    async def disable_motor(self, motor_id: int) -> None:
        self.calls.append(("disable_motor", motor_id))

    async def enable_motor(self, motor_id: int) -> None:
        self.calls.append(("enable_motor", motor_id))


def _cmd(mid: int, kp: float = 10.0) -> ServoCommand:
    return ServoCommand(servo_id=mid, position=0.5, kp=kp, kd=1.0, torque_ff=0.2)


@pytest.mark.asyncio
async def test_healthy_passthrough_and_telemetry() -> None:
    inner = _StubBackend()
    sb = SafetyBackend(inner)
    state = await sb.get_state()
    assert state.power_telemetry is not None
    cmds = [_cmd(M1)]
    await sb.send_commands(cmds)
    assert len(inner.sent) == 1
    out = inner.sent[0]
    assert [(c.servo_id, c.position, c.kp, c.kd, c.torque_ff) for c in out] == [(M1, 0.5, 10.0, 1.0, 0.2)]
    assert inner.calls == []


@pytest.mark.asyncio
async def test_empty_commands_not_forwarded() -> None:
    inner = _StubBackend()
    sb = SafetyBackend(inner)
    await sb.get_state()
    await sb.send_commands([])
    assert inner.sent == []


@pytest.mark.asyncio
async def test_hard_cutoff_disables_one_motor_and_blocks_its_commands() -> None:
    inner = _StubBackend()
    inner.state = _state(ids=[M1, M2], drive_temps={M1: 67, M2: 30})
    sb = SafetyBackend(inner)
    await sb.get_state()
    assert ("disable_motor", M1) in inner.calls
    assert ("disable_torques",) not in inner.calls
    await sb.send_commands([_cmd(M1), _cmd(M2)])
    sent_ids = {c.servo_id for c in inner.sent[0]}
    assert M1 not in sent_ids and M2 in sent_ids


@pytest.mark.asyncio
async def test_global_emergency_disables_all_torques() -> None:
    inner = _StubBackend()
    inner.state = _state(ids=[M1, M2], drive_temps={M1: 80})
    sb = SafetyBackend(inner)
    await sb.get_state()
    assert ("disable_torques",) in inner.calls
    await sb.send_commands([_cmd(M1), _cmd(M2)])
    assert inner.sent == []  # everything gated


@pytest.mark.asyncio
async def test_voltage_cutoff_disables_torques(monkeypatch: pytest.MonkeyPatch) -> None:
    inner = _StubBackend()
    pm = PowerManager()
    monkeypatch.setattr(pm, "drain_voltage_cutoff", lambda: True)
    sb = SafetyBackend(inner, pm)
    await sb.get_state()
    assert ("disable_torques",) in inner.calls


@pytest.mark.asyncio
async def test_compliance_scaling_does_not_mutate_callers_commands() -> None:
    inner = _StubBackend()
    inner.state = _state(ids=[M1], drive_temps={M1: 58})  # soft warning → 0.5
    sb = SafetyBackend(inner)
    await sb.get_state()
    original = _cmd(M1)
    await sb.send_commands([original])
    out = inner.sent[0][0]
    assert (out.kp, out.kd, out.torque_ff) == (5.0, 0.5, 0.1)
    assert (original.kp, original.kd, original.torque_ff) == (10.0, 1.0, 0.2)


@pytest.mark.asyncio
async def test_budget_pending_motors_get_idle_commands_at_current_position() -> None:
    inner = _StubBackend()
    pm = PowerManager(bin_policy=BinPackPolicy(priority=[M1, M2, M3]), budget_override=2.0)
    sb = SafetyBackend(inner, pm)
    await sb.get_state()
    await sb.send_commands([_cmd(M1), _cmd(M2), _cmd(M3)])
    out = {c.servo_id: c for c in inner.sent[0]}
    assert set(out) == {M1, M2, M3}
    idle = [c for c in out.values() if c.kp == 0.0 and c.kd == 0.0 and c.torque_ff == 0.0]
    assert len(idle) == 2
    for c in idle:
        assert c.position == inner.state.servo_positions[c.servo_id]
    assert out[M1].kp == 10.0  # priority motor stays active


@pytest.mark.asyncio
async def test_power_reset_denied_while_motor_hot_then_reenables_on_success() -> None:
    inner = _StubBackend()
    thresholds = PowerThresholds(temp_hysteresis_cooldown_s=0.0)
    pm = PowerManager(thresholds)
    sb = SafetyBackend(inner, pm)

    inner.state = _state(ids=[M1, M2], drive_temps={M1: 67})
    await sb.get_state()  # M1 disabled
    inner.calls.clear()
    assert await sb.power_reset() is False
    assert not any(c[0] == "enable_motor" for c in inner.calls)

    inner.state = _state(ids=[M1, M2], drive_temps={M1: 30})
    await sb.get_state()  # cool; starts cooldown timer
    inner.calls.clear()
    assert await sb.power_reset() is True
    assert inner.calls == [("enable_motor", M1), ("enable_motor", M2)]
