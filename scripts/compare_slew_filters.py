"""
compare_slew_filters.py — SHAPE_LAYER Stage 1.4/1.5 offline prep (mock-only).

Compares candidate arrangements for:

  1.4 Who owns the joint servo — Controller's `_apply_slew_to_commands` (LPF +
      delta cap), RobotBackend's `send_commands` (ramp + anti-windup), or both
      in series (today's default)?
  1.5 What rate should the control loop run at — matched to `motor_update_hz`
      (50 Hz), or today's 4x oversample where `_pending_frames` is overwritten
      three times out of four before the TX loop ever sends it?

This is NOT the decision. Open decision #2 in docs/plans/SHAPE_LAYER.md is
explicit that who-owns-the-servo is Nick's call and "wants hardware time and
probably a before/after recording." This script produces comparison data to
bring to that session — it does not touch `controller.py` or `robot.py`, and
it never talks to a real robot.

Design: reuses the real, unmodified `Controller._apply_slew_to_commands` and
`RobotBackend.send_commands` — not reimplementations of their math — driving
the real `MockBackend._step_dynamics` (Stage 1.3's second-order joint model)
as the physical plant. The only thing this script adds is the staging logic
that decides which of those real methods run each tick, and how often a new
setpoint actually reaches the simulated wire (mirroring `_pending_frames`
being overwritten between `_motor_tx_loop` pops at `motor_update_hz`).

Run: python scripts/compare_slew_filters.py
"""

from __future__ import annotations

import asyncio
import math
from dataclasses import dataclass
from typing import Callable, Literal
from unittest.mock import patch

from petctl.backends.mock import MockBackend
from petctl.backends.robot import RobotBackend, _parse_slcan
from petctl.config import LOOP_LIMITS, MOTOR_LIMITS
from petctl.controller import Controller
from petctl.motors.base import uint_to_float
from petctl.types import ServoCommand

SID = 1
WIRE_HZ = LOOP_LIMITS.motor_update_hz          # 50 Hz — real cadence a setpoint reaches the motor
OVERSAMPLE_HZ = WIRE_HZ * 4                     # today's controller loop rate (Controller._loop)
DURATION_S = 4.0

Owner = Literal["controller", "backend", "both"]


class _FakeControllerState:
    """Minimal stand-in exposing exactly what `_apply_slew_to_commands` reads
    (`self._slew_last_sent_rad`, `self._state.dt`, `self._state.servo_positions`,
    `self.speed_gain`), so we can call the real unbound method without spinning
    up Controller's threads, viz queue, or keyboard listener."""

    def __init__(self, dt: float) -> None:
        self._slew_last_sent_rad: dict[int, float] = {}
        self._state = type("_S", (), {"dt": dt, "servo_positions": {}})()
        self.speed_gain = 1.0


def _make_backend() -> RobotBackend:
    b = RobotBackend()
    b._connected = True
    b._discovered_motors = [SID]
    b._motor_state[SID] = {"pos": 0.0, "vel": 0.0, "torque": 0.0, "drive_temp": 0, "motor_temp": 0, "err_code": 0}
    return b


def _make_mock() -> MockBackend:
    m = MockBackend(num_modules=2)  # servo 1 only
    m._connected = True
    return m


def _decode_pos_from_frame(rb: RobotBackend, frame: str) -> float:
    _, payload = _parse_slcan(frame)
    p_uint = (payload[0] << 8) | payload[1]
    e = rb._profile.encoding
    return uint_to_float(p_uint, 16, e.pos_min, e.pos_max)


@dataclass
class Sample:
    t: float
    target: float
    commanded: float
    actual: float


async def simulate(*, owner: Owner, loop_hz: float, target_fn: Callable[[float], float]) -> list[Sample]:
    """Run one candidate: `owner` picks which real filter(s) run each loop tick;
    `loop_hz` picks the control-loop rate. A new setpoint reaches the mock's
    physical plant only once per wire period (1/WIRE_HZ), mirroring
    `_pending_frames` being overwritten by the controller loop between
    `_motor_tx_loop` pops — the actual mechanism SHAPE_LAYER 1.5 flags.
    """
    dt_loop = 1.0 / loop_hz
    wire_period = 1.0 / WIRE_HZ
    n_ticks = int(DURATION_S / dt_loop)

    fake_ctrl = _FakeControllerState(dt=dt_loop)
    rb = _make_backend()
    mb = _make_mock()

    samples: list[Sample] = []
    t = 0.0
    time_since_wire = wire_period  # force a wire update on the first tick
    commanded = 0.0

    # RobotBackend.send_commands times itself off time.monotonic() (real
    # wall clock), not off dt_loop — a tight synthetic loop would otherwise
    # see near-zero elapsed time between calls regardless of the loop_hz
    # under test. Drive a virtual clock instead so its ramp filter sees the
    # intended dt_loop, exactly as it would running at that rate for real.
    virtual_now = [0.0]
    with patch("time.monotonic", side_effect=lambda: virtual_now[0]):
        for _ in range(n_ticks):
            virtual_now[0] = t
            target = target_fn(t)
            cmd = ServoCommand(servo_id=SID, position=target)

            if owner in ("controller", "both"):
                Controller._apply_slew_to_commands(fake_ctrl, [cmd])
            filtered_pos = cmd.position

            if owner in ("backend", "both"):
                # Seed the backend's physical-feedback view from the mock plant —
                # this is what a real CAN reply frame would report.
                rb._motor_state[SID]["pos"] = mb._servo_positions.get(SID, 0.0)
                await rb.send_commands([ServoCommand(servo_id=SID, position=filtered_pos)])
                frame = rb._pending_frames.get(SID)
                if frame is not None:
                    filtered_pos = _decode_pos_from_frame(rb, frame)

            commanded = filtered_pos

            # Wire discard: a new setpoint reaches the plant only once per
            # wire period, exactly like _pending_frames being overwritten by
            # the oversampled loop between motor_tx_loop pops.
            time_since_wire += dt_loop
            if time_since_wire >= wire_period - 1e-9:
                time_since_wire = 0.0
                await mb.send_commands([
                    ServoCommand(
                        servo_id=SID, position=commanded,
                        kp=MOTOR_LIMITS.kp_default, kd=MOTOR_LIMITS.kd_default,
                    )
                ])

            mb._step_dynamics(dt_loop)
            actual = mb._servo_positions.get(SID, 0.0)

            samples.append(Sample(t=t, target=target, commanded=commanded, actual=actual))
            t += dt_loop

    return samples


def step_target(t: float) -> float:
    return 0.0 if t < 0.1 else 1.0


def sine_target(t: float) -> float:
    # Matches LOOP_LIMITS.max_speed_rad_s's own reference case: snuggle,
    # ±40 deg at 0.4 Hz, peak ~100 deg/s.
    return math.radians(40.0) * math.sin(2 * math.pi * 0.4 * t)


def metrics_for_step(samples: list[Sample]) -> dict[str, float]:
    target_final = samples[-1].target
    eps = 0.02
    settle_t = 0.0
    for s in reversed(samples):
        if abs(s.actual - target_final) >= eps:
            settle_t = s.t
            break
    overshoot = max(0.0, max(s.actual for s in samples) - target_final)
    max_gap = max(abs(s.commanded - s.actual) for s in samples)
    return {"settle_time_s": settle_t, "overshoot_rad": overshoot, "max_cmd_actual_gap_rad": max_gap}


def metrics_for_sine(samples: list[Sample]) -> dict[str, float]:
    tail = [s for s in samples if s.t >= DURATION_S * 0.5]  # steady state
    tracking_err = max(abs(s.actual - s.target) for s in tail)
    cmd_actual_gap = max(abs(s.commanded - s.actual) for s in tail)
    return {"peak_tracking_err_rad": tracking_err, "peak_cmd_actual_gap_rad": cmd_actual_gap}


async def main() -> None:
    owners: list[Owner] = ["controller", "backend", "both"]
    loop_rates = [("matched_50hz", WIRE_HZ), ("oversample_200hz", OVERSAMPLE_HZ)]

    print(f"{'candidate':<28} {'signal':<8} {'metric':<24} {'value':>10}")
    print("-" * 74)
    for owner in owners:
        for rate_label, loop_hz in loop_rates:
            label = f"{owner}/{rate_label}"

            step_samples = await simulate(owner=owner, loop_hz=loop_hz, target_fn=step_target)
            for k, v in metrics_for_step(step_samples).items():
                print(f"{label:<28} {'step':<8} {k:<24} {v:>10.4f}")

            sine_samples = await simulate(owner=owner, loop_hz=loop_hz, target_fn=sine_target)
            for k, v in metrics_for_sine(sine_samples).items():
                print(f"{label:<28} {'sine':<8} {k:<24} {v:>10.4f}")


if __name__ == "__main__":
    asyncio.run(main())
