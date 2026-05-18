"""
Nine motion patterns for PET — standalone control schemes.

Each produces a distinct body movement and can be selected via:
    petctl run --control <name>

Patterns:
    ripple    — two full wave crests visible simultaneously (shorter wavelength)
    pulse     — all joints in phase, whole-body flex and release
    breathe   — slow, tiny-amplitude in-phase breathing
    sway      — travelling wave with amplitude tapering head→tail
    cascade   — travelling wave with amplitude growing head→tail
    slalom    — alternating odd/even sign → S-shape rocking
    twitch    — smoothed per-joint Brownian noise (jittery, organic)
    freeze    — hold all joints at home (0°)
    coil      — quadratic spatial phase → tighter curl toward the tail
"""

from __future__ import annotations

import math
import random
import time
from typing import TYPE_CHECKING

from petctl.protocols import ControlScheme
from petctl.types import RobotState, ServoCommand

if TYPE_CHECKING:
    from petctl.controller import Controller


class RippleControlScheme(ControlScheme):
    """Two full wave crests across the body simultaneously."""

    name = "ripple"

    def __init__(self, amplitude_deg: float = 40.0, hz: float = 0.4) -> None:
        self.amplitude_deg = amplitude_deg
        self.hz = hz
        self._start: float = 0.0

    def on_start(self, controller: "Controller") -> None:
        self._start = time.monotonic()
        print(f"[Ripple] ±{self.amplitude_deg}° at {self.hz} Hz, 2× spatial frequency. Ctrl-C to stop.")

    def update(self, state: RobotState) -> list[ServoCommand]:
        t = time.monotonic() - self._start
        ids = sorted(state.active_servo_ids)
        n = len(ids)
        if n == 0:
            return []
        return [
            ServoCommand.from_angle(
                servo_id=sid,
                angle_deg=self.amplitude_deg * math.sin(
                    2 * math.pi * self.hz * t + (i / n) * 4 * math.pi
                ),
            )
            for i, sid in enumerate(ids)
        ]


class PulseControlScheme(ControlScheme):
    """All joints in phase — whole-body contraction and release."""

    name = "pulse"

    def __init__(self, amplitude_deg: float = 50.0, hz: float = 0.25) -> None:
        self.amplitude_deg = amplitude_deg
        self.hz = hz
        self._start: float = 0.0

    def on_start(self, controller: "Controller") -> None:
        self._start = time.monotonic()
        print(f"[Pulse] ±{self.amplitude_deg}° at {self.hz} Hz, all joints in phase. Ctrl-C to stop.")

    def update(self, state: RobotState) -> list[ServoCommand]:
        t = time.monotonic() - self._start
        angle = self.amplitude_deg * math.sin(2 * math.pi * self.hz * t)
        return [
            ServoCommand.from_angle(servo_id=sid, angle_deg=angle)
            for sid in sorted(state.active_servo_ids)
        ]


class BreatheControlScheme(ControlScheme):
    """Very gentle, slow in-phase motion — aliveness at rest."""

    name = "breathe"

    def __init__(self, amplitude_deg: float = 12.0, hz: float = 0.08) -> None:
        self.amplitude_deg = amplitude_deg
        self.hz = hz
        self._start: float = 0.0

    def on_start(self, controller: "Controller") -> None:
        self._start = time.monotonic()
        print(f"[Breathe] ±{self.amplitude_deg}° at {self.hz} Hz (~{1/self.hz:.0f}s period). Ctrl-C to stop.")

    def update(self, state: RobotState) -> list[ServoCommand]:
        t = time.monotonic() - self._start
        angle = self.amplitude_deg * math.sin(2 * math.pi * self.hz * t)
        return [
            ServoCommand.from_angle(servo_id=sid, angle_deg=angle)
            for sid in sorted(state.active_servo_ids)
        ]


class SwayControlScheme(ControlScheme):
    """Travelling wave with amplitude tapering head→tail (head leads, tail damps)."""

    name = "sway"

    def __init__(
        self,
        amplitude_deg: float = 60.0,
        hz: float = 0.15,
        tail_fraction: float = 0.15,
    ) -> None:
        self.amplitude_deg = amplitude_deg
        self.hz = hz
        self.tail_fraction = tail_fraction
        self._start: float = 0.0

    def on_start(self, controller: "Controller") -> None:
        self._start = time.monotonic()
        print(
            f"[Sway] head ±{self.amplitude_deg}° → tail ±{self.amplitude_deg * self.tail_fraction:.0f}°"
            f" at {self.hz} Hz. Ctrl-C to stop."
        )

    def update(self, state: RobotState) -> list[ServoCommand]:
        t = time.monotonic() - self._start
        ids = sorted(state.active_servo_ids)
        n = len(ids)
        if n == 0:
            return []
        cmds = []
        for i, sid in enumerate(ids):
            frac = i / max(n - 1, 1)
            taper = 1.0 - (1.0 - self.tail_fraction) * frac
            spatial_phase = frac * 2 * math.pi
            angle = self.amplitude_deg * taper * math.sin(2 * math.pi * self.hz * t + spatial_phase)
            cmds.append(ServoCommand.from_angle(servo_id=sid, angle_deg=angle))
        return cmds


class CascadeControlScheme(ControlScheme):
    """Travelling wave with amplitude growing head→tail (crack-the-whip effect)."""

    name = "cascade"

    def __init__(
        self,
        amplitude_deg: float = 60.0,
        hz: float = 0.2,
        head_fraction: float = 0.1,
    ) -> None:
        self.amplitude_deg = amplitude_deg
        self.hz = hz
        self.head_fraction = head_fraction
        self._start: float = 0.0

    def on_start(self, controller: "Controller") -> None:
        self._start = time.monotonic()
        print(
            f"[Cascade] head ±{self.amplitude_deg * self.head_fraction:.0f}° → tail ±{self.amplitude_deg}°"
            f" at {self.hz} Hz. Ctrl-C to stop."
        )

    def update(self, state: RobotState) -> list[ServoCommand]:
        t = time.monotonic() - self._start
        ids = sorted(state.active_servo_ids)
        n = len(ids)
        if n == 0:
            return []
        cmds = []
        for i, sid in enumerate(ids):
            frac = i / max(n - 1, 1)
            taper = self.head_fraction + (1.0 - self.head_fraction) * frac
            spatial_phase = frac * 2 * math.pi
            angle = self.amplitude_deg * taper * math.sin(2 * math.pi * self.hz * t + spatial_phase)
            cmds.append(ServoCommand.from_angle(servo_id=sid, angle_deg=angle))
        return cmds


class SlalomControlScheme(ControlScheme):
    """Odd/even joints get opposing phase → persistent S-shape that rocks side to side."""

    name = "slalom"

    def __init__(self, amplitude_deg: float = 45.0, hz: float = 0.2) -> None:
        self.amplitude_deg = amplitude_deg
        self.hz = hz
        self._start: float = 0.0

    def on_start(self, controller: "Controller") -> None:
        self._start = time.monotonic()
        print(f"[Slalom] ±{self.amplitude_deg}° S-shape rocking at {self.hz} Hz. Ctrl-C to stop.")

    def update(self, state: RobotState) -> list[ServoCommand]:
        t = time.monotonic() - self._start
        ids = sorted(state.active_servo_ids)
        return [
            ServoCommand.from_angle(
                servo_id=sid,
                angle_deg=(1.0 if i % 2 == 0 else -1.0)
                * self.amplitude_deg
                * math.sin(2 * math.pi * self.hz * t),
            )
            for i, sid in enumerate(ids)
        ]


class TwitchControlScheme(ControlScheme):
    """Each joint wanders independently via smoothed Brownian noise."""

    name = "twitch"

    def __init__(self, amplitude_deg: float = 30.0, smoothing: float = 0.06) -> None:
        self.amplitude_deg = amplitude_deg
        # EMA alpha: lower = smoother/slower drift, higher = jerkier
        self.smoothing = smoothing
        self._current: dict[int, float] = {}
        self._target: dict[int, float] = {}

    def on_start(self, controller: "Controller") -> None:
        self._current = {}
        self._target = {}
        print(f"[Twitch] ±{self.amplitude_deg}° random noise per joint. Ctrl-C to stop.")

    def update(self, state: RobotState) -> list[ServoCommand]:
        ids = sorted(state.active_servo_ids)
        cmds = []
        for sid in ids:
            if sid not in self._current:
                self._current[sid] = 0.0
                self._target[sid] = 0.0
            self._target[sid] = max(
                -self.amplitude_deg,
                min(self.amplitude_deg, self._target[sid] + random.gauss(0.0, 0.3) * self.amplitude_deg),
            )
            self._current[sid] += self.smoothing * (self._target[sid] - self._current[sid])
            cmds.append(ServoCommand.from_angle(servo_id=sid, angle_deg=self._current[sid]))
        return cmds


class FreezeControlScheme(ControlScheme):
    """Command all joints to home (0°) and hold there."""

    name = "freeze"

    def on_start(self, controller: "Controller") -> None:
        print("[Freeze] Holding all joints at home (0°). Ctrl-C to stop.")

    def update(self, state: RobotState) -> list[ServoCommand]:
        return [
            ServoCommand.from_angle(servo_id=sid, angle_deg=0.0)
            for sid in sorted(state.active_servo_ids)
        ]


class CoilControlScheme(ControlScheme):
    """Quadratic spatial phase distribution — tighter curl accumulates toward the tail."""

    name = "coil"

    def __init__(self, amplitude_deg: float = 55.0, hz: float = 0.15) -> None:
        self.amplitude_deg = amplitude_deg
        self.hz = hz
        self._start: float = 0.0

    def on_start(self, controller: "Controller") -> None:
        self._start = time.monotonic()
        print(f"[Coil] ±{self.amplitude_deg}° coiling motion at {self.hz} Hz. Ctrl-C to stop.")

    def update(self, state: RobotState) -> list[ServoCommand]:
        t = time.monotonic() - self._start
        ids = sorted(state.active_servo_ids)
        n = len(ids)
        if n == 0:
            return []
        return [
            ServoCommand.from_angle(
                servo_id=sid,
                angle_deg=self.amplitude_deg * math.sin(
                    2 * math.pi * self.hz * t + (i / max(n - 1, 1)) ** 2 * 2 * math.pi
                ),
            )
            for i, sid in enumerate(ids)
        ]


ALL_PATTERNS: list[type[ControlScheme]] = [
    RippleControlScheme,
    PulseControlScheme,
    BreatheControlScheme,
    SwayControlScheme,
    CascadeControlScheme,
    SlalomControlScheme,
    TwitchControlScheme,
    FreezeControlScheme,
    CoilControlScheme,
]

PATTERN_NAMES: list[str] = [cls.name for cls in ALL_PATTERNS]
