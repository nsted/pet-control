"""
Ten motion patterns for PET — standalone control schemes.

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
    curl      — ramp all joints to same large angle, coil body into a tight ball and hold
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


class Spin7ControlScheme(ControlScheme):
    """Continuously rotate joint 7 (tail); all other joints hold at home.

    Every full revolution the motor's software zero is reset to its current
    physical position, keeping commanded angles in the safe MIT encoding range
    indefinitely.
    """

    name = "spin7"

    def __init__(self, speed_deg_per_s: float = 30.0) -> None:
        self.speed_deg_per_s = speed_deg_per_s
        self._pos_deg: float = 0.0
        self._last_t: float = 0.0
        self._backend = None

    def on_start(self, controller: "Controller") -> None:
        self._pos_deg = 0.0
        self._last_t = time.monotonic()
        self._backend = controller.backend
        print(f"[Spin7] Joint 7 spinning at {self.speed_deg_per_s}°/s. Ctrl-C to stop.")

    def update(self, state: RobotState) -> list[ServoCommand]:
        now = time.monotonic()
        self._pos_deg += self.speed_deg_per_s * (now - self._last_t)
        self._last_t = now

        if self._pos_deg >= 360.0:
            self._pos_deg -= 360.0
            if hasattr(self._backend, "reset_motor_zero"):
                self._backend.reset_motor_zero(7)

        cmds = []
        for sid in sorted(state.active_servo_ids):
            angle = self._pos_deg if sid == 7 else 0.0
            cmds.append(ServoCommand.from_angle(servo_id=sid, angle_deg=angle))
        return cmds


class StrokeReactControlScheme(ControlScheme):
    """React to lateral stroke by continuously spinning toward the touching hand.

    Each module drives its own servo independently from its capacitive sensors.
    Touch drives rotation velocity — the joint keeps turning as long as the hand
    is present. Lifting the hand holds the current angle. When left and right
    activation are equal, the joint picks a random spin direction at first contact.
    """

    name = "stroke"

    TOUCH_THRESHOLD: float = 0.05   # mean pad activation below this = no touch
    EQUAL_DEADZONE: float = 0.2     # |net_norm| below this = treat as equal
    MAX_SPEED_DEG_PER_S: float = 60.0
    POS_LIMIT_DEG: float = math.degrees(12.5)  # stay inside MIT encoding range

    def __init__(self) -> None:
        self._angle_deg: dict[int, float] = {}
        self._rand_dir: dict[int, float] = {}
        self._was_touching: dict[int, bool] = {}

    def on_start(self, controller: "Controller") -> None:
        self._angle_deg = {}
        self._rand_dir = {}
        self._was_touching = {}
        print("[StrokeReact] Touch left → spin right; right → spin left. Hand off = hold. Ctrl-C to stop.")

    def update(self, state: RobotState) -> list[ServoCommand]:
        dt = max(state.dt, 1e-4)
        cmds: list[ServoCommand] = []

        for module_id, sensors in state.sensors.items():
            servo_id = module_id          # module 1–7 maps directly to servo 1–7
            if servo_id not in state.active_servo_ids:
                continue

            if servo_id not in self._angle_deg:
                self._angle_deg[servo_id] = math.degrees(
                    state.servo_positions.get(servo_id, 0.0)
                )

            left = sensors.touch_left
            right = sensors.touch_right
            total = left + right

            if total < self.TOUCH_THRESHOLD:
                self._was_touching[servo_id] = False
                self._rand_dir.pop(servo_id, None)
                # hold — send current angle so the joint doesn't drift
                cmds.append(ServoCommand.from_angle(
                    servo_id=servo_id, angle_deg=self._angle_deg[servo_id]
                ))
                continue

            net_norm = (left - right) / max(total, 1e-6)   # –1 (right) … +1 (left)

            if abs(net_norm) < self.EQUAL_DEADZONE:
                if not self._was_touching.get(servo_id, False) or servo_id not in self._rand_dir:
                    self._rand_dir[servo_id] = random.choice([-1.0, 1.0])
                direction = self._rand_dir[servo_id]
            else:
                self._rand_dir.pop(servo_id, None)
                direction = net_norm   # proportional to lateral bias

            self._angle_deg[servo_id] = max(
                -self.POS_LIMIT_DEG,
                min(self.POS_LIMIT_DEG, self._angle_deg[servo_id] + direction * self.MAX_SPEED_DEG_PER_S * dt),
            )
            self._was_touching[servo_id] = True
            cmds.append(ServoCommand.from_angle(
                servo_id=servo_id, angle_deg=self._angle_deg[servo_id]
            ))

        return cmds


class WanderControlScheme(ControlScheme):
    """Each joint turns until it stalls under load, then reverses direction.

    Stall requires two conditions to both be true within STALL_WINDOW_S:
      1. Position has not advanced STALL_THRESHOLD_RAD (motor is stuck).
      2. Peak torque during the window has reached STALL_TORQUE_NM (motor is
         actively pushing — not just drifting or coasting to a slow stop).

    On reversal, _pos_cmd syncs to actual position and the controller slew
    filter is reset to match via take_slew_resets(), so the motor begins
    moving in the new direction on the very next tick.
    """

    name = "wander"

    SPEED_DEG_PER_S: float = 45.0
    STALL_THRESHOLD_RAD: float = 0.3   # ~17° minimum travel per window
    STALL_WINDOW_S: float = 0.8
    STALL_TORQUE_NM: float = 0.9       # peak torque that must be seen during the window
    REVERSAL_COOLDOWN_S: float = 0.5
    MAX_POS_RAD: float = 12.4

    def __init__(self, speed_deg_per_s: float = SPEED_DEG_PER_S) -> None:
        self._speed_rad_s = math.radians(speed_deg_per_s)
        self._pos_cmd: dict[int, float] = {}
        self._direction: dict[int, float] = {}
        self._stall_since: dict[int, float] = {}
        self._stall_pos_snap: dict[int, float] = {}
        self._stall_peak_torque: dict[int, float] = {}
        self._reversed_at: dict[int, float] = {}
        self._pending_slew_resets: dict[int, float] = {}

    def on_start(self, controller: "Controller") -> None:
        self._pos_cmd = {}
        self._direction = {}
        self._stall_since = {}
        self._stall_pos_snap = {}
        self._stall_peak_torque = {}
        self._reversed_at = {}
        self._pending_slew_resets = {}
        print(
            f"[Wander] Each joint turns at {math.degrees(self._speed_rad_s):.0f}°/s, "
            "reversing on stall. Ctrl-C to stop."
        )

    def take_slew_resets(self) -> dict[int, float]:
        """Called by the controller before _apply_slew_to_commands each tick."""
        resets = self._pending_slew_resets
        self._pending_slew_resets = {}
        return resets

    def update(self, state: RobotState) -> list[ServoCommand]:
        now = time.monotonic()
        dt = max(state.dt, 1e-4)
        cmds: list[ServoCommand] = []

        for sid in sorted(state.active_servo_ids):
            if sid not in self._pos_cmd:
                self._pos_cmd[sid] = state.servo_positions.get(sid, 0.0)
                self._direction[sid] = random.choice([-1.0, 1.0])
                self._stall_since[sid] = 0.0
                self._reversed_at[sid] = now

            in_cooldown = (now - self._reversed_at[sid]) < self.REVERSAL_COOLDOWN_S

            stall_triggered = False
            if not in_cooldown:
                actual = state.servo_positions.get(sid)
                torque = abs(state.motor_torques.get(sid, 0.0))

                if actual is not None:
                    if sid not in self._stall_pos_snap:
                        self._stall_since[sid] = now
                        self._stall_pos_snap[sid] = actual
                        self._stall_peak_torque[sid] = torque
                    elif abs(actual - self._stall_pos_snap[sid]) >= self.STALL_THRESHOLD_RAD:
                        # Motor moved — slide the window forward and reset peak
                        self._stall_since[sid] = now
                        self._stall_pos_snap[sid] = actual
                        self._stall_peak_torque[sid] = torque
                    else:
                        self._stall_peak_torque[sid] = max(self._stall_peak_torque[sid], torque)
                        if now - self._stall_since[sid] >= self.STALL_WINDOW_S:
                            if self._stall_peak_torque[sid] >= self.STALL_TORQUE_NM:
                                stall_triggered = True
                            else:
                                # Position stalled but not under load — restart window
                                self._stall_since[sid] = now
                                self._stall_pos_snap[sid] = actual
                                self._stall_peak_torque[sid] = torque
            else:
                self._stall_since[sid] = 0.0
                self._stall_pos_snap.pop(sid, None)
                self._stall_peak_torque.pop(sid, None)

            # Fallback: always reverse at the MIT encoding ceiling.
            if not stall_triggered and not in_cooldown and abs(self._pos_cmd[sid]) >= self.MAX_POS_RAD:
                stall_triggered = True

            if stall_triggered:
                actual = state.servo_positions.get(sid, self._pos_cmd[sid])
                clamped = max(-self.MAX_POS_RAD, min(self.MAX_POS_RAD, actual))
                peak_t = self._stall_peak_torque.get(sid, 0.0)
                self._pos_cmd[sid] = clamped
                self._pending_slew_resets[sid] = clamped
                self._direction[sid] *= -1.0
                self._reversed_at[sid] = now
                self._stall_since[sid] = 0.0
                self._stall_pos_snap.pop(sid, None)
                self._stall_peak_torque.pop(sid, None)
                print(f"[Wander] Motor {sid} → reversing at {math.degrees(clamped):.1f}° (peak torque {peak_t:.2f} Nm)")

            self._pos_cmd[sid] += self._direction[sid] * self._speed_rad_s * dt
            self._pos_cmd[sid] = max(-self.MAX_POS_RAD, min(self.MAX_POS_RAD, self._pos_cmd[sid]))

            cmds.append(ServoCommand(servo_id=sid, position=self._pos_cmd[sid]))

        return cmds


class ExploreControlScheme(ControlScheme):
    """Copy of WanderControlScheme for parameter tuning.

    Identical behaviour to wander — tweak the class constants here
    without disturbing the reference.
    """

    name = "explore"

    SPEED_DEG_PER_S: float = 45.0
    STALL_VEL_THRESHOLD: float = 0.15
    STALL_WINDOW_S: float = 0.6
    REVERSAL_COOLDOWN_S: float = 0.4
    MAX_POS_RAD: float = 12.4

    def __init__(self, speed_deg_per_s: float = SPEED_DEG_PER_S) -> None:
        self._speed_rad_s = math.radians(speed_deg_per_s)
        self._pos_cmd: dict[int, float] = {}
        self._direction: dict[int, float] = {}
        self._stall_since: dict[int, float] = {}
        self._reversed_at: dict[int, float] = {}

    def on_start(self, controller: "Controller") -> None:
        self._pos_cmd = {}
        self._direction = {}
        self._stall_since = {}
        self._reversed_at = {}
        print(
            f"[Explore] Each joint turns at {math.degrees(self._speed_rad_s):.0f}°/s, "
            "reversing on stall. Ctrl-C to stop."
        )

    def update(self, state: RobotState) -> list[ServoCommand]:
        now = time.monotonic()
        dt = max(state.dt, 1e-4)
        cmds: list[ServoCommand] = []

        for sid in sorted(state.active_servo_ids):
            if sid not in self._pos_cmd:
                self._pos_cmd[sid] = state.servo_positions.get(sid, 0.0)
                self._direction[sid] = random.choice([-1.0, 1.0])
                self._stall_since[sid] = 0.0
                self._reversed_at[sid] = now

            vel = state.motor_velocities.get(sid, 0.0)
            in_cooldown = (now - self._reversed_at[sid]) < self.REVERSAL_COOLDOWN_S

            stall_triggered = False
            if not in_cooldown:
                if abs(vel) < self.STALL_VEL_THRESHOLD:
                    if self._stall_since[sid] == 0.0:
                        self._stall_since[sid] = now
                    elif now - self._stall_since[sid] >= self.STALL_WINDOW_S:
                        stall_triggered = True
                else:
                    self._stall_since[sid] = 0.0
            else:
                self._stall_since[sid] = 0.0

            if not stall_triggered and abs(self._pos_cmd[sid]) >= self.MAX_POS_RAD:
                stall_triggered = True

            if stall_triggered:
                self._pos_cmd[sid] = state.servo_positions.get(sid, self._pos_cmd[sid])
                self._direction[sid] *= -1.0
                self._reversed_at[sid] = now
                self._stall_since[sid] = 0.0
                print(f"[Explore] Motor {sid} → reversing (vel={vel:.3f} rad/s)")

            self._pos_cmd[sid] += self._direction[sid] * self._speed_rad_s * dt
            self._pos_cmd[sid] = max(-self.MAX_POS_RAD, min(self.MAX_POS_RAD, self._pos_cmd[sid]))

            cmds.append(ServoCommand(servo_id=sid, position=self._pos_cmd[sid]))

        return cmds


class CurlControlScheme(ControlScheme):
    """Ramp all joints to the same angle on one side, coiling the body into a tight ball, then hold."""

    name = "curl"

    def __init__(self, target_deg: float = 360.0) -> None:
        self.target_deg = target_deg

    def on_start(self, controller: "Controller") -> None:
        print(f"[Curl] Coiling all joints to {self.target_deg:.0f}°. Ctrl-C to stop.")

    def update(self, state: RobotState) -> list[ServoCommand]:
        return [
            ServoCommand.from_angle(servo_id=sid, angle_deg=self.target_deg)
            for sid in sorted(state.active_servo_ids)
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
    CurlControlScheme,
    Spin7ControlScheme,
    StrokeReactControlScheme,
    WanderControlScheme,
    ExploreControlScheme,
]

PATTERN_NAMES: list[str] = [cls.name for cls in ALL_PATTERNS]
