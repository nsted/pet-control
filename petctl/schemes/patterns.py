"""
Motion patterns for PET — standalone control schemes.

Each produces a distinct body movement and can be selected via:
    petctl run --control <name>

Patterns:
    snuggle          — two full wave crests visible simultaneously (shorter wavelength)
    pulse            — all joints in phase, whole-body flex and release
    breathe          — slow, tiny-amplitude in-phase breathing
    sway             — travelling wave with amplitude tapering head→tail
    cascade          — travelling wave with amplitude growing head→tail
    slalom           — alternating odd/even sign → S-shape rocking
    twitch           — smoothed per-joint Brownian noise (jittery, organic)
    freeze           — hold all joints at home (0°)
    coil             — quadratic spatial phase → tighter curl toward the tail
    curl             — ramp with slalom signs to 70°, looping head-to-tail, then hold
    balanced-torque  — seek pose where all motors share equal load near a 1A target
    purr-ripple      — kd-vibration wave propagating head→tail; speed scales ripple rate
"""

from __future__ import annotations

import logging
import math
import random
import time
from typing import TYPE_CHECKING

from petctl.config import MOTOR_LIMITS
from petctl.protocols import Motion
from petctl.types import RobotState, ServoCommand

if TYPE_CHECKING:
    from petctl.controller import Controller

logger = logging.getLogger(__name__)


class SnuggleMotion(Motion):
    """Two full wave crests across the body simultaneously."""

    name = "snuggle"

    def is_active(self) -> bool:
        return True

    def __init__(self, amplitude_deg: float = 40.0, hz: float = 0.4) -> None:
        self.amplitude_deg = amplitude_deg
        self.hz = hz
        self._start: float = 0.0

    def on_start(self, controller: "Controller") -> None:
        self._start = time.monotonic()
        logger.info("[BEHAVIOR] Snuggle")
        logger.debug("[BEHAVIOR] Snuggle: ±%.0f° at %.1f Hz, 2× spatial frequency.", self.amplitude_deg, self.hz)

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


class PulseMotion(Motion):
    """All joints in phase — whole-body contraction and release."""

    name = "pulse"

    def is_active(self) -> bool:
        return True

    def __init__(self, amplitude_deg: float = 50.0, hz: float = 0.25) -> None:
        self.amplitude_deg = amplitude_deg
        self.hz = hz
        self._start: float = 0.0

    def on_start(self, controller: "Controller") -> None:
        self._start = time.monotonic()
        logger.info("[BEHAVIOR] Pulse")
        logger.debug("[BEHAVIOR] Pulse: ±%.0f° at %.1f Hz, all joints in phase.", self.amplitude_deg, self.hz)

    def update(self, state: RobotState) -> list[ServoCommand]:
        t = time.monotonic() - self._start
        angle = self.amplitude_deg * math.sin(2 * math.pi * self.hz * t)
        return [
            ServoCommand.from_angle(servo_id=sid, angle_deg=angle)
            for sid in sorted(state.active_servo_ids)
        ]


class BreatheMotion(Motion):
    """Very gentle, slow in-phase motion — aliveness at rest."""

    name = "breathe"

    def is_active(self) -> bool:
        return True

    def __init__(self, amplitude_deg: float = 12.0, hz: float = 0.08) -> None:
        self.amplitude_deg = amplitude_deg
        self.hz = hz
        self._start: float = 0.0

    def on_start(self, controller: "Controller") -> None:
        self._start = time.monotonic()
        logger.info("[BEHAVIOR] Breathe")
        logger.debug("[BEHAVIOR] Breathe: ±%.0f° at %.2f Hz (~%.0fs period).", self.amplitude_deg, self.hz, 1.0 / self.hz)

    def update(self, state: RobotState) -> list[ServoCommand]:
        t = time.monotonic() - self._start
        angle = self.amplitude_deg * math.sin(2 * math.pi * self.hz * t)
        return [
            ServoCommand.from_angle(servo_id=sid, angle_deg=angle)
            for sid in sorted(state.active_servo_ids)
        ]


class SwayMotion(Motion):
    """Travelling wave with amplitude tapering head→tail (head leads, tail damps)."""

    name = "sway"

    def is_active(self) -> bool:
        return True

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
        logger.info("[BEHAVIOR] Sway")
        logger.debug(
            "[BEHAVIOR] Sway: head ±%.0f° → tail ±%.0f° at %.1f Hz.",
            self.amplitude_deg, self.amplitude_deg * self.tail_fraction, self.hz,
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


class CascadeMotion(Motion):
    """Travelling wave with amplitude growing head→tail (crack-the-whip effect)."""

    name = "cascade"

    def is_active(self) -> bool:
        return True

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
        logger.info("[BEHAVIOR] Cascade")
        logger.debug(
            "[BEHAVIOR] Cascade: head ±%.0f° → tail ±%.0f° at %.1f Hz.",
            self.amplitude_deg * self.head_fraction, self.amplitude_deg, self.hz,
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


class SlalomMotion(Motion):
    """Odd/even joints get opposing phase → persistent S-shape that rocks side to side."""

    name = "slalom"

    def is_active(self) -> bool:
        return True

    def __init__(self, amplitude_deg: float = 45.0, hz: float = 0.2) -> None:
        self.amplitude_deg = amplitude_deg
        self.hz = hz
        self._start: float = 0.0

    def on_start(self, controller: "Controller") -> None:
        self._start = time.monotonic()
        logger.info("[BEHAVIOR] Slalom")
        logger.debug("[BEHAVIOR] Slalom: ±%.0f° S-shape rocking at %.1f Hz.", self.amplitude_deg, self.hz)

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


class TwitchMotion(Motion):
    """Each joint wanders independently via smoothed Brownian noise."""

    name = "twitch"

    def is_active(self) -> bool:
        return True

    def __init__(self, amplitude_deg: float = 30.0, smoothing: float = 0.06) -> None:
        self.amplitude_deg = amplitude_deg
        # EMA alpha: lower = smoother/slower drift, higher = jerkier
        self.smoothing = smoothing
        self._current: dict[int, float] = {}
        self._target: dict[int, float] = {}

    def on_start(self, controller: "Controller") -> None:
        self._current = {}
        self._target = {}
        logger.info("[BEHAVIOR] Twitch")
        logger.debug("[BEHAVIOR] Twitch: ±%.0f° random noise per joint.", self.amplitude_deg)

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


class FreezeMotion(Motion):
    """Command all joints to home (0°) and hold there."""

    name = "freeze"

    def on_start(self, controller: "Controller") -> None:
        logger.info("[BEHAVIOR] Freeze")
        logger.debug("[BEHAVIOR] Freeze: holding all joints at home (0°).")

    def update(self, state: RobotState) -> list[ServoCommand]:
        return [
            ServoCommand.from_angle(servo_id=sid, angle_deg=0.0)
            for sid in sorted(state.active_servo_ids)
        ]


class IdleMotion(Motion):
    """Stay in MIT mode (motors on, green light) but freespin — kp=kd=0, no torque."""

    name = "idle"

    def on_start(self, controller: "Controller") -> None:
        logger.info("[BEHAVIOR] Idle")
        logger.debug("[BEHAVIOR] Idle: motors on, freespinning.")

    def update(self, state: RobotState) -> list[ServoCommand]:
        return [
            ServoCommand(servo_id=sid, position=0.0, kp=0.0, kd=0.0, torque_ff=0.0)
            for sid in sorted(state.active_servo_ids)
        ]


class CoilMotion(Motion):
    """Quadratic spatial phase distribution — tighter curl accumulates toward the tail."""

    name = "coil"

    def is_active(self) -> bool:
        return True

    def __init__(self, amplitude_deg: float = 55.0, hz: float = 0.15) -> None:
        self.amplitude_deg = amplitude_deg
        self.hz = hz
        self._start: float = 0.0

    def on_start(self, controller: "Controller") -> None:
        self._start = time.monotonic()
        logger.info("[BEHAVIOR] Coil")
        logger.debug("[BEHAVIOR] Coil: ±%.0f° coiling motion at %.1f Hz.", self.amplitude_deg, self.hz)

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


class Spin7Motion(Motion):
    """Continuously rotate joint 7 (tail); all other joints hold at home.

    Every full revolution the motor's software zero is reset to its current
    physical position, keeping commanded angles in the safe MIT encoding range
    indefinitely.
    """

    name = "spin7"

    def is_active(self) -> bool:
        return True

    def __init__(self, speed_deg_per_s: float = 30.0) -> None:
        self.speed_deg_per_s = speed_deg_per_s
        self._pos_deg: float = 0.0
        self._last_t: float = 0.0
        self._backend = None

    def on_start(self, controller: "Controller") -> None:
        self._pos_deg = 0.0
        self._last_t = time.monotonic()
        self._backend = controller.backend
        logger.info("[BEHAVIOR] Spin7")
        logger.debug("[BEHAVIOR] Spin7: joint 7 spinning at %.0f°/s.", self.speed_deg_per_s)

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


class StrokeReactMotion(Motion):
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
    POS_LIMIT_DEG: float = math.degrees(MOTOR_LIMITS.pos_max)

    def __init__(self) -> None:
        self._angle_deg: dict[int, float] = {}
        self._rand_dir: dict[int, float] = {}
        self._was_touching: dict[int, bool] = {}

    def on_start(self, controller: "Controller") -> None:
        self._angle_deg = {}
        self._rand_dir = {}
        self._was_touching = {}
        logger.info("[BEHAVIOR] StrokeReact")
        logger.debug("[BEHAVIOR] StrokeReact: left → spin right, right → spin left; hold on release.")

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


class _WanderBase(Motion):
    """Position+torque stall detection and direction reversal, shared by Wander and Drift.

    Stall requires two conditions within STALL_WINDOW_S:
      1. Position has not advanced STALL_THRESHOLD_RAD (motor is stuck).
      2. Peak torque during the window has reached STALL_TORQUE_NM (actively
         pushing — not just drifting or coasting to a slow stop).

    On reversal, _pos_cmd syncs to actual position and the controller slew
    filter is reset via take_slew_resets(), so the motor begins moving in the
    new direction on the very next tick.
    """

    def is_active(self) -> bool:
        return True

    STALL_THRESHOLD_RAD: float = 0.3   # ~17° minimum travel per window
    STALL_WINDOW_S: float = 0.8
    STALL_TORQUE_NM: float = 0.9       # peak torque that must be seen during the window
    REVERSAL_COOLDOWN_S: float = 0.5
    MAX_POS_RAD: float = MOTOR_LIMITS.pos_max

    def __init__(self) -> None:
        self._pos_cmd: dict[int, float] = {}
        self._direction: dict[int, float] = {}
        self._stall_since: dict[int, float] = {}
        self._stall_pos_snap: dict[int, float] = {}
        self._stall_peak_torque: dict[int, float] = {}
        self._reversed_at: dict[int, float] = {}
        self._pending_slew_resets: dict[int, float] = {}

    def _init_stall_state(self) -> None:
        self._pos_cmd = {}
        self._direction = {}
        self._stall_since = {}
        self._stall_pos_snap = {}
        self._stall_peak_torque = {}
        self._reversed_at = {}
        self._pending_slew_resets = {}

    def take_slew_resets(self) -> dict[int, float]:
        """Called by the controller before _apply_slew_to_commands each tick."""
        resets = self._pending_slew_resets
        self._pending_slew_resets = {}
        return resets

    def _init_servo(self, sid: int, state: RobotState, now: float) -> None:
        self._pos_cmd[sid] = state.servo_positions.get(sid, 0.0)
        self._direction[sid] = random.choice([-1.0, 1.0])
        self._stall_since[sid] = 0.0
        self._reversed_at[sid] = now

    def _check_stall(self, sid: int, state: RobotState, now: float) -> bool:
        """Return True if stall is triggered for this servo this tick."""
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
            # Fallback: always reverse at the MIT encoding ceiling.
            if not stall_triggered and abs(self._pos_cmd[sid]) >= self.MAX_POS_RAD:
                stall_triggered = True
        else:
            self._stall_since[sid] = 0.0
            self._stall_pos_snap.pop(sid, None)
            self._stall_peak_torque.pop(sid, None)
        return stall_triggered

    def _do_reversal(self, sid: int, state: RobotState, now: float) -> tuple[float, float]:
        """Sync pos_cmd to actual, flip direction, reset slew. Returns (clamped_rad, peak_torque_nm)."""
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
        return clamped, peak_t


class ExploreMotion(_WanderBase):
    """Each joint turns at a fixed speed, reversing on position+torque stall or ceiling."""

    name = "explore"

    SPEED_DEG_PER_S: float = 45.0

    def __init__(self, speed_deg_per_s: float = SPEED_DEG_PER_S) -> None:
        super().__init__()
        self._speed_rad_s = math.radians(speed_deg_per_s)

    def on_start(self, controller: "Controller") -> None:
        self._init_stall_state()
        logger.info("[BEHAVIOR] Explore")
        logger.debug("[BEHAVIOR] Explore: each joint turns at %.0f°/s, reversing on stall.", math.degrees(self._speed_rad_s))

    def update(self, state: RobotState) -> list[ServoCommand]:
        now = time.monotonic()
        dt = max(state.dt, 1e-4)
        cmds: list[ServoCommand] = []
        for sid in sorted(state.active_servo_ids):
            if sid not in self._pos_cmd:
                self._init_servo(sid, state, now)
            if self._check_stall(sid, state, now):
                clamped, peak_t = self._do_reversal(sid, state, now)
                logger.debug("[Explore] Motor %d → reversing at %.1f° (peak torque %.2f Nm)", sid, math.degrees(clamped), peak_t)
            self._pos_cmd[sid] += self._direction[sid] * self._speed_rad_s * dt
            self._pos_cmd[sid] = max(-self.MAX_POS_RAD, min(self.MAX_POS_RAD, self._pos_cmd[sid]))
            cmds.append(ServoCommand(servo_id=sid, position=self._pos_cmd[sid]))
        return cmds


class SeekTouchMotion(_WanderBase):
    """Explore until each module detects contact, then hold in place.

    Each servo moves independently at a fixed speed, reversing on stall or
    ceiling — exactly like ExploreMotion.  As soon as touch_total on a module
    exceeds TOUCH_THRESHOLD the servo freezes (zero-torque hold) until contact
    is released, then resumes seeking.
    """

    name = "seek-touch"

    SPEED_DEG_PER_S: float = 45.0
    TOUCH_THRESHOLD: float = 0.06

    def __init__(self) -> None:
        super().__init__()
        self._speed_rad_s = math.radians(self.SPEED_DEG_PER_S)
        self._holding: set[int] = set()

    def on_start(self, controller: "Controller") -> None:
        self._init_stall_state()
        self._holding = set()
        logger.info("[BEHAVIOR] SeekTouch")

    def update(self, state: RobotState) -> list[ServoCommand]:
        now = time.monotonic()
        dt = max(state.dt, 1e-4)
        cmds: list[ServoCommand] = []
        for sid in sorted(state.active_servo_ids):
            if sid not in self._pos_cmd:
                self._init_servo(sid, state, now)
            s = state.sensors.get(sid)
            if s is not None and s.touch_total >= self.TOUCH_THRESHOLD:
                self._holding.add(sid)
            else:
                self._holding.discard(sid)
            if sid in self._holding:
                pos = state.servo_positions.get(sid, 0.0)
                cmds.append(ServoCommand(servo_id=sid, position=pos, kp=0.0, kd=0.0, torque_ff=0.0))
            else:
                if self._check_stall(sid, state, now):
                    clamped, peak_t = self._do_reversal(sid, state, now)
                    logger.debug("[SeekTouch] s%d reversed at %.1f°", sid, math.degrees(clamped))
                self._pos_cmd[sid] += self._direction[sid] * self._speed_rad_s * dt
                self._pos_cmd[sid] = max(-self.MAX_POS_RAD, min(self.MAX_POS_RAD, self._pos_cmd[sid]))
                cmds.append(ServoCommand(servo_id=sid, position=self._pos_cmd[sid]))
        return cmds


class AvoidTouchMotion(_WanderBase):
    """Hold idle until touched, then move away until contact ends.

    Each servo is still when its module is not touched.  When touch_total
    exceeds TOUCH_THRESHOLD the servo starts moving: direction is chosen
    based on which face is more active (away from that face), or toward home
    if the face balance is unclear.  Stall detection and ceiling reversal
    remain active while fleeing.
    """

    name = "avoid-touch"

    SPEED_DEG_PER_S: float = 45.0
    TOUCH_THRESHOLD: float = 0.06
    FACE_BIAS: float = 0.05  # min left/right imbalance to pick a face direction

    def __init__(self) -> None:
        super().__init__()
        self._speed_rad_s = math.radians(self.SPEED_DEG_PER_S)
        self._fleeing: set[int] = set()

    def on_start(self, controller: "Controller") -> None:
        self._init_stall_state()
        self._fleeing = set()
        logger.info("[BEHAVIOR] AvoidTouch")

    def _flee_dir(self, sid: int, state: RobotState) -> float:
        """Pick a direction that moves away from the more-active face."""
        s = state.sensors.get(sid)
        if s is not None:
            diff = s.touch_right - s.touch_left
            if diff > self.FACE_BIAS:
                # right face more active → flee left → use same sign as curl-left for this servo
                return -(1.0 if sid % 2 == 1 else -1.0)
            if diff < -self.FACE_BIAS:
                # left face more active → flee right
                return (1.0 if sid % 2 == 1 else -1.0)
        # No clear face bias — move toward home (reduce |pos|)
        pos = state.servo_positions.get(sid, 0.0)
        if abs(pos) > 0.1:
            return -math.copysign(1.0, pos)
        return self._direction.get(sid, 1.0)

    def update(self, state: RobotState) -> list[ServoCommand]:
        now = time.monotonic()
        dt = max(state.dt, 1e-4)
        cmds: list[ServoCommand] = []
        for sid in sorted(state.active_servo_ids):
            if sid not in self._pos_cmd:
                self._init_servo(sid, state, now)
            s = state.sensors.get(sid)
            touched = s is not None and s.touch_total >= self.TOUCH_THRESHOLD
            was_fleeing = sid in self._fleeing
            if touched:
                if not was_fleeing:
                    # First contact: set flee direction before entering fleeing state
                    self._direction[sid] = self._flee_dir(sid, state)
                    self._pos_cmd[sid] = state.servo_positions.get(sid, self._pos_cmd[sid])
                self._fleeing.add(sid)
            else:
                self._fleeing.discard(sid)
            if sid in self._fleeing:
                if self._check_stall(sid, state, now):
                    clamped, peak_t = self._do_reversal(sid, state, now)
                    logger.debug("[AvoidTouch] s%d reversed at %.1f°", sid, math.degrees(clamped))
                self._pos_cmd[sid] += self._direction[sid] * self._speed_rad_s * dt
                self._pos_cmd[sid] = max(-self.MAX_POS_RAD, min(self.MAX_POS_RAD, self._pos_cmd[sid]))
                cmds.append(ServoCommand(servo_id=sid, position=self._pos_cmd[sid]))
            else:
                pos = state.servo_positions.get(sid, 0.0)
                cmds.append(ServoCommand(servo_id=sid, position=pos, kp=0.0, kd=0.0, torque_ff=0.0))
        return cmds


class DriftMotion(_WanderBase):
    """Wander variant where all joints share one speed that varies over time.

    Speed is driven by a sine oscillator between MIN_SPEED_DEG_PER_S and
    MAX_SPEED_DEG_PER_S with period SPEED_PERIOD_S.  Every motor receives the
    same speed value each tick; stall detection and reversal are identical to
    ExploreMotion.
    """

    name = "drift"

    MIN_SPEED_DEG_PER_S: float = 15.0
    MAX_SPEED_DEG_PER_S: float = 90.0  # intentionally 2× Explore — explores a wider speed range
    SPEED_PERIOD_S: float = 12.0

    def __init__(self) -> None:
        super().__init__()
        self._start_time: float = 0.0

    def on_start(self, controller: "Controller") -> None:
        self._start_time = time.monotonic()
        self._init_stall_state()
        logger.info("[BEHAVIOR] Drift")
        logger.debug(
            "[BEHAVIOR] Drift: shared speed %.0f–%.0f°/s over %.0fs, reversing on stall.",
            self.MIN_SPEED_DEG_PER_S, self.MAX_SPEED_DEG_PER_S, self.SPEED_PERIOD_S,
        )

    def _current_speed_rad_s(self, now: float) -> float:
        t = now - self._start_time
        phase = (t % self.SPEED_PERIOD_S) / self.SPEED_PERIOD_S
        # sine oscillates 0→1→0 over one period
        alpha = 0.5 * (1.0 - math.cos(2.0 * math.pi * phase))
        lo = math.radians(self.MIN_SPEED_DEG_PER_S)
        hi = math.radians(self.MAX_SPEED_DEG_PER_S)
        return lo + alpha * (hi - lo)

    def update(self, state: RobotState) -> list[ServoCommand]:
        now = time.monotonic()
        dt = max(state.dt, 1e-4)
        speed_rad_s = self._current_speed_rad_s(now)
        cmds: list[ServoCommand] = []
        for sid in sorted(state.active_servo_ids):
            if sid not in self._pos_cmd:
                self._init_servo(sid, state, now)
            if self._check_stall(sid, state, now):
                clamped, peak_t = self._do_reversal(sid, state, now)
                logger.debug("[Drift] Motor %d → reversing at %.1f° (peak torque %.2f Nm, speed %.0f°/s)", sid, math.degrees(clamped), peak_t, math.degrees(speed_rad_s))
            self._pos_cmd[sid] += self._direction[sid] * speed_rad_s * dt
            self._pos_cmd[sid] = max(-self.MAX_POS_RAD, min(self.MAX_POS_RAD, self._pos_cmd[sid]))
            cmds.append(ServoCommand(servo_id=sid, position=self._pos_cmd[sid]))
        return cmds


class StruggleMotion(Motion):
    """Copy of ExploreMotion for parameter tuning.

    Identical behaviour to explore — tweak the class constants here
    without disturbing the reference.
    """

    name = "struggle"

    def is_active(self) -> bool:
        return True

    SPEED_DEG_PER_S: float = 45.0
    STALL_VEL_THRESHOLD: float = 0.15
    STALL_WINDOW_S: float = 0.6
    REVERSAL_COOLDOWN_S: float = 0.4
    MAX_POS_RAD: float = MOTOR_LIMITS.pos_max

    def __init__(self, speed_deg_per_s: float = SPEED_DEG_PER_S) -> None:
        self._speed_rad_s = math.radians(speed_deg_per_s)
        self._pos_cmd: dict[int, float] = {}
        self._direction: dict[int, float] = {}
        self._stall_since: dict[int, float] = {}
        self._reversed_at: dict[int, float] = {}
        self._pending_slew_resets: dict[int, float] = {}

    def on_start(self, controller: "Controller") -> None:
        self._pos_cmd = {}
        self._direction = {}
        self._stall_since = {}
        self._reversed_at = {}
        self._pending_slew_resets = {}
        logger.info("[BEHAVIOR] Struggle")
        logger.debug("[BEHAVIOR] Struggle: each joint turns at %.0f°/s, reversing on stall.", math.degrees(self._speed_rad_s))

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
                actual = state.servo_positions.get(sid, self._pos_cmd[sid])
                clamped = max(-self.MAX_POS_RAD, min(self.MAX_POS_RAD, actual))
                self._pos_cmd[sid] = clamped
                self._pending_slew_resets[sid] = clamped
                self._direction[sid] *= -1.0
                self._reversed_at[sid] = now
                self._stall_since[sid] = 0.0
                logger.debug("[Struggle] Motor %d → reversing (vel=%.3f rad/s)", sid, vel)

            self._pos_cmd[sid] += self._direction[sid] * self._speed_rad_s * dt
            self._pos_cmd[sid] = max(-self.MAX_POS_RAD, min(self.MAX_POS_RAD, self._pos_cmd[sid]))

            cmds.append(ServoCommand(servo_id=sid, position=self._pos_cmd[sid]))

        return cmds


class CurlMotion(Motion):
    """Ramp joints to loop head-to-tail using slalom sign pattern, then hold.

    Alternating signs (odd-enumeration-index +, even −) match the slalom curl
    direction, producing a consistent right-side curl that brings the head joint
    origin within ~0.4 cm of the tail at TARGET_DEG per FK geometry.
    """

    name = "curl"

    def is_active(self) -> bool:
        return True

    # FK geometry: head/tail MODULE BODIES just touch at ~70° (joint-origin distance ≈ 7 cm = 1 module width).
    # 88° brings joint origins to 0.4 cm but the snake has spiralled 1.7 turns — deep body overlap.
    TARGET_DEG: float = 70.0
    RAMP_S: float = 8.0

    def __init__(
        self,
        target_deg: float = TARGET_DEG,
        ramp_s: float = RAMP_S,
    ) -> None:
        self.target_deg = target_deg
        self.ramp_s = ramp_s
        self._start: float = 0.0

    def on_start(self, controller: "Controller") -> None:
        self._start = time.monotonic()
        logger.info("[BEHAVIOR] Curl")
        logger.debug("[BEHAVIOR] Curl: ±%.0f° over %.0fs, then hold.", self.target_deg, self.ramp_s)

    def update(self, state: RobotState) -> list[ServoCommand]:
        t = time.monotonic() - self._start
        alpha = min(1.0, t / self.ramp_s)
        angle = alpha * self.target_deg
        ids = sorted(state.active_servo_ids)
        return [
            ServoCommand.from_angle(
                servo_id=sid,
                angle_deg=(1.0 if i % 2 == 0 else -1.0) * angle,
            )
            for i, sid in enumerate(ids)
        ]


def _sense_face_direction(state: RobotState, pad_threshold: float, direction_threshold: float) -> float:
    """Return +1.0 (right), -1.0 (left), or 0.0 (ambiguous) from face pad balance."""
    total_right = total_left = 0.0
    for sens in state.sensors.values():
        for v in sens.touch_right_pads:
            if v >= pad_threshold:
                total_right += v
        for v in sens.touch_left_pads:
            if v >= pad_threshold:
                total_left += v
    total = total_right + total_left
    if total < 1e-6:
        return 0.0
    balance = (total_right - total_left) / total
    if balance > direction_threshold:
        return 1.0
    if balance < -direction_threshold:
        return -1.0
    return 0.0


class StrokeCurlMotion(Motion):
    """
    Each module curls toward the touched face while the hand is on it.

    Touch on the right face → curl right; left face → curl left.
    Each module tracks touch independently: it curls while touched, holds
    its position for HOLD_S after the hand moves away, then returns home.
    """

    name = "stroke-curl"

    TARGET_DEG: float = 45.0
    HOLD_S: float = 2.0             # seconds to hold after hand leaves a module
    DECAY_DEG_PER_S: float = 15.0   # return-to-home speed after hold expires
    DIRECTION_THRESHOLD: float = 0.15  # min right-left imbalance to commit to a side
    MODULE_TOUCH_THRESHOLD: float = 0.06  # touch_total to count as "hand present"
    PROGRESS_THRESHOLD_RAD: float = 0.05  # must move this much toward home per window
    PROGRESS_WINDOW_S: float = 0.5        # idle immediately if no progress within this window
    DIRECTION_SIGN: float = 1.0           # +1 = curl toward touch, -1 = curl away

    def __init__(self) -> None:
        from petctl.perception.stroke import PAD_THRESHOLD
        self._PAD_THRESHOLD = PAD_THRESHOLD
        self._curl_target: dict[int, float] = {}
        self._release_time: dict[int, float] = {}  # sid → monotonic time of last release
        self._curl_dir: float = 0.0
        self._progress_snap: dict[int, tuple[float, float]] = {}  # sid → (abs_pos, time)

    def on_start(self, controller: "Controller") -> None:
        self._curl_target = {}
        self._release_time = {}
        self._curl_dir = 0.0
        self._progress_snap = {}
        logger.info("[BEHAVIOR] StrokeCurl")
        logger.debug("[BEHAVIOR] StrokeCurl: right → curl right, left → curl left; holds on release.")

    def update(self, state: RobotState) -> list[ServoCommand]:
        now = state.timestamp
        dt = max(state.dt, 1e-4)

        new_dir = self._sense_dir(state)
        if new_dir != 0.0:
            self._curl_dir = new_dir
        elif self._curl_dir == 0.0:
            self._curl_dir = 1.0

        active_set = state.active_servo_ids
        directly_touched: set[int] = {
            sid for sid in active_set
            if (s := state.sensors.get(sid)) is not None and s.touch_total >= self.MODULE_TOUCH_THRESHOLD
        }
        touching: set[int] = directly_touched | {
            nb for sid in directly_touched for nb in (sid - 1, sid + 1)
            if nb in active_set
        }

        cmds = []
        # Invariant: module N has servo N (modules 1–7). Module 0 is head, no servo.
        for sid in sorted(active_set):
            touched = sid in touching

            if touched:
                sign = (1.0 if sid % 2 == 1 else -1.0) * self._curl_dir * self.DIRECTION_SIGN
                self._curl_target[sid] = sign * self.TARGET_DEG
                self._release_time.pop(sid, None)
                self._progress_snap.pop(sid, None)
            else:
                cur = self._curl_target.get(sid, 0.0)
                if cur != 0.0:
                    if sid not in self._release_time:
                        self._release_time[sid] = now
                    elif now - self._release_time[sid] > self.HOLD_S:
                        # Decay phase: idle immediately if motor stops making progress toward home.
                        abs_pos = abs(state.servo_positions.get(sid, 0.0))
                        if sid not in self._progress_snap:
                            self._progress_snap[sid] = (abs_pos, now)
                        else:
                            snap_pos, snap_t = self._progress_snap[sid]
                            if snap_pos - abs_pos >= self.PROGRESS_THRESHOLD_RAD:
                                self._progress_snap[sid] = (abs_pos, now)
                            elif now - snap_t >= self.PROGRESS_WINDOW_S:
                                self._curl_target[sid] = 0.0
                                self._release_time.pop(sid, None)
                                self._progress_snap.pop(sid, None)
                                cur = 0.0

                        if cur != 0.0:
                            decay = self.DECAY_DEG_PER_S * dt
                            if abs(cur) <= decay:
                                self._curl_target[sid] = 0.0
                                self._release_time.pop(sid, None)
                                self._progress_snap.pop(sid, None)
                            else:
                                self._curl_target[sid] = cur - math.copysign(decay, cur)

            target = self._curl_target.get(sid, 0.0)
            if target == 0.0:
                cmds.append(ServoCommand(servo_id=sid, position=0.0, kp=0.0, kd=0.0, torque_ff=0.0))
            else:
                cmds.append(ServoCommand.from_angle(sid, target))

        return cmds

    def _sense_dir(self, state: RobotState) -> float:
        return _sense_face_direction(state, self._PAD_THRESHOLD, self.DIRECTION_THRESHOLD)


class CurlTowardsMotion(StrokeCurlMotion):
    """Curl toward the touched face. Identical to stroke-curl; provided as a named alias."""

    name = "curl-towards"

    def on_start(self, controller: "Controller") -> None:
        self._curl_target = {}
        self._release_time = {}
        self._curl_dir = 0.0
        self._progress_snap = {}
        logger.info("[BEHAVIOR] CurlTowards")


class CurlAwayMotion(StrokeCurlMotion):
    """Curl away from the touched face — the mirror of curl-towards."""

    name = "curl-away"
    DIRECTION_SIGN: float = -1.0

    def on_start(self, controller: "Controller") -> None:
        self._curl_target = {}
        self._release_time = {}
        self._curl_dir = 0.0
        self._progress_snap = {}
        logger.info("[BEHAVIOR] CurlAway")


class StrokeSnuggleMotion(Motion):
    """
    Like stroke-curl, but after 10s of continuous stroking transitions to
    snuggle for 8s, then commands all joints home.

    States:
      curl    — per-module touch tracking (same as stroke-curl)
      snuggle — full-body snuggle wave for RIPPLE_DURATION_S
      home   — all joints commanded to 0° and held there
    """

    name = "stroke-snuggle"

    # Curl phase
    TARGET_DEG: float = 45.0
    HOLD_S: float = 2.0
    DECAY_DEG_PER_S: float = 15.0
    DIRECTION_THRESHOLD: float = 0.15
    MODULE_TOUCH_THRESHOLD: float = 0.06

    # Curl → snuggle trigger
    STROKE_TRIGGER_S: float = 15.0
    TOUCH_GAP_GRACE_S: float = 1.0   # gap before resetting the stroke timer

    # Ripple phase
    RIPPLE_AMPLITUDE_DEG: float = 40.0
    RIPPLE_HZ: float = 0.4
    RIPPLE_DURATION_S: float = 15.0

    _CURL = "curl"
    _SNUGGLE = "snuggle"
    _HOME = "home"

    def is_active(self) -> bool:
        return self._state == self._SNUGGLE

    def __init__(self) -> None:
        from petctl.perception.stroke import PAD_THRESHOLD
        self._PAD_THRESHOLD = PAD_THRESHOLD
        self._curl_target: dict[int, float] = {}
        self._release_time: dict[int, float] = {}
        self._curl_dir: float = 0.0
        self._state: str = self._CURL
        self._stroke_start: float | None = None
        self._last_touch: float | None = None
        self._snuggle_start: float = 0.0

    def on_start(self, controller: "Controller") -> None:
        self._curl_target = {}
        self._release_time = {}
        self._curl_dir = 0.0
        self._state = self._CURL
        self._stroke_start = None
        self._last_touch = None
        self._snuggle_start = 0.0
        logger.info("[BEHAVIOR] StrokeSnuggle")
        logger.debug(
            "[BEHAVIOR] StrokeSnuggle: stroke for %.0fs → snuggle %.0fs → home.",
            self.STROKE_TRIGGER_S, self.RIPPLE_DURATION_S,
        )

    def update(self, state: RobotState) -> list[ServoCommand]:
        if self._state == self._SNUGGLE:
            return self._do_snuggle(state)
        if self._state == self._HOME:
            return self._do_home(state)
        return self._do_curl(state)

    # ------------------------------------------------------------------

    def _do_curl(self, state: RobotState) -> list[ServoCommand]:
        now = state.timestamp
        dt = max(state.dt, 1e-4)

        new_dir = self._sense_dir(state)
        if new_dir != 0.0:
            self._curl_dir = new_dir
        elif self._curl_dir == 0.0:
            self._curl_dir = 1.0

        any_touched = False
        cmds: list[ServoCommand] = []

        # Invariant: module N has servo N (modules 1–7). Module 0 is head, no servo.
        for sid in sorted(state.active_servo_ids):
            sens = state.sensors.get(sid)
            touched = sens is not None and sens.touch_total >= self.MODULE_TOUCH_THRESHOLD
            if touched:
                any_touched = True
                sign = (1.0 if sid % 2 == 1 else -1.0) * self._curl_dir
                self._curl_target[sid] = sign * self.TARGET_DEG
                self._release_time.pop(sid, None)
            else:
                cur = self._curl_target.get(sid, 0.0)
                if cur != 0.0:
                    if sid not in self._release_time:
                        self._release_time[sid] = now
                    elif now - self._release_time[sid] > self.HOLD_S:
                        decay = self.DECAY_DEG_PER_S * dt
                        if abs(cur) <= decay:
                            self._curl_target[sid] = 0.0
                            self._release_time.pop(sid, None)
                        else:
                            self._curl_target[sid] = cur - math.copysign(decay, cur)
            cmds.append(ServoCommand.from_angle(sid, self._curl_target.get(sid, 0.0)))

        if any_touched:
            if self._stroke_start is None:
                self._stroke_start = now
            self._last_touch = now
        elif self._last_touch is not None and (now - self._last_touch) > self.TOUCH_GAP_GRACE_S:
            self._stroke_start = None
            self._last_touch = None

        if self._stroke_start is not None and (now - self._stroke_start) >= self.STROKE_TRIGGER_S:
            logger.info("[StrokeSnuggle] %.0fs stroke → snuggle", self.STROKE_TRIGGER_S)
            self._state = self._SNUGGLE
            self._snuggle_start = now

        return cmds

    def _do_snuggle(self, state: RobotState) -> list[ServoCommand]:
        t = state.timestamp - self._snuggle_start
        if t >= self.RIPPLE_DURATION_S:
            logger.info("[StrokeSnuggle] snuggle done → home")
            self._state = self._HOME
            return self._do_home(state)
        ids = sorted(state.active_servo_ids)
        n = len(ids)
        if n == 0:
            return []
        return [
            ServoCommand.from_angle(
                servo_id=sid,
                angle_deg=self.RIPPLE_AMPLITUDE_DEG * math.sin(
                    2 * math.pi * self.RIPPLE_HZ * t + (i / n) * 4 * math.pi
                ),
            )
            for i, sid in enumerate(ids)
        ]

    def _do_home(self, state: RobotState) -> list[ServoCommand]:
        return [
            ServoCommand.from_angle(servo_id=sid, angle_deg=0.0)
            for sid in sorted(state.active_servo_ids)
        ]

    def _sense_dir(self, state: RobotState) -> float:
        return _sense_face_direction(state, self._PAD_THRESHOLD, self.DIRECTION_THRESHOLD)




class YieldStiffMotion(Motion):
    """Yield by drifting the commanded position toward displacement while torque is high.

    Each tick: if |motor_torque| > TORQUE_BASELINE_NM on a servo, the commanded
    position for that servo drifts toward the actual (displaced) position at
    YIELD_RATE_RAD_S.  Drifting stops when torque returns to baseline; the servo
    holds wherever it ended up — no automatic return to home.

    No touch detection required: compliance is triggered purely by motor load.
    """

    name = "yield-stiff"

    YIELD_RATE_RAD_S: float = 3.0    # how fast the setpoint chases actual (rad/s)
    TORQUE_BASELINE_NM: float = 0.15  # stop drifting when torque drops below this

    def __init__(self) -> None:
        self._commanded: dict[int, float] = {}

    def on_start(self, controller: "Controller") -> None:  # noqa: ARG002
        logger.info("[BEHAVIOR] YieldStiff")
        logger.debug("[BEHAVIOR] YieldStiff: push to yield; setpoint holds at new position on release.")

    def update(self, state: RobotState) -> list[ServoCommand]:
        dt = max(state.dt, 1.0 / 60.0)
        commands = []

        for sid in sorted(state.active_servo_ids):
            cmd = self._commanded.get(sid, 0.0)
            actual = state.servo_positions.get(sid, 0.0)
            torque = abs(state.motor_torques.get(sid, 0.0))

            if torque > self.TORQUE_BASELINE_NM:
                error = actual - cmd
                if abs(error) > 1e-6:
                    step = min(abs(error), self.YIELD_RATE_RAD_S * dt)
                    cmd += step if error > 0 else -step

            self._commanded[sid] = cmd
            commands.append(ServoCommand(servo_id=sid, position=cmd))

        return commands


class PoseMotion(Motion):
    """Track actual position while a hand is present and moving; lock on release.

    A joint enters following only when BOTH conditions hold:
      1. Any hand contact is detected anywhere on the robot.
      2. The joint is displaced > FOLLOW_DISP_ON_RAD from its held position.

    Displacement (not velocity) is the entry gate: PD settling keeps the joint
    near the commanded setpoint, so velocity spikes during settling never trigger
    following.  Manual manipulation overcomes motor resistance and moves the joint
    clearly away from commanded.

    A joint locks when velocity drops below VELOCITY_OFF_THRESHOLD_RAD_S or the
    hand leaves.  Pass --log-touch to log transitions.  Ctrl-C to stop.
    """

    name = "pose"

    # Entry gate: joint must be this far from its held position to start following.
    FOLLOW_DISP_ON_RAD: float = 0.15   # ~8.6°
    # Exit gate: lock when velocity drops below this while following.
    VELOCITY_OFF_THRESHOLD_RAD_S: float = 0.20
    # Touch hysteresis: enter hand-present above ON, exit below OFF.
    # EMA smooths the raw touch signal before thresholding.
    TOUCH_ON_THRESHOLD: float = 0.12
    TOUCH_OFF_THRESHOLD: float = 0.07
    TOUCH_EMA_ALPHA: float = 0.05   # per controller tick (~70 ms time constant at 280 Hz)
    # After locking, ignore re-entry for this long while the motor settles.
    LOCK_SETTLE_S: float = 0.18

    def __init__(self) -> None:
        self._commanded: dict[int, float] = {}
        self._following: set[int] = set()
        self._locked_at: dict[int, float] = {}
        self._verbose = False
        self._touch_ema: float = 0.0
        self._hand_state: bool = False

    def on_start(self, controller: "Controller") -> None:
        self._commanded = {}
        self._following = set()
        self._locked_at = {}
        self._verbose = controller.log_touch
        self._touch_ema = 0.0
        self._hand_state = False
        logger.info("[BEHAVIOR] Pose")
        logger.debug("[BEHAVIOR] Pose: rotate a joint by hand — it holds that position on release.")

    def _update_hand(self, state: RobotState) -> bool:
        """Update EMA + hysteresis hand state. Returns True if hand is present."""
        raw = max((s.touch_total for s in state.sensors.values()), default=0.0)
        self._touch_ema = self.TOUCH_EMA_ALPHA * raw + (1.0 - self.TOUCH_EMA_ALPHA) * self._touch_ema
        if self._touch_ema >= self.TOUCH_ON_THRESHOLD:
            self._hand_state = True
        elif self._touch_ema < self.TOUCH_OFF_THRESHOLD:
            self._hand_state = False
        # else: stay as-is (hysteresis zone — no transition)
        return self._hand_state

    def update(self, state: RobotState) -> list[ServoCommand]:
        now = time.monotonic()
        commands = []
        following_now: set[int] = set()
        prev_hand = self._hand_state
        hand = self._update_hand(state)

        if self._verbose and hand != prev_hand:
            if hand:
                logger.info("[Pose] hand ON  (ema=%.3f)", self._touch_ema)
            else:
                logger.info("[Pose] hand OFF  (ema=%.3f)", self._touch_ema)

        for sid in sorted(state.active_servo_ids):
            actual = state.servo_positions.get(sid, 0.0)
            if sid not in self._commanded:
                self._commanded[sid] = actual

            velocity = abs(state.motor_velocities.get(sid, 0.0))
            if hand:
                was_following = sid in self._following
                settled = (now - self._locked_at.get(sid, 0.0)) >= self.LOCK_SETTLE_S
                if was_following:
                    if velocity >= self.VELOCITY_OFF_THRESHOLD_RAD_S:
                        following_now.add(sid)
                        self._commanded[sid] = actual
                elif settled and abs(actual - self._commanded[sid]) > self.FOLLOW_DISP_ON_RAD:
                    following_now.add(sid)
                    self._commanded[sid] = actual

            commands.append(ServoCommand(servo_id=sid, position=self._commanded[sid]))

        newly_locked = self._following - following_now
        for sid in newly_locked:
            self._locked_at[sid] = now

        if self._verbose:
            for sid in following_now - self._following:
                logger.info("[Pose] s%d following  vel=%.3f rad/s", sid, abs(state.motor_velocities.get(sid, 0.0)))
            for sid in newly_locked:
                logger.info("[Pose] s%d locked  pos=%.3f rad", sid, self._commanded.get(sid, 0.0))
        self._following = following_now

        return commands


class NeighborAssistDriftMotion(Motion):
    """DriftMotion kinematics with neighbor-assist stall recovery.

    When a motor stalls (position stuck + torque high), its neighboring motors
    briefly nudge toward neutral to relieve mechanical constraints.  If the
    stalled motor recovers it continues its drift trajectory; otherwise a
    normal direction reversal follows after MAX_ASSIST_ATTEMPTS tries.

    Stall detection mirrors _WanderBase (position + torque compound check).
    Neighbor assist is a new layer on top — the stalled motor keeps pushing
    while neighbors temporarily override their drift targets.
    """

    name = "neighbor-assist-drift"

    # Drift kinematics — mirrors DriftMotion
    MIN_SPEED_DEG_PER_S: float = 15.0
    MAX_SPEED_DEG_PER_S: float = 90.0
    SPEED_PERIOD_S: float = 12.0
    MAX_POS_RAD: float = MOTOR_LIMITS.pos_max

    # Stall detection — same thresholds as _WanderBase
    STALL_THRESHOLD_RAD: float = 0.3
    STALL_WINDOW_S: float = 0.8
    STALL_TORQUE_NM: float = 0.9
    REVERSAL_COOLDOWN_S: float = 0.5

    # Neighbor assist
    ASSIST_WIGGLE_RAD: float = math.radians(20.0)
    ASSIST_DURATION_S: float = 0.5
    RECOVERY_RAD: float = 0.08
    MAX_ASSIST_ATTEMPTS: int = 2
    ASSIST_COOLDOWN_S: float = 1.5

    def __init__(self) -> None:
        self._start_time: float = 0.0
        self._pos_cmd: dict[int, float] = {}
        self._direction: dict[int, float] = {}
        # stall detection
        self._stall_since: dict[int, float] = {}
        self._stall_pos_snap: dict[int, float] = {}
        self._stall_peak_torque: dict[int, float] = {}
        self._reversed_at: dict[int, float] = {}
        # neighbor assist
        self._assist_phase: dict[int, str] = {}       # "normal" | "assisting" | "cooldown"
        self._assist_start: dict[int, float] = {}
        self._assist_attempt: dict[int, int] = {}
        self._assist_snap_pos: dict[int, float] = {}
        self._assist_dir: dict[int, int] = {}         # +1 = wiggle toward 0°, -1 = away
        self._cooldown_until: dict[int, float] = {}
        self._pending_slew_resets: dict[int, float] = {}

    def is_active(self) -> bool:
        return True

    def take_slew_resets(self) -> dict[int, float]:
        resets = self._pending_slew_resets
        self._pending_slew_resets = {}
        return resets

    def on_start(self, controller: "Controller") -> None:
        self._start_time = time.monotonic()
        self._pos_cmd.clear()
        self._direction.clear()
        self._stall_since.clear()
        self._stall_pos_snap.clear()
        self._stall_peak_torque.clear()
        self._reversed_at.clear()
        self._assist_phase.clear()
        self._assist_start.clear()
        self._assist_attempt.clear()
        self._assist_snap_pos.clear()
        self._assist_dir.clear()
        self._cooldown_until.clear()
        self._pending_slew_resets.clear()
        logger.info("[BEHAVIOR] NeighborAssistDrift")

    def _current_speed_rad_s(self, now: float) -> float:
        t = now - self._start_time
        phase = (t % self.SPEED_PERIOD_S) / self.SPEED_PERIOD_S
        alpha = 0.5 * (1.0 - math.cos(2.0 * math.pi * phase))
        lo = math.radians(self.MIN_SPEED_DEG_PER_S)
        hi = math.radians(self.MAX_SPEED_DEG_PER_S)
        return lo + alpha * (hi - lo)

    def _neighbors(self, sid: int, active: set[int]) -> list[int]:
        return [n for n in (sid - 1, sid + 1) if n in active]

    def _init_servo(self, sid: int, state: RobotState, now: float) -> None:
        self._pos_cmd[sid] = state.servo_positions.get(sid, 0.0)
        self._direction[sid] = random.choice([-1.0, 1.0])
        self._stall_since[sid] = 0.0
        self._reversed_at[sid] = now
        self._assist_phase[sid] = "normal"
        self._cooldown_until[sid] = 0.0

    def _check_stall(self, sid: int, state: RobotState, now: float) -> bool:
        if (now - self._reversed_at[sid]) < self.REVERSAL_COOLDOWN_S:
            self._stall_since[sid] = 0.0
            self._stall_pos_snap.pop(sid, None)
            self._stall_peak_torque.pop(sid, None)
            return False
        actual = state.servo_positions.get(sid)
        torque = abs(state.motor_torques.get(sid, 0.0))
        if actual is None:
            return False
        if sid not in self._stall_pos_snap:
            self._stall_since[sid] = now
            self._stall_pos_snap[sid] = actual
            self._stall_peak_torque[sid] = torque
            return False
        if abs(actual - self._stall_pos_snap[sid]) >= self.STALL_THRESHOLD_RAD:
            self._stall_since[sid] = now
            self._stall_pos_snap[sid] = actual
            self._stall_peak_torque[sid] = torque
            return False
        self._stall_peak_torque[sid] = max(self._stall_peak_torque[sid], torque)
        if now - self._stall_since[sid] >= self.STALL_WINDOW_S:
            if self._stall_peak_torque[sid] >= self.STALL_TORQUE_NM:
                return True
            # Position stuck but no load — restart window
            self._stall_since[sid] = now
            self._stall_pos_snap[sid] = actual
            self._stall_peak_torque[sid] = torque
        return abs(self._pos_cmd[sid]) >= self.MAX_POS_RAD  # ceiling fallback

    def _do_reversal(self, sid: int, state: RobotState, now: float) -> None:
        actual = state.servo_positions.get(sid, self._pos_cmd[sid])
        clamped = max(-self.MAX_POS_RAD, min(self.MAX_POS_RAD, actual))
        self._pos_cmd[sid] = clamped
        self._pending_slew_resets[sid] = clamped
        self._direction[sid] *= -1.0
        self._reversed_at[sid] = now
        self._stall_since[sid] = 0.0
        self._stall_pos_snap.pop(sid, None)
        self._stall_peak_torque.pop(sid, None)

    def _assist_neighbor_target(self, nid: int, state: RobotState, toward_neutral: bool) -> float:
        current = state.servo_positions.get(nid, self._pos_cmd.get(nid, 0.0))
        if toward_neutral:
            # Step toward 0°
            step = math.copysign(min(self.ASSIST_WIGGLE_RAD, abs(current)), current) if abs(current) > 0.01 else 0.0
            target = current - step
        else:
            # Step away from 0° (using drift direction as hint)
            hint = self._direction.get(nid, 1.0)
            target = current + math.copysign(self.ASSIST_WIGGLE_RAD, hint)
        return max(-self.MAX_POS_RAD, min(self.MAX_POS_RAD, target))

    def update(self, state: RobotState) -> list[ServoCommand]:
        now = time.monotonic()
        dt = max(state.dt, 1e-4)
        speed_rad_s = self._current_speed_rad_s(now)
        active = state.active_servo_ids

        # Pass 1: advance state machine, build neighbor override map
        neighbor_overrides: dict[int, float] = {}  # nid → target_rad
        for sid in sorted(active):
            if sid not in self._pos_cmd:
                self._init_servo(sid, state, now)

            phase = self._assist_phase[sid]

            if phase == "normal":
                if self._check_stall(sid, state, now):
                    self._assist_phase[sid] = "assisting"
                    self._assist_start[sid] = now
                    self._assist_attempt[sid] = 0
                    self._assist_dir[sid] = 1
                    self._assist_snap_pos[sid] = state.servo_positions.get(sid, self._pos_cmd[sid])
                    logger.info(
                        "[NeighborAssist] s%d stalled at %.2f° — starting neighbor assist",
                        sid, math.degrees(self._assist_snap_pos[sid]),
                    )

            elif phase == "assisting":
                elapsed = now - self._assist_start[sid]
                if elapsed >= self.ASSIST_DURATION_S:
                    actual = state.servo_positions.get(sid, self._pos_cmd[sid])
                    moved = abs(actual - self._assist_snap_pos[sid])
                    if moved >= self.RECOVERY_RAD:
                        logger.info(
                            "[NeighborAssist] s%d recovered (moved %.3f rad) — entering cooldown",
                            sid, moved,
                        )
                        self._assist_phase[sid] = "cooldown"
                        self._cooldown_until[sid] = now + self.ASSIST_COOLDOWN_S
                        self._stall_since[sid] = 0.0
                        self._stall_pos_snap.pop(sid, None)
                        self._stall_peak_torque.pop(sid, None)
                    elif self._assist_attempt[sid] < self.MAX_ASSIST_ATTEMPTS - 1:
                        self._assist_attempt[sid] += 1
                        self._assist_dir[sid] *= -1
                        self._assist_start[sid] = now
                        self._assist_snap_pos[sid] = actual
                        logger.info(
                            "[NeighborAssist] s%d still stuck — attempt %d (dir %+d)",
                            sid, self._assist_attempt[sid] + 1, self._assist_dir[sid],
                        )
                    else:
                        logger.info("[NeighborAssist] s%d assist exhausted — reversing", sid)
                        self._do_reversal(sid, state, now)
                        self._assist_phase[sid] = "cooldown"
                        self._cooldown_until[sid] = now + self.ASSIST_COOLDOWN_S
                else:
                    # Still in assist window: schedule neighbor overrides
                    toward_neutral = (self._assist_dir[sid] == 1)
                    for nid in self._neighbors(sid, active):
                        if nid not in neighbor_overrides:  # first stalled motor wins
                            neighbor_overrides[nid] = self._assist_neighbor_target(nid, state, toward_neutral)

            elif phase == "cooldown":
                if now >= self._cooldown_until[sid]:
                    self._assist_phase[sid] = "normal"
                    self._stall_since[sid] = 0.0
                    self._stall_pos_snap.pop(sid, None)
                    self._stall_peak_torque.pop(sid, None)

        # Pass 2: generate commands
        cmds: list[ServoCommand] = []
        for sid in sorted(active):
            if sid in neighbor_overrides:
                # Freeze drift integrator so neighbor resumes from sensible position
                self._pos_cmd[sid] = state.servo_positions.get(sid, self._pos_cmd.get(sid, 0.0))
                cmds.append(ServoCommand(servo_id=sid, position=neighbor_overrides[sid]))
            else:
                self._pos_cmd[sid] += self._direction[sid] * speed_rad_s * dt
                self._pos_cmd[sid] = max(-self.MAX_POS_RAD, min(self.MAX_POS_RAD, self._pos_cmd[sid]))
                cmds.append(ServoCommand(servo_id=sid, position=self._pos_cmd[sid]))

        return cmds


class BalancedTorqueMotion(Motion):
    """Find a pose where all joints share equal load at a 1 A total-current objective.

    Each tick, per-motor MIT torque feedback drives a position integrator toward
    two simultaneous objectives:
      1. Equalise: nudge each joint so all motors carry similar torque.
      2. Level:    bias the cross-motor mean toward
                   (TARGET_CURRENT_A × KT_NM_PER_AMP) / n_motors.

    Under-loaded joints creep outward (slalom-curl direction); over-loaded joints
    retreat toward home.  Starting from rest the robot slowly finds a pose where
    gravity loads all joints equally at the combined 1 A current level.
    """

    name = "balanced-torque"

    TARGET_CURRENT_A: float = 1.0     # total target Amps across all motors
    KT_NM_PER_AMP: float = 0.3       # GL40 II torque constant estimate (Nm/A)
    GAIN_EQ: float = 1.0             # equalization term weight
    GAIN_LVL: float = 0.5            # level term weight
    TORQUE_TO_POS_GAIN: float = 0.5  # rad / (Nm · s)
    MAX_POS_RAD: float = MOTOR_LIMITS.pos_max
    LOG_INTERVAL_S: float = 3.0

    def __init__(
        self,
        target_current_a: float = TARGET_CURRENT_A,
        kt_nm_per_a: float = KT_NM_PER_AMP,
    ) -> None:
        self._target_total_nm = target_current_a * kt_nm_per_a
        self._pos_cmd: dict[int, float] = {}
        self._last_log: float = 0.0

    def is_active(self) -> bool:
        return True

    def on_start(self, controller: "Controller") -> None:
        self._pos_cmd = {}
        self._last_log = 0.0
        logger.info(
            "[BEHAVIOR] BalancedTorque: %.1fA total × %.3f Nm/A = %.3f Nm total target",
            self._target_total_nm / (self.KT_NM_PER_AMP or 1e-9),
            self.KT_NM_PER_AMP,
            self._target_total_nm,
        )

    def update(self, state: RobotState) -> list[ServoCommand]:
        now = time.monotonic()
        dt = max(state.dt, 1e-4)
        ids = sorted(state.active_servo_ids)
        if not ids:
            return []

        # Per-motor target scales with how many motors are active
        target_per_motor = self._target_total_nm / len(ids)

        torques = {sid: abs(state.motor_torques.get(sid, 0.0)) for sid in ids}
        mean_torque = sum(torques.values()) / len(ids)

        if now - self._last_log >= self.LOG_INTERVAL_S:
            self._last_log = now
            logger.debug(
                "[BalancedTorque] mean=%.3f Nm  target/motor=%.3f Nm  %s",
                mean_torque,
                target_per_motor,
                "  ".join(f"s{sid}:{torques[sid]:.2f}" for sid in ids),
            )

        cmds = []
        for sid in ids:
            if sid not in self._pos_cmd:
                self._pos_cmd[sid] = state.servo_positions.get(sid, 0.0)

            t_i = torques[sid]
            e_eq = mean_torque - t_i                  # + → under-loaded vs peers
            e_lvl = target_per_motor - mean_torque    # + → all motors under target
            net_error = self.GAIN_EQ * e_eq + self.GAIN_LVL * e_lvl
            step = net_error * self.TORQUE_TO_POS_GAIN * dt

            pos = self._pos_cmd[sid]
            if step > 0.0:
                # Under-loaded: push outward in slalom-curl direction
                dir_ = 1.0 if sid % 2 == 1 else -1.0
                if abs(pos) > 0.05:
                    dir_ = math.copysign(1.0, pos)
                pos += dir_ * step
            elif step < 0.0:
                # Over-loaded: retreat toward home
                retreat = min(abs(step), abs(pos))
                if abs(pos) > 1e-6:
                    pos -= math.copysign(retreat, pos)

            self._pos_cmd[sid] = max(-self.MAX_POS_RAD, min(self.MAX_POS_RAD, pos))
            cmds.append(ServoCommand(servo_id=sid, position=self._pos_cmd[sid]))

        return cmds


class PurrRippleMotion(Motion):
    """kd-vibration travelling wave propagating head to tail.

    Each motor's kd is modulated by a sine wave with a spatial phase offset so
    the peak ripples smoothly from head to tail with overlapping envelopes.
    Multiple motors are simultaneously elevated. Position stays at 0° —
    elevated kd resists velocity perturbations, producing a purring feel.
    Speed controls ripple frequency (ripples per second).
    """

    name = "purr-ripple"

    KD_TARGET: float = 0.08  # peak kd — causes vibration
    BASE_HZ: float = 0.3    # ripple frequency at speed=1.0 (~3.3s per pass)
    CREST_POWER: int = 0.5    # exponent on sin envelope; higher = narrower active crest

    def __init__(self, speed: float = 1.0) -> None:
        self.speed = max(speed, 0.01)
        self._start: float = 0.0

    def on_start(self, controller: "Controller") -> None:
        self._start = time.monotonic()
        hz = self.BASE_HZ * self.speed
        logger.info("[BEHAVIOR] PurrRipple: speed=%.1f (%.2f Hz)", self.speed, hz)
        logger.debug("[BEHAVIOR] PurrRipple: kd target=%.2f, %.2f Hz.", self.KD_TARGET, hz)

    def is_active(self) -> bool:
        return True

    def update(self, state: RobotState) -> list[ServoCommand]:
        t = time.monotonic() - self._start
        ids = sorted(sid for sid in state.active_servo_ids if sid != 7)
        n = len(ids)
        if n == 0:
            return []

        hz = self.BASE_HZ * self.speed
        cmds = []
        for i, sid in enumerate(ids):
            # One wavelength across the body — each motor gets a unique phase offset
            spatial_phase = (i / max(n - 1, 1)) * 2 * math.pi
            phase = 2 * math.pi * hz * t - spatial_phase
            if math.sin(phase) <= 0.0:
                # Between crests: idle (freespin) until next turn
                cmds.append(ServoCommand(servo_id=sid, position=0.0, kp=0.0, kd=0.0, torque_ff=0.0))
            else:
                # Higher CREST_POWER narrows the active lobe without changing motor ordering
                envelope = math.sin(phase) ** self.CREST_POWER
                kd = MOTOR_LIMITS.kd_default + envelope * (self.KD_TARGET - MOTOR_LIMITS.kd_default)
                cmds.append(ServoCommand(servo_id=sid, position=0.0, kp=MOTOR_LIMITS.kp_default, kd=kd, torque_ff=0.0))

        return cmds


ALL_PATTERNS: list[type[Motion]] = [
    SnuggleMotion,
    PulseMotion,
    BreatheMotion,
    SwayMotion,
    CascadeMotion,
    SlalomMotion,
    TwitchMotion,
    FreezeMotion,
    CoilMotion,
    CurlMotion,
    Spin7Motion,
    StrokeReactMotion,
    StrokeCurlMotion,
    CurlTowardsMotion,
    CurlAwayMotion,
    StrokeSnuggleMotion,
    ExploreMotion,
    SeekTouchMotion,
    AvoidTouchMotion,
    DriftMotion,
    StruggleMotion,
    NeighborAssistDriftMotion,
    YieldStiffMotion,
    PoseMotion,
    BalancedTorqueMotion,
    PurrRippleMotion,
]

PATTERN_NAMES: list[str] = [cls.name for cls in ALL_PATTERNS]
