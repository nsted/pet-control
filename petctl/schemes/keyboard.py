"""
KeyboardControlScheme — control servos with the keyboard.

Controls:
  Ctrl+Shift+K   Enable/disable module angle adjustment (disabled by default)
  0-8            Select module (when adjustment enabled)
  ↑ / ↓          Adjust selected module angle by step_deg (when adjustment enabled)
  Ctrl+Shift+R   Reset all servos to 0°
  Ctrl+Shift+H   Save current positions as EEPROM home
  Ctrl+Shift+D   Deactivate all motors (exit MIT mode)
  Ctrl+Shift+L   Toggle sensor pad labels in visualizer
  Ctrl-C         Quit (SIGINT only — q/Esc do not quit)

Uses pynput for cross-platform keyboard capture in a background thread,
so it works from any terminal without requiring a GUI window.

Install: pip install pynput
"""

from __future__ import annotations

import logging
import threading
import time
from typing import TYPE_CHECKING, Optional

from petctl.protocols import ControlScheme
from petctl.types import RobotState, ServoCommand

logger = logging.getLogger(__name__)

if TYPE_CHECKING:
    from petctl.controller import Controller

# Number of modules selectable via digit keys (0 through 8)
_MAX_KEYBOARD_MODULES = 9


class KeyboardControlScheme(ControlScheme):
    """
    Keyboard-driven servo control.

    Args:
        step_deg:       Degrees to move per keypress (default: 4°)
        servo_offset:   Offset added to the selected key number to get
                        servo_id (default: 0, so key 1 → servo 1).
    """

    name = "keyboard"

    def __init__(
        self,
        step_deg: float = 4.0,
        servo_offset: int = 0,
    ) -> None:
        self.step_deg = step_deg
        self.servo_offset = servo_offset

        # Per-module target angles (degrees)
        self._angles: dict[int, float] = {}
        # Currently selected module (0-8)
        self._selected: int = 0
        # Pending angle deltas to apply on next update()
        self._pending: dict[int, float] = {}
        self._reset_requested: bool = False
        self._save_home_requested: bool = False
        self._deactivate_requested: bool = False
        self._adjustment_enabled: bool = False
        self._lock = threading.Lock()
        # Timestamp of last key event that changed a motor target.
        # Position commands are re-issued for _SLEW_SETTLE_S after each key
        # press so the slew filter can reach the target, then the scheme goes
        # quiet and the controller falls back to MIT enable frames only.
        self._last_key_time: float = 0.0

        self._controller: Optional["Controller"] = None
        self._listener = None
        self._ctrl_held: bool = False

    # ------------------------------------------------------------------
    # ControlScheme interface
    # ------------------------------------------------------------------

    def on_start(self, controller: "Controller") -> None:
        self._controller = controller
        self._start_listener()
        logger.info(
            "[Keyboard] Ready (adjustment disabled).\n"
            "  Ctrl+Shift+K: enable/disable adjustment  |  Ctrl+Shift+R: reset  |  Ctrl+Shift+H: save home  |  Ctrl+Shift+D: deactivate motors  |  Ctrl+Shift+L: toggle sensor labels  |  Ctrl-C: quit"
        )

    # Seconds to keep re-issuing position commands after the last key press.
    # Long enough for the slew filter to reach the target; after this the
    # controller falls back to MIT enable frames, stopping the motor clicking.
    _SLEW_SETTLE_S: float = 0.5

    def update(self, state: RobotState) -> list[ServoCommand]:
        with self._lock:
            reset = self._reset_requested
            self._reset_requested = False
            pending = dict(self._pending)
            self._pending.clear()

        commands: list[ServoCommand] = []

        if reset:
            zeroed: list[int] = []
            for mod in range(_MAX_KEYBOARD_MODULES):
                servo_id = mod + self.servo_offset
                if servo_id not in state.active_servo_ids:
                    continue
                self._angles[mod] = 0.0
                commands.append(ServoCommand.from_angle(servo_id, 0.0))
                zeroed.append(servo_id)
            self._last_key_time = time.monotonic()
            logger.info("[Keyboard] reset: servos %s → 0°", zeroed)
            return commands

        for mod, delta in pending.items():
            servo_id = mod + self.servo_offset
            if servo_id not in state.active_servo_ids:
                continue  # servo not on this backend — skip
            if mod not in self._angles:
                # Seed from actual servo position so first keypress is relative
                # to where the robot (or mock) currently is, not from 0°.
                raw = state.servo_positions.get(servo_id)
                if raw is None:
                    continue
                self._angles[mod] = ServoCommand.position_to_angle(raw)
            current = self._angles[mod]
            new_angle = current + delta
            self._angles[mod] = new_angle
            logger.info("[Keyboard] servo %d: %.1f° → %.1f°", servo_id, current, new_angle)

        # Re-issue targets only while the slew filter is still settling.
        # Once quiet, the controller sends MIT enable frames and the motor holds
        # its last position internally — no need to keep commanding it.
        if self._angles and (time.monotonic() - self._last_key_time) <= self._SLEW_SETTLE_S:
            for mod, angle_deg in self._angles.items():
                servo_id = mod + self.servo_offset
                if servo_id not in state.active_servo_ids:
                    continue
                commands.append(ServoCommand.from_angle(servo_id, angle_deg))

        return commands

    def on_stop(self) -> None:
        if self._listener is not None:
            try:
                self._listener.stop()
            except Exception as e:
                logger.warning("[Keyboard] listener teardown error: %s", e)
        self._listener = None

    # ------------------------------------------------------------------
    # Status helpers (useful for visualizers / CLI display)
    # ------------------------------------------------------------------

    @property
    def selected_module(self) -> int:
        return self._selected

    @property
    def angles(self) -> dict[int, float]:
        """Current target angles per module (copy)."""
        with self._lock:
            return dict(self._angles)

    def take_save_home(self) -> bool:
        """Consume and return the save-home request flag (set by Ctrl+Shift+H key).

        Returns True exactly once per key event; subsequent calls return False.
        Also resets internal angle targets to 0 so the scheme commands the new
        home position (stay in place) rather than driving back to the old target.
        """
        with self._lock:
            val = self._save_home_requested
            self._save_home_requested = False
            if val:
                # Reset internal angle tracking to match the new reference frame.
                # Do NOT set _reset_requested — that would emit position commands
                # and activate motor torque.
                self._angles.clear()
        return val

    def take_deactivate(self) -> bool:
        """Consume and return the deactivate-motors request flag (set by Ctrl+Shift+D).

        Returns True exactly once per key event; subsequent calls return False.
        """
        with self._lock:
            val = self._deactivate_requested
            self._deactivate_requested = False
        return val

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _dispatch_toggle_labels(self) -> None:
        if self._controller is None:
            return
        for viz in self._controller.visualizers:
            toggle = getattr(viz, "toggle_pad_labels", None)
            if toggle is not None:
                toggle()

    def _start_listener(self) -> None:
        try:
            from pynput import keyboard
        except ImportError:
            logger.error("[Keyboard] pynput not installed. Run: pip install pynput")
            return

        ctrl_keys = frozenset({keyboard.Key.ctrl, keyboard.Key.ctrl_l, keyboard.Key.ctrl_r})

        def on_press(key):
            toggle_labels = False
            msg = None
            with self._lock:
                if key in ctrl_keys:
                    self._ctrl_held = True
                    return

                # Character keys — select module, reset, or toggle
                try:
                    char = key.char
                    if char and char.isdigit():
                        if self._adjustment_enabled:
                            self._selected = int(char)
                            msg = "module %s selected" % char
                        return
                    if char in ("K", "k") and self._ctrl_held:
                        self._adjustment_enabled = not self._adjustment_enabled
                        msg = "adjustment %s" % ("enabled" if self._adjustment_enabled else "disabled")
                        return
                    if char in ("R", "r") and self._ctrl_held:
                        self._reset_requested = True
                        msg = "reset requested"
                        return
                    if char in ("H", "h") and self._ctrl_held:
                        self._save_home_requested = True
                        msg = "save home requested"
                        return
                    if char in ("D", "d") and self._ctrl_held:
                        self._deactivate_requested = True
                        msg = "deactivate motors requested"
                        return
                    if char in ("L", "l") and self._ctrl_held:
                        toggle_labels = True
                        msg = "toggle sensor labels"
                except AttributeError:
                    pass

                # Arrow keys — only active when adjustment is enabled
                if not self._adjustment_enabled:
                    return
                if key == keyboard.Key.up:
                    self._pending[self._selected] = (
                        self._pending.get(self._selected, 0.0) + self.step_deg
                    )
                    self._last_key_time = time.monotonic()
                    msg = "↑ module %d (+%.1f°)" % (self._selected, self.step_deg)
                elif key == keyboard.Key.down:
                    self._pending[self._selected] = (
                        self._pending.get(self._selected, 0.0) - self.step_deg
                    )
                    self._last_key_time = time.monotonic()
                    msg = "↓ module %d (-%.1f°)" % (self._selected, self.step_deg)

            # Dispatch actions outside the lock
            if msg:
                logger.info("[Keyboard] %s", msg)
            if toggle_labels:
                self._dispatch_toggle_labels()

        def on_release(key):
            if key in ctrl_keys:
                with self._lock:
                    self._ctrl_held = False

        self._listener = keyboard.Listener(on_press=on_press, on_release=on_release)
        self._listener.daemon = True
        self._listener.start()
