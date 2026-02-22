"""
KeyboardControlScheme — control servos with the keyboard.

Controls:
  1-7        Select servo to control (key N → servo N)
  ↑ / ↑      Increase angle by step_deg
  ↓ / ↓      Decrease angle by step_deg
  r          Reset all servos to 0°
  q / Esc    Request controller shutdown

Uses pynput for cross-platform keyboard capture in a background thread,
so it works from any terminal without requiring a GUI window.

Install: pip install pynput
"""

from __future__ import annotations

import threading
from typing import TYPE_CHECKING, Optional

from petctl.protocols import ControlScheme
from petctl.types import RobotState, ServoCommand

if TYPE_CHECKING:
    from petctl.controller import Controller

# Number of modules selectable via digit keys (0 through 8)
_MAX_KEYBOARD_MODULES = 9


class KeyboardControlScheme(ControlScheme):
    """
    Keyboard-driven servo control.

    Args:
        step_deg:       Degrees to move per keypress (default: 5°)
        angle_limit:    Maximum absolute angle in degrees (default: 150°)
        servo_offset:   Offset added to the selected key number to get
                        servo_id (default: 0, so key 1 → servo 1).
    """

    name = "keyboard"

    def __init__(
        self,
        step_deg: float = 5.0,
        angle_limit: float = 150.0,
        servo_offset: int = 0,
    ) -> None:
        self.step_deg = step_deg
        self.angle_limit = angle_limit
        self.servo_offset = servo_offset

        # Per-module target angles (degrees)
        self._angles: dict[int, float] = {}
        # Currently selected module (0-8)
        self._selected: int = 0
        # Pending angle deltas to apply on next update()
        self._pending: dict[int, float] = {}
        self._reset_requested: bool = False
        self._lock = threading.Lock()

        self._controller: Optional["Controller"] = None
        self._listener = None

    # ------------------------------------------------------------------
    # ControlScheme interface
    # ------------------------------------------------------------------

    def on_start(self, controller: "Controller") -> None:
        self._controller = controller
        self._start_listener()
        print(
            "[Keyboard] Ready.\n"
            "  0-8: select module  |  ↑/↓: adjust angle  |  r: reset  |  q/Esc: quit"
        )

    def update(self, state: RobotState) -> list[ServoCommand]:
        with self._lock:
            reset = self._reset_requested
            self._reset_requested = False
            pending = dict(self._pending)
            self._pending.clear()

        commands: list[ServoCommand] = []

        if reset:
            for mod in range(_MAX_KEYBOARD_MODULES):
                servo_id = mod + self.servo_offset
                if servo_id not in state.active_servo_ids:
                    continue
                self._angles[mod] = 0.0
                commands.append(ServoCommand.from_angle(servo_id, 0.0))
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
            new_angle = max(-self.angle_limit, min(self.angle_limit, current + delta))
            self._angles[mod] = new_angle
            commands.append(ServoCommand.from_angle(servo_id, new_angle))

        return commands

    def on_stop(self) -> None:
        if self._listener is not None:
            try:
                self._listener.stop()
            except Exception:
                pass
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

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _start_listener(self) -> None:
        try:
            from pynput import keyboard
        except ImportError:
            print("[Keyboard] pynput not installed. Run: pip install pynput")
            return

        def on_press(key):
            stop_requested = False
            with self._lock:
                # Character keys — select module, reset, or quit
                try:
                    char = key.char
                    if char and char.isdigit():
                        self._selected = int(char)
                        return
                    if char in ("r", "R"):
                        self._reset_requested = True
                        return
                    if char in ("q", "Q"):
                        stop_requested = True
                        # No return — fall through so stop() is called after lock
                except AttributeError:
                    pass

                # Special keys (skip if already stopping)
                if not stop_requested:
                    if key == keyboard.Key.up:
                        self._pending[self._selected] = (
                            self._pending.get(self._selected, 0.0) + self.step_deg
                        )
                    elif key == keyboard.Key.down:
                        self._pending[self._selected] = (
                            self._pending.get(self._selected, 0.0) - self.step_deg
                        )
                    elif key == keyboard.Key.esc:
                        stop_requested = True

            # Call stop() outside the lock — thread-safe via call_soon_threadsafe
            if stop_requested and self._controller:
                self._controller.stop()

        self._listener = keyboard.Listener(on_press=on_press)
        self._listener.daemon = True
        self._listener.start()
