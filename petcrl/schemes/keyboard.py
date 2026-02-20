"""
KeyboardControlScheme — control servos with the keyboard.

Controls:
  0-8        Select which module/servo to control
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
from typing import Dict, List, Optional, TYPE_CHECKING

from petcrl.protocols import ControlScheme
from petcrl.types import RobotState, ServoCommand

if TYPE_CHECKING:
    from petcrl.controller import Controller


class KeyboardControlScheme(ControlScheme):
    """
    Keyboard-driven servo control.

    Args:
        step_deg:       Degrees to move per keypress (default: 5°)
        angle_limit:    Maximum absolute angle in degrees (default: 150°)
        servo_offset:   Offset added to the selected module number to get
                        servo_id (default: 1, so module 0 → servo 1).
    """

    name = "keyboard"

    def __init__(
        self,
        step_deg: float = 5.0,
        angle_limit: float = 150.0,
        servo_offset: int = 1,
    ) -> None:
        self.step_deg = step_deg
        self.angle_limit = angle_limit
        self.servo_offset = servo_offset

        # Per-module target angles (degrees)
        self._angles: Dict[int, float] = {}
        # Currently selected module (0-8)
        self._selected: int = 0
        # Pending angle deltas to apply on next update()
        self._pending: Dict[int, float] = {}
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

    def update(self, state: RobotState) -> List[ServoCommand]:
        with self._lock:
            reset = self._reset_requested
            self._reset_requested = False
            pending = dict(self._pending)
            self._pending.clear()

        commands: List[ServoCommand] = []

        if reset:
            for mod in range(9):
                self._angles[mod] = 0.0
                commands.append(ServoCommand.from_angle(mod + self.servo_offset, 0.0))
            return commands

        for mod, delta in pending.items():
            current = self._angles.get(mod, 0.0)
            new_angle = max(-self.angle_limit, min(self.angle_limit, current + delta))
            self._angles[mod] = new_angle
            commands.append(
                ServoCommand.from_angle(mod + self.servo_offset, new_angle)
            )

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
    def angles(self) -> Dict[int, float]:
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
            with self._lock:
                # Digit keys — select module
                try:
                    char = key.char
                    if char and char.isdigit():
                        self._selected = int(char)
                        return
                    if char in ("r", "R"):
                        self._reset_requested = True
                        return
                    if char in ("q", "Q"):
                        if self._controller:
                            self._controller.stop()
                        return
                except AttributeError:
                    pass

                # Special keys
                if key == keyboard.Key.up:
                    self._pending[self._selected] = (
                        self._pending.get(self._selected, 0.0) + self.step_deg
                    )
                elif key == keyboard.Key.down:
                    self._pending[self._selected] = (
                        self._pending.get(self._selected, 0.0) - self.step_deg
                    )
                elif key == keyboard.Key.esc:
                    if self._controller:
                        self._controller.stop()

        self._listener = keyboard.Listener(on_press=on_press)
        self._listener.daemon = True
        self._listener.start()
