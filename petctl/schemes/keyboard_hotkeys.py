"""
KeyboardHotkeys — always-on keyboard shortcuts for admin/controller actions.

Active regardless of the active control scheme.

Shortcuts:
  Ctrl+Shift+H   Save current positions as EEPROM home
  Ctrl+Shift+D   Deactivate all motors (exit MIT mode)
  Ctrl+Shift+C   Recalibrate MPR121 cap sensor baselines (fix stuck pads)
  Ctrl+Shift+T   Tare IMU orientation in the visualizer
"""

from __future__ import annotations

import logging
import threading
from typing import TYPE_CHECKING, Optional

if TYPE_CHECKING:
    from petctl.controller import Controller

logger = logging.getLogger(__name__)


class KeyboardHotkeys:
    """Always-on keyboard listener for admin shortcuts.

    Started by the Controller at run-time; active with any motion scheme.
    Ctrl+C (SIGINT) is handled by the process signal handler, not here.
    """

    def __init__(self) -> None:
        self._save_home_requested: bool = False
        self._deactivate_requested: bool = False
        self._calibrate_touch_requested: bool = False
        self._tare_imu_requested: bool = False
        self._ctrl_held: bool = False
        self._lock = threading.Lock()
        self._controller: Optional[Controller] = None
        self._listener = None

    def start(self, controller: Controller) -> None:
        """Start the listener and bind to the controller for dispatch."""
        self._controller = controller
        self._start_listener()
        logger.info(
            "[Hotkeys] Ctrl+Shift+H: save home  |  Ctrl+Shift+D: deactivate motors  |  "
            "Ctrl+Shift+C: recal cap sensors  |  Ctrl+Shift+T: tare IMU"
        )

    def stop(self) -> None:
        """Stop the keyboard listener."""
        if self._listener is not None:
            try:
                self._listener.stop()
            except Exception as e:
                logger.warning("[Hotkeys] listener teardown error: %s", e)
        self._listener = None

    # ------------------------------------------------------------------
    # Consume flags (called from the controller loop)
    # ------------------------------------------------------------------

    def take_save_home(self) -> bool:
        """Consume and return the save-home request flag. Returns True once per event."""
        with self._lock:
            val = self._save_home_requested
            self._save_home_requested = False
        return val

    def take_deactivate(self) -> bool:
        """Consume and return the deactivate-motors request flag. Returns True once per event."""
        with self._lock:
            val = self._deactivate_requested
            self._deactivate_requested = False
        return val

    def take_calibrate_touch(self) -> bool:
        """Consume and return the cap-sensor recalibration request flag. Returns True once per event."""
        with self._lock:
            val = self._calibrate_touch_requested
            self._calibrate_touch_requested = False
        return val

    def take_tare_imu(self) -> bool:
        """Consume and return the IMU tare request flag. Returns True once per event."""
        with self._lock:
            val = self._tare_imu_requested
            self._tare_imu_requested = False
        return val

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _start_listener(self) -> None:
        try:
            from pynput import keyboard
        except ImportError:
            logger.error("[Hotkeys] pynput not installed. Run: pip install pynput")
            return

        ctrl_keys = frozenset({keyboard.Key.ctrl, keyboard.Key.ctrl_l, keyboard.Key.ctrl_r})

        def on_press(key):
            msg = None
            with self._lock:
                if key in ctrl_keys:
                    self._ctrl_held = True
                    return

                try:
                    char = key.char
                    if char in ("H", "h", "\x08") and self._ctrl_held:
                        self._save_home_requested = True
                        msg = "save home requested"
                    elif char in ("D", "d", "\x04") and self._ctrl_held:
                        self._deactivate_requested = True
                        msg = "deactivate motors requested"
                    elif char in ("C", "c") and self._ctrl_held:
                        self._calibrate_touch_requested = True
                        msg = "cap sensor recalibration requested"
                    elif char == "T" and self._ctrl_held:
                        self._tare_imu_requested = True
                        msg = "IMU tare requested"
                except AttributeError:
                    pass

            if msg:
                logger.info("[Hotkeys] %s", msg)

        def on_release(key):
            if key in ctrl_keys:
                with self._lock:
                    self._ctrl_held = False

        self._listener = keyboard.Listener(on_press=on_press, on_release=on_release)
        self._listener.daemon = True
        self._listener.start()
