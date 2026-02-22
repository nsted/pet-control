"""
Controller — the central hub of petctl.

Owns a RobotBackend, a ControlScheme, and a list of Visualizers.
Runs a single async loop:

  1. state  = await backend.get_state()
  2. cmds   = scheme.update(state)
  3. if not dry_run: await backend.send_commands(cmds)
  4. for viz in visualizers: viz.update(state)
  5. sleep for remainder of tick

Usage:

  from petctl import Controller
  from petctl.backends.mock import MockBackend
  from petctl.schemes.keyboard import KeyboardControlScheme
  from petctl.visualizers.rerun_viz import RerunVisualizer

  async def main():
      ctrl = Controller(
          backend=MockBackend(),
          scheme=KeyboardControlScheme(),
          visualizers=[RerunVisualizer()],
      )
      await ctrl.run()

  asyncio.run(main())
"""

from __future__ import annotations

import asyncio
import signal
import time
from typing import Optional

from petctl.protocols import ControlScheme, RobotBackend, Visualizer
from petctl.types import RobotState


class Controller:
    """
    Central coordinator for petctl.

    Args:
        backend:      A RobotBackend (RobotBackend, MockBackend, etc.)
        scheme:       A ControlScheme (keyboard, AI, passthrough, etc.)
        visualizers:  List of Visualizer instances (Rerun, etc.)
        poll_hz:      How often to run the control loop (default: 20 Hz)
        dry_run:      If True, read sensors and run the scheme but never
                      send servo commands.  Useful for testing.
    """

    def __init__(
        self,
        backend: RobotBackend,
        scheme: ControlScheme,
        visualizers: Optional[list[Visualizer]] = None,
        poll_hz: float = 20.0,
        dry_run: bool = False,
    ) -> None:
        self.backend = backend
        self.scheme = scheme
        self.visualizers: list[Visualizer] = visualizers or []
        self.poll_interval = 1.0 / poll_hz
        self.dry_run = dry_run

        self._state: RobotState = RobotState.empty()
        self._running = False
        # Created inside run() to bind to the correct event loop
        self._stop_event: Optional[asyncio.Event] = None
        self._event_loop: Optional[asyncio.AbstractEventLoop] = None

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    @property
    def state(self) -> RobotState:
        """The most recently polled RobotState (read-only)."""
        return self._state

    def set_scheme(self, scheme: ControlScheme) -> None:
        """
        Hot-swap the active control scheme.

        Calls on_stop() on the current scheme and on_start() on the new one
        without dropping the backend connection.
        """
        if self._running:
            self.scheme.on_stop()
        self.scheme = scheme
        if self._running:
            scheme.on_start(self)

    def stop(self) -> None:
        """Request a graceful shutdown of the control loop.

        Thread-safe: may be called from any thread (e.g. keyboard listener).
        """
        if self._stop_event is not None and self._event_loop is not None:
            self._event_loop.call_soon_threadsafe(self._stop_event.set)

    async def run(self) -> None:
        """
        Connect to the backend and start the control loop.

        Runs until stop() is called or a KeyboardInterrupt is received.
        """
        print(f"[Controller] Connecting via {self.backend.__class__.__name__}...")
        ok = await self.backend.connect()
        if not ok:
            print("[Controller] Backend failed to connect. Exiting.")
            return

        self._running = True
        self._event_loop = asyncio.get_running_loop()
        self._stop_event = asyncio.Event()

        # Activate scheme and visualizers
        self.scheme.on_start(self)
        for viz in self.visualizers:
            viz.on_start(self)

        print("[Controller] Running. Press Ctrl-C to stop.")

        # Install signal handlers so Ctrl-C triggers graceful shutdown
        loop = asyncio.get_running_loop()
        registered_signals: list[signal.Signals] = []
        for sig in (signal.SIGINT, signal.SIGTERM):
            try:
                loop.add_signal_handler(sig, self.stop)
                registered_signals.append(sig)
            except NotImplementedError:
                # Windows doesn't support add_signal_handler for all signals
                pass

        try:
            await self._loop()
        finally:
            for sig in registered_signals:
                try:
                    loop.remove_signal_handler(sig)
                except Exception:
                    pass
            await self._shutdown()

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    async def _loop(self) -> None:
        assert self._stop_event is not None
        while not self._stop_event.is_set():
            tick_start = time.monotonic()

            # 1. Get state from backend
            self._state = await self.backend.get_state()

            # 2. Run control scheme
            try:
                commands = self.scheme.update(self._state)
            except Exception as e:
                print(f"[Controller] Scheme '{self.scheme.name}' error: {e}")
                commands = []

            # 3. Send commands (unless dry run)
            if not self.dry_run and commands:
                try:
                    await self.backend.send_commands(commands)
                except Exception as e:
                    print(f"[Controller] Backend send_commands error: {e}")

            # 4. Update visualizers
            for viz in self.visualizers:
                try:
                    viz.update(self._state)
                except Exception as e:
                    print(f"[Controller] Visualizer {viz.name} error: {e}")

            # 5. Sleep for remainder of tick
            elapsed = time.monotonic() - tick_start
            sleep_time = self.poll_interval - elapsed
            if sleep_time > 0:
                await asyncio.sleep(sleep_time)

    async def _shutdown(self) -> None:
        self._running = False
        print("\n[Controller] Shutting down...")
        self.scheme.on_stop()
        for viz in self.visualizers:
            viz.on_stop()
        await self.backend.disconnect()
        print("[Controller] Done.")
