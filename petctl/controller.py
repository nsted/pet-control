"""
Controller — the central hub of petctl.

Owns a RobotBackend, a ControlScheme, and a list of Visualizers.
Runs an async loop as fast as possible:

  1. state  = await backend.get_state()
  2. cmds   = scheme.update(state)
  2b. smooth = LPF toward scheme targets + per-tick delta cap (LOOP_LIMITS.*)
  3. if not dry_run: await backend.send_commands(cmds)
  4. visualizers.update(state)  ← daemon thread with bounded queue; frames are
     dropped (never blocking) if Rerun can't keep up
  5. asyncio.sleep(0) to yield to motor TX, sensor, and receive tasks

The loop itself has no rate limit. Motor TX fires at motor_update_hz (its own task);
sensor polling fires at sensor_poll_hz (its own task). state.dt reflects the
true wall-clock tick period and is available to schemes. Timing stats are
printed every few seconds so you can see the real rate.

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
import math
import queue
import signal
import threading
import time
from typing import Optional

from petctl.config import LOOP_LIMITS
from petctl.protocols import ControlScheme, RobotBackend, Visualizer
from petctl.types import RobotState, ServoCommand


class Controller:
    """
    Central coordinator for petctl.

    Args:
        backend:      A RobotBackend (RobotBackend, MockBackend, etc.)
        scheme:       A ControlScheme (keyboard, AI, passthrough, etc.)
        visualizers:  List of Visualizer instances (Rerun, etc.)
        dry_run:      If True, read sensors and run the scheme but never
                      send servo commands.  Useful for testing.
        limp:         If True, disable motor torque after connecting so joints
                      can be moved freely by hand.  Implies dry_run=True so
                      commands never re-enable torque during the session.
    """

    # How often to print loop timing stats (seconds)
    _STATS_INTERVAL = 5.0

    def __init__(
        self,
        backend: RobotBackend,
        scheme: ControlScheme,
        visualizers: Optional[list[Visualizer]] = None,
        dry_run: bool = False,
        limp: bool = False,
        log_mit: bool = False,
        log_touch: bool = False,
        log_loop: bool = False,
    ) -> None:
        self.backend = backend
        self.scheme = scheme
        self.visualizers: list[Visualizer] = visualizers or []
        self.dry_run = dry_run or limp  # limp implies no commands
        self.limp = limp
        self.log_mit = log_mit
        self.log_touch = log_touch
        self.log_loop = log_loop

        self._state: RobotState = RobotState.empty()
        self._running = False
        # Created inside run() to bind to the correct event loop
        self._stop_event: Optional[asyncio.Event] = None
        self._event_loop: Optional[asyncio.AbstractEventLoop] = None
        self._last_pos_print: float = 0.0

        # Rolling loop timing stats
        self._loop_times: list[float] = []   # seconds per iteration
        self._last_stats_print: float = 0.0

        # Daemon thread for visualizer updates. Bounded queue with maxsize=1 so
        # the control loop never blocks: put_nowait drops frames when Rerun can't
        # keep up. None is the shutdown sentinel.
        self._viz_queue: queue.Queue[Optional[RobotState]] = queue.Queue(maxsize=1)
        self._viz_thread = threading.Thread(
            target=self._viz_worker, name="viz", daemon=True
        )
        self._viz_thread.start()

        # Last commanded position (rad) after slew — used to cap per-tick jumps.
        self._slew_last_sent_rad: dict[int, float] = {}


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

        if self.limp:
            print("[Controller] Limp mode: disabling torques — move joints freely by hand.")
            await self.backend.disable_torques()

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

    def _apply_slew_to_commands(self, commands: list[ServoCommand]) -> None:
        """
        Smooth each position command toward the scheme output, then clamp delta.

        A first-order low-pass (tau from LOOP_LIMITS) avoids brick-wall velocity
        in command space, which tends to feel like small “pops” at 20–50 Hz.
        """
        if not commands:
            return
        max_step = math.radians(LOOP_LIMITS.max_angle_step_per_tick_deg)
        tau = LOOP_LIMITS.command_smoothing_tau_s
        alpha_cap = LOOP_LIMITS.command_smoothing_max_alpha
        dt_floor = 1e-3
        dt = max(self._state.dt, dt_floor)

        for cmd in commands:
            if cmd.position is None:
                continue
            sid = cmd.servo_id
            target = cmd.position
            prev = self._slew_last_sent_rad.get(sid)
            if prev is None:
                prev = self._state.servo_positions.get(sid)
            if prev is None:
                cmd.position = target
                self._slew_last_sent_rad[sid] = cmd.position
                continue

            if tau <= 0.0:
                new_pos = target
            else:
                alpha = min(alpha_cap, dt / tau)
                new_pos = prev + alpha * (target - prev)

            delta = new_pos - prev
            if max_step > 0.0:
                delta = max(-max_step, min(max_step, delta))
            cmd.position = prev + delta
            self._slew_last_sent_rad[sid] = cmd.position

    async def _loop(self) -> None:
        assert self._stop_event is not None
        while not self._stop_event.is_set():
            t0 = time.monotonic()

            # 1. Get state from backend
            self._state = await self.backend.get_state()

            # 2. Run control scheme
            try:
                commands = self.scheme.update(self._state)
            except Exception as e:
                print(f"[Controller] Scheme '{self.scheme.name}' error: {e}")
                commands = []

            # Let schemes reset the slew state for specific servos before this
            # tick's filter runs — needed when a stall reversal must take effect
            # immediately rather than waiting for the filter to unwind.
            _take_slew_resets = getattr(self.scheme, "take_slew_resets", None)
            if _take_slew_resets is not None:
                for _sid, _pos in _take_slew_resets().items():
                    self._slew_last_sent_rad[_sid] = _pos

            self._apply_slew_to_commands(commands)

            # 3. Send commands (unless dry run)
            if not self.dry_run and commands:
                try:
                    await self.backend.send_commands(commands)
                except Exception as e:
                    print(f"[Controller] Backend send_commands error: {e}")

            # 3b. Save-home: write EEPROM offsets so current position reports as 0
            take_save_home = getattr(self.scheme, "take_save_home", None)
            if take_save_home is not None and take_save_home():
                print("[Controller] Saving home offsets...")
                try:
                    await self.backend.write_home_offsets()
                    # Clear slew state so the filter doesn't hold pre-home positions.
                    self._slew_last_sent_rad.clear()
                    print("[Controller] Home saved — positions reset to 0.")
                except Exception as e:
                    print(f"[Controller] write_home_offsets error: {e}")

            # 4. Hand latest state to the viz thread; drop if the previous frame
            # hasn't been consumed yet (Rerun backed up → never stall the loop).
            if self.visualizers:
                try:
                    self._viz_queue.put_nowait(self._state)
                except queue.Full:
                    pass

            # 5. Timing / debug prints (before pacing sleep)
            now = time.monotonic()

            if self.log_loop and now - self._last_stats_print >= self._STATS_INTERVAL:
                self._print_stats(now)

            # 6. Periodically print MIT state (only when --log-mit)
            if self.log_mit and now - self._last_pos_print >= 2.0:
                sids = sorted(self._state.servo_positions)
                if sids:
                    header = f"{'id':>3}  {'pos°':>8}  {'vel r/s':>8}  {'torq Nm':>8}  {'temp°C':>7}  {'err':>4}"
                    print(f"[MIT]\n{header}")
                    for sid in sids:
                        p = math.degrees(self._state.servo_positions.get(sid, 0.0))
                        v = self._state.motor_velocities.get(sid, 0.0)
                        t = self._state.motor_torques.get(sid, 0.0)
                        tmp = self._state.motor_temperatures.get(sid, 0)
                        err = self._state.motor_errors.get(sid, 0)
                        print(f"  {sid:>3}  {p:>8.2f}  {v:>8.3f}  {t:>8.3f}  {tmp:>7}  {err:>4}")
                self._last_pos_print = now

            # 7. Yield to motor TX, sensor, and receive tasks.
            # Run 4× motor_update_hz so commands are always fresh before the next TX tick.
            await asyncio.sleep(1.0 / (LOOP_LIMITS.motor_update_hz * 4))
            self._loop_times.append(time.monotonic() - t0)

    def _viz_worker(self) -> None:
        """Daemon thread: pull states from the queue and drive all visualizers."""
        while True:
            state = self._viz_queue.get()
            if state is None:
                break
            for viz in self.visualizers:
                try:
                    viz.update(state)
                except Exception as e:
                    print(f"[Controller] Visualizer {viz.name} error: {e}")

    def _print_stats(self, now: float) -> None:
        """Print loop timing stats and reset the accumulator.

        Samples are full wall-clock periods between iterations (including pacing
        sleep), so the printed Hz matches the effective control tick rate.
        """
        times = self._loop_times
        if not times:
            return
        n = len(times)
        mean_ms = sum(times) / n * 1000
        min_ms  = min(times) * 1000
        max_ms  = max(times) * 1000
        hz      = 1000.0 / mean_ms if mean_ms > 0 else 0.0
        print(f"[loop] {hz:.1f} Hz  (min {min_ms:.0f} ms  mean {mean_ms:.0f} ms  max {max_ms:.0f} ms)  n={n}")
        self._loop_times = []
        self._last_stats_print = now

    async def _shutdown(self) -> None:
        self._running = False
        print("\n[Controller] Shutting down...")
        self.scheme.on_stop()
        # Signal the viz worker and wait briefly. If Rerun's channel is backed
        # up the thread may still be blocked in rr.log(); the daemon flag ensures
        # it won't prevent process exit even if join times out.
        self._viz_queue.put(None)
        self._viz_thread.join(timeout=2.0)
        for viz in self.visualizers:
            viz.on_stop()
        await self.backend.disable_torques()
        await self.backend.disconnect()
        print("[Controller] Done.")
