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
import logging
import math
import queue
import signal
import threading
import time
from typing import Optional

from petctl.config import LOOP_LIMITS
from petctl.power_manager import PowerManager
from petctl.protocols import ControlScheme, RobotBackend, Visualizer
from petctl.types import RobotState, ServoCommand

logger = logging.getLogger(__name__)


_CONTACT_LABELS: dict[str, str] = {
    "stroke":   "STROKE  ",
    "hold":     "HOLD    ",
    "squeeze":  "SQUEEZE ",
    "restrict": "RESTRICT",
    "wrench":   "WRENCH  ",
}

_STROKE_END_GRACE_S: float = 0.45  # don't fire end until stroke absent this long


class _TouchLogger:
    """Runs stroke/hold/contact detection every tick and prints transitions."""

    def __init__(self) -> None:
        from petctl.perception.contact import ContactClassifier
        from petctl.perception.stroke import HoldDetector, StrokeDetector
        self._stroke = StrokeDetector()
        self._hold = HoldDetector()
        self._clf = ContactClassifier()
        self._prev: str | None = None
        self._prev_direction: str | None = None
        self._stroke_last_t: float | None = None
        self._stroke_start_centroid: float | None = None
        self._stroke_last_centroid: float | None = None
        self._stroke_last_speed: float = 0.0

    def update(self, state: RobotState) -> None:
        stroke = self._stroke.update(state)
        hold = self._hold.update(state)

        if stroke is not None:
            curr = "stroke"
            if curr != self._prev:
                self._stroke_start_centroid = stroke.centroid
                self._transition(self._prev, curr,
                    f"centroid={stroke.centroid:.1f}  side={stroke.side}")
            self._prev = curr
            self._prev_direction = stroke.direction
            self._stroke_last_t = state.timestamp
            self._stroke_last_centroid = stroke.centroid
            self._stroke_last_speed = stroke.speed
            self._clf.reset()
            self._hold.reset()  # flush stroke frames so hold fires promptly after stroke ends
            return

        # Grace period: a brief hold/squeeze or no-touch gap doesn't end a stroke.
        if self._prev == "stroke" and self._stroke_last_t is not None:
            if state.timestamp - self._stroke_last_t < _STROKE_END_GRACE_S:
                return

        if hold is None:
            if self._prev is not None:
                if self._prev == "stroke" and self._stroke_start_centroid is not None:
                    s, e = self._stroke_start_centroid, self._stroke_last_centroid
                    arrow = "→" if e > s else "←"
                    logger.info("[STROKE  ] end  %s from=%.1f to=%.1f  speed=%.1f mod/s",
                        arrow, s, e, self._stroke_last_speed)
                else:
                    logger.info("[%s] end", _CONTACT_LABELS.get(self._prev, self._prev.upper()[:8]))
            self._prev = None
            self._prev_direction = None
            self._stroke_last_t = None
            self._stroke_start_centroid = None
            self._stroke_last_centroid = None
            self._stroke_last_speed = 0.0
            self._clf.reset()
            return

        cr = self._clf.classify(hold, state)
        curr = cr.contact_type.value
        if curr != self._prev:
            prev_for_transition = self._prev
            if self._prev == "stroke" and self._stroke_start_centroid is not None:
                s, e = self._stroke_start_centroid, self._stroke_last_centroid
                arrow = "→" if e > s else "←"
                logger.info("[STROKE  ] end  %s from=%.1f to=%.1f  speed=%.1f mod/s",
                    arrow, s, e, self._stroke_last_speed)
                prev_for_transition = None
            self._stroke_start_centroid = None
            self._stroke_last_centroid = None
            self._stroke_last_speed = 0.0
            self._stroke_last_t = None
            blobs = " ".join(f"[{','.join(str(m) for m in b.modules)}]" for b in hold.q_blobs)
            servos = f"  servos={cr.affected_servos}" if cr.affected_servos else ""
            torque = f"  torque={cr.torque_peak:.2f}Nm" if cr.torque_peak else ""
            pressure = f"  pressure={cr.pressure_peak:.2f}" if cr.pressure_peak else ""
            self._transition(prev_for_transition, curr,
                f"blobs={blobs}  dur={hold.duration:.1f}s{servos}{torque}{pressure}")
        self._prev = curr

    def _transition(self, prev: str | None, curr: str, details: str) -> None:
        if prev is not None:
            logger.info("[%s] end", _CONTACT_LABELS.get(prev, prev.upper()[:8]))
        logger.info("[%s] start  %s", _CONTACT_LABELS.get(curr, curr.upper()[:8]), details)


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
        self._touch_logger: Optional[_TouchLogger] = _TouchLogger() if log_touch else None

        self.power_manager = PowerManager()
        self._power_reset_requested: bool = False

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

    def request_power_reset(self) -> None:
        """Ask the control loop to attempt a power-manager emergency reset.

        Thread-safe: may be called from the keyboard listener thread.
        Actual reset executes on the next control tick; conditions are validated then.
        """
        self._power_reset_requested = True

    async def run(self) -> None:
        """
        Connect to the backend and start the control loop.

        Runs until stop() is called or a KeyboardInterrupt is received.
        """
        logger.info("[Controller] Connecting via %s...", self.backend.__class__.__name__)
        ok = await self.backend.connect()
        if not ok:
            logger.error("[Controller] Backend failed to connect. Exiting.")
            return

        if self.limp:
            logger.info("[Controller] Limp mode: disabling torques — move joints freely by hand.")
            await self.backend.disable_torques()

        self._running = True
        self._event_loop = asyncio.get_running_loop()
        self._stop_event = asyncio.Event()

        # Activate scheme and visualizers
        self.scheme.on_start(self)
        for viz in self.visualizers:
            viz.on_start(self)

        logger.info("[Controller] Running. Press Ctrl-C to stop.")

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

            # 1b. Power management — evaluate protection conditions
            pm = self.power_manager
            pm.update(self._state, t0)
            disable_ids, is_global_emergency = pm.drain_disable_events()
            if is_global_emergency:
                try:
                    await self.backend.disable_torques()
                except Exception as e:
                    logger.error("[Controller] Emergency disable_torques error: %s", e)
            elif disable_ids:
                for _mid in disable_ids:
                    try:
                        await self.backend.disable_motor(_mid)
                    except Exception as e:
                        logger.error("[Controller] disable_motor(%d) error: %s", _mid, e)
            self._state.power_telemetry = pm.get_telemetry(self._state.battery_voltage_v)

            # Handle operator power-reset request
            if self._power_reset_requested:
                self._power_reset_requested = False
                if pm.operator_reset():
                    logger.info("[Controller] Power manager reset: system re-enabled.")
                else:
                    logger.warning("[Controller] Power manager reset denied: conditions not safe.")

            # 2. Run control scheme
            try:
                commands = self.scheme.update(self._state)
            except Exception as e:
                logger.error("[Controller] Scheme '%s' error: %s", self.scheme.name, e)
                commands = []

            if self._touch_logger is not None:
                self._touch_logger.update(self._state)

            # Let schemes reset the slew state for specific servos before this
            # tick's filter runs — needed when a stall reversal must take effect
            # immediately rather than waiting for the filter to unwind.
            _take_slew_resets = getattr(self.scheme, "take_slew_resets", None)
            if _take_slew_resets is not None:
                for _sid, _pos in _take_slew_resets().items():
                    self._slew_last_sent_rad[_sid] = _pos

            self._apply_slew_to_commands(commands)

            # 3. Send commands (unless dry run), filtered and scaled by PowerManager
            if not self.dry_run and commands:
                safe_commands = []
                for cmd in commands:
                    if not pm.is_motor_enabled(cmd.servo_id):
                        continue
                    scale = pm.get_compliance_scale(cmd.servo_id)
                    if scale != 1.0:
                        cmd.kp *= scale
                        cmd.kd *= scale
                        cmd.torque_ff *= scale
                    safe_commands.append(cmd)
                if safe_commands:
                    try:
                        await self.backend.send_commands(safe_commands)
                    except Exception as e:
                        logger.error("[Controller] Backend send_commands error: %s", e)

            # 3b. Save-home: write EEPROM offsets so current position reports as 0
            take_save_home = getattr(self.scheme, "take_save_home", None)
            if take_save_home is not None and take_save_home():
                logger.info("[Controller] Saving home offsets...")
                try:
                    await self.backend.write_home_offsets()
                    # Clear slew state so the filter doesn't hold pre-home positions.
                    self._slew_last_sent_rad.clear()
                    logger.info("[Controller] Home saved — positions reset to 0.")
                except Exception as e:
                    logger.error("[Controller] write_home_offsets error: %s", e)

            # 3c. Deactivate: exit MIT mode on all motors
            take_deactivate = getattr(self.scheme, "take_deactivate", None)
            if take_deactivate is not None and take_deactivate():
                logger.info("[Controller] Deactivating all motors...")
                try:
                    await self.backend.disable_torques()
                    logger.info("[Controller] All motors deactivated.")
                except Exception as e:
                    logger.error("[Controller] disable_torques error: %s", e)

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
                    logger.info("[MIT]\n%s", header)
                    for sid in sids:
                        p = math.degrees(self._state.servo_positions.get(sid, 0.0))
                        v = self._state.motor_velocities.get(sid, 0.0)
                        t = self._state.motor_torques.get(sid, 0.0)
                        tmp = self._state.motor_temperatures.get(sid, 0)
                        err = self._state.motor_err_codes.get(sid, 0)
                        logger.info("  %3d  %8.2f  %8.3f  %8.3f  %7s  %4s", sid, p, v, t, tmp, err)
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
                    logger.error("[Controller] Visualizer %s error: %s", viz.name, e)

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
        logger.info("[loop] %.1f Hz  (min %.0f ms  mean %.0f ms  max %.0f ms)  n=%d", hz, min_ms, mean_ms, max_ms, n)
        self._loop_times = []
        self._last_stats_print = now

    async def _shutdown(self) -> None:
        self._running = False
        logger.info("[Controller] Shutting down...")
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
        logger.info("[Controller] Done.")
