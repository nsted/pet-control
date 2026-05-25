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
from typing import Callable, Optional

from petctl.config import LOOP_LIMITS
from petctl.perception.contact import ContactType
from petctl.power_manager import PowerManager
from petctl.protocols import ControlScheme, RobotBackend, Visualizer
from petctl.types import RobotState, ServoCommand, TouchEvent, TouchSummary

logger = logging.getLogger(__name__)


# Budge and twist are the same passive-motion gesture at different travel thresholds.
# Transitions between them don't start a new gesture — only entry from "none" does.
_PASSIVE_MOTION_FAMILY: frozenset[str] = frozenset({"budge", "twist"})

# Downgrade hysteresis: how many consecutive frames at a lower level are required
# before committing a downgrade.  Upgrades are always immediate.
_DOWNGRADE_GRACE_FRAMES: int = 8  # ~265ms at 30Hz

# Contact levels for hysteresis comparison (higher = more specific).
_CONTACT_LEVEL: dict[str, int] = {
    "none":     0,
    "budge":    1,
    "touch":    2,
    "hold":     3,
    "squeeze":  4,
    "restrict": 5,
    "twist":    6,
    "wrench":   7,
    "stroke":   8,
}

# Gesture lifecycle thresholds for the touch emitter.
_UPDATE_INTERVAL_S: float = 0.5   # interval between "running" status updates
_MERGE_GAP_S: float = 0.25        # merge a new gesture if the gap since last end ≤ this
_END_HOLD_S: float = 0.40         # hold before emitting "complete" — bridges sensor gaps
_MIN_GESTURE_DURATION_S: float = 0.22  # suppress gestures shorter than this
_TOUCH_LOG_FLUSH_S: float = 0.35  # flush buffered "complete" as [end] after this silence


def _event_level(event: TouchEvent) -> int:
    if event.stroke is not None:
        return _CONTACT_LEVEL["stroke"]
    if event.contact is not None:
        return _CONTACT_LEVEL.get(event.contact.contact_type.value, 1)
    return _CONTACT_LEVEL["none"]


class _TouchProcessor:
    """Owns the touch detectors and runs them once per tick.

    Called by the Controller immediately after get_state(). The result is
    attached to state.touch so schemes, visualizers, and _TouchLogger all
    read from the same computed value rather than each running their own detectors.

    Hysteresis: upgrades (touch → hold → squeeze/restrict/wrench) are immediate.
    Downgrades require _DOWNGRADE_GRACE_FRAMES consecutive frames at the lower
    level before being committed.  This suppresses sub-200ms oscillations at
    hold/touch and stroke/touch boundaries caused by qualifying-blob flicker.
    """

    def __init__(self) -> None:
        from petctl.perception.contact import ContactClassifier, ContactReading
        from petctl.perception.stroke import HoldDetector, StrokeDetector, qualifying_contact
        self._stroke = StrokeDetector()
        self._hold = HoldDetector()
        self._clf = ContactClassifier()
        self._qualifying_contact = qualifying_contact
        self._ContactReading = ContactReading
        self._committed: TouchEvent = TouchEvent()
        self._downgrade_count: int = 0

    def update(self, state: RobotState) -> TouchEvent:
        raw = self._detect(state)
        if _event_level(raw) < _event_level(self._committed):
            self._downgrade_count += 1
            if self._downgrade_count < _DOWNGRADE_GRACE_FRAMES:
                return self._committed  # hold the higher state
        else:
            self._downgrade_count = 0
        self._committed = raw
        return self._committed

    def _detect(self, state: RobotState) -> TouchEvent:
        stroke = self._stroke.update(state)
        if stroke is not None:
            # Flush hold velocity window so hold fires promptly after stroke ends.
            self._hold.reset()
            self._clf.reset()
            return TouchEvent(stroke=stroke)

        hold = self._hold.update(state)
        if hold is None:
            motion = self._clf.classify_no_hold(state)
            if motion is not None:
                # Motor motion (BUDGE or TWIST) takes priority over bare contact —
                # incidental sensor activation from movement is not a real touch.
                # BUDGE is suppressed only when classify() has a qualifying hold blob.
                return TouchEvent(contact=motion)
            centroid, side = self._qualifying_contact(state)
            if centroid is not None:
                return TouchEvent(contact=self._ContactReading(
                    contact_type=ContactType.TOUCH,
                    centroid=centroid,
                    side=side,
                ))
            if not self._clf.has_active_motion():
                self._clf.reset()  # no motion, no contact — full reset
            return TouchEvent()

        contact = self._clf.classify(hold, state)
        return TouchEvent(hold=hold, contact=contact)


class _TouchLogger:
    """Logs touch events to console in the same format sent to the LLM.

    Offsets are time since the last WS reset (epoch_fn()).  Buffers
    "complete" events briefly: if another touch follows within
    _TOUCH_LOG_FLUSH_S the buffered event is emitted as [ongoing]; after
    silence it flushes as [end].
    """

    def __init__(self, epoch_fn: Callable[[], float]) -> None:
        self._epoch_fn = epoch_fn
        self._pending: tuple[TouchSummary, float] | None = None  # (summary, offset)

    def tick(self, now: float) -> None:
        """Call each controller tick to flush stale pending events."""
        if self._pending is not None and now - self._pending[0].timestamp >= _TOUCH_LOG_FLUSH_S:
            s, offset = self._pending
            logger.info("[TOUCH  ] +%.1fs: %s", offset, s.describe())
            self._pending = None

    def on_summary(self, summary: TouchSummary) -> None:
        if summary.touch_type == "none":
            return

        offset = summary.timestamp - self._epoch_fn()

        # New event arrived — flush pending complete as [ongoing] (it wasn't truly the end)
        if self._pending is not None:
            s, pending_offset = self._pending
            logger.info("[TOUCH  ] +%.1fs: %s", pending_offset, s.describe(status_override="running"))
            self._pending = None

        if summary.status == "complete":
            self._pending = (summary, offset)
        else:
            logger.info("[TOUCH  ] +%.1fs: %s", offset, summary.describe())


class _TouchEventEmitter:
    """Stages touch gestures for the LLM and logger with lifecycle tracking.

    Gestures shorter than _MIN_GESTURE_DURATION_S are suppressed entirely.
    Longer gestures emit status="started" once they pass that threshold,
    status="running" every _UPDATE_INTERVAL_S, and status="complete" on end.

    End-hold: when the gesture level drops (sensor gap, inter-module dead zone,
    brief velocity dip) the session is held open for _END_HOLD_S before "complete"
    is emitted.  If the gesture recovers within that window the hold is cancelled
    and the session continues uninterrupted.  This bridges slow-stroke inter-module
    gaps without the layer interactions of merge-after-end.

    _MERGE_GAP_S still applies as a fallback: if the hold expires and a new contact
    follows quickly, the sessions are stitched together.

    Called every tick; acts on time as well as type transitions.  The queue
    is bounded — summaries are dropped (never blocking) when the consumer is slow.
    """

    def __init__(self, queue: asyncio.Queue) -> None:
        self._queue = queue
        self._on_emit_cbs: list[Callable[[TouchSummary], None]] = []

        self._sess_type: str = "none"
        self._sess_start: float = 0.0
        self._sess_touch: TouchEvent | None = None
        self._sess_staged: bool = False   # True once "started" has been emitted
        self._sess_last_t: float = 0.0   # time of last staged emission
        self._end_hold_since: float | None = None  # when the end-hold timer started

        # State saved when a session ends, for continuity merge checks.
        self._prev_end_type: str = "none"
        self._prev_end_t: float = 0.0
        self._prev_end_start: float = 0.0
        self._prev_end_staged: bool = False
        self._prev_end_direction: str | None = None

    def set_logger(self, callback: Callable[[TouchSummary], None]) -> None:
        self._on_emit_cbs.append(callback)

    def add_callback(self, callback: Callable[[TouchSummary], None]) -> None:
        self._on_emit_cbs.append(callback)

    def update(self, state: RobotState) -> None:
        touch = state.touch
        if touch is None:
            return
        now = state.timestamp
        curr = self._touch_type(touch)

        if self._sess_type == "none":
            if curr == "none":
                return
            # Starting a new session — check whether to merge with the previous one.
            # Strokes with opposite directions are never merged (direction change = new stroke).
            curr_dir = touch.stroke.direction if touch.stroke is not None else None
            same_dir = (
                curr_dir is None
                or self._prev_end_direction is None
                or curr_dir == self._prev_end_direction
            )
            if (
                self._prev_end_type != "none"
                and now - self._prev_end_t <= _MERGE_GAP_S
                and (
                    _CONTACT_LEVEL.get(curr, 0) >= _CONTACT_LEVEL.get(self._prev_end_type, 0)
                    or self._prev_end_type == "stroke"
                )
                and same_dir
            ):
                self._sess_start = self._prev_end_start
                self._sess_staged = self._prev_end_staged
                self._sess_last_t = self._prev_end_t
            else:
                self._sess_start = now
                self._sess_staged = False
                self._sess_last_t = now
            self._sess_type = curr
            self._prev_end_type = "none"  # consume merge window
            self._end_hold_since = None

        elif curr in _PASSIVE_MOTION_FAMILY and self._sess_type in _PASSIVE_MOTION_FAMILY:
            self._sess_type = curr  # budge↔twist: same passive-motion gesture
            self._end_hold_since = None

        elif _CONTACT_LEVEL.get(curr, 0) >= _CONTACT_LEVEL.get(self._sess_type, 0):
            self._sess_type = curr  # promoted (e.g. hold → squeeze): continue session
            self._end_hold_since = None

        else:
            # Contact dropped below session level — hold before committing the end.
            if self._end_hold_since is None:
                self._end_hold_since = now
            if now - self._end_hold_since < _END_HOLD_S:
                return  # within hold: keep session alive, don't update touch or emit
            # Hold expired — finalize the session.
            self._on_contact_end(now, emit_none=(curr == "none"))
            self._end_hold_since = None
            if curr != "none":
                self._sess_type = curr
                self._sess_start = now
                self._sess_staged = False
                self._sess_last_t = now
            return

        self._sess_touch = touch

        elapsed = now - self._sess_start
        if not self._sess_staged and elapsed >= _MIN_GESTURE_DURATION_S:
            self._sess_staged = True
            self._sess_last_t = now
            self._emit(self._make_summary("started", now, elapsed))
        elif self._sess_staged and now - self._sess_last_t >= _UPDATE_INTERVAL_S:
            self._sess_last_t = now
            self._emit(self._make_summary("running", now, elapsed))

    def _on_contact_end(self, now: float, *, emit_none: bool = True) -> None:
        if self._sess_type == "none":
            return
        elapsed = now - self._sess_start
        if elapsed < _MIN_GESTURE_DURATION_S:
            self._sess_type = "none"
            self._sess_touch = None
            self._sess_staged = False
            return
        # Catch the race: gesture passed _MIN_GESTURE_DURATION_S between ticks but
        # contact ended before the next update() call emitted "started".
        if not self._sess_staged and elapsed >= _MIN_GESTURE_DURATION_S:
            self._sess_staged = True
            self._emit(self._make_summary("started", now, elapsed))
        # Only emit "complete" (→ [end]) when "started" was emitted — prevents
        # orphan [end] events from short sessions that were never staged.
        was_staged = self._sess_staged
        if was_staged:
            self._emit(self._make_summary("complete", now, elapsed))
        # Always update merge-window state so quick-follow gestures can rejoin
        # even if this session produced no output.
        self._prev_end_type = self._sess_type
        self._prev_end_t = now
        self._prev_end_start = self._sess_start
        self._prev_end_staged = self._sess_staged
        self._prev_end_direction = (
            self._sess_touch.stroke.direction
            if self._sess_touch is not None and self._sess_touch.stroke is not None
            else None
        )
        self._sess_type = "none"
        self._sess_touch = None
        self._sess_staged = False
        if was_staged and emit_none:
            self._emit_none(now)

    def _make_summary(self, status: str, now: float, elapsed: float) -> TouchSummary:
        return TouchSummary.from_touch_event(self._sess_touch, now, elapsed, status=status)  # type: ignore[arg-type]

    def _emit(self, summary: TouchSummary) -> None:
        try:
            self._queue.put_nowait(summary)
        except asyncio.QueueFull:
            pass
        for cb in self._on_emit_cbs:
            cb(summary)

    def _emit_none(self, now: float) -> None:
        self._emit(TouchSummary(
            touch_type="none",
            timestamp=now,
            duration=0.0,
            intensity=0.0,
            centroid=None,
            side="",
            modules=[],
            velocity=None,
            direction=None,
            confidence=None,
            pressure_peak=None,
            torque_peak=None,
        ))

    @staticmethod
    def _touch_type(touch: TouchEvent) -> str:
        if touch.stroke is not None:
            return "stroke"
        if touch.contact is not None:
            return touch.contact.contact_type.value
        return "none"


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
        self.power_manager = PowerManager()
        self._touch_processor = _TouchProcessor()
        self._power_reset_requested: bool = False

        # Async queue of TouchSummary events, emitted on type transitions.
        # Bounded so a slow consumer never stalls the control loop.
        # Read from this in any async task to receive touch events for LLM or other consumers.
        self.touch_events: asyncio.Queue[TouchSummary] = asyncio.Queue(maxsize=64)
        self._touch_emitter = _TouchEventEmitter(self.touch_events)
        self._ws_epoch: float = time.monotonic()
        self._was_connected: bool = False
        self._touch_logger: _TouchLogger | None = None
        if log_touch:
            self._touch_logger = _TouchLogger(epoch_fn=lambda: self._ws_epoch)
            self._touch_emitter.set_logger(self._touch_logger.on_summary)

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

    def register_touch_callback(self, callback: Callable[[TouchSummary], None]) -> None:
        """Register a callback to receive TouchSummary events from the emitter."""
        self._touch_emitter.add_callback(callback)

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
        self._ws_epoch = time.monotonic()
        self._was_connected = True

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
            self._state.servo_commanded_positions = dict(self._slew_last_sent_rad)
            if self._state.connected and not self._was_connected:
                self._ws_epoch = self._state.timestamp
            self._was_connected = self._state.connected
            self._state.is_behavior_active = self.scheme.is_active()
            self._state.touch = self._touch_processor.update(self._state)
            self._touch_emitter.update(self._state)
            if self._touch_logger is not None:
                self._touch_logger.tick(self._state.timestamp)

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
                    # Re-enter MIT mode for all servos — hardware is still in exit-motor-mode
                    # after a thermal/compliance disable. enable_motor also resets ramp state,
                    # preventing a position snap on the first command post-recovery.
                    for _mid in sorted(self._state.active_servo_ids):
                        try:
                            await self.backend.enable_motor(_mid)
                        except Exception as e:
                            logger.error("[Controller] enable_motor(%d) after reset error: %s", _mid, e)
                else:
                    logger.warning("[Controller] Power manager reset denied: conditions not safe.")

            # 2. Run control scheme
            try:
                commands = self.scheme.update(self._state)
            except Exception as e:
                logger.error("[Controller] Scheme '%s' error: %s", self.scheme.name, e)
                commands = []

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
