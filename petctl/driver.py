"""
Driver — hosts a Backend on the message bus.

The driver owns the hardware side of petctl: a ``SafetyBackend``-wrapped
backend (RobotBackend, MockBackend, later a MuJoCo backend). A brain process
(Controller + motions) talks to it through ``RemoteBackend`` using the
formats in ``petctl.wire``:

  publishes  pet/state       RobotState hardware facts, at the Controller's old
                             tick rate (4 × motor_update_hz) so the PowerManager
                             sees the same sample cadence as before the split
  subscribes pet/cmd         ServoCommand batches, applied in order
  serves     pet/svc/<name>  enable_motor, disable_motor, disable_torques, ...

The Backend must already include safety (``SafetyBackend``); the driver adds none.

Command watchdog: if no command batch arrives for ``cmd_timeout_s`` (a dead or
hung brain), the driver ramps kp / kd / torque_ff of the last commands linearly
to zero over ``ramp_s``, holding the last position targets — never a hard cut.
A new command batch resumes normal operation immediately.
"""

from __future__ import annotations

import asyncio
import logging
import signal
import time
from typing import Any, Callable, Optional

from petctl import wire
from petctl.config import LIVENESS_LIMITS, LOOP_LIMITS, POWER_BUDGET
from petctl.latency import SampleStats
from petctl.protocols import Backend
from petctl.transport import Bus
from petctl.types import ServoCommand

logger = logging.getLogger(__name__)

_COMMAND_QUEUE_MAX = 256
_SERVICE_TIMEOUT_S = 15.0   # RemoteBackend waits up to 20 s for the slow ones


class Driver:
    """Runs one Backend and exposes it on a Bus."""

    def __init__(
        self,
        backend: Backend,
        bus: Bus,
        *,
        cmd_timeout_s: Optional[float] = None,
        ramp_s: Optional[float] = None,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self.backend = backend
        self.bus = bus
        self._cmd_timeout_s = LIVENESS_LIMITS.driver_cmd_timeout_s if cmd_timeout_s is None else cmd_timeout_s
        self._ramp_s = LIVENESS_LIMITS.safe_hold_ramp_s if ramp_s is None else ramp_s
        self._clock = clock
        self._last_cmd_t: Optional[float] = None
        self._last_cmds: dict[int, ServoCommand] = {}
        self._ramp_start_t = 0.0
        # "ok" -> "ramping" -> "held"; any new command batch returns to "ok".
        self.watchdog_state = "ok"
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._stop: Optional[asyncio.Event] = None
        self._commands: Optional[asyncio.Queue[bytes]] = None
        # Stage 6 instrumentation (same-machine monotonic clock on both ends).
        self.cmd_latency = SampleStats()        # brain send -> just before driver applies (incl. queueing)
        self.cmd_interarrival = SampleStats()   # gap between command batches arriving
        self.commands_dropped = 0
        self.commands_applied = 0
        self.states_published = 0

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    async def run(self) -> None:
        """Connect the backend, serve the bus until stop() or SIGINT/SIGTERM, then shut down."""
        self._loop = asyncio.get_running_loop()
        self._stop = asyncio.Event()
        self._commands = asyncio.Queue(maxsize=_COMMAND_QUEUE_MAX)

        logger.info("[Driver] Connecting via %s...", type(self.backend).__name__)
        if not await self.backend.connect():
            logger.error("[Driver] Backend failed to connect. Exiting.")
            return
        if self.backend.is_real_hardware and POWER_BUDGET.max_bus_current_a <= 2.0:
            logger.warning(
                "[Driver] Real robot running with dev power budget (%.1fA) — "
                "set max_bus_current_a in config.py for production",
                POWER_BUDGET.max_bus_current_a,
            )

        self.bus.subscribe(wire.KEY_COMMANDS, self._on_command_bytes)
        for name in wire.SERVICES:
            self.bus.serve(wire.service_key(name), self._make_service_handler(name))

        registered: list[signal.Signals] = []
        for sig in (signal.SIGINT, signal.SIGTERM):
            try:
                self._loop.add_signal_handler(sig, self.stop)
                registered.append(sig)
            except NotImplementedError:
                pass

        logger.info("[Driver] Serving. Press Ctrl-C to stop.")
        cmd_task = asyncio.create_task(self._command_loop())
        try:
            await self._state_loop()
        finally:
            cmd_task.cancel()
            for sig in registered:
                try:
                    self._loop.remove_signal_handler(sig)
                except Exception:
                    pass
            await self._shutdown()

    def stop(self) -> None:
        """Ask run() to return (safe to call from the event loop thread)."""
        if self._stop is not None:
            self._stop.set()

    async def _shutdown(self) -> None:
        logger.info(
            "[Driver] Shutting down (published %d states, applied %d command batches, dropped %d).",
            self.states_published, self.commands_applied, self.commands_dropped,
        )
        logger.info("[Driver] %s", self.cmd_latency.format("command latency"))
        logger.info("[Driver] %s", self.cmd_interarrival.format("command inter-arrival"))
        try:
            await self.backend.disable_torques()
        finally:
            await self.backend.disconnect()
            self.bus.close()
        logger.info("[Driver] Done.")

    # ------------------------------------------------------------------
    # State out
    # ------------------------------------------------------------------

    async def _state_loop(self) -> None:
        assert self._stop is not None
        period = 1.0 / (LOOP_LIMITS.motor_update_hz * 4)
        while not self._stop.is_set():
            try:
                state = await self.backend.get_state()
                self.bus.publish(wire.KEY_STATE, wire.encode_state(state))
                self.states_published += 1
                await self._watchdog_tick()
            except Exception:
                logger.exception("[Driver] state loop error")
            await asyncio.sleep(period)

    # ------------------------------------------------------------------
    # Commands in
    # ------------------------------------------------------------------

    def _on_command_bytes(self, data: bytes) -> None:
        """Bus thread: hand the payload to the event loop."""
        self.cmd_interarrival.add_arrival(time.monotonic())
        loop = self._loop
        if loop is not None:
            loop.call_soon_threadsafe(self._enqueue_command, data)

    def _enqueue_command(self, data: bytes) -> None:
        assert self._commands is not None
        if self._commands.full():
            # Stale commands are worthless — drop the oldest, keep the newest.
            self._commands.get_nowait()
            self.commands_dropped += 1
            if self.commands_dropped in (1, 10, 100) or self.commands_dropped % 1000 == 0:
                logger.warning("[Driver] command queue full; dropped %d so far", self.commands_dropped)
        self._commands.put_nowait(data)

    async def _command_loop(self) -> None:
        assert self._commands is not None
        while True:
            data = await self._commands.get()
            try:
                commands, sent_at = wire.decode_commands(data)
                if sent_at is not None:
                    self.cmd_latency.add(time.monotonic() - sent_at)
                self._note_commands(commands)
                await self.backend.send_commands(commands)
                self.commands_applied += 1
            except Exception:
                logger.exception("[Driver] failed to apply command batch")

    # ------------------------------------------------------------------
    # Command watchdog
    # ------------------------------------------------------------------

    def _note_commands(self, commands: list[ServoCommand]) -> None:
        """Record the latest command per servo and clear any watchdog ramp."""
        for c in commands:
            self._last_cmds[c.servo_id] = c
        self._last_cmd_t = self._clock()
        if self.watchdog_state != "ok":
            logger.info("[Driver] Commands resumed — watchdog cleared.")
            self.watchdog_state = "ok"

    async def _watchdog_tick(self) -> None:
        """Called every state tick: start, advance or finish the safe-hold ramp."""
        if self._last_cmd_t is None or not self._last_cmds:
            return  # no brain has commanded yet; nothing to wind down
        now = self._clock()
        if self.watchdog_state == "ok":
            if now - self._last_cmd_t <= self._cmd_timeout_s:
                return
            self.watchdog_state = "ramping"
            self._ramp_start_t = now
            logger.warning(
                "[Driver] No commands for %.2fs — ramping gains to zero over %.2fs.",
                now - self._last_cmd_t, self._ramp_s,
            )
        if self.watchdog_state == "ramping":
            frac = 1.0 if self._ramp_s <= 0 else max(0.0, 1.0 - (now - self._ramp_start_t) / self._ramp_s)
            await self.backend.send_commands([
                ServoCommand(
                    servo_id=c.servo_id,
                    position=c.position,
                    velocity=c.velocity,
                    kp=c.kp * frac,
                    kd=c.kd * frac,
                    torque_ff=c.torque_ff * frac,
                )
                for c in self._last_cmds.values()
            ])
            if frac == 0.0:
                self.watchdog_state = "held"
                logger.warning("[Driver] Safe hold reached (gains zero). Waiting for the brain.")

    # ------------------------------------------------------------------
    # Services
    # ------------------------------------------------------------------

    def _make_service_handler(self, name: str):  # type: ignore[no-untyped-def]
        def handler(data: bytes) -> bytes:
            """Bus thread: run the call on the event loop and wait for it."""
            try:
                req_name, args = wire.decode_request(data)
                if req_name != name:
                    raise ValueError(f"request for {req_name!r} sent to {name!r}")
                assert self._loop is not None
                fut = asyncio.run_coroutine_threadsafe(self._call(name, args), self._loop)
                return wire.encode_reply(fut.result(timeout=_SERVICE_TIMEOUT_S))
            except Exception as e:
                logger.error("[Driver] service %s failed: %s", name, e)
                return wire.encode_reply(error=f"{type(e).__name__}: {e}")

        return handler

    async def _call(self, name: str, args: dict[str, Any]) -> Any:
        b = self.backend
        if name == "enable_motor":
            await b.enable_motor(args["motor_id"])
        elif name == "disable_motor":
            await b.disable_motor(args["motor_id"])
        elif name == "disable_torques":
            await b.disable_torques()
        elif name == "write_home_offsets":
            await b.write_home_offsets()
        elif name == "set_stagger":
            b.set_stagger(args["motor_id"], args["delay_s"])
        elif name == "calibrate_touch":
            return await b.calibrate_touch()
        elif name == "set_touch_threshold":
            return await b.set_touch_threshold(
                args["touch12"], args["release12"], args.get("touch3"), args.get("release3")
            )
        elif name == "power_reset":
            return await b.power_reset()
        else:  # pragma: no cover - SERVICES and this table are tested to agree
            raise ValueError(f"unhandled service {name!r}")
        return None
