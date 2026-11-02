"""
RemoteBackend — a Backend whose hardware lives in a separate driver process.

The brain (Controller + motions) uses this exactly like RobotBackend or
MockBackend; underneath, state arrives on ``pet/state``, commands leave on
``pet/cmd`` and the non-streaming calls are request/reply services
(see ``petctl.wire`` and ``petctl.driver``).

Safety is NOT applied here — the driver's SafetyBackend owns it.

Liveness: if no state arrives for ``state_timeout_s`` the driver is presumed
dead. ``get_state()`` then returns the last state with ``connected=False`` (as
the Backend contract requires), commands are not published, and service calls
return immediately instead of blocking — so a Controller shutting down never
hangs on a dead driver (the driver's own watchdog winds the motors down).
Reception resumes automatically if the driver comes back.

Behavior-neutrality notes:
  * ``get_state()`` is non-blocking and returns the latest cached message.
    Every call returns a fresh shallow copy, because the Controller writes
    derived fields (gesture, is_motion_active, servo_commanded_positions)
    onto the object it receives.
  * ``dt`` is time since the previous ``get_state()`` call — what the in-process
    backends report and what the Controller's slew filter was tuned against —
    not time since the last message.
  * The Controller loop (~200 Hz) and the driver's state rate (also 4 ×
    motor_update_hz) are not locked, so a given state may be returned more
    than once or skipped. Logged for SHAPE_LAYER 1.5.
"""

from __future__ import annotations

import asyncio
import dataclasses
import logging
import time
from typing import Any, Callable, Optional

from petctl import wire
from petctl.config import LIVENESS_LIMITS
from petctl.protocols import Backend
from petctl.transport import Bus
from petctl.types import RobotState, ServoCommand

logger = logging.getLogger(__name__)

_FAST_TIMEOUT_S = 5.0
_SLOW_TIMEOUT_S = 20.0   # calibrations and EEPROM writes; the driver waits up to 15 s


class RemoteBackend(Backend):
    """Backend proxy for a driver reachable over a ``Bus``."""

    def __init__(
        self,
        bus: Bus,
        connect_timeout_s: float = 10.0,
        *,
        state_timeout_s: Optional[float] = None,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self._bus = bus
        self._connect_timeout_s = connect_timeout_s
        self._state_timeout_s = (
            LIVENESS_LIMITS.brain_state_timeout_s if state_timeout_s is None else state_timeout_s
        )
        self._clock = clock
        # Arrival time of the newest state, stamped on the bus thread. The event loop can
        # be blocked for over a second (e.g. the Rerun viewer starting); staleness must
        # reflect when the driver last spoke, not when the loop got round to the message.
        self._latest_rx_t = 0.0
        self._driver_lost = False
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._latest: Optional[RobotState] = None
        self._first_state: Optional[asyncio.Event] = None
        self._last_get_t: Optional[float] = None
        self._open = False
        self.states_received = 0

    # ------------------------------------------------------------------
    # Connection
    # ------------------------------------------------------------------

    async def connect(self) -> bool:
        """Subscribe to the driver's state stream; True once the first state arrives."""
        self._loop = asyncio.get_running_loop()
        self._first_state = asyncio.Event()
        self._bus.subscribe(wire.KEY_STATE, self._on_state_bytes)
        self._open = True
        try:
            await asyncio.wait_for(self._first_state.wait(), timeout=self._connect_timeout_s)
        except asyncio.TimeoutError:
            logger.error("[Remote] No state from the driver within %.0fs — is `petctl driver` running?",
                         self._connect_timeout_s)
            return False
        logger.info("[Remote] Connected to driver.")
        return True

    async def disconnect(self) -> None:
        self._open = False
        self._bus.close()

    @property
    def is_connected(self) -> bool:
        return self._open and not self._is_stale() and self._latest is not None and self._latest.connected

    def _is_stale(self) -> bool:
        """True if no state has arrived within state_timeout_s (or ever)."""
        return self._latest is None or self._clock() - self._latest_rx_t > self._state_timeout_s

    # ------------------------------------------------------------------
    # State in / commands out
    # ------------------------------------------------------------------

    def _on_state_bytes(self, data: bytes) -> None:
        """Bus thread: decode, then hand over to the event loop."""
        try:
            state = wire.decode_state(data)
        except Exception:
            logger.exception("[Remote] bad state message")
            return
        self._latest_rx_t = self._clock()
        loop = self._loop
        if loop is not None:
            loop.call_soon_threadsafe(self._set_state, state)

    def _set_state(self, state: RobotState) -> None:
        self._latest = state
        self.states_received += 1
        if self._first_state is not None:
            self._first_state.set()

    async def get_state(self) -> RobotState:
        now = time.monotonic()
        dt = 0.0 if self._last_get_t is None else now - self._last_get_t
        self._last_get_t = now
        if self._latest is None:
            return RobotState.empty()
        if self._is_stale():
            if not self._driver_lost:
                self._driver_lost = True
                logger.error("[Remote] No state from the driver for %.1fs — reporting disconnected.",
                             self._state_timeout_s)
            return dataclasses.replace(self._latest, connected=False, dt=dt)
        if self._driver_lost:
            self._driver_lost = False
            logger.info("[Remote] Driver is back.")
        return dataclasses.replace(self._latest, dt=dt)

    async def send_commands(self, commands: list[ServoCommand]) -> None:
        if not commands or not self._open or self._is_stale():
            return
        self._bus.publish(wire.KEY_COMMANDS, wire.encode_commands(commands, sent_at=time.monotonic()))

    # ------------------------------------------------------------------
    # Services
    # ------------------------------------------------------------------

    async def _service(self, name: str, timeout: float = _FAST_TIMEOUT_S, default: Any = None, **args: Any) -> Any:
        """Call a driver service. Returns ``default`` (and logs) if the driver is unreachable."""
        if self._is_stale():
            logger.error("[Remote] Driver unreachable; skipping %s().", name)
            return default
        loop = asyncio.get_running_loop()
        try:
            raw = await loop.run_in_executor(
                None, self._bus.request, wire.service_key(name), wire.encode_request(name, **args), timeout
            )
        except TimeoutError:
            logger.error("[Remote] %s() timed out after %.0fs.", name, timeout)
            return default
        return wire.decode_reply(raw)   # a remote error still raises RuntimeError

    async def disable_torques(self) -> None:
        await self._service("disable_torques")

    async def disable_motor(self, motor_id: int) -> None:
        await self._service("disable_motor", motor_id=motor_id)

    async def enable_motor(self, motor_id: int) -> None:
        await self._service("enable_motor", motor_id=motor_id)

    async def write_home_offsets(self) -> None:
        await self._service("write_home_offsets", timeout=_SLOW_TIMEOUT_S)

    def set_stagger(self, motor_id: int, delay_s: float) -> None:
        """Fire-and-forget (the Backend method is synchronous)."""
        loop = self._loop
        if loop is None:
            return

        async def _send() -> None:
            try:
                await self._service("set_stagger", motor_id=motor_id, delay_s=delay_s)
            except Exception as e:
                logger.warning("[Remote] set_stagger(%d) failed: %s", motor_id, e)

        loop.call_soon_threadsafe(lambda: asyncio.ensure_future(_send()))

    async def calibrate_touch(self) -> bool:
        return bool(await self._service("calibrate_touch", timeout=_SLOW_TIMEOUT_S, default=False))

    async def set_touch_threshold(
        self,
        touch12: int,
        release12: int,
        touch3: int | None = None,
        release3: int | None = None,
    ) -> bool:
        return bool(await self._service(
            "set_touch_threshold", timeout=_SLOW_TIMEOUT_S, default=False,
            touch12=touch12, release12=release12, touch3=touch3, release3=release3,
        ))

    async def power_reset(self) -> bool:
        return bool(await self._service("power_reset", default=False))
