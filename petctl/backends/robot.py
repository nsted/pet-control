"""Robot backend for CubeMars GL40 II motors over SLCAN text WebSocket."""

from __future__ import annotations

import asyncio
import collections
import itertools
import logging
import os
import socket
import time
from typing import Optional, Sequence

import websockets

from petctl.config import LOOP_LIMITS, MOTOR_LIMITS, NUM_MODULES, SENSOR_LIMITS
from petctl.protocols import Backend as _BackendBase
from petctl.types import ImuReading, ModuleSensors, RobotState, ServoCommand

logger = logging.getLogger(__name__)

_WS_SETTLE_S: float = 0.05    # allow WS handshake + firmware ACK before first command
_SLCAN_SETTLE_S: float = 0.05  # allow Arduino to open CAN bus before pinging again

# Default connection parameters (also imported by cli.py to keep a single source of truth)
ROBOT_DEFAULT_HOST = "pet-robot.local"
ROBOT_DEFAULT_PORT = 8080


# Lines CAN→WebSocket may broadcast (see grapple-arduino head.ino). Not API
# ("<digit>:") and not MIT motor frames (handled via "t"/"T" + hex).
_SLCAN_WS_NOISE_FIRST = frozenset("SOCZzFNvV\x07")


def _ping_payload_ok(data: Optional[str]) -> bool:
    """
    True if `data` (text after first ':' in the API line) counts as a ping ACK.

    grapple-arduino `head.ino` calls `sendResponseWithReqId(requestId)` with no
    second argument, so the reply is literally \"<id>:\" with an empty body — not
    \"<id>:pong\". We accept either empty / whitespace-only or any payload
    containing \"pong\" (case-insensitive).
    """
    if data is None:
        return False
    stripped = data.strip().lower()
    return "pong" in stripped or stripped == ""



class RobotBackend(_BackendBase):
    """Backend for the real PET robot."""

    def __init__(
        self,
        host: str = ROBOT_DEFAULT_HOST,
        port: int = ROBOT_DEFAULT_PORT,
        calibrate_on_connect: bool = False,
        calibration_samples: int = 10,
        auto_reconnect: bool = True,
        reconnect_delay: float = 2.0,
        motor_ids: Optional[Sequence[int]] = None,
        fsr_offsets: Optional[dict[int, dict[str, float]]] = None,
    ) -> None:
        self.host = host
        self.port = port
        self.calibrate_on_connect = calibrate_on_connect
        self.calibration_samples = calibration_samples
        self.auto_reconnect = auto_reconnect
        self.reconnect_delay = reconnect_delay
        # Per-module FSR zero-pressure offsets: {module_id: {face: float}}.
        # Subtracted after normalization; values from scripts/calibrate_fsr.py.
        self._fsr_offsets: dict[int, dict[str, float]] = fsr_offsets or {}
        if motor_ids is None:
            self._configured_motor_ids: Optional[tuple[int, ...]] = None
        else:
            t = tuple(int(m) for m in motor_ids)
            self._configured_motor_ids = t if t else None

        self._ws: Optional[websockets.ClientConnection] = None
        self._receive_task: Optional[asyncio.Task] = None
        self._pending_requests: dict[int, asyncio.Future] = {}

        self._connected = False
        self._resolved_ip: Optional[str] = None
        # Start at 1 — some firmware treats request id 0 as invalid / broadcast.
        self._req_id_counter = itertools.count(1)

        self._discovered_modules: list[int] = []
        self._discovered_motors: list[int] = []

        self._motor_state: dict[int, dict[str, float]] = {}
        self._angle_offsets: dict[int, float] = {}
        # Last MIT absolute position / wall time for velocity feedforward.
        self._last_mit_abs_pos: dict[int, float] = {}
        self._last_mit_wall_s: dict[int, float] = {}
        # TX task double-buffer: pending (latest from control loop) and last sent (for poll).
        self._pending_frames: dict[int, str] = {}
        self._last_sent_frames: dict[int, str] = {}
        # Monotonic time of the last real position command for each motor.
        # Used to revert to zero-torque after LOOP_LIMITS.idle_hold_s of inactivity.
        self._last_command_time: dict[int, float] = {}
        self._motor_tx_task: Optional[asyncio.Task] = None
        self._module_poll_task: Optional[asyncio.Task] = None
        self._motor_poll_task: Optional[asyncio.Task] = None

        self._last_state = RobotState.empty()
        self._reconnecting = False
        self._reconnect_task: Optional[asyncio.Task] = None
        self._rx_unhandled_logged: int = 0
        self._rx_binary_logged: int = 0
        # Set by _try_bare_ping; receive loop completes it on a plain "pong" line.
        self._bare_reply_fut: Optional[asyncio.Future[str]] = None

        # Latest sensor data populated by _handle_sensor_push — updated at ~50 Hz.
        self._latest_sensors: Optional[dict] = None
        self._latest_sensor_ts: float = 0.0
        # Latest IMU data per module, populated by _handle_imu_push — updated at ~25 Hz.
        self._latest_imu: dict[int, ImuReading] = {}
        # Per-module, per-face sliding windows for cap moving average.
        # Keyed by module_id → {"left": [deque,...], "right": [...], "middle": [...]}.
        self._cap_filter: dict[int, dict[str, list[collections.deque]]] = {}
        self._latest_battery_current_raw: int = 0
        self._latest_battery_voltage_raw: int = 0
        self._disabled_motor_ids: set[int] = set()
        # Stagger: motor_id → monotonic time before which this motor's frame is held
        self._stagger_until: dict[int, float] = {}

        # Auto-recalibration: track how long each module has had cap active with no FSR.
        self._stuck_since: dict[int, float] = {}   # mod_id → monotonic time stuck started
        self._last_touch_recal_t: float = 0.0
        self._needs_touch_recal: bool = False

        # Monotonic timestamp of the last WS message received from the robot.
        # Used by _motor_tx_loop to detect silent TCP drops (no WS close frame).
        self._last_rx_time: float = 0.0

        self._ws_send_lock: asyncio.Lock = asyncio.Lock()

    # ------------------------------------------------------------------
    # Backend interface
    # ------------------------------------------------------------------

    async def connect(self) -> bool:
        ip = await self._resolve_host()
        if not ip:
            logger.error("[RobotBackend] Could not resolve %s", self.host)
            return False
        self._resolved_ip = ip
        return await self._connect_with_ip(ip)

    async def disconnect(self) -> None:
        self._connected = False
        if self._reconnect_task and not self._reconnect_task.done():
            self._reconnect_task.cancel()
            self._reconnect_task = None
        if self._motor_tx_task:
            self._motor_tx_task.cancel()
            self._motor_tx_task = None
        if self._module_poll_task:
            self._module_poll_task.cancel()
            self._module_poll_task = None
        if self._receive_task:
            self._receive_task.cancel()
            self._receive_task = None
        if self._ws:
            await self._ws.close()
            self._ws = None
        for fut in self._pending_requests.values():
            if not fut.done():
                fut.cancel()
        self._pending_requests.clear()
        if self._bare_reply_fut is not None and not self._bare_reply_fut.done():
            self._bare_reply_fut.cancel()
        self._bare_reply_fut = None
        self._last_mit_abs_pos.clear()
        self._last_mit_wall_s.clear()

    async def get_state(self) -> RobotState:
        if not self._connected:
            return RobotState(
                timestamp=time.monotonic(),
                sensors=self._last_state.sensors,
                servo_positions=self._last_state.servo_positions,
                motor_velocities=self._last_state.motor_velocities,
                motor_torques=self._last_state.motor_torques,
                battery_current_raw=self._last_state.battery_current_raw,
                battery_voltage_raw=self._last_state.battery_voltage_raw,
                active_modules=self._discovered_modules,
                active_servo_ids=set(self._discovered_motors),
                connected=False,
                dt=0.0,
            )
        try:
            # Use the most recent sensor data from the background _sensor_loop.
            # On startup (before the first sensor read completes) use empty sensors
            # rather than firing a competing request — _sensor_loop owns that channel.
            sensors = self._latest_sensors or {}
            battery_current_raw = self._latest_battery_current_raw
            battery_voltage_raw = self._latest_battery_voltage_raw

            now = time.monotonic()
            state = RobotState(
                timestamp=now,
                sensor_timestamp=self._latest_sensor_ts,
                sensors=sensors,
                servo_positions={
                    mid: data["pos"] - self._angle_offsets.get(mid, 0.0)
                    for mid, data in self._motor_state.items()
                },
                motor_velocities={mid: data["vel"] for mid, data in self._motor_state.items()},
                motor_torques={mid: data["torque"] for mid, data in self._motor_state.items()},
                motor_temperatures={mid: data.get("drive_temp", 0) for mid, data in self._motor_state.items()},
                motor_winding_temperatures={mid: data.get("motor_temp", 0) for mid, data in self._motor_state.items()},
                motor_err_codes={mid: data.get("err_code", 0) for mid, data in self._motor_state.items()},
                battery_current_raw=battery_current_raw,
                battery_voltage_raw=battery_voltage_raw,
                imu=dict(self._latest_imu),
                active_modules=self._discovered_modules,
                active_servo_ids=set(self._discovered_motors),
                connected=True,
                dt=now - self._last_state.timestamp,
            )
            self._last_state = state
            return state

        except Exception as e:
            logger.error("[RobotBackend] get_state error: %s", e)
            return RobotState(
                timestamp=time.monotonic(),
                sensors=self._last_state.sensors,
                servo_positions=self._last_state.servo_positions,
                motor_velocities=self._last_state.motor_velocities,
                motor_torques=self._last_state.motor_torques,
                battery_current_raw=self._last_state.battery_current_raw,
                battery_voltage_raw=self._last_state.battery_voltage_raw,
                active_modules=self._discovered_modules,
                active_servo_ids=set(self._discovered_motors),
                connected=False,
                dt=0.0,
            )

    async def send_commands(self, commands: list[ServoCommand]) -> None:
        if not self._connected:
            return
        now = time.monotonic()
        for cmd in commands:
            if cmd.position is None:
                continue
            sid = cmd.servo_id
            if sid in self._disabled_motor_ids:
                continue  # don't drift tracking state while motor is off
            p_target = cmd.position + self._angle_offsets.get(sid, 0.0)
            last_t = self._last_mit_wall_s.get(sid)
            phys = self._motor_state.get(sid)
            prev_last_p = self._last_mit_abs_pos.get(sid)

            if prev_last_p is None:
                # First command: seed from physical to avoid a snap on the first move.
                last_p = phys["pos"] if phys is not None else p_target
            elif phys is not None:
                # Anti-windup: clamp ramp state to within window of actual position.
                # During normal motion phys ≈ prev_last_p so clamp never fires.
                # During occlusion it saturates at physical ± window, bounding torque.
                w = LOOP_LIMITS.anti_windup_rad
                last_p = max(min(prev_last_p, phys["pos"] + w), phys["pos"] - w)
            else:
                last_p = prev_last_p
            if last_t is not None:
                # Cap dt at one motor-TX period so a gap (scheme swap, slow tick,
                # reconnect) never inflates max_step into a position snap.
                dt = max(min(now - last_t, 1.0 / LOOP_LIMITS.motor_update_hz), 1.0 / 120.0)
                max_step = LOOP_LIMITS.max_speed_rad_s * dt
                delta = p_target - last_p
                pos_rad = last_p + max(min(delta, max_step), -max_step)
                if cmd.velocity is not None:
                    vel = cmd.velocity
                else:
                    vel = (pos_rad - last_p) / dt
                vel = max(MOTOR_LIMITS.vel_min, min(MOTOR_LIMITS.vel_max, vel))
            else:
                pos_rad = last_p
                vel = cmd.velocity if cmd.velocity is not None else 0.0
            self._last_mit_abs_pos[sid] = pos_rad
            self._last_mit_wall_s[sid] = now
            self._pending_frames[sid] = _encode_mit_packet(sid, pos_rad, vel, cmd.kp, cmd.kd, cmd.torque_ff)

    def set_stagger(self, motor_id: int, delay_s: float) -> None:
        """Delay the next TX frame for motor_id by delay_s seconds."""
        self._stagger_until[motor_id] = time.monotonic() + delay_s

    @property
    def is_connected(self) -> bool:
        return self._connected

    @property
    def discovered_modules(self) -> list[int]:
        return list(self._discovered_modules)

    @property
    def discovered_servos(self) -> list[int]:
        return list(self._discovered_motors)

    # ------------------------------------------------------------------
    # Connection helpers
    # ------------------------------------------------------------------

    async def _resolve_host(self) -> Optional[str]:
        """
        Resolve hostname to IP. Uses cached result for fast reconnects.
        Falls back to socket.getaddrinfo (system mDNS on macOS via Bonjour).
        """
        try:
            socket.inet_aton(self.host)
            return self.host        # already an IP
        except OSError:
            pass

        if self._resolved_ip:
            return self._resolved_ip

        logger.debug("[RobotBackend] Resolving %s...", self.host)
        loop = asyncio.get_running_loop()
        try:
            infos = await asyncio.wait_for(
                loop.run_in_executor(
                    None,
                    lambda: socket.getaddrinfo(self.host, self.port, socket.AF_INET),
                ),
                timeout=10.0,
            )
            if infos:
                ip = infos[0][4][0]
                logger.debug("[RobotBackend] Resolved to %s", ip)
                return ip
        except (asyncio.TimeoutError, OSError) as e:
            logger.warning("[RobotBackend] Resolution failed: %s", e)
        return None

    async def _connect_with_ip(self, ip: str) -> bool:
        uri = f"ws://{ip}:{self.port}"
        logger.debug("[RobotBackend] Connecting to %s...", uri)
        try:
            self._ws = await asyncio.wait_for(
                websockets.connect(uri, ping_interval=None),
                timeout=2.0,
            )
        except asyncio.TimeoutError:
            logger.warning("[RobotBackend] Connection timed out")
            return False
        except Exception as e:
            logger.warning("[RobotBackend] Connect failed: %s", e)
            return False

        self._receive_task = asyncio.create_task(self._receive_loop())
        self._connected = True
        await asyncio.sleep(_WS_SETTLE_S)

        async def _numbered_ping_ok() -> bool:
            data = await self._send_text("ping", timeout=3.0)
            return _ping_payload_ok(data)

        slcan_preopened = False
        ok = await _numbered_ping_ok()
        if not ok:
            logger.debug("[RobotBackend] Numbered ping failed; opening SLCAN (S8/O) then retrying...")
            await self._ws.send("S8")
            await self._ws.send("O")
            await asyncio.sleep(_SLCAN_SETTLE_S)
            slcan_preopened = True
            ok = await _numbered_ping_ok()
        if not ok:
            logger.debug("[RobotBackend] Retrying with bare text \"ping\" (expect line \"pong\")...")
            ok = await self._try_bare_ping(timeout=2.5)
        if not ok:
            logger.error(
                "[RobotBackend] Text handshake failed — no reply to numbered ping. "
                "grapple-arduino normally sends \"<id>:\" (empty body). "
                "Optional: bare \"pong\" after \"ping\". "
                "Re-run with PETCTL_ROBOT_TRACE=1 to log incoming text lines."
            )
            await self.disconnect()
            return False

        if not slcan_preopened:
            await self._ws.send("S8")
            await self._ws.send("O")
            await asyncio.sleep(_SLCAN_SETTLE_S)

        self._disabled_motor_ids.clear()

        self._discovered_modules = await self._discover_modules()
        logger.debug("[RobotBackend] Discovered modules: %s", self._discovered_modules)
        if len(self._discovered_modules) < NUM_MODULES:
            logger.warning(
                "[RobotBackend] Only %d/%d modules found — polling every 2s",
                len(self._discovered_modules), NUM_MODULES,
            )
        if self._configured_motor_ids is not None:
            self._discovered_motors = list(self._configured_motor_ids)
            logger.debug("[RobotBackend] Using fixed motor IDs (--motors): %s", self._discovered_motors)
            for mid in self._discovered_motors:
                await self._ws.send(_encode_mit_enable(mid))
            await asyncio.sleep(0.5)
        elif self._discovered_motors:
            # Reconnect fast path: reuse cached IDs, just re-enable and wait for first CAN echo.
            logger.debug("[RobotBackend] Reusing motor IDs %s (reconnect)", self._discovered_motors)
            for mid in self._discovered_motors:
                await self._ws.send(_encode_mit_enable(mid))
            await asyncio.sleep(0.3)
        else:
            self._discovered_motors = await self._discover_motors()

        if self.calibrate_on_connect and self._discovered_modules:
            await self._calibrate()

        # Cancel any leftover tasks from a previous connection, then start fresh.
        for task in (self._motor_tx_task, self._module_poll_task, self._motor_poll_task):
            if task and not task.done():
                task.cancel()
        self._latest_sensors = None
        self._motor_tx_task = asyncio.create_task(self._motor_tx_loop())
        self._module_poll_task = asyncio.create_task(self._module_poll_loop())
        self._motor_poll_task = asyncio.create_task(self._motor_poll_loop())

        logger.debug("[RobotBackend] Connected (text channel + SLCAN OK)")
        return True

    # ------------------------------------------------------------------
    # Text command helpers
    # ------------------------------------------------------------------

    async def _try_bare_ping(self, timeout: float) -> bool:
        """Some firmware answers plain \"pong\" to plain \"ping\" (no request id)."""
        if self._ws is None:
            return False
        loop = asyncio.get_running_loop()
        self._bare_reply_fut = loop.create_future()
        try:
            await self._ws.send("ping")
            await asyncio.wait_for(self._bare_reply_fut, timeout=timeout)
            return True
        except (asyncio.TimeoutError, asyncio.CancelledError):
            return False
        finally:
            self._bare_reply_fut = None

    def _next_req_id(self) -> int:
        val = next(self._req_id_counter)
        if val >= 65535:
            # Wrap back to 1 — firmware treats 0 as invalid/broadcast.
            self._req_id_counter = itertools.count(1)
            return 1
        return val

    async def _send_text(
        self, command: str, timeout: float = 2.0
    ) -> Optional[str]:
        if self._ws is None:
            return None
        req_id = self._next_req_id()
        fut = asyncio.get_running_loop().create_future()
        self._pending_requests[req_id] = fut
        async with self._ws_send_lock:
            await self._ws.send(f"{req_id}:{command}")
        try:
            return await asyncio.wait_for(fut, timeout=timeout)
        except asyncio.TimeoutError:
            self._pending_requests.pop(req_id, None)
            return None

    # ------------------------------------------------------------------
    # Discovery
    # ------------------------------------------------------------------

    async def _discover_modules(self) -> list[int]:
        data = await self._send_text("getmodules", timeout=5.0)
        if not data:
            return []
        try:
            return [int(x.strip()) for x in data.split(",") if x.strip()]
        except ValueError:
            return []

    async def _module_poll_loop(self) -> None:
        """Poll getmodules every 2s, logging any changes."""
        while True:
            await asyncio.sleep(2.0)
            modules = await self._discover_modules()
            if not modules:
                continue
            if set(modules) != set(self._discovered_modules):
                prev = set(self._discovered_modules)
                curr = set(modules)
                added = sorted(curr - prev)
                dropped = sorted(prev - curr)
                note = ", ".join(
                    ([f"+{len(added)} added {added}"] if added else [])
                    + ([f"-{len(dropped)} dropped {dropped}"] if dropped else [])
                )
                self._discovered_modules = modules
                logger.debug("[RobotBackend] Modules: %s (%s)", self._discovered_modules, note)
                if len(self._discovered_modules) == NUM_MODULES:
                    logger.debug("[RobotBackend] All %d modules present", NUM_MODULES)

    async def _discover_motors(self) -> list[int]:
        if self._ws is None:
            return []
        self._motor_state.clear()
        for mid in range(1, 8):
            await self._ws.send(_encode_mit_enable(mid))
        await asyncio.sleep(1.0)
        discovered = sorted(self._motor_state.keys())
        if not discovered:
            logger.warning(
                "[RobotBackend] No motor CAN feedback during discovery; defaulting to IDs 1–7. "
                "For a partial bench (e.g. only motor 1), pass --motors 1."
            )
            return list(range(1, 8))
        logger.debug("[RobotBackend] Discovered motors: %s", discovered)
        return discovered

    async def _motor_poll_loop(self) -> None:
        """Poll all possible motor IDs every 5s; enable any newly-responding ones."""
        while True:
            await asyncio.sleep(5.0)
            if self._ws is None or self._configured_motor_ids is not None:
                continue
            for mid in range(1, 8):
                if self._ws is None:
                    break
                try:
                    await self._ws.send(_encode_mit_enable(mid))
                except Exception:
                    break
            await asyncio.sleep(0.5)
            now = set(self._motor_state.keys())
            known = set(self._discovered_motors)
            added = sorted(now - known)
            if added:
                self._discovered_motors = sorted(now)
                logger.debug("[RobotBackend] Motors: %s (+%d added %s)", self._discovered_motors, len(added), added)

    # ------------------------------------------------------------------
    # Sensor reading
    # ------------------------------------------------------------------

    def _parse_sensor_response(
        self, data: Optional[str]
    ) -> tuple[Optional[dict[int, ModuleSensors]], int, int]:
        """Parse a CSV sensor response string into (modules, battery_current_raw, battery_voltage_raw).

        Wire format (68 bytes):
          bytes  0–15: touch bits, 2 bytes per module × 8 modules
                       bits 0–3: mpr1 (right face), bits 4–7: mpr2 (left face), bits 8–13: mpr3 (top/middle)
          bytes 16–63: FSR, 6 bytes per module × 8 modules (fsrR, fsrL, fsrM — big-endian int16)
          bytes 64–67: head-only extended (current 2B, voltage 2B)
        """
        if not data:
            return None, 0, 0
        try:
            raw_bytes = [int(x.strip()) for x in data.split(",")]
        except ValueError:
            return None, 0, 0
        if len(raw_bytes) < 68:
            return None, 0, 0

        touch = raw_bytes[0:16]    # 2 bytes × 8 modules (14 binary touch bits per module)
        fsr = raw_bytes[16:64]     # 6 bytes × 8 modules (3 FSR int16 values per module)
        battery_current_raw = (raw_bytes[64] << 8) | raw_bytes[65]
        battery_voltage_raw = (raw_bytes[66] << 8) | raw_bytes[67]

        modules: dict[int, ModuleSensors] = {}
        # Always parse all NUM_MODULES slots — the sensor payload is fixed-size
        # regardless of discovery state. Partial discovery (e.g. after reconnect)
        # must not silently discard 7/8 modules' data.
        module_ids = (
            self._discovered_modules
            if len(self._discovered_modules) == NUM_MODULES
            else list(range(NUM_MODULES))
        )
        for mod_idx, mod_id in enumerate(module_ids[:8]):
            # 14-bit touch word, little-endian (low byte first)
            touch_word = touch[mod_idx * 2] | (touch[mod_idx * 2 + 1] << 8)
            raw_right   = tuple(float((touch_word >> i) & 1) for i in range(0, 4))
            raw_left    = tuple(float((touch_word >> (4 + i)) & 1) for i in range(4))
            middle_pads = tuple(float((touch_word >> (8 + i)) & 1) for i in range(6))

            f = fsr[mod_idx * 6:(mod_idx + 1) * 6]
            right_p = _int16_be(f[0], f[1])
            left_p = _int16_be(f[2], f[3])
            middle_p = _int16_be(f[4], f[5])

            if mod_id % 2 == 1:
                raw_left, raw_right = raw_right, raw_left
                left_p, right_p = right_p, left_p

            # Canonical pad order: 0=top_head, 1=top_mid, 2=top_rear, 3=bottom_solo.
            # Hardware right face is reversed along the top row: hw[0,1,2,3] = [top_rear,top_mid,top_head,bottom_solo].
            right_pads = (raw_right[2], raw_right[1], raw_right[0], raw_right[3])
            left_pads  = raw_left  # hw left already: [top_head, top_mid, top_rear, bottom_solo]

            # Odd-module swap puts left-face data into raw_right and vice versa, inverting the
            # top-row reorder above. Fix by reversing top row on both faces after the swap.
            # m0 (head) has the same top-row reversal issue plus reversed middle pads.
            if mod_id % 2 == 1 or mod_id == 0:
                left_pads  = (left_pads[2],  left_pads[1],  left_pads[0],  left_pads[3])
                right_pads = (right_pads[2], right_pads[1], right_pads[0], right_pads[3])
            if mod_id == 0:
                middle_pads = middle_pads[::-1]

            fsr_off = self._fsr_offsets.get(mod_id, {})
            modules[mod_id] = ModuleSensors(
                module_id=mod_id,
                touch_left_pads=left_pads,
                touch_right_pads=right_pads,
                touch_middle_pads=middle_pads,
                pressure_left=max(0.0, _normalize_pressure(left_p) - fsr_off.get("left", 0.0)),
                pressure_right=max(0.0, _normalize_pressure(right_p) - fsr_off.get("right", 0.0)),
                pressure_middle=max(0.0, _normalize_pressure(middle_p) - fsr_off.get("middle", 0.0)),
            )
        return modules, battery_current_raw, battery_voltage_raw

    # ------------------------------------------------------------------
    # Motor TX and sensor polling background tasks
    # ------------------------------------------------------------------

    async def _motor_tx_loop(self) -> None:
        """Motor frame TX loop at motor_update_hz.

        Batches all motor frames into one WS message per tick (newline-separated
        SLCAN). The Arduino splits on newlines and processes each frame. This
        reduces Python→Arduino WS messages from N_motors×Hz to Hz, and lets
        the Arduino batch CAN responses, keeping its broadcastTXT() call rate low.
        """
        _RX_SILENCE_TIMEOUT = 5.0
        period = 1.0 / LOOP_LIMITS.motor_update_hz

        while self._connected:
            try:
                t0 = time.monotonic()
                ids = self._discovered_motors or list(range(1, 8))

                # RX watchdog — detect silent TCP drops.
                if self._last_rx_time > 0 and t0 - self._last_rx_time > _RX_SILENCE_TIMEOUT:
                    logger.warning(
                        "[RobotBackend] No message received in %.1fs — connection silent, reconnecting.",
                        t0 - self._last_rx_time,
                    )
                    self._connected = False
                    if self.auto_reconnect and not self._reconnecting:
                        self._reconnect_task = asyncio.get_running_loop().create_task(
                            self._reconnect_loop()
                        )
                    break

                # Build one batched WS message with all motor frames (newline-separated).
                # Arduino splits on '\n' and processes each SLCAN command in order.
                # Staggered motors (set_stagger) hold their last frame until the delay expires.
                if self._ws is not None:
                    frames = []
                    for mid in ids:
                        if mid in self._disabled_motor_ids:
                            continue
                        if t0 < self._stagger_until.get(mid, 0.0):
                            # Stagger active: hold last sent frame, leave pending frame for next tick
                            frame = self._last_sent_frames.get(mid, _encode_mit_zero(mid))
                        else:
                            pending = self._pending_frames.pop(mid, None)
                            if pending is not None:
                                frame = pending
                                self._last_command_time[mid] = t0
                            else:
                                idle_s = t0 - self._last_command_time.get(mid, 0.0)
                                if idle_s <= LOOP_LIMITS.idle_hold_s:
                                    frame = self._last_sent_frames.get(mid, _encode_mit_zero(mid))
                                else:
                                    frame = _encode_mit_zero(mid)
                        self._last_sent_frames[mid] = frame
                        frames.append(frame)
                    if frames:
                        async with self._ws_send_lock:
                            await self._ws.send("\n".join(frames))

                elapsed = time.monotonic() - t0
                remaining = period - elapsed
                if remaining > 0:
                    await asyncio.sleep(remaining)
                else:
                    await asyncio.sleep(0)  # yield to event loop even on overrun
                    logger.warning(
                        "[RobotBackend] Motor TX overrun: %.1fms (budget %.1fms)",
                        elapsed * 1000, period * 1000,
                    )

            except asyncio.CancelledError:
                raise
            except websockets.ConnectionClosed:
                break
            except Exception as e:
                if not self._connected:
                    break
                logger.error("[RobotBackend] Motor TX loop error: %s", e)
                await asyncio.sleep(0.01)

    def _apply_cap_filter(self, sensors: dict[int, ModuleSensors]) -> dict[int, ModuleSensors]:
        """Apply per-pad asymmetric cap filter.

        Activation: sliding-window average over cap_filter_window frames (smooth, noise-tolerant).
        Release: forced to 0.0 once cap_clear_frames consecutive hardware zeros arrive, overriding
        any historical 1s still in the window.  Prevents post-release noise or MPR121 baseline drift
        from keeping pads elevated indefinitely.
        """
        window = SENSOR_LIMITS.cap_filter_window
        clear_n = SENSOR_LIMITS.cap_clear_frames
        result: dict[int, ModuleSensors] = {}
        for mod_id, ms in sensors.items():
            if mod_id not in self._cap_filter:
                self._cap_filter[mod_id] = {
                    "left":   [collections.deque(maxlen=window) for _ in ms.touch_left_pads],
                    "right":  [collections.deque(maxlen=window) for _ in ms.touch_right_pads],
                    "middle": [collections.deque(maxlen=window) for _ in ms.touch_middle_pads],
                }
            f = self._cap_filter[mod_id]
            for dq, v in zip(f["left"],   ms.touch_left_pads):   dq.append(v)
            for dq, v in zip(f["right"],  ms.touch_right_pads):  dq.append(v)
            for dq, v in zip(f["middle"], ms.touch_middle_pads): dq.append(v)

            def _filtered(dq: collections.deque) -> float:
                # Count consecutive zeros at the tail.
                n_zeros = 0
                for val in reversed(dq):
                    if val == 0.0:
                        n_zeros += 1
                    else:
                        break
                if n_zeros >= clear_n:
                    return 0.0
                return sum(dq) / len(dq)

            result[mod_id] = ModuleSensors(
                module_id=mod_id,
                touch_left_pads=tuple(_filtered(dq) for dq in f["left"]),
                touch_right_pads=tuple(_filtered(dq) for dq in f["right"]),
                touch_middle_pads=tuple(_filtered(dq) for dq in f["middle"]),
                pressure_left=ms.pressure_left,
                pressure_right=ms.pressure_right,
                pressure_middle=ms.pressure_middle,
            )
        return result

    def _handle_sensor_push(self, data: str) -> None:
        """Process an unsolicited sensor push from the Arduino (~50 Hz)."""
        sensors, batt_cur, batt_vol = self._parse_sensor_response(data)
        if sensors is not None:
            self._latest_sensors = self._apply_cap_filter(sensors)
            self._latest_sensor_ts = time.monotonic()
            self._latest_battery_current_raw = batt_cur
            self._latest_battery_voltage_raw = batt_vol
            self._check_stuck_touch(self._latest_sensors)
        else:
            logger.debug("[RobotBackend] Sensor push parse failed")

    def _check_stuck_touch(self, sensors: dict[int, "ModuleSensors"]) -> None:
        """Detect pads stuck on due to MPR121 baseline drift and flag for recalibration.

        A module is considered stuck when any face's average is above
        cap_stuck_min_face AND all FSRs are below cap_stuck_max_fsr — i.e. the
        capacitive pad reads as touched but there is no physical pressure.  After
        cap_stuck_timeout_s of continuous stuck state, _needs_touch_recal is set
        so _receive_loop can schedule calibrate_touch() as a background task.
        """
        lim = SENSOR_LIMITS
        now = time.monotonic()
        if self._needs_touch_recal:
            return  # already flagged; wait for it to fire
        if now - self._last_touch_recal_t < lim.cap_stuck_recal_cooldown_s:
            return  # in cooldown

        for mod_id, ms in sensors.items():
            max_face = max(ms.touch_left, ms.touch_right, ms.touch_middle)
            no_pressure = ms.pressure_total < lim.cap_stuck_max_fsr
            if max_face > lim.cap_stuck_min_face and no_pressure:
                if mod_id not in self._stuck_since:
                    self._stuck_since[mod_id] = now
                elif now - self._stuck_since[mod_id] >= lim.cap_stuck_timeout_s:
                    logger.info(
                        "[RobotBackend] Module %d stuck (cap=%.2f fsr=%.3f) for %.0fs — "
                        "scheduling touch recalibration",
                        mod_id, max_face, ms.pressure_total,
                        now - self._stuck_since[mod_id],
                    )
                    self._needs_touch_recal = True
                    self._stuck_since.clear()
                    return
            else:
                self._stuck_since.pop(mod_id, None)

    def _handle_imu_push(self, data: str) -> None:
        """Process an unsolicited IMU push from the Arduino (~25 Hz).

        Wire format: "<mid>,<qr>,<qi>,<qj>,<qk>,<ax>,<ay>,<az>,<gx>,<gy>,<gz>"
        One message per active module (currently only module 7).
        Quaternion is the BNO085 rotation vector in absolute world frame.
        """
        try:
            parts = data.split(",")
            if len(parts) != 11:
                return
            mid = int(parts[0])
            self._latest_imu[mid] = ImuReading(
                module_id=mid,
                qr=float(parts[1]), qi=float(parts[2]),
                qj=float(parts[3]), qk=float(parts[4]),
                ax=float(parts[5]), ay=float(parts[6]), az=float(parts[7]),
                gx=float(parts[8]), gy=float(parts[9]), gz=float(parts[10]),
                timestamp=time.monotonic(),
            )
        except (ValueError, IndexError):
            logger.debug("[RobotBackend] IMU push parse failed: %r", data[:80])

    # ------------------------------------------------------------------
    # Sensor calibration & normalization
    # ------------------------------------------------------------------

    async def _calibrate(self) -> None:
        await asyncio.sleep(0.05 * max(1, self.calibration_samples))
        await self.set_home()

    async def set_hardware_zero(self) -> None:
        """Write the current physical position to each motor's EEPROM as its zero.

        Sends the CubeMars MIT 0xFE command ('set zero') to every discovered
        motor. The motor saves its current encoder position to non-volatile storage
        so commanding 0.0 returns to this exact physical pose even after a motor
        power cycle. Software angle offsets are cleared afterward because the motor
        itself now reports 0 at this position.
        """
        if self._ws is None:
            return
        ids = self._discovered_motors or list(range(1, 8))
        logger.debug("[RobotBackend] Writing hardware zero to motor(s) %s...", ids)
        for mid in ids:
            async with self._ws_send_lock:
                await self._ws.send(_encode_mit_set_zero(mid))
        # Allow EEPROM write to complete and next CAN frame to arrive with pos≈0.
        await asyncio.sleep(0.25)
        self._angle_offsets.clear()
        # After hardware zero the motor will report 0.0 at the current position.
        # Seed ramp at 0.0 so the first command after zeroing doesn't snap.
        now = time.monotonic()
        self._last_mit_abs_pos = {mid: 0.0 for mid in ids}
        self._last_mit_wall_s = {mid: now for mid in ids}
        logger.debug("[RobotBackend] Hardware zero written — %d motor(s) zeroed.", len(ids))

    async def poll_positions(self) -> None:
        """Solicit a state reply from every motor.

        Resends the last transmitted frame verbatim so the MIT control law is
        unchanged. Falls back to zero-torque for motors never commanded. Bypasses
        the TX task — used only during calibration where synchronous ordering matters.
        """
        if self._ws is None:
            return
        ids = self._configured_motor_ids if self._configured_motor_ids else range(1, 8)
        for mid in ids:
            frame = self._last_sent_frames.get(mid, _encode_mit_zero(mid))
            async with self._ws_send_lock:
                await self._ws.send(frame)

    async def disable_torques(self) -> None:
        if self._ws is None:
            return
        try:
            ids = self._discovered_motors or list(range(1, 8))
            for mid in ids:
                async with self._ws_send_lock:
                    await self._ws.send(_encode_mit_disable(mid))
                self._disabled_motor_ids.add(mid)
        except Exception:
            pass

    async def disable_motor(self, motor_id: int) -> None:
        """Send exit-motor-mode to a single motor (per-motor thermal disable)."""
        if self._ws is None:
            return
        try:
            async with self._ws_send_lock:
                await self._ws.send(_encode_mit_disable(motor_id))
            self._disabled_motor_ids.add(motor_id)
        except Exception:
            pass

    async def enable_motor(self, motor_id: int) -> None:
        """Send MIT enter-motor-mode and resume TX loop for a previously disabled motor."""
        if self._ws is not None:
            try:
                async with self._ws_send_lock:
                    await self._ws.send(_encode_mit_enable(motor_id))
            except Exception:
                pass
        # Clear tracking so send_commands re-seeds from physical position on next call,
        # preventing a snap if the motor drifted while torque was off.
        self._last_mit_abs_pos.pop(motor_id, None)
        self._last_mit_wall_s.pop(motor_id, None)
        self._pending_frames.pop(motor_id, None)
        self._disabled_motor_ids.discard(motor_id)

    async def write_home_offsets(self) -> None:
        # Solicit a fresh frame so _motor_state is current, then write hardware zero.
        await self.poll_positions()
        await asyncio.sleep(0.08)
        await self.set_hardware_zero()

    async def calibrate_touch(self) -> bool:
        """Re-init all MPR121 ICs so they recalibrate their baseline from scratch.

        Sends "calibratetouch" to the head, which reinitialises its three MPR121
        chips and CAN-broadcasts CALIBRATE_COMMAND to body modules. Use when pads
        are stuck "on" after a prolonged touch — the MPR121 adaptive baseline drifts
        during extended contact and this resets it.

        Returns True if the robot acknowledged the command.
        """
        data = await self._send_text("calibratetouch", timeout=5.0)
        return data is not None

    async def _auto_recalibrate_touch(self) -> None:
        """Background task: recalibrate all MPR121 baselines after stuck-pad detection."""
        ok = await self.calibrate_touch()
        if ok:
            logger.info("[RobotBackend] Auto-recalibration complete — MPR121 baselines reset.")
        else:
            logger.warning("[RobotBackend] Auto-recalibration failed (no ACK from robot).")

    async def set_touch_threshold(
        self,
        touch12: int,
        release12: int,
        touch3: int | None = None,
        release3: int | None = None,
    ) -> bool:
        """Set MPR121 touch/release thresholds on all modules via CAN broadcast.

        touch12/release12: thresholds for side-face electrodes (mpr1, mpr2).
        touch3/release3:   thresholds for top/middle electrodes (mpr3).
                           If omitted, mpr12 values are used for mpr3 as well.

        Firmware defaults: head touch12=5/release12=2, touch3=6/release3=3;
                           body touch12=5/release12=2, touch3=3/release3=1.
        Constraint: touch > release > 0.

        Returns True if the robot acknowledged the command.
        """
        if touch3 is not None and release3 is not None:
            cmd = f"touchthreshold {touch12} {release12} {touch3} {release3}"
        else:
            cmd = f"touchthreshold {touch12} {release12}"
        data = await self._send_text(cmd, timeout=5.0)
        ok = data is not None and "invalid" not in data
        if ok:
            logger.info(
                "[RobotBackend] Touch thresholds set: mpr12 %d/%d  mpr3 %s/%s",
                touch12, release12,
                touch3 if touch3 is not None else touch12,
                release3 if release3 is not None else release12,
            )
        else:
            logger.warning("[RobotBackend] Touch threshold command failed: %s", data)
        return ok

    # ------------------------------------------------------------------
    # Auto-reconnect
    # ------------------------------------------------------------------

    async def _reconnect_loop(self) -> None:
        if self._reconnecting:
            return  # deduplicate concurrent invocations
        self._reconnecting = True
        try:
            # Tear down the old connection so the Arduino releases its slot.
            # Without an explicit close, the server sees the old TCP connection
            # as still alive and refuses new connection attempts.
            if self._receive_task and not self._receive_task.done():
                self._receive_task.cancel()
                self._receive_task = None
            if self._motor_tx_task and not self._motor_tx_task.done():
                self._motor_tx_task.cancel()
                self._motor_tx_task = None
            if self._module_poll_task and not self._module_poll_task.done():
                self._module_poll_task.cancel()
                self._module_poll_task = None
            if self._motor_poll_task and not self._motor_poll_task.done():
                self._motor_poll_task.cancel()
                self._motor_poll_task = None
            if self._ws is not None:
                try:
                    await asyncio.wait_for(self._ws.close(), timeout=0.2)
                except Exception:
                    pass
                self._ws = None
            for fut in self._pending_requests.values():
                if not fut.done():
                    fut.cancel()
            self._pending_requests.clear()

            attempt = 0
            while not self._connected:
                attempt += 1
                logger.debug("[RobotBackend] Reconnect attempt %d...", attempt)
                try:
                    ip = self._resolved_ip or await self._resolve_host()
                    if ip and await self._connect_with_ip(ip):
                        logger.debug("[RobotBackend] Reconnected.")
                        break
                except Exception as e:
                    logger.error("[RobotBackend] Reconnect error: %s", e)
                await asyncio.sleep(self.reconnect_delay)
        finally:
            self._reconnecting = False

    async def set_home(self) -> None:
        captured: list[int] = []
        for mid, state in self._motor_state.items():
            self._angle_offsets[mid] = state["pos"]
            captured.append(mid)
        missing = sorted(set(self._discovered_motors) - set(captured))
        if missing:
            logger.warning("[RobotBackend] set_home: no data yet for motor(s) %s — offsets unchanged", missing)
        if captured:
            logger.debug(
                "[RobotBackend] set_home: captured %s",
                "  ".join(f"{mid}:{self._angle_offsets[mid]:.4f}" for mid in sorted(captured)),
            )
        # Seed ramp state from current physical positions so the first post-home command
        # doesn't snap. Motors with no data are dropped (old absolute coords are invalid).
        now = time.monotonic()
        self._last_mit_abs_pos = {mid: self._motor_state[mid]["pos"] for mid in captured}
        self._last_mit_wall_s = {mid: now for mid in captured}

    def reset_motor_zero(self, motor_id: int) -> None:
        """Redefine the current physical position of motor_id as software zero.

        Called synchronously from control schemes that need to wrap a motor's
        commanded position (e.g. continuous spin) without hitting the ±12.5 rad
        MIT encoding ceiling.
        """
        state = self._motor_state.get(motor_id)
        if state is None:
            return
        self._angle_offsets[motor_id] = state["pos"]
        # Seed ramp at the new absolute coordinate (== current physical pos) so
        # the first command after the wrap doesn't snap.
        self._last_mit_abs_pos[motor_id] = state["pos"]
        self._last_mit_wall_s[motor_id] = time.monotonic()

    async def _receive_loop(self) -> None:
        if self._ws is None:
            return
        try:
            async for msg in self._ws:
                self._last_rx_time = time.monotonic()
                if isinstance(msg, bytes):
                    if self._rx_binary_logged < 4:
                        self._rx_binary_logged += 1
                        preview = msg[:48].hex()
                        logger.warning(
                            "[RobotBackend] Ignoring binary WS frame (%d B), "
                            "hex[:48]=%s… — firmware should use text for API.",
                            len(msg), preview,
                        )
                    continue
                # One WS text frame may contain multiple lines (\n or \r\n).
                for raw_line in msg.splitlines():
                    line = raw_line.strip()
                    if not line:
                        continue
                    if os.environ.get("PETCTL_ROBOT_TRACE"):
                        logger.debug("[RobotBackend] << %r", line[:240])
                    if line.startswith(("t", "T")):
                        self._handle_slcan_frame(line)
                    elif line.startswith("push:"):
                        self._handle_sensor_push(line[5:])
                        if self._needs_touch_recal:
                            self._needs_touch_recal = False
                            self._last_touch_recal_t = time.monotonic()
                            asyncio.create_task(self._auto_recalibrate_touch())
                    elif line.startswith("imu:"):
                        self._handle_imu_push(line[4:])
                    elif line.startswith("boot:"):
                        logger.info("[RobotBackend] firmware boot: %s", line[5:])
                    elif line[0].isdigit():
                        self._handle_api_response(line)
                    elif self._bare_reply_fut is not None and not self._bare_reply_fut.done():
                        low = line.lower()
                        if low == "pong" or low.startswith("pong"):
                            self._bare_reply_fut.set_result(line)
                    elif line[0] in _SLCAN_WS_NOISE_FIRST:
                        # SLCAN-side traffic echoed on WS (e.g. "z"); ignore — not petctl API.
                        continue
                    elif self._rx_unhandled_logged < 12:
                        self._rx_unhandled_logged += 1
                        logger.warning("[RobotBackend] Unhandled text (sample): %r", line[:160])
        except asyncio.CancelledError:
            return
        except Exception as e:
            logger.error("[RobotBackend] WebSocket receive loop ended: %r", e)
        else:
            try:
                code = self._ws.close_code if self._ws else None
                reason = self._ws.close_reason if self._ws else ""
                detail = f" code={code} reason={reason!r}" if code is not None else ""
            except Exception:
                detail = ""
            logger.debug("[RobotBackend] WebSocket closed cleanly by server%s", detail)
        # Reach here on clean close OR exception (not CancelledError) — trigger reconnect.
        if self._connected:
            self._connected = False
            if self.auto_reconnect and not self._reconnecting:
                self._reconnect_task = asyncio.get_running_loop().create_task(self._reconnect_loop())

    def _handle_api_response(self, msg: str) -> None:
        req_raw, _, data = msg.partition(":")
        req_raw = req_raw.strip()
        if not req_raw.isdigit():
            return
        req_id = int(req_raw)
        fut = self._pending_requests.pop(req_id, None)
        if fut is not None and not fut.done():
            fut.set_result(data)

    def _handle_slcan_frame(self, frame: str) -> None:
        can_id, payload = _parse_slcan(frame)
        if can_id != 0x000 or len(payload) < 6:
            return
        motor_id = payload[0] & 0xF
        err_code = (payload[0] >> 4) & 0xF
        p_raw = (payload[1] << 8) | payload[2]
        v_raw = (payload[3] << 4) | (payload[4] >> 4)
        t_raw = ((payload[4] & 0xF) << 8) | payload[5]
        pos = _uint_to_float(p_raw, 16, MOTOR_LIMITS.pos_min, MOTOR_LIMITS.pos_max)
        vel = _uint_to_float(v_raw, 12, MOTOR_LIMITS.vel_min, MOTOR_LIMITS.vel_max)
        torque = _uint_to_float(t_raw, 12, MOTOR_LIMITS.torque_min, MOTOR_LIMITS.torque_max)
        drive_temp = _byte_to_int8(payload[6]) if len(payload) >= 7 else 0
        motor_temp = _byte_to_int8(payload[7]) if len(payload) >= 8 else 0
        self._motor_state[motor_id] = {
            "pos": pos, "vel": vel, "torque": torque,
            "drive_temp": drive_temp, "motor_temp": motor_temp, "err_code": err_code,
        }


def _byte_to_int8(b: int) -> int:
    """Reinterpret an unsigned byte as a signed int8 (two's complement)."""
    return b if b < 128 else b - 256


def _normalize_pressure(value: int) -> float:
    return max(0.0, min(1.0, float(value) / SENSOR_LIMITS.fsr_max_raw))


def _int16_be(msb: int, lsb: int) -> int:
    raw = (msb << 8) | lsb
    if raw >= 0x8000:
        raw -= 0x10000
    return raw


def _float_to_uint(value: float, bits: int, min_value: float, max_value: float) -> int:
    span = max_value - min_value
    clipped = max(min_value, min(max_value, value))
    scale = (1 << bits) - 1
    return int((clipped - min_value) * scale / span)


def _uint_to_float(value: int, bits: int, min_value: float, max_value: float) -> float:
    span = max_value - min_value
    scale = (1 << bits) - 1
    return min_value + float(value) * span / float(scale)


def _encode_mit_packet(motor_id: int, pos: float, vel: float, kp: float, kd: float, torque: float) -> str:
    p_uint = _float_to_uint(pos, 16, MOTOR_LIMITS.pos_min, MOTOR_LIMITS.pos_max)
    v_uint = _float_to_uint(vel, 12, MOTOR_LIMITS.vel_min, MOTOR_LIMITS.vel_max)
    kp_uint = _float_to_uint(kp, 12, 0.0, 500.0)
    kd_uint = _float_to_uint(kd, 12, 0.0, 5.0)
    t_uint = _float_to_uint(torque, 12, MOTOR_LIMITS.torque_min, MOTOR_LIMITS.torque_max)
    payload = [
        (p_uint >> 8) & 0xFF,
        p_uint & 0xFF,
        (v_uint >> 4) & 0xFF,
        ((v_uint & 0xF) << 4) | ((kp_uint >> 8) & 0xF),
        kp_uint & 0xFF,
        (kd_uint >> 4) & 0xFF,
        ((kd_uint & 0xF) << 4) | ((t_uint >> 8) & 0xF),
        t_uint & 0xFF,
    ]
    return f"t{motor_id:03X}8{''.join(f'{b:02X}' for b in payload)}"


def _encode_mit_enable(motor_id: int) -> str:
    return f"t{motor_id:03X}8FFFFFFFFFFFFFFFC"


def _encode_mit_disable(motor_id: int) -> str:
    return f"t{motor_id:03X}8FFFFFFFFFFFFFFFD"


def _encode_mit_set_zero(motor_id: int) -> str:
    """CubeMars MIT 0xFE — write current encoder position to EEPROM as zero."""
    return f"t{motor_id:03X}8FFFFFFFFFFFFFFFE"


def _encode_mit_zero(motor_id: int) -> str:
    """MIT packet with pos=0, vel=0, kp=0, kd=0, torque=0 — zero torque state query."""
    return _encode_mit_packet(motor_id, pos=0.0, vel=0.0, kp=0.0, kd=0.0, torque=0.0)


def _parse_slcan(frame: str) -> tuple[int, list[int]]:
    text = frame.strip()
    if not text or text[0] not in ("t", "T"):
        return -1, []
    can_id = int(text[1:4], 16)
    dlc = int(text[4], 16)
    data_hex = text[5:5 + dlc * 2]
    payload = [int(data_hex[i:i + 2], 16) for i in range(0, len(data_hex), 2)]
    return can_id, payload
