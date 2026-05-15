"""Robot backend for CubeMars GL40 II motors over SLCAN text WebSocket."""

from __future__ import annotations

import asyncio
import itertools
import os
import socket
import time
from typing import Optional, Sequence

import websockets

from petctl.config import LOOP_LIMITS, MOTOR_LIMITS
from petctl.protocols import RobotBackend as _BackendBase
from petctl.types import ModuleSensors, RobotState, ServoCommand

# Default connection parameters (also imported by cli.py to keep a single source of truth)
ROBOT_DEFAULT_HOST = "pet-robot.local"
ROBOT_DEFAULT_PORT = 8080


# Lines CAN→WebSocket may broadcast (see grapple-arduino head.ino). Not API
# ("<digit>:") and not MIT motor frames (handled via "t"/"T" + hex).
_SLCAN_WS_NOISE_FIRST = frozenset("SOCZzFNvV")


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


class _TxTokenBucket:
    """Leaky-bucket rate limiter for outbound WebSocket messages."""

    def __init__(self, rate: float) -> None:
        self._rate = rate
        self._tokens = rate
        self._last = time.monotonic()

    async def acquire(self) -> None:
        while True:
            now = time.monotonic()
            self._tokens = min(self._rate, self._tokens + (now - self._last) * self._rate)
            self._last = now
            if self._tokens >= 1.0:
                self._tokens -= 1.0
                return
            await asyncio.sleep((1.0 - self._tokens) / self._rate)


class RobotBackend(_BackendBase):
    """Backend for the real PET robot."""

    def __init__(
        self,
        host: str = ROBOT_DEFAULT_HOST,
        port: int = ROBOT_DEFAULT_PORT,
        calibrate_on_connect: bool = True,
        calibration_samples: int = 10,
        auto_reconnect: bool = True,
        reconnect_delay: float = 2.0,
        motor_ids: Optional[Sequence[int]] = None,
    ) -> None:
        self.host = host
        self.port = port
        self.calibrate_on_connect = calibrate_on_connect
        self.calibration_samples = calibration_samples
        self.auto_reconnect = auto_reconnect
        self.reconnect_delay = reconnect_delay
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

        self._last_state = RobotState.empty()
        self._reconnecting = False
        self._reconnect_task: Optional[asyncio.Task] = None
        self._rx_unhandled_logged: int = 0
        self._rx_binary_logged: int = 0
        # Set by _try_bare_ping; receive loop completes it on a plain "pong" line.
        self._bare_reply_fut: Optional[asyncio.Future[str]] = None

        # Latest sensor data from background poll task — decouples sensor reads
        # from the control loop so get_state() never blocks on a network round-trip.
        self._latest_sensors: Optional[dict] = None
        self._latest_battery_current_raw: int = 0
        self._latest_battery_voltage_raw: int = 0
        self._sensor_poll_task: Optional[asyncio.Task] = None

        # Outbound WS serialization: lock prevents motor batches and sensor requests
        # from interleaving; bucket caps total messages/sec across all channels.
        self._ws_send_lock: asyncio.Lock = asyncio.Lock()
        self._tx_bucket: _TxTokenBucket = _TxTokenBucket(LOOP_LIMITS.ws_max_tx_messages_per_sec)
        self._idle_poll_idx: int = 0

    # ------------------------------------------------------------------
    # RobotBackend interface
    # ------------------------------------------------------------------

    async def connect(self) -> bool:
        ip = await self._resolve_host()
        if not ip:
            print(f"[RobotBackend] Could not resolve {self.host}")
            return False
        self._resolved_ip = ip
        return await self._connect_with_ip(ip)

    async def disconnect(self) -> None:
        self._connected = False
        if self._reconnect_task and not self._reconnect_task.done():
            self._reconnect_task.cancel()
            self._reconnect_task = None
        if self._sensor_poll_task:
            self._sensor_poll_task.cancel()
            self._sensor_poll_task = None
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
            # Use the most recent data from the background sensor poll task.
            # This is non-blocking — the poll loop runs concurrently and updates
            # _latest_sensors as fast as the firmware responds (~20-50 ms round-trip).
            # Falls back to a synchronous read only on the very first tick.
            if self._latest_sensors is None:
                sensors, battery_current_raw, battery_voltage_raw = await self._read_sensors()
                if sensors is not None:
                    self._latest_sensors = sensors
                    self._latest_battery_current_raw = battery_current_raw
                    self._latest_battery_voltage_raw = battery_voltage_raw
            else:
                sensors = self._latest_sensors
                battery_current_raw = self._latest_battery_current_raw
                battery_voltage_raw = self._latest_battery_voltage_raw

            if sensors is None:
                raise RuntimeError("Sensor read returned None")

            now = time.monotonic()
            state = RobotState(
                timestamp=now,
                sensors=sensors,
                servo_positions={
                    mid: data["pos"] - self._angle_offsets.get(mid, 0.0)
                    for mid, data in self._motor_state.items()
                },
                motor_velocities={mid: data["vel"] for mid, data in self._motor_state.items()},
                motor_torques={mid: data["torque"] for mid, data in self._motor_state.items()},
                battery_current_raw=battery_current_raw,
                battery_voltage_raw=battery_voltage_raw,
                active_modules=self._discovered_modules,
                active_servo_ids=set(self._discovered_motors),
                connected=True,
                dt=now - self._last_state.timestamp,
            )
            self._last_state = state
            return state

        except Exception as e:
            print(f"[RobotBackend] get_state error: {e}")
            self._connected = False
            if self.auto_reconnect and not self._reconnecting:
                self._reconnect_task = asyncio.get_running_loop().create_task(self._reconnect_loop())
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
        if not self._connected or self._ws is None:
            return
        now = time.monotonic()
        frames: list[str] = []
        for cmd in commands:
            if cmd.position is None:
                continue
            sid = cmd.servo_id
            pos_rad = cmd.position + self._angle_offsets.get(sid, 0.0)
            last_p = self._last_mit_abs_pos.get(sid)
            last_t = self._last_mit_wall_s.get(sid)
            if last_p is not None and last_t is not None:
                dt_cmd = max(now - last_t, 1.0 / 120.0)
                vel = (pos_rad - last_p) / dt_cmd
                vel = max(MOTOR_LIMITS.vel_min, min(MOTOR_LIMITS.vel_max, vel))
            else:
                vel = 0.0
            self._last_mit_abs_pos[sid] = pos_rad
            self._last_mit_wall_s[sid] = now
            frames.append(_encode_mit_packet(sid, pos_rad, vel, cmd.kp, cmd.kd, cmd.torque_ff))
        await self._ws_send_batch(frames)

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

        print(f"[RobotBackend] Resolving {self.host}...")
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
                print(f"[RobotBackend] Resolved to {ip}")
                return ip
        except (asyncio.TimeoutError, OSError) as e:
            print(f"[RobotBackend] Resolution failed: {e}")
        return None

    async def _connect_with_ip(self, ip: str, *, _is_reconnect: bool = False) -> bool:
        uri = f"ws://{ip}:{self.port}"
        print(f"[RobotBackend] Connecting to {uri}...")
        try:
            self._ws = await asyncio.wait_for(
                websockets.connect(uri, ping_interval=None),
                timeout=10.0,
            )
        except asyncio.TimeoutError:
            print("[RobotBackend] Connection timed out")
            return False
        except Exception as e:
            print(f"[RobotBackend] Connect failed: {e}")
            return False

        self._receive_task = asyncio.create_task(self._receive_loop())
        self._connected = True
        await asyncio.sleep(0.05)

        async def _numbered_ping_ok() -> bool:
            data = await self._send_text("ping", timeout=3.0)
            return _ping_payload_ok(data)

        slcan_preopened = False
        ok = await _numbered_ping_ok()
        if not ok:
            print("[RobotBackend] Numbered ping failed; opening SLCAN (S8/O) then retrying...")
            await self._ws.send("S8")
            await self._ws.send("O")
            await asyncio.sleep(0.05)
            slcan_preopened = True
            ok = await _numbered_ping_ok()
        if not ok:
            print("[RobotBackend] Retrying with bare text \"ping\" (expect line \"pong\")...")
            ok = await self._try_bare_ping(timeout=2.5)
        if not ok:
            print(
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
            await asyncio.sleep(0.05)

        if _is_reconnect and self._discovered_modules and self._discovered_motors:
            # Reuse previously discovered topology — skip the 1-second motor scan and
            # re-calibration so home offsets stay valid across a transient drop.
            print(
                f"[RobotBackend] Reconnect: reusing modules={self._discovered_modules} "
                f"motors={self._discovered_motors}"
            )
            for mid in self._discovered_motors:
                await self._ws.send(_encode_mit_enable(mid))
            await asyncio.sleep(0.3)
        else:
            self._discovered_modules = await self._discover_modules()

            if self._configured_motor_ids is not None:
                self._discovered_motors = list(self._configured_motor_ids)
                print(f"[RobotBackend] Using fixed motor IDs (--motors): {self._discovered_motors}")
                for mid in self._discovered_motors:
                    await self._ws.send(_encode_mit_enable(mid))
                await asyncio.sleep(0.5)
            else:
                self._discovered_motors = await self._discover_motors()

            if self.calibrate_on_connect and self._discovered_modules:
                await self._calibrate()

        # Cancel any leftover poll task from a previous connection, then start fresh.
        if self._sensor_poll_task and not self._sensor_poll_task.done():
            self._sensor_poll_task.cancel()
        self._latest_sensors = None
        self._sensor_poll_task = asyncio.create_task(self._sensor_poll_loop())

        print("[RobotBackend] Connected (text channel + SLCAN OK)")
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
        return next(self._req_id_counter) % 65536

    async def _send_text(
        self, command: str, timeout: float = 2.0
    ) -> Optional[str]:
        if self._ws is None:
            return None
        req_id = self._next_req_id()
        fut = asyncio.get_running_loop().create_future()
        self._pending_requests[req_id] = fut
        await self._tx_bucket.acquire()
        async with self._ws_send_lock:
            await self._ws.send(f"{req_id}:{command}")
        try:
            return await asyncio.wait_for(fut, timeout=timeout)
        except asyncio.TimeoutError:
            self._pending_requests.pop(req_id, None)
            return None

    async def _ws_send_batch(self, frames: list[str]) -> None:
        """Rate-limit and serialize a batch of outbound SLCAN/text frames.

        Joins all frames into one WebSocket message so the Arduino handles a single
        frame instead of N, reducing its WebSocket processing overhead by up to 7×
        during active motor control.
        """
        if not frames or self._ws is None:
            return
        await self._tx_bucket.acquire()
        async with self._ws_send_lock:
            await self._ws.send("\n".join(frames))

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

    async def _discover_motors(self) -> list[int]:
        if self._ws is None:
            return []
        before = set(self._motor_state.keys())
        for mid in range(1, 8):
            await self._ws.send(_encode_mit_enable(mid))
        await asyncio.sleep(1.0)
        after = set(self._motor_state.keys())
        discovered = sorted(after - before)
        if not discovered:
            print(
                "[RobotBackend] No motor CAN feedback during discovery; defaulting to IDs 1–7. "
                "For a partial bench (e.g. only motor 1), pass --motors 1."
            )
            return list(range(1, 8))
        print(f"[RobotBackend] Discovered motors: {discovered}")
        return discovered

    # ------------------------------------------------------------------
    # Sensor reading
    # ------------------------------------------------------------------

    async def _read_sensors(self) -> tuple[Optional[dict[int, ModuleSensors]], int, int]:
        data = await self._send_text("snsr 0 108", timeout=0.5)
        if not data:
            return None, 0, 0
        try:
            raw_bytes = [int(x.strip()) for x in data.split(",")]
        except ValueError:
            return None, 0, 0
        if len(raw_bytes) < 108:
            return None, 0, 0

        touch = raw_bytes[0:56]
        fsr = raw_bytes[56:104]
        battery_current_raw = (raw_bytes[104] << 8) | raw_bytes[105]
        battery_voltage_raw = (raw_bytes[106] << 8) | raw_bytes[107]

        modules: dict[int, ModuleSensors] = {}
        module_ids = self._discovered_modules or list(range(8))
        for mod_idx, mod_id in enumerate(module_ids[:8]):
            t = touch[mod_idx * 7:(mod_idx + 1) * 7]
            nibbles: list[int] = []
            for b in t:
                nibbles.append((b >> 4) & 0xF)
                nibbles.append(b & 0xF)
            right_touch = sum(nibbles[0:4]) / 60.0
            left_touch = sum(nibbles[4:8]) / 60.0
            middle_touch = sum(nibbles[8:14]) / 90.0

            f = fsr[mod_idx * 6:(mod_idx + 1) * 6]
            right_p = _int16_be(f[0], f[1])
            left_p = _int16_be(f[2], f[3])
            middle_p = _int16_be(f[4], f[5])

            if mod_id % 2 == 1:
                left_touch, right_touch = right_touch, left_touch
                left_p, right_p = right_p, left_p

            modules[mod_id] = ModuleSensors(
                module_id=mod_id,
                touch_middle=max(0.0, min(1.0, middle_touch)),
                touch_left=max(0.0, min(1.0, left_touch)),
                touch_right=max(0.0, min(1.0, right_touch)),
                pressure_middle=_normalize_pressure(middle_p),
                pressure_left=_normalize_pressure(left_p),
                pressure_right=_normalize_pressure(right_p),
            )
        return modules, battery_current_raw, battery_voltage_raw

    # ------------------------------------------------------------------
    # Background sensor polling
    # ------------------------------------------------------------------

    async def _sensor_poll_loop(self) -> None:
        """Continuously poll touch/FSR sensors in the background.

        Motor state (positions, velocities, torques) arrives asynchronously via
        the CAN receive loop. Sensor data requires a request-reply round-trip that
        can take 20-50 ms on real hardware. Running this loop in the background
        decouples sensor latency from the 50 Hz control tick.

        Caps to 10 Hz regardless of firmware response time to reduce Arduino load.
        Triggers a reconnect after 2 consecutive timeouts (0.5 s each) so a dead
        connection is detected within ~1 s rather than waiting for a higher-level exception.
        """
        consecutive_failures = 0
        while self._connected:
            try:
                t0 = time.monotonic()
                sensors, batt_cur, batt_vol = await self._read_sensors()
                if sensors is not None:
                    self._latest_sensors = sensors
                    self._latest_battery_current_raw = batt_cur
                    self._latest_battery_voltage_raw = batt_vol
                    consecutive_failures = 0
                else:
                    consecutive_failures += 1
                    if consecutive_failures >= 2:
                        print(
                            f"[RobotBackend] Sensor read timed out {consecutive_failures}× — "
                            "declaring connection dead and reconnecting."
                        )
                        self._connected = False
                        if self.auto_reconnect and not self._reconnecting:
                            self._reconnect_task = asyncio.get_running_loop().create_task(self._reconnect_loop())
                        break
                # Cap to sensor_poll_hz so we don't flood the Arduino with back-to-back requests.
                elapsed = time.monotonic() - t0
                period = 1.0 / LOOP_LIMITS.sensor_poll_hz
                if elapsed < period:
                    await asyncio.sleep(period - elapsed)
            except asyncio.CancelledError:
                raise
            except Exception:
                if not self._connected:
                    break
                await asyncio.sleep(0.05)  # brief pause before retrying after error

    # ------------------------------------------------------------------
    # Sensor calibration & normalization
    # ------------------------------------------------------------------

    async def _calibrate(self) -> None:
        await asyncio.sleep(0.05 * max(1, self.calibration_samples))
        await self.set_home()

    async def poll_positions(self) -> None:
        """Send a zero-torque MIT packet to each motor to solicit a state reply.

        All control params (kp, kd, torque_ff) are 0 so no force is applied.
        Avoids the MIT mode-enter frame (0xFC) which can cause audible clicking
        as the motor controller briefly re-initialises.
        """
        if self._ws is None:
            return
        # Poll all configured IDs (or 1–7) rather than just _discovered_motors, so
        # motors that missed the discovery window still get fresh state each tick.
        ids = self._configured_motor_ids if self._configured_motor_ids else range(1, 8)
        frames = [_encode_mit_zero(mid) for mid in ids]
        await self._ws_send_batch(frames)

    async def poll_next_motor(self) -> None:
        """Poll one motor in round-robin order and advance the index.

        Call once per idle tick instead of poll_positions() to spread CAN
        transactions evenly across the control period rather than bursting
        all motors at once.
        """
        if self._ws is None:
            return
        ids = list(self._configured_motor_ids) if self._configured_motor_ids else list(range(1, 8))
        mid = ids[self._idle_poll_idx % len(ids)]
        self._idle_poll_idx += 1
        await self._ws_send_batch([_encode_mit_zero(mid)])

    async def disable_torques(self) -> None:
        if self._ws is None:
            return
        try:
            ids = self._discovered_motors or list(range(1, 8))
            frames = [_encode_mit_disable(mid) for mid in ids]
            await self._ws_send_batch(frames)
        except Exception:
            pass

    async def write_home_offsets(self) -> None:
        # Solicit a fresh frame from every motor, then yield to the receive
        # loop so the responses land in _motor_state before we snapshot it.
        await self.poll_positions()
        await asyncio.sleep(0.08)
        await self.set_home()

    # ------------------------------------------------------------------
    # Auto-reconnect
    # ------------------------------------------------------------------

    async def _reconnect_loop(self) -> None:
        self._reconnecting = True
        try:
            # Tear down the old connection so the Arduino releases its slot.
            # Without an explicit close, the server sees the old TCP connection
            # as still alive and refuses new connection attempts.
            if self._receive_task and not self._receive_task.done():
                self._receive_task.cancel()
                self._receive_task = None
            if self._sensor_poll_task and not self._sensor_poll_task.done():
                self._sensor_poll_task.cancel()
                self._sensor_poll_task = None
            if self._ws is not None:
                try:
                    await asyncio.wait_for(self._ws.close(), timeout=2.0)
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
                print(f"[RobotBackend] Reconnect attempt {attempt}...")
                try:
                    ip = self._resolved_ip or await self._resolve_host()
                    if ip and await self._connect_with_ip(ip, _is_reconnect=True):
                        print("[RobotBackend] Reconnected.")
                        break
                except Exception as e:
                    print(f"[RobotBackend] Reconnect error: {e}")
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
            print(f"[RobotBackend] set_home: no data yet for motor(s) {missing} — offsets unchanged")
        if captured:
            print(
                "[RobotBackend] set_home: captured "
                + "  ".join(f"{mid}:{self._angle_offsets[mid]:.4f}" for mid in sorted(captured))
            )
        self._last_mit_abs_pos.clear()
        self._last_mit_wall_s.clear()

    async def _receive_loop(self) -> None:
        if self._ws is None:
            return
        try:
            async for msg in self._ws:
                if isinstance(msg, bytes):
                    if self._rx_binary_logged < 4:
                        self._rx_binary_logged += 1
                        preview = msg[:48].hex()
                        print(
                            f"[RobotBackend] Ignoring binary WS frame ({len(msg)} B), "
                            f"hex[:48]={preview}… — firmware should use text for API."
                        )
                    continue
                # One WS text frame may contain multiple lines (\n or \r\n).
                for raw_line in msg.splitlines():
                    line = raw_line.strip()
                    if not line:
                        continue
                    if os.environ.get("PETCTL_ROBOT_TRACE"):
                        print(f"[RobotBackend] << {line[:240]!r}")
                    if line.startswith(("t", "T")):
                        self._handle_slcan_frame(line)
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
                        print(f"[RobotBackend] Unhandled text (sample): {line[:160]!r}")
        except asyncio.CancelledError:
            return
        except Exception as e:
            print(f"[RobotBackend] WebSocket receive loop ended: {e!r}")
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
        p_raw = (payload[1] << 8) | payload[2]
        v_raw = (payload[3] << 4) | (payload[4] >> 4)
        t_raw = ((payload[4] & 0xF) << 8) | payload[5]
        pos = _uint_to_float(p_raw, 16, MOTOR_LIMITS.pos_min, MOTOR_LIMITS.pos_max)
        vel = _uint_to_float(v_raw, 12, MOTOR_LIMITS.vel_min, MOTOR_LIMITS.vel_max)
        torque = _uint_to_float(t_raw, 12, MOTOR_LIMITS.torque_min, MOTOR_LIMITS.torque_max)
        self._motor_state[motor_id] = {"pos": pos, "vel": vel, "torque": torque}


def _normalize_pressure(value: int) -> float:
    return max(0.0, min(1.0, float(value) / 32767.0))


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
