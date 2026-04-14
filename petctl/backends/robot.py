"""Robot backend for CubeMars GL40 II motors over SLCAN text WebSocket."""

from __future__ import annotations

import asyncio
import itertools
import socket
import time
from typing import Optional

import websockets

from petctl.config import MOTOR_LIMITS
from petctl.protocols import RobotBackend as _BackendBase
from petctl.types import ModuleSensors, RobotState, ServoCommand

# Default connection parameters (also imported by cli.py to keep a single source of truth)
ROBOT_DEFAULT_HOST = "pet-robot.local"
ROBOT_DEFAULT_PORT = 8080


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
    ) -> None:
        self.host = host
        self.port = port
        self.calibrate_on_connect = calibrate_on_connect
        self.calibration_samples = calibration_samples
        self.auto_reconnect = auto_reconnect
        self.reconnect_delay = reconnect_delay

        self._ws: Optional[websockets.ClientConnection] = None
        self._receive_task: Optional[asyncio.Task] = None
        self._pending_requests: dict[int, asyncio.Future] = {}

        self._connected = False
        self._resolved_ip: Optional[str] = None
        self._req_id_counter = itertools.count()

        self._discovered_modules: list[int] = []
        self._discovered_motors: list[int] = []

        self._motor_state: dict[int, dict[str, float]] = {}
        self._angle_offsets: dict[int, float] = {}

        self._last_state = RobotState.empty()
        self._reconnecting = False

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
            sensors, battery_current_raw, battery_voltage_raw = await self._read_sensors()
            if sensors is None:
                raise RuntimeError("Sensor read returned None")

            now = time.monotonic()
            state = RobotState(
                timestamp=now,
                sensors=sensors,
                servo_positions={mid: data["pos"] for mid, data in self._motor_state.items()},
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
                asyncio.get_running_loop().create_task(self._reconnect_loop())
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
        for cmd in commands:
            if cmd.position is None:
                continue
            pos_rad = cmd.position + self._angle_offsets.get(cmd.servo_id, 0.0)
            frame = _encode_mit_packet(cmd.servo_id, pos_rad, 0.0, cmd.kp, cmd.kd, cmd.torque_ff)
            await self._ws.send(frame)

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

    async def _connect_with_ip(self, ip: str) -> bool:
        uri = f"ws://{ip}:{self.port}"
        print(f"[RobotBackend] Connecting to {uri}...")
        try:
            self._ws = await asyncio.wait_for(
                websockets.connect(uri),
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

        pong = await self._send_text("ping", timeout=3.0)
        if pong is None or "pong" not in pong.lower():
            print("[RobotBackend] Ping failed")
            await self.disconnect()
            return False

        await self._ws.send("S8")
        await self._ws.send("O")

        self._discovered_modules = await self._discover_modules()
        self._discovered_motors = await self._discover_motors()

        if self.calibrate_on_connect and self._discovered_modules:
            await self._calibrate()

        print("[RobotBackend] Connected and ping OK")
        return True

    # ------------------------------------------------------------------
    # Text command helpers
    # ------------------------------------------------------------------

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
        await self._ws.send(f"{req_id}:{command}")
        try:
            return await asyncio.wait_for(fut, timeout=timeout)
        except asyncio.TimeoutError:
            self._pending_requests.pop(req_id, None)
            return None
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
            print("[RobotBackend] No motor feedback during discovery; defaulting to IDs 1-7")
            return list(range(1, 8))
        return discovered

    # ------------------------------------------------------------------
    # Sensor reading
    # ------------------------------------------------------------------

    async def _read_sensors(self) -> tuple[Optional[dict[int, ModuleSensors]], int, int]:
        data = await self._send_text("snsr 0 108", timeout=2.0)
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
    # Sensor calibration & normalization
    # ------------------------------------------------------------------

    async def _calibrate(self) -> None:
        await asyncio.sleep(0.05 * max(1, self.calibration_samples))

    async def disable_torques(self) -> None:
        if self._ws is None:
            return
        for mid in self._discovered_motors or list(range(1, 8)):
            await self._ws.send(_encode_mit_disable(mid))

    async def write_home_offsets(self) -> None:
        await self.set_home()

    # ------------------------------------------------------------------
    # Auto-reconnect
    # ------------------------------------------------------------------

    async def _reconnect_loop(self) -> None:
        self._reconnecting = True
        attempt = 0
        while not self._connected:
            attempt += 1
            print(f"[RobotBackend] Reconnect attempt {attempt}...")
            try:
                ip = self._resolved_ip or await self._resolve_host()
                if ip and await self._connect_with_ip(ip):
                    self._connected = True
                    print("[RobotBackend] Reconnected.")
                    break
            except Exception as e:
                print(f"[RobotBackend] Reconnect error: {e}")
                self._resolved_ip = None
            await asyncio.sleep(self.reconnect_delay)
        self._reconnecting = False

    async def set_home(self) -> None:
        for mid, state in self._motor_state.items():
            self._angle_offsets[mid] = state["pos"]

    async def _receive_loop(self) -> None:
        if self._ws is None:
            return
        try:
            async for msg in self._ws:
                if not isinstance(msg, str):
                    continue
                if msg.startswith(("t", "T")):
                    self._handle_slcan_frame(msg)
                elif msg and msg[0].isdigit():
                    self._handle_api_response(msg)
        except Exception:
            self._connected = False
            if self.auto_reconnect and not self._reconnecting:
                asyncio.get_running_loop().create_task(self._reconnect_loop())

    def _handle_api_response(self, msg: str) -> None:
        req_raw, _, data = msg.partition(":")
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
        motor_id = payload[0] >> 4
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


def _parse_slcan(frame: str) -> tuple[int, list[int]]:
    text = frame.strip()
    if not text or text[0] not in ("t", "T"):
        return -1, []
    can_id = int(text[1:4], 16)
    dlc = int(text[4], 16)
    data_hex = text[5:5 + dlc * 2]
    payload = [int(data_hex[i:i + 2], 16) for i in range(0, len(data_hex), 2)]
    return can_id, payload
