"""
GrappleBackend — connects to the PET robot using the Feetech WebSocket SDK.

Uses ftservo-python-websockets (scservo_sdk) as the transport layer:
  https://github.com/robotstack-dev/ftservo-python-websockets

The SDK is synchronous (websocket-client under the hood), so all SDK calls
run in a single background thread via ThreadPoolExecutor and are awaited
from the async control loop.

TEXT CHANNEL  (WebSocket text frames — handled directly via port_handler)
  Request:  "{req_id}:{command}"
  Response: "{req_id}:{data}"
  Commands:
    ping           → "pong"
    getmodules     → "0,1,2,..."   (comma-separated module IDs)
    snsr 0 {n}     → "{byte0},{byte1},..."  (n sensor bytes as CSV)

BINARY CHANNEL  (WebSocket binary frames — handled by hls_scs packet handler)
  servo commands use hls_scs.WritePosEx(id, position, speed, acc)
  servo reads use hls_scs.ReadPos(id)

SENSOR DATA FORMAT
  All touch sensors first, then all pressure sensors.
  Each sensor value is 2 bytes big-endian in the CSV stream:
    [tM_hi, tM_lo, tL_hi, tL_lo, tR_hi, tR_lo,  ← module 0 touch
     tM_hi, tM_lo, ...                             ← module 1 touch
     ...
     pM_hi, pM_lo, pL_hi, pL_lo, pR_hi, pR_lo,  ← module 0 pressure
     ...]
  Odd module IDs have left/right swapped for body-frame consistency.
"""

from __future__ import annotations

import asyncio
import socket
import time
from concurrent.futures import ThreadPoolExecutor, TimeoutError as FuturesTimeoutError
from typing import Dict, List, Optional, Tuple

from scservo_sdk.port_handler_factory import get_port_handler
from scservo_sdk.hls_scs import hls_scs as HlsScs

from petcrl.protocols import RobotBackend
from petcrl.types import ModuleSensors, RobotState, ServoCommand

# Sensor fields in layout order
_SENSOR_FIELDS = (
    "touch_middle", "touch_left", "touch_right",
    "pressure_middle", "pressure_left", "pressure_right",
)

# Bytes per module: 6 sensors × 2 bytes each
_BYTES_PER_MODULE = 12


class GrappleBackend(RobotBackend):
    """
    Backend for the real PET robot.

    Args:
        host:                  mDNS hostname or IP  (default: "pet-robot.local")
        port:                  WebSocket port       (default: 8080)
        calibrate_on_connect:  Collect baseline sensor samples on first connect.
                               Required for normalized 0-1 sensor values.
        calibration_samples:   Number of idle samples to collect (default: 10)
        auto_reconnect:        Retry automatically when connection drops.
        reconnect_delay:       Seconds between reconnect attempts.
    """

    def __init__(
        self,
        host: str = "pet-robot.local",
        port: int = 8080,
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

        self._port_handler = None
        self._packet_handler: Optional[HlsScs] = None
        # Single-worker executor: serializes all synchronous SDK calls
        self._executor = ThreadPoolExecutor(max_workers=1)

        self._connected = False
        self._resolved_ip: Optional[str] = None
        self._req_id = 0

        self._discovered_modules: List[int] = []
        self._discovered_servos: List[int] = []

        # Calibration: per-(module_id, sensor_field) baseline and range
        self._baseline: Dict[Tuple[int, str], float] = {}
        self._range:    Dict[Tuple[int, str], float] = {}
        self._calibrated = False

        # Last known state for graceful degradation during reconnect
        self._last_state = RobotState.empty()
        # Servo positions tracked locally — updated on send, not re-read each tick
        self._servo_positions: Dict[int, int] = {}

        self._reconnecting = False

    # ------------------------------------------------------------------
    # RobotBackend interface
    # ------------------------------------------------------------------

    async def connect(self) -> bool:
        ip = await self._resolve_host()
        if not ip:
            print(f"[GrappleBackend] Could not resolve {self.host}")
            return False
        self._resolved_ip = ip

        ok = await self._do_connect(ip)
        if not ok:
            return False

        self._discovered_modules = await self._discover_modules()
        self._discovered_servos  = await self._discover_servos()
        print(f"[GrappleBackend] Modules: {self._discovered_modules}")
        print(f"[GrappleBackend] Servos:  {self._discovered_servos}")

        if self.calibrate_on_connect and self._discovered_modules:
            await self._calibrate()

        return True

    async def disconnect(self) -> None:
        self._connected = False
        if self._port_handler:
            loop = asyncio.get_running_loop()
            await loop.run_in_executor(self._executor, self._port_handler.closePort)
            self._port_handler = None
            self._packet_handler = None

    async def get_state(self) -> RobotState:
        if not self._connected:
            return RobotState(
                timestamp=time.monotonic(),
                sensors=self._last_state.sensors,
                servo_positions=self._servo_positions,
                active_modules=self._discovered_modules,
                connected=False,
                dt=0.0,
            )
        try:
            raw = await self._read_sensors()
            if raw is None:
                raise RuntimeError("Sensor read returned None")

            sensors = self._normalize(raw)
            now = time.monotonic()
            state = RobotState(
                timestamp=now,
                sensors=sensors,
                servo_positions=dict(self._servo_positions),
                active_modules=self._discovered_modules,
                connected=True,
                dt=now - self._last_state.timestamp,
            )
            self._last_state = state
            return state

        except Exception as e:
            print(f"[GrappleBackend] get_state error: {e}")
            self._connected = False
            if self.auto_reconnect and not self._reconnecting:
                asyncio.get_event_loop().create_task(self._reconnect_loop())
            return RobotState(
                timestamp=time.monotonic(),
                sensors=self._last_state.sensors,
                servo_positions=self._servo_positions,
                active_modules=self._discovered_modules,
                connected=False,
                dt=0.0,
            )

    async def send_commands(self, commands: List[ServoCommand]) -> None:
        if not self._connected:
            return
        for cmd in commands:
            try:
                if cmd.position is not None:
                    await self._write_pos_ex(cmd.servo_id, cmd.position, 100, cmd.acceleration)
                    self._servo_positions[cmd.servo_id] = cmd.position
                elif cmd.speed is not None:
                    await self._write_speed(cmd.servo_id, cmd.speed, cmd.acceleration)
            except Exception as e:
                print(f"[GrappleBackend] send_commands error (servo {cmd.servo_id}): {e}")
                self._connected = False
                if self.auto_reconnect and not self._reconnecting:
                    asyncio.get_event_loop().create_task(self._reconnect_loop())
                break

    @property
    def is_connected(self) -> bool:
        return self._connected

    @property
    def discovered_modules(self) -> List[int]:
        return list(self._discovered_modules)

    @property
    def discovered_servos(self) -> List[int]:
        return list(self._discovered_servos)

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

        print(f"[GrappleBackend] Resolving {self.host}...")
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
                print(f"[GrappleBackend] Resolved to {ip}")
                return ip
        except (asyncio.TimeoutError, OSError) as e:
            print(f"[GrappleBackend] Resolution failed: {e}")
        return None

    async def _do_connect(self, ip: str) -> bool:
        """Initialise the Feetech SDK port + packet handler, verify with ping."""
        uri = f"ws://{ip}:{self.port}"
        print(f"[GrappleBackend] Connecting to {uri}...")

        loop = asyncio.get_running_loop()

        def _init():
            ph = get_port_handler(uri)
            if not ph.openPort():
                return None, None
            ph.setBaudRate(1000000)
            return ph, HlsScs(ph)

        try:
            ph, pkt = await asyncio.wait_for(
                loop.run_in_executor(self._executor, _init),
                timeout=10.0,
            )
        except asyncio.TimeoutError:
            print("[GrappleBackend] Connection timed out")
            return False

        if ph is None:
            print("[GrappleBackend] openPort() failed")
            return False

        self._port_handler  = ph
        self._packet_handler = pkt
        self._connected = True

        # Verify with a ping
        pong = await self._send_text("ping", timeout=3.0)
        if pong is None:
            print("[GrappleBackend] Ping failed")
            await self.disconnect()
            return False

        print("[GrappleBackend] Connected and ping OK")
        return True

    # ------------------------------------------------------------------
    # Text command helpers
    # ------------------------------------------------------------------

    def _next_req_id(self) -> int:
        req_id = self._req_id
        self._req_id = (self._req_id + 1) % 65536
        return req_id

    async def _send_text(
        self, command: str, timeout: float = 2.0
    ) -> Optional[Tuple[int, str]]:
        """
        Send "req_id:command" text frame and receive "req_id:data" text response.
        Runs in the thread executor so the synchronous SDK calls don't block the loop.
        Returns (req_id, data) or None on failure.
        """
        req_id = self._next_req_id()
        msg = f"{req_id}:{command}"
        loop = asyncio.get_running_loop()

        def _do():
            # writePort with a str sends a text WebSocket frame
            self._port_handler.writePort(msg)
            # Receive response directly from underlying websocket connection
            response = self._port_handler.websocket.recv()
            if isinstance(response, bytes):
                response = response.decode("utf-8")
            return response

        try:
            response = await asyncio.wait_for(
                loop.run_in_executor(self._executor, _do),
                timeout=timeout,
            )
            if response and ":" in response:
                parts = response.split(":", 1)
                return (int(parts[0]), parts[1])
        except (asyncio.TimeoutError, Exception) as e:
            print(f"[GrappleBackend] Text command '{command}' failed: {e}")
        return None

    # ------------------------------------------------------------------
    # Discovery
    # ------------------------------------------------------------------

    async def _discover_modules(self) -> List[int]:
        result = await self._send_text("getmodules", timeout=5.0)
        if not result:
            return []
        _, data = result
        try:
            return [int(x.strip()) for x in data.split(",") if x.strip()]
        except ValueError:
            return []

    async def _discover_servos(self, servo_range: range = range(1, 10)) -> List[int]:
        """Ping each servo ID; collect those that respond."""
        found = []
        loop = asyncio.get_running_loop()
        for sid in servo_range:
            def _ping(servo_id=sid):
                success, _model = self._packet_handler.Ping(servo_id)
                return success
            try:
                ok = await asyncio.wait_for(
                    loop.run_in_executor(self._executor, _ping),
                    timeout=0.5,
                )
                if ok:
                    found.append(sid)
            except (asyncio.TimeoutError, FuturesTimeoutError, Exception):
                pass
        return found

    # ------------------------------------------------------------------
    # Sensor reading
    # ------------------------------------------------------------------

    async def _read_sensors(self) -> Optional[Dict[int, Dict[str, int]]]:
        """Send 'snsr 0 {n}' and parse the raw 16-bit sensor values."""
        n = len(self._discovered_modules) * _BYTES_PER_MODULE
        if n == 0:
            return {}

        result = await self._send_text(f"snsr 0 {n}", timeout=2.0)
        if not result:
            return None

        _, data = result
        try:
            raw_bytes = [int(x.strip()) for x in data.split(",")]
        except ValueError:
            return None

        if len(raw_bytes) < n:
            return None

        # Recompose 16-bit values from high/low byte pairs
        words = [
            (raw_bytes[i] << 8) | raw_bytes[i + 1]
            for i in range(0, n, 2)
        ]

        num_mod = len(self._discovered_modules)
        touch_words    = words[:num_mod * 3]
        pressure_words = words[num_mod * 3:]

        modules: Dict[int, Dict[str, int]] = {}
        for idx, mod_id in enumerate(self._discovered_modules):
            t = touch_words[idx * 3: idx * 3 + 3]
            p = pressure_words[idx * 3: idx * 3 + 3]
            inv = (mod_id % 2) == 1   # odd modules have left/right swapped
            modules[mod_id] = {
                "touch_middle":    t[0],
                "touch_left":      t[2] if inv else t[1],
                "touch_right":     t[1] if inv else t[2],
                "pressure_middle": p[0],
                "pressure_left":   p[2] if inv else p[1],
                "pressure_right":  p[1] if inv else p[2],
            }
        return modules

    # ------------------------------------------------------------------
    # Sensor calibration & normalization
    # ------------------------------------------------------------------

    async def _calibrate(self) -> None:
        """Collect baseline samples with robot idle; record per-sensor baseline+range."""
        print(f"[GrappleBackend] Calibrating ({self.calibration_samples} samples)...")
        accumulated: Dict[Tuple[int, str], List[float]] = {}

        for _ in range(self.calibration_samples):
            raw = await self._read_sensors()
            if not raw:
                continue
            for mod_id, vals in raw.items():
                for field, value in vals.items():
                    accumulated.setdefault((mod_id, field), []).append(float(value))
            await asyncio.sleep(0.05)

        for key, samples in accumulated.items():
            baseline = sum(samples) / len(samples)
            self._baseline[key] = baseline
            # Range: 10% of full 16-bit scale or observed headroom, whichever is larger
            observed_max = max(samples)
            self._range[key] = max(observed_max - baseline, 6554.0)

        self._calibrated = True
        print("[GrappleBackend] Calibration complete.")

    def _normalize(self, raw: Dict[int, Dict[str, int]]) -> Dict[int, ModuleSensors]:
        """Convert raw 16-bit sensor values to normalized 0-1 ModuleSensors."""
        result: Dict[int, ModuleSensors] = {}
        for mod_id, vals in raw.items():
            normalized = {}
            for field in _SENSOR_FIELDS:
                value = float(vals.get(field, 0))
                if self._calibrated:
                    key = (mod_id, field)
                    baseline = self._baseline.get(key, 0.0)
                    rng = self._range.get(key, 6554.0)
                    normalized[field] = max(0.0, min(1.0, (value - baseline) / rng))
                else:
                    normalized[field] = value / 65535.0
            result[mod_id] = ModuleSensors(module_id=mod_id, **normalized)
        return result

    # ------------------------------------------------------------------
    # Servo commands (via hls_scs packet handler)
    # ------------------------------------------------------------------

    async def _write_pos_ex(
        self, servo_id: int, position: int, speed: int, acc: int
    ) -> bool:
        """Send WritePosEx servo command through the SDK."""
        loop = asyncio.get_running_loop()

        def _do():
            # hls_scs.WritePosEx handles direction inversion internally via
            # the scs_toscs helper for signed values; raw position [0-4095]
            # is treated as unsigned here. Direction inversion (odd servos)
            # is handled in ServoCommand / Controller layer if needed.
            comm_result, error = self._packet_handler.WritePosEx(
                servo_id, position, speed, acc
            )
            return comm_result == 0 and error == 0

        try:
            return await asyncio.wait_for(
                loop.run_in_executor(self._executor, _do),
                timeout=0.5,
            )
        except (asyncio.TimeoutError, Exception) as e:
            print(f"[GrappleBackend] WritePosEx error (servo {servo_id}): {e}")
            return False

    async def _write_speed(self, servo_id: int, speed: int, acc: int) -> bool:
        """Send WriteSpeed servo command (continuous rotation / wheel mode)."""
        loop = asyncio.get_running_loop()

        def _do():
            comm_result, error = self._packet_handler.WriteSpeed(servo_id, speed, acc)
            return comm_result == 0 and error == 0

        try:
            return await asyncio.wait_for(
                loop.run_in_executor(self._executor, _do),
                timeout=0.5,
            )
        except (asyncio.TimeoutError, Exception) as e:
            print(f"[GrappleBackend] WriteSpeed error (servo {servo_id}): {e}")
            return False

    async def read_servo_position(self, servo_id: int) -> Optional[int]:
        """Read current servo position (0-4095). Available for advanced use."""
        loop = asyncio.get_running_loop()

        def _do():
            position, comm_result, error = self._packet_handler.ReadPos(servo_id)
            if comm_result == 0:
                return position
            return None

        try:
            return await asyncio.wait_for(
                loop.run_in_executor(self._executor, _do),
                timeout=0.5,
            )
        except (asyncio.TimeoutError, Exception):
            return None

    # ------------------------------------------------------------------
    # Auto-reconnect
    # ------------------------------------------------------------------

    async def _reconnect_loop(self) -> None:
        self._reconnecting = True
        attempt = 0
        while not self._connected:
            attempt += 1
            print(f"[GrappleBackend] Reconnect attempt {attempt}...")
            try:
                ip = self._resolved_ip or await self._resolve_host()
                if ip and await self._do_connect(ip):
                    self._connected = True
                    self._discovered_modules = await self._discover_modules()
                    print("[GrappleBackend] Reconnected.")
                    break
            except Exception as e:
                print(f"[GrappleBackend] Reconnect error: {e}")
                self._resolved_ip = None
            await asyncio.sleep(self.reconnect_delay)
        self._reconnecting = False
