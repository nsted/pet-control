"""
RobotBackend — connects to the PET robot using the Feetech WebSocket SDK.

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
import itertools
import socket
import time
from concurrent.futures import ThreadPoolExecutor
from typing import Optional

from scservo_sdk.port_handler_factory import get_port_handler
from scservo_sdk.hls_scs import hls_scs as HlsScs, HLSS_TORQUE_SWITCH

from petctl.protocols import RobotBackend as _BackendBase
from petctl.types import ModuleSensors, RobotState, ServoCommand

# Sensor fields in layout order
_SENSOR_FIELDS = (
    "touch_middle", "touch_left", "touch_right",
    "pressure_middle", "pressure_left", "pressure_right",
)

# Bytes per module: 6 sensors × 2 bytes each
_BYTES_PER_MODULE = 12

# Default connection parameters (also imported by cli.py to keep a single source of truth)
ROBOT_DEFAULT_HOST = "pet-robot.local"
ROBOT_DEFAULT_PORT = 8080


class RobotBackend(_BackendBase):
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
        host: str = ROBOT_DEFAULT_HOST,
        port: int = ROBOT_DEFAULT_PORT,
        calibrate_on_connect: bool = True,
        calibration_samples: int = 10,
        auto_reconnect: bool = True,
        reconnect_delay: float = 2.0,
        torque_limit: int = 100,
    ) -> None:
        """
        torque_limit: TARGET_TORQUE value written with every position command
            (6.5 mA units; 100 ≈ 650 mA, 980 = MAX_TORQUE / full rated torque).
            Firmware v43+ interprets 0 as "no torque", so this must be > 0.
        """
        self.host = host
        self.port = port
        self.calibrate_on_connect = calibrate_on_connect
        self.calibration_samples = calibration_samples
        self.auto_reconnect = auto_reconnect
        self.reconnect_delay = reconnect_delay
        self.torque_limit = torque_limit

        self._port_handler = None
        self._packet_handler: Optional[HlsScs] = None
        # Single-worker executor: serializes all synchronous SDK calls
        self._executor = ThreadPoolExecutor(max_workers=1)

        self._connected = False
        self._resolved_ip: Optional[str] = None
        self._req_id_counter = itertools.count()

        self._discovered_modules: list[int] = []
        self._discovered_servos: list[int] = []

        # Calibration: per-(module_id, sensor_field) baseline and range
        self._baseline: dict[tuple[int, str], float] = {}
        self._range:    dict[tuple[int, str], float] = {}
        self._calibrated = False

        # Last known state for graceful degradation during reconnect
        self._last_state = RobotState.empty()
        # Servo positions tracked locally — updated on send, not re-read each tick
        self._servo_positions: dict[int, int] = {}

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

        ok = await self._do_connect(ip)
        if not ok:
            return False

        self._discovered_modules = await self._discover_modules()
        self._discovered_servos  = await self._discover_servos()
        print(f"[RobotBackend] Modules: {self._discovered_modules}")
        print(f"[RobotBackend] Servos:  {self._discovered_servos}")

        await self._enable_torques()

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
                active_servo_ids=set(self._discovered_servos),
                connected=False,
                dt=0.0,
            )
        try:
            raw = await self._read_sensors()
            if raw is None:
                raise RuntimeError("Sensor read returned None")

            # Read actual servo positions from hardware so the visualization
            # reflects the real robot pose rather than last commanded positions.
            read_positions = await self._read_all_servo_positions()
            self._servo_positions.update(read_positions)

            sensors = self._normalize(raw)
            now = time.monotonic()
            state = RobotState(
                timestamp=now,
                sensors=sensors,
                servo_positions=dict(self._servo_positions),
                active_modules=self._discovered_modules,
                active_servo_ids=set(self._discovered_servos),
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
                servo_positions=self._servo_positions,
                active_modules=self._discovered_modules,
                active_servo_ids=set(self._discovered_servos),
                connected=False,
                dt=0.0,
            )

    async def send_commands(self, commands: list[ServoCommand]) -> None:
        if not self._connected:
            return
        for cmd in commands:
            try:
                if cmd.position is not None:
                    ok = await self._write_pos_ex(cmd.servo_id, cmd.position, 100, cmd.acceleration)
                    if ok:
                        self._servo_positions[cmd.servo_id] = cmd.position
                elif cmd.speed is not None:
                    await self._write_speed(cmd.servo_id, cmd.speed, cmd.acceleration)
            except Exception as e:
                print(f"[RobotBackend] send_commands error (servo {cmd.servo_id}): {e}")
                self._connected = False
                if self.auto_reconnect and not self._reconnecting:
                    asyncio.get_running_loop().create_task(self._reconnect_loop())
                break

    @property
    def is_connected(self) -> bool:
        return self._connected

    @property
    def discovered_modules(self) -> list[int]:
        return list(self._discovered_modules)

    @property
    def discovered_servos(self) -> list[int]:
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

    async def _do_connect(self, ip: str) -> bool:
        """Initialise the Feetech SDK port + packet handler, verify with ping."""
        uri = f"ws://{ip}:{self.port}"
        print(f"[RobotBackend] Connecting to {uri}...")

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
            print("[RobotBackend] Connection timed out")
            return False

        if ph is None:
            print("[RobotBackend] openPort() failed")
            return False

        self._port_handler  = ph
        self._packet_handler = pkt
        self._connected = True

        # Verify with a ping
        pong = await self._send_text("ping", timeout=3.0)
        if pong is None:
            print("[RobotBackend] Ping failed")
            await self.disconnect()
            return False

        print("[RobotBackend] Connected and ping OK")
        return True

    # ------------------------------------------------------------------
    # Text command helpers
    # ------------------------------------------------------------------

    def _next_req_id(self) -> int:
        return next(self._req_id_counter) % 65536

    async def _send_text(
        self, command: str, timeout: float = 2.0
    ) -> Optional[tuple[int, str]]:
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
                resp_id = int(parts[0])
                if resp_id != req_id:
                    print(f"[RobotBackend] req_id mismatch: sent {req_id}, got {resp_id}")
                    return None
                return (resp_id, parts[1])
        except asyncio.TimeoutError:
            print(f"[RobotBackend] Text command '{command}' timed out")
        except OSError as e:
            print(f"[RobotBackend] Text command '{command}' network error: {e}")
        return None

    # ------------------------------------------------------------------
    # Discovery
    # ------------------------------------------------------------------

    async def _discover_modules(self) -> list[int]:
        result = await self._send_text("getmodules", timeout=5.0)
        if not result:
            return []
        _, data = result
        try:
            return [int(x.strip()) for x in data.split(",") if x.strip()]
        except ValueError:
            return []

    async def _discover_servos(self, servo_range: range = range(1, 10)) -> list[int]:
        """
        Probe each servo ID via ReadPos; collect those that respond.

        Sets a short WebSocket recv timeout (0.4 s) during discovery so that
        missing servos fail fast instead of waiting the SDK's default ~10 s.
        The original timeout is restored afterward.  All probes run in a
        single blocking executor call so the worker is fully released before
        the first sensor read.
        """
        loop = asyncio.get_running_loop()
        servo_ids = list(servo_range)

        def _do() -> list[int]:
            found: list[int] = []
            ws = self._port_handler.websocket
            old_timeout = ws.gettimeout()
            ws.settimeout(0.4)
            try:
                for sid in servo_ids:
                    pos, comm_result, error = self._packet_handler.ReadPos(sid)
                    if comm_result == 0:
                        found.append(sid)
                        self._servo_positions[sid] = pos
            finally:
                ws.settimeout(old_timeout)
            return found

        try:
            # With 0.4 s per probe × 9 probes = 3.6 s worst case
            return await asyncio.wait_for(
                loop.run_in_executor(self._executor, _do),
                timeout=10.0,
            )
        except (asyncio.TimeoutError, Exception) as e:
            print(f"[RobotBackend] _discover_servos error: {e}")
            return []

    # ------------------------------------------------------------------
    # Sensor reading
    # ------------------------------------------------------------------

    async def _read_sensors(self) -> Optional[dict[int, dict[str, int]]]:
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

        modules: dict[int, dict[str, int]] = {}
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
        print(f"[RobotBackend] Calibrating ({self.calibration_samples} samples)...")
        accumulated: dict[tuple[int, str], list[float]] = {}

        for _ in range(self.calibration_samples):
            raw = await self._read_sensors()
            if not raw:
                continue
            for mod_id, vals in raw.items():
                for field, value in vals.items():
                    accumulated.setdefault((mod_id, field), []).append(float(value))
            await asyncio.sleep(0.05)

        if not accumulated:
            print("[RobotBackend] Calibration failed: no samples collected.")
            return  # _calibrated remains False

        for key, samples in accumulated.items():
            baseline = sum(samples) / len(samples)
            self._baseline[key] = baseline
            # Range: 10% of full 16-bit scale or observed headroom, whichever is larger
            observed_max = max(samples)
            self._range[key] = max(observed_max - baseline, 6554.0)

        self._calibrated = True
        print("[RobotBackend] Calibration complete.")

    def _normalize(self, raw: dict[int, dict[str, int]]) -> dict[int, ModuleSensors]:
        """Convert raw 16-bit sensor values to normalized 0-1 ModuleSensors."""
        result: dict[int, ModuleSensors] = {}
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
        """Send WritePosEx servo command through the SDK.

        Writes 8 bytes starting at HLSS_TORQUE_SWITCH (40), so torque is
        atomically enabled with every position command:
          [40] TORQUE_SWITCH = 1
          [41] ACCELERATION  = acc
          [42] TARGET_POS_L  = pos_lo
          [43] TARGET_POS_H  = pos_hi
          [44] TARGET_TORQUE_L = torque_lo  (rated torque = 980)
          [45] TARGET_TORQUE_H = torque_hi
          [46] RUNNING_SPEED_L = speed_lo
          [47] RUNNING_SPEED_H = speed_hi
        This ensures servos that boot with torque off respond without needing
        a separate enable step, and survive reconnects without losing torque.

        TARGET_TORQUE must be non-zero: firmware v43 changed the semantics so
        that TARGET_TORQUE=0 means "apply zero torque" rather than "use default".
        We write MAX_TORQUE (980) so both v42 and v43 servos apply full torque.
        """
        loop = asyncio.get_running_loop()
        ph = self._packet_handler
        torque_limit = self.torque_limit

        def _do():
            txpacket = [
                1,                                  # HLSS_TORQUE_SWITCH = 1 (on)
                acc,
                ph.scs_lobyte(position),
                ph.scs_hibyte(position),
                ph.scs_lobyte(torque_limit),        # TARGET_TORQUE lo — must be non-zero on v43
                ph.scs_hibyte(torque_limit),        # TARGET_TORQUE hi
                ph.scs_lobyte(speed),
                ph.scs_hibyte(speed),
            ]
            comm_result, error = ph.writeTxRx(
                servo_id, HLSS_TORQUE_SWITCH, len(txpacket), txpacket
            )
            return comm_result == 0 and error == 0

        try:
            return await asyncio.wait_for(
                loop.run_in_executor(self._executor, _do),
                timeout=0.5,
            )
        except asyncio.TimeoutError:
            print(f"[RobotBackend] WritePosEx timed out (servo {servo_id})")
            return False
        except OSError as e:
            print(f"[RobotBackend] WritePosEx network error (servo {servo_id}): {e}")
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
        except asyncio.TimeoutError:
            print(f"[RobotBackend] WriteSpeed timed out (servo {servo_id})")
            return False
        except OSError as e:
            print(f"[RobotBackend] WriteSpeed network error (servo {servo_id}): {e}")
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

    async def _read_all_servo_positions(self) -> dict[int, int]:
        """
        Read current positions for all discovered servos in one executor call.

        All reads are serialized in the single-worker executor to avoid
        concurrent SDK calls. Returns only servos that responded successfully;
        caller merges with last known positions as fallback.
        """
        if not self._discovered_servos or self._packet_handler is None:
            return {}

        loop = asyncio.get_running_loop()
        servo_ids = list(self._discovered_servos)

        def _do() -> dict[int, int]:
            positions: dict[int, int] = {}
            for sid in servo_ids:
                pos, comm_result, error = self._packet_handler.ReadPos(sid)
                if comm_result == 0:
                    positions[sid] = pos
            return positions

        try:
            return await asyncio.wait_for(
                loop.run_in_executor(self._executor, _do),
                timeout=1.0,
            )
        except (asyncio.TimeoutError, Exception) as e:
            print(f"[RobotBackend] _read_all_servo_positions error: {e}")
            return {}

    async def _enable_torques(self) -> None:
        """
        Enable torque for every discovered servo.

        Some HLS servos boot with HLSS_TORQUE_SWITCH = 0 (torque off).
        In that state ReadPos still works and WritePosEx is accepted at the
        protocol level (comm=0, err=0), but the motor does not move.
        Writing 1 to HLSS_TORQUE_SWITCH causes the servo to hold its
        current position — safe to call without setting a target first.
        """
        if not self._discovered_servos or self._packet_handler is None:
            return
        loop = asyncio.get_running_loop()
        servo_ids = list(self._discovered_servos)

        def _do() -> None:
            for sid in servo_ids:
                self._packet_handler.write1ByteTxRx(sid, HLSS_TORQUE_SWITCH, 1)

        try:
            await asyncio.wait_for(
                loop.run_in_executor(self._executor, _do),
                timeout=5.0,
            )
            print(f"[RobotBackend] Torque enabled: {servo_ids}")
        except (asyncio.TimeoutError, Exception) as e:
            print(f"[RobotBackend] _enable_torques error: {e}")

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
                if ip and await self._do_connect(ip):
                    self._connected = True
                    self._discovered_modules = await self._discover_modules()
                    print("[RobotBackend] Reconnected.")
                    break
            except Exception as e:
                print(f"[RobotBackend] Reconnect error: {e}")
                self._resolved_ip = None
            await asyncio.sleep(self.reconnect_delay)
        self._reconnecting = False
