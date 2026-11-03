"""
SafetyBackend — wraps any Backend and owns power/thermal protection.

Everything that used to live in ``Controller._loop`` between "get state" and
"send commands" lives here, so a control scheme (or a future second brain)
cannot bypass it:

  get_state():     inner state → PowerManager.update → emergency / thermal /
                   voltage-cutoff disables → attach ``power_telemetry``
  send_commands(): thermal gating → compliance scaling → budget bin-packing
                   (pending motors get idle commands) → inner.send_commands
  power_reset():   operator reset → re-enter MIT mode on all servos

All other Backend methods delegate to the wrapped backend.
"""

from __future__ import annotations

import logging
import time
from typing import Optional

from petctl.power_manager import PowerManager
from petctl.protocols import Backend
from petctl.types import RobotState, ServoCommand

logger = logging.getLogger(__name__)


class SafetyBackend(Backend):
    """Backend decorator that applies PowerManager protection to an inner backend."""

    def __init__(self, inner: Backend, power_manager: Optional[PowerManager] = None) -> None:
        self._inner = inner
        self._pm = power_manager if power_manager is not None else PowerManager()
        self._last_state: Optional[RobotState] = None

    @property
    def inner(self) -> Backend:
        """The wrapped backend."""
        return self._inner

    @property
    def power_manager(self) -> PowerManager:
        """The PowerManager this wrapper owns."""
        return self._pm

    # --- connection / pass-through -------------------------------------

    async def connect(self) -> bool:
        return await self._inner.connect()

    async def disconnect(self) -> None:
        await self._inner.disconnect()

    @property
    def is_connected(self) -> bool:
        return self._inner.is_connected

    @property
    def is_real_hardware(self) -> bool:
        return self._inner.is_real_hardware

    async def disable_torques(self) -> None:
        await self._inner.disable_torques()

    async def disable_motor(self, motor_id: int) -> None:
        await self._inner.disable_motor(motor_id)

    async def enable_motor(self, motor_id: int) -> None:
        await self._inner.enable_motor(motor_id)

    async def write_home_offsets(self) -> None:
        await self._inner.write_home_offsets()

    def set_stagger(self, motor_id: int, delay_s: float) -> None:
        self._inner.set_stagger(motor_id, delay_s)

    async def calibrate_touch(self) -> bool:
        return await self._inner.calibrate_touch()

    async def set_touch_threshold(
        self,
        touch12: int,
        release12: int,
        touch3: int | None = None,
        release3: int | None = None,
    ) -> bool:
        return await self._inner.set_touch_threshold(touch12, release12, touch3, release3)

    # --- protection ----------------------------------------------------

    async def get_state(self) -> RobotState:
        """Inner state, evaluated by the PowerManager, with ``power_telemetry`` attached."""
        state = await self._inner.get_state()
        pm = self._pm
        pm.update(state, time.monotonic())

        disable_ids, is_global_emergency = pm.drain_disable_events()
        if is_global_emergency:
            try:
                await self._inner.disable_torques()
            except Exception as e:
                logger.error("[Safety] Emergency disable_torques error: %s", e)
        elif disable_ids:
            for mid in disable_ids:
                try:
                    await self._inner.disable_motor(mid)
                except Exception as e:
                    logger.error("[Safety] disable_motor(%d) error: %s", mid, e)
        if pm.drain_voltage_cutoff():
            try:
                await self._inner.disable_torques()
                logger.warning("[Safety] Voltage cutoff: all motor torques disabled.")
            except Exception as e:
                logger.error("[Safety] voltage_cutoff disable_torques error: %s", e)

        state.power_telemetry = pm.get_telemetry(state.battery_voltage_v)
        self._last_state = state
        return state

    async def send_commands(self, commands: list[ServoCommand]) -> None:
        """Gate, scale and budget-limit commands, then forward them. Never mutates the caller's list."""
        if not commands:
            return
        pm = self._pm
        state = self._last_state if self._last_state is not None else RobotState.empty()

        # Thermal gating — skip disabled motors
        enabled = [cmd for cmd in commands if pm.is_motor_enabled(cmd.servo_id)]

        # Thermal compliance scale (handles WARNING state); work on copies
        scaled: list[ServoCommand] = []
        for cmd in enabled:
            scale = pm.get_compliance_scale(cmd.servo_id)
            if scale != 1.0:
                cmd = ServoCommand(
                    servo_id=cmd.servo_id,
                    position=cmd.position,
                    velocity=cmd.velocity,
                    kp=cmd.kp * scale,
                    kd=cmd.kd * scale,
                    torque_ff=cmd.torque_ff * scale,
                )
            scaled.append(cmd)

        # Budget bin-pack: active bin fires now; pending motors are idled in place
        active_commands, pending_ids = pm.allocate_budget(scaled, state)
        idle_commands = [
            ServoCommand(
                servo_id=mid,
                position=state.servo_positions.get(mid, 0.0),
                kp=0.0,
                kd=0.0,
                torque_ff=0.0,
            )
            for mid in pending_ids
        ]
        to_send = active_commands + idle_commands
        if to_send:
            await self._inner.send_commands(to_send)

    async def power_reset(self) -> bool:
        """Operator reset; on success re-enter MIT mode on every active servo."""
        if not self._pm.operator_reset():
            logger.warning("[Safety] Power manager reset denied: conditions not safe.")
            return False
        logger.info("[Safety] Power manager reset: system re-enabled.")
        # Hardware is still in exit-motor-mode after a thermal/compliance disable.
        # enable_motor also resets ramp state, preventing a position snap on the
        # first command after recovery.
        ids = sorted(self._last_state.active_servo_ids) if self._last_state is not None else []
        for mid in ids:
            try:
                await self._inner.enable_motor(mid)
            except Exception as e:
                logger.error("[Safety] enable_motor(%d) after reset error: %s", mid, e)
        return True
