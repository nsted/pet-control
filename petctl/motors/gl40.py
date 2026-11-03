"""
petctl.motors.gl40 — CubeMars GL40 II motor profile (SHAPE_LAYER Stage 1.7).

MIT-mode over CAN (SLCAN text), as wired for PET. Bit field widths
(16/12/12/12/12) and the enable/disable/set-zero magic frames are this
driver's own frame layout — a different driver may pack something else
entirely, not just use different numeric bounds.
"""

from __future__ import annotations

from typing import Optional

from petctl.motors.base import (
    MotorEncodingRanges,
    MotorFeedback,
    MotorGains,
    MotorPowerModel,
    MotorProfile,
    MotorThermalLimits,
    byte_to_int8,
    float_to_uint,
    uint_to_float,
)


class GL40_II(MotorProfile):
    """CubeMars GL40 II BLDC motor in MIT mode."""

    name = "GL40_II"

    encoding = MotorEncodingRanges(
        pos_min=-12.5, pos_max=12.5,
        vel_min=-0.5, vel_max=0.5,
        torque_min=-1.0, torque_max=1.0,
        kp_wire_max=500.0, kd_wire_max=5.0,
    )
    gains = MotorGains(
        # Softer defaults — high kp tracks each MIT setpoint sharply (feels "poppy").
        kp_default=0.8, kd_default=0.035,
        kp_max=1.5, kd_max=0.04,
    )
    thermal = MotorThermalLimits(
        temp_soft_warning_c=55.0,       # reduce Kp/Kd/tau_ff by 50%
        temp_hard_cutoff_c=65.0,        # exit motor mode for this motor
        temp_global_emergency_c=75.0,   # exit motor mode for ALL motors
        temp_hysteresis_recovery_c=50.0,
        temp_hysteresis_cooldown_s=30.0,
    )
    power = MotorPowerModel(
        # torque_coeff: clamped-motor calibration 2026-06-25, VMAX=0.5/TMAX=1 (correct
        # encoding). Motor 7, kp=0 kd_max, tau_ff swept 0.02-0.12 Nm both directions,
        # other motors relaxed. R^2=0.9939, residual rms=0.023A, n=1008, V_bus=14.70V.
        per_motor_base_a=0.06,
        per_motor_torque_coeff=72.52,
        # mech_coeff: uncalibratable from free-spin (tau^2 and tau*omega collinear on
        # a free rotor). 0.3 is a conservative estimate; the reactive EMA backstop
        # in power_manager.py covers residual error.
        per_motor_mech_coeff=0.3,
        per_motor_worst_case_w=24.0,
        # Mock joint dynamics (SHAPE_LAYER Stage 1.3) — initial guesses, not
        # measured; fit from petctl/recorder.py recordings once real motor step
        # responses are available.
        mock_inertia_kg_m2=0.015,
        mock_viscous_friction_nm_s_per_rad=0.03,
    )

    def encode_command(
        self, motor_id: int, pos: float, vel: float, kp: float, kd: float, torque: float
    ) -> str:
        e = self.encoding
        p_uint = float_to_uint(pos, 16, e.pos_min, e.pos_max)
        v_uint = float_to_uint(vel, 12, e.vel_min, e.vel_max)
        kp_uint = float_to_uint(kp, 12, 0.0, e.kp_wire_max)
        kd_uint = float_to_uint(kd, 12, 0.0, e.kd_wire_max)
        t_uint = float_to_uint(torque, 12, e.torque_min, e.torque_max)
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

    def encode_enable(self, motor_id: int) -> str:
        return f"t{motor_id:03X}8FFFFFFFFFFFFFFFC"

    def encode_disable(self, motor_id: int) -> str:
        return f"t{motor_id:03X}8FFFFFFFFFFFFFFFD"

    def encode_set_zero(self, motor_id: int) -> str:
        """CubeMars MIT 0xFE — write current encoder position to EEPROM as zero."""
        return f"t{motor_id:03X}8FFFFFFFFFFFFFFFE"

    def decode_feedback(self, can_id: int, payload: list[int]) -> Optional[MotorFeedback]:
        if can_id != 0x000 or len(payload) < 6:
            return None
        e = self.encoding
        motor_id = payload[0] & 0xF
        err_code = (payload[0] >> 4) & 0xF
        p_raw = (payload[1] << 8) | payload[2]
        v_raw = (payload[3] << 4) | (payload[4] >> 4)
        t_raw = ((payload[4] & 0xF) << 8) | payload[5]
        pos = uint_to_float(p_raw, 16, e.pos_min, e.pos_max)
        vel = uint_to_float(v_raw, 12, e.vel_min, e.vel_max)
        torque = uint_to_float(t_raw, 12, e.torque_min, e.torque_max)
        drive_temp = byte_to_int8(payload[6]) if len(payload) >= 7 else 0
        motor_temp = byte_to_int8(payload[7]) if len(payload) >= 8 else 0
        return MotorFeedback(
            motor_id=motor_id, pos=pos, vel=vel, torque=torque,
            drive_temp=drive_temp, motor_temp=motor_temp, err_code=err_code,
        )
