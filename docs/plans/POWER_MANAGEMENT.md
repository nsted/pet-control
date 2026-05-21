# petctl Power Management System — Implementation Plan

## Context

PET uses 8× CubeMars GL40 II BLDC gimbal motors on a CAN bus (1 Mbps) in MIT impedance mode. Two recent incidents motivate this work:

1. **Thermal incident:** Two motors overheated to ~100°C during an extended idle session (motors holding pose against gravity). A thermal limit set via the Upper Computer was not honored — consistent with the known behavior that the GL40 II does not enforce its configured current limit in MIT mode, and likely does not enforce its thermal limit either. The motors must be protected by petctl, not by the driver firmware.
2. **Voltage anomalies:** Battery voltage readings (via ESP32-S3 internal ADC, 100k:10k divider) have shown intermittent spikes to ~27V on a system with no legitimate source above ~14.7V. Spikes correlate with anomalous reply-frame data on motor 6 (non-zero velocity/torque while motors are disabled and robot is stationary). Root cause not yet identified. Components are rated to ≥25V (most) or ≥32V (regulator, motors), so spikes are not immediately damaging, but they indicate something abnormal that needs visibility.

PET will be exhibited at ICRA 2026 (Vienna, June 1–5) and ICSR 2026 (London, July 1–4), where it will run for multi-hour gallery sessions and be handled by the public. Touch safety, component longevity, unattended-run reliability, and post-event diagnostic value all matter.

## Goals

1. **Thermal monitoring:** Read drive and motor temperatures from every reply frame, for every motor.
2. **Thermal protection:** Tiered response (soft back-off → hard per-motor disable → global emergency disable) with hysteresis.
3. **Voltage monitoring:** Read bus voltage at a defined rate, with sanity filtering for ADC garbage.
4. **Voltage anomaly detection:** Identify out-of-range readings, classify as transient vs sustained, count and log spike events.
5. **Voltage protection:** Hard emergency disable if voltage exceeds absolute threshold (defense in depth; should never fire under healthy operation).
6. **Driver error code surfacing:** Read and log ERR field from every reply frame.
7. **Telemetry:** Surface all state to rerun for live monitoring and post-event diagnostics.

## Non-goals

- Hardware changes (bus capacitance, TVS clamps, ground re-routing, thermal mass) — handled separately.
- Diagnosing root cause of the voltage spikes — this system makes them visible and survivable, doesn't solve them.
- Active cooling control — PET has no fans or pumps; this is a sensing-and-disable layer only.
- Battery state-of-charge estimation, balancing, or charge management — that's a BMS concern.

## Reference: GL40 II reply frame format

From the CubeMars GL II user manual (verified from project knowledge):

| Byte | Field |
|------|-------|
| 0 | ID (lower 4 bits) + ERR code (upper 4 bits) |
| 1–2 | Position (16-bit, maps to ±12.5 rad) |
| 3 + high nibble of 4 | Velocity (12-bit, maps to ±200 r/s) |
| Low nibble of 4 + 5 | Torque (12-bit, maps to ±10 N·m) |
| 6 | **Drive temperature (int8, °C, range -128 to +127)** |
| 7 | **Motor temperature (int8, °C, range -128 to +127)** |

ERR codes (upper nibble of byte 0):
- `0` — Disable
- `1` — Enable
- `9` — Under-voltage
- `A` — Over-current
- `B` — MOS over-temperature
- `C` — Motor winding over-temperature
- `D` — Communication loss
- `E` — Overload

Critical: ERR codes B and C must be treated as authoritative — if the motor itself flags overtemp, we trust it immediately regardless of what the temperature field says, because something in the driver believes the limit has been crossed.

## Thresholds

All thresholds in a single dataclass at the top of the module, easy to tune.

### Thermal (per-motor, applied to whichever is higher: drive temp or motor temp)

| Tier | Threshold | Action |
|------|-----------|--------|
| Soft warning | 55°C | Log event; reduce commanded τ_ff magnitude and Kp/Kd on that motor by 50% (motor becomes more compliant) |
| Hard cutoff | 65°C | Send exit-motor-mode CAN command to that specific motor; log; do not accept commands for that motor until hysteresis recovery |
| Hysteresis recovery | 50°C | Below this temperature *and* without a recent ERR=B/C event for ≥30s, motor may be re-enabled (re-enable is operator-initiated, not automatic) |
| Global emergency | 75°C on *any* motor | Send exit-motor-mode to *all* motors; log; require manual reset |

### Voltage (system bus, sampled via ESP32-S3 ADC + 100k:10k divider)

| Threshold | Meaning | Action |
|-----------|---------|--------|
| Normal range | 13.5–15.0V | Expected for 3S charged with DC plugged in |
| Low warning | <12.0V | Log; flag for operator (battery getting low, or DC unplugged with discharge progressing) |
| Critical low | <10.5V | Send exit-motor-mode to all motors; log (3S cells at 3.5V/cell, hard cutoff) |
| Spike threshold | >16V | Log timestamp and peak value; increment spike counter. **No protective action on individual spikes.** |
| Spike rate emergency | >5 spikes/minute | Send exit-motor-mode to all motors; log; require manual reset |
| Absolute emergency | >30V single sample (after sanity filter) | Send exit-motor-mode to all motors; log; require manual reset |

**Rationale:** Individual voltage spikes are currently being investigated; components have headroom and we don't have enough information to attribute spikes to specific causes yet. The system records every spike for diagnostic value but does not act on isolated events. Sustained or extreme conditions (rate or magnitude) trigger a global emergency stop because at that point the cause is no longer "occasional anomaly" but "something is actively wrong."

### ADC sanity filtering (defends against garbage readings)

Before treating an ADC sample as a real voltage measurement:
- Reject samples outside [0V, 40V] range (physically impossible given divider)
- Apply a 5-sample median filter before threshold comparisons (kills isolated bit-flip glitches)
- Keep the raw sample stream for telemetry (so we can still see the unfiltered spike events for diagnosis)

## Architecture

The power management system is a new module `petctl/power_manager.py` that runs as part of the existing safety/self-preservation layer (layer 1 of the four-layer petctl architecture).

### State machine (per-motor thermal state)

```
NORMAL ──(temp > 55°C)──▶ WARNING ──(temp > 65°C)──▶ DISABLED
   ▲                          │                          │
   │                          │                          │
   └──(temp < 50°C)───────────┘                          │
                                                         │
   ┌─────────────────────────────────────────────────────┘
   │
   └──(temp < 50°C for ≥30s AND no recent ERR=B/C AND operator_reset())──▶ NORMAL
```

Per-motor state is driven purely by that motor's thermal data and ERR field. Voltage-related conditions trip the global state machine instead.

### State machine (global system state)

```
RUNNING ──(any motor temp > 75°C
           OR voltage spike rate > 5/min
           OR voltage > 30V (absolute)
           OR voltage < 10.5V (critical low))─────▶ EMERGENCY_STOPPED
   ▲                                                       │
   │                                                       │
   └──(operator_reset AND all motors < 50°C AND voltage normal)
```

Per-motor disables from thermal cutoff are scoped to the affected motor — the system keeps running on the remaining motors. Only system-wide conditions (sustained spike rate, absolute overvoltage, very low battery, or a single motor reaching the global emergency thermal threshold) trip the global state and disable everything.

### Data flow

1. CAN reply frame received → existing parser extracts position, velocity, torque, **temperatures, ERR**
2. ADC voltage sample read at the current ADC rate → sanity filter applied → fed to PowerManager
3. PowerManager evaluates state on every tick:
   - Per-motor thermal state from latest temperatures and ERR codes
   - Voltage state from latest filtered sample (logs spikes, tracks rate)
4. Control layer queries PowerManager before sending any command: `if not pm.is_motor_enabled(motor_id): skip command`
5. Control layer reads PowerManager-modulated Kp/Kd/τ_ff scaling: `kp_scale = pm.get_compliance_scale(motor_id)`
6. PowerManager logs state changes and all spike events to rerun with timestamps and reasons

### Telemetry to add to rerun

New entities under `/power/`:
- `/power/voltage/raw` — unfiltered ADC voltage
- `/power/voltage/filtered` — median-filtered voltage
- `/power/voltage/spike_count` — running count of spike events since boot
- `/power/voltage/spike_rate_per_min` — rolling 60-second spike rate
- `/power/voltage/last_spike_peak` — peak voltage of most recent spike event
- `/power/voltage/state` — text label (NORMAL / LOW_WARNING / CRITICAL_LOW / SPIKE_RATE_EMERGENCY / ABSOLUTE_EMERGENCY)
- `/power/motors/{id}/state` — text label (NORMAL / WARNING / DISABLED)
- `/power/motors/{id}/disable_reason` — text describing why a motor is in DISABLED (e.g., "thermal_cutoff @ 66°C", "global_emergency: voltage spike rate")
- `/power/motors/{id}/compliance_scale` — current Kp/Kd/τ_ff multiplier (1.0, 0.5, or 0.0)
- `/power/motors/{id}/err_code` — ERR field from reply frame, as text
- `/power/global_state` — RUNNING / EMERGENCY_STOPPED
- `/power/events/` — text log entries on every state transition and every spike event with reason

## Implementation steps

Tackle in this order; each step should be independently testable.

### [x] Step 1: Extend reply-frame parser
Fixed `_handle_slcan_frame` in `backends/robot.py`: ERR nibble now extracted from upper 4 bits of byte 0; bytes 6 and 7 parsed as signed int8 via `_byte_to_int8()`. `RobotState` gains `motor_winding_temperatures` (byte 7) and `motor_err_codes` (ERR nibble); `motor_temperatures` corrected to drive temp (byte 6). Unit tests in `tests/test_power_manager.py` cover positive/negative/boundary temperatures and all ERR codes.

### [x] Step 2: Add voltage sanity filter
Voltage sanity filter and 5-sample median live inside `PowerManager.update()`. Raw voltage (`state.battery_voltage_v`, already calibrated via two-point fit) is sanity-checked against [0V, 40V] before being fed to the median window. Both raw and filtered values are exposed in `PowerTelemetry`. Unit tests cover sanity rejection, median smoothing of single spikes, and spike rate accumulation.

### [x] Step 3: Build PowerManager class
`petctl/power_manager.py` — pure logic, no I/O. Implements per-motor thermal state machine (NORMAL → WARNING → DISABLED) and global system state machine (RUNNING → EMERGENCY_STOPPED). `PowerThresholds` dataclass holds all tunable values. `drain_disable_events()` returns pending CAN actions; `is_motor_enabled()` and `get_compliance_scale()` gate command flow. `operator_reset(now)` validates hysteresis conditions before re-enabling.

### [x] Step 4: Wire PowerManager into the control loop
`Controller` instantiates `PowerManager` and calls `pm.update(state, now)` each tick. Disable events are drained and routed to `backend.disable_motor(id)` (new per-motor method) or `backend.disable_torques()` (global). Commands are filtered through `pm.is_motor_enabled()` and scaled by `pm.get_compliance_scale()`. `controller.request_power_reset()` is thread-safe for use from keyboard listener.

### [x] Step 5: Add rerun telemetry
`PowerTelemetry` dataclass attached to `RobotState.power_telemetry` each tick. `RerunViz._log_power_telemetry()` logs voltage (raw + filtered), spike count/rate, voltage state, per-motor state and compliance scale, global state, and event transitions. Winding temperature added under `motors/winding_temperature/`. Verify visually with the robot running.

### [ ] Step 6: Bench test
- **Thermal:** Lock a motor against a load to drive temperature up. Verify soft warning fires at 55°C and the compliance scale drops. Verify hard cutoff fires at 65°C and the motor disables. Let it cool, verify hysteresis prevents re-enable until <50°C.
- **Voltage:** Inject synthetic spikes (modify the ADC source temporarily, or use a test hook). Verify the spike counter increments. Verify rate emergency fires after enough spikes.
- **ERR codes:** If you can cause the motor to assert ERR=B or ERR=C (e.g., by genuinely overheating one motor in a controlled way), verify petctl reacts immediately even before its own threshold is hit.
- **Recovery:** Verify operator_reset() works only when conditions are actually safe.

### Step 7: Calibration session
Before the cutoffs are trusted in the gallery, run normal operation logs for 30+ minutes across several typical motion profiles and confirm the steady-state temperature distribution sits well below 55°C. If motors routinely exceed 55°C during normal operation, raise the thresholds (after understanding why — that's high for benign motion) OR fix the underlying issue (load distribution, cooling, motion profile).

## Open questions for the implementer to confirm before starting

1. Does the existing reply-frame parser already extract bytes 6 and 7 and the ERR field? Search petctl for the CAN reply decoding logic. If yes, skip step 1.
2. Where does the existing ADC voltage reading live in petctl? Identify the read path before adding the sanity filter.
3. What is the exact format of the "exit motor mode" CAN command for the GL40 II? Reference the GL II user manual sections on control mode entry/exit. The manual indicates the red light = exited state and the green light = entered state; the command bytes for transition need to be verified from the manual.
4. What CAN ID is currently assigned to each of the 8 motors? Hardcode the list or read from existing config?
5. What is the current control loop rate, and what voltage sample rate does the ADC run at? The median filter window of 5 samples needs to fit within a reasonable latency budget (<100ms total).
6. Does petctl currently have a concept of "operator_reset()" — a UI action or CAN message the operator can issue to clear an emergency state? If not, this needs to be added (could be as simple as a file-watch or a keypress in the console).
7. Are there any existing places in the code that bypass the layer-1 safety system and send motor commands directly? Those need to be audited and routed through PowerManager.

## Acceptance criteria

- Long-duration bench test (≥2 hours) in a normal idle posture without any motor exceeding 60°C.
- Synthetic high-load test in which a motor is deliberately stressed to the cutoff trips reliably with no driver damage.
- All voltage spike events visible in rerun with timestamps; no false emergency triggers under normal operation; emergency triggers reliably when synthetic emergencies are injected.
- No regressions in normal robot motion behavior — compliance scaling at 1.0 in normal state means no change to existing tuning.
- After an emergency stop, robot does not re-enable automatically — operator must explicitly reset.
- All state transitions logged with reason strings sufficient for post-event diagnosis.

## Notes for the implementer

- This is a safety-critical module. Err on the side of more conservative behavior: when in doubt, disable. False positives (unnecessary disables) are merely annoying; false negatives (missed overtemp) cook motors.
- Keep PowerManager pure-logic where possible. Hardware I/O lives at the edges. This makes the state machines easy to unit-test exhaustively.
- The voltage spike behavior is poorly understood as of this writing. Implement the monitoring and the conservative emergency thresholds, but don't tune the per-minute spike rate threshold tightly until we have data on how often spikes occur during clean operation vs anomalous operation.
- Component voltage ratings (≥25V for most, ≥32V for regulator and motors) mean we have real headroom. The voltage thresholds above are conservative — adjust only after the cause of spikes is understood.
- All thresholds, including the per-motor temperature triers and all voltage levels, should live in a single configuration dataclass at the top of the module so they can be tuned without code changes elsewhere.
