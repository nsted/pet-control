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

## Per-motor torque cap: demand-weighted allocation (2026-11-03, Nick)

Decision: the total stays at the effective budget (4 A battery, 6 A wall), but it need not be split
equally between motors; it may be divided unequally and differently from tick to tick. Per motor, a hard
**2 A peak** and a **sustained average below 1 A**.

Implemented (`PowerBudgetConfig.per_motor_cap_mode`, default `"demand"`; `"equal"` is the old behaviour):
- Each active motor's *demand* is the model current of the larger of its commanded torque
  (`|torque_ff|`, `kp·|position error|`) and its measured torque, limited to its own allowance.
- The budget (minus the idle base current of any pending motors) is split **max-min fair (water-filling)**
  over those demands: a motor asking for little keeps what it asks for, the rest flows to the motors asking
  for more, and when everything is demanded at once it reduces to the old equal share. The predicted total
  can never exceed the budget.
- Per-motor allowance: peak 2 A (≈0.164 N·m by the model) while that motor's running-average current is low,
  eased linearly to 1 A as the average (time constant 3 s) goes from 0.8 A to 1 A, tighter above 1 A.
- The cap is published each tick in `PowerTelemetry.per_motor_torque_cap_nm` and
  `per_motor_sustained_current_a` (it reflects the *current* requests, so an idle motor shows ≈0).
- Unchanged: the reactive measured-current backstop, the sustained-peak emergency stop, voltage cutoff,
  thermal gating, and the cold-start staging of motors.

Caveats: (1) the current model was calibrated only for tau_ff 0.02–0.12 N·m, so 2 A is an extrapolation and
has **not been checked against measured bus current on the robot**; (2) the kp·error demand uses kp in
our units, which the code comments say do not map linearly to N·m; (3) the per-motor current is
model-estimated, the robot only measures total bus current. Hardware validation (step the torque up while
logging bus current against the model) is still to do before this is merged.

### Hardware validation, first run (2026-11-03, servo 4, wall supply 14.55 V, `scripts/validate_power_allocation.py`)

Open-loop `torque_ff` steps on one middle joint, others holding; abort limits 4.5 A bus / 52 °C / 0.4 rad window.
- **Forwarded torque = requested** at 0.040, 0.080, 0.098 N·m; demand cap tracked the request (0.040, 0.081, 0.100). Nothing clipped at these levels, so the larger-than-old-cap region (> 0.105 N·m) is **not yet exercised**.
- **Servo 4 is not clamped under sustained torque:** it moved 0.39 rad during the 1.5 s hold at 0.08 N·m (it did not move in the 60 ms pulses to 0.1). The run stopped at the 0.4 rad window on the third level, as designed. Max bus current 1.64 A, max motor temperature 48 °C (motors were already 45–48 °C; they were 36–40 °C at the start of the day).
- **Current model vs measurement is NOT validated.** Steady bus-current rise over the idle 0.62 A: +0.02 A at 0.04 N·m, +0.10 A at 0.08, about +0.12 A at 0.10 (rising with speed), against 0.09, 0.38, 0.59 A predicted: the model over-predicts by about 4–5x in these moving, sustained holds. But a single transient, the return-to-hold after the abort (measured torque 0.139 N·m, moving), peaked at 1.64 A (+1.0 A), which matches the model (+1.15 A). The two disagree, so bus current is not a simple function of torque here (speed, direction, braking and the 26 Hz current sensor all plausible). A **clamped, non-moving** hold is needed for a clean answer; a hanging robot held against gravity provides one.
- The model errs in the conservative direction in the steady case, so the allocation is not unsafe on this evidence, but its absolute numbers (2 A peak ≈ 0.164 N·m) are unconfirmed.

### Hanging-robot test (built 2026-11-03; not yet run on hardware)

`scripts/hang_test.py` + `petctl/hang_analysis.py`. Robot held by the head (module 0); servo k joins modules k-1 and k.
Each joint is stepped through small angles from both directions while the others hold; fits gravity stiffness K,
Coulomb friction from the hysteresis, and `delta_bus = a·tau²` over static holds (the clean check of the current model
behind the torque cap). The command is slewed at 0.2 rad/s so joint speed stays under the 0.45 rad/s guard. Refuses to
start if a static torque exceeds 0.05 N·m (not hanging straight) or a motor is above 50 °C; stops at 4.5 A bus, 52 °C,
0.6 rad, or any power-manager intervention.

Geometry (from Nick's Fusion export, 2026-11-03, `pet_mass_properties.json`): adjacent joint axes 5.86 cm apart, adjacent
motor centres 5.9 cm apart along the chain; each motor's centre of mass lies on its joint axis; the axes alternate between
world y and z, i.e. every axis is 45° to the chain. For a chain hanging straight down and an axis at β to it,
`K = m·g·r·sin²β` (0.5·m·g·r at 45°). With the mass at the motor axes and 0.275 kg per module the predicted stiffness is
`0.0796 · n(n-1)/2` N·m/rad for n modules below: servo 1: 1.67, 2: 1.19, 3: 0.80, 4: 0.48, 5: 0.24, 6: 0.08, 7: 0 (the tail
joint measures friction only). The module masses are NOT known from the CAD (see below); they need weighing.

#### Hanging test, first robot attempts on servo 7 (2026-11-03; robot held by the head, no holds completed)

The robot hung straight (all static torques ≤ 0.03 N·m; motors 33–38 °C; bus 1.4 A). No fit was possible:
- **kp 0.8 / kd 0.035 (default):** the free-hanging tail **chatters** while holding still: angle ±0.02 rad, torque ±0.07 N·m, velocity
  spikes to ±0.48 rad/s. It was damped by the floor in earlier tests. A light joint (J ~ 5e-4 kg·m²) under the default PD with
  50 Hz bursty feedback and 20–100 ms delay is marginally unstable. The speed guard tripped on one spike.
- **kp 0.15 / kd 0.015:** worse: with the command at zero the tail ran away from 0.016 to 0.248 rad within 0.5 s while the motor
  pushed back (−0.02…−0.037 N·m, i.e. kp·angle), so an external torque larger than the weak spring acts on it, or the joint has
  negative gravity stiffness (tail centre of mass not hanging below its axis), or the hanging chain was moving. Not resolved.
- Script now has `--kp/--kd` for the joint under test, a windowed (0.1 s mean) speed guard, and reports the hold-still jitter.
Next: a passive observation of servo 7 with zero gains (equilibrium angle, ring-down) to separate those causes.

#### Hanging test, servo 7 at the default gains (kp 0.8, kd 0.035; Nick: these are the usable gains), 2026-11-03

Connected by IP (192.168.18.33) because `pet-robot.local` briefly failed to resolve. Robot hung straight (static torques ≤ 0.007 N·m, motors 34 °C,
bus 0.9–1.45 A). One hold completed, then the 0.45 rad/s mean-speed guard tripped on chatter; not a safety event.
- **The free-hanging tail is in a sustained ~6.5 Hz limit cycle while holding still:** angle ±0.02 rad (std 0.0146), torque ±0.076 N·m
  (std 0.052, ~25x a stable joint), mean |velocity| 0.21 rad/s. This is the PD loop oscillating at sqrt(kp/J) with the 20-100 ms feedback delay.
- **Tail inertia from the oscillation frequency:** J ≈ kp/(2π·6.5)² ≈ 4.8e-4 kg·m² (approximate; delay lowers the frequency). Agrees with the
  ~5e-4 estimated earlier from the 70 ms rise to 0.3 rad/s; the mock's 0.015 is ~30x too large for the tail.
- Tail friction cannot be read through that chatter (torque jitter 0.05 vs friction 0.004-0.0075 N·m), and its predicted gravity lever is zero, so
  the hanging test is not informative for servo 7. Proceed to the loaded joints (servos 4, 3, 2, 1).
- For the RL actuator model: the default PD gains limit-cycle on a light, free joint; the sim needs the feedback delay and 50 Hz sampling to reproduce it.

#### CORRECTION (2026-11-03) to the two servo-7 hanging entries above

- The statement that the 20-100 ms telemetry delay destabilises the tail's PD loop was **wrong**: the MIT-mode PD runs inside the motor driver
  at a high rate; the 50 Hz stream only carries the target out and telemetry back, so that delay is not in the loop. The cause of the 6.5 Hz
  limit cycle on the free-hanging tail is not established (backlash/stiction with a light output inertia, or wrong effective gains, are candidates).
- The tail inertia "≈ 4.8e-4 kg·m² from the oscillation frequency" assumed kp is a true 0.8 N·m/rad and **is withdrawn**. A regression of the
  measured torque on the tail's own trace gives `tau ≈ 1.8·(command - angle) + 0.032·(-velocity)` (R² 0.27): the effective kp looks ~2.2x the
  nominal 0.8 while kd matches. If so, the same 6.5 Hz implies an inertia over twice as large. (The independent ~5e-4 estimate from the 70 ms rise to
  0.3 rad/s used measured torque and does not depend on kp, but assumed negligible friction.)
- Nick's view: the default kp/kd are off for this joint, and servo 7 (nothing downstream) is the right place to find usable ones.
  `scripts/tune_tail_gains.py` sweeps (kp, kd) soft to stiff on a hanging tail, measures hold jitter, step response and the effective kp/kd the
  motor applies (it recovers kp within 6% on the mock), and reports the stiffest stable pair. Not yet run on the robot.

#### Tail gain sweep (servo 7 hanging from the head, 2026-11-03, `scripts/tune_tail_gains.py`, data/tune/tune_s7_b.jsonl)

First attempt (kp 0.1, kd 0.005) ran away: after 1.1 s at rest the angle grew ~10x in 0.19 s with only −0.004…−0.010 N·m of restoring torque, i.e. an unstable
equilibrium (suspected small negative gravity stiffness of the tail at the hanging pose; unverified). Gains kp 0.15/kd 0.02 held the joint perfectly still
(angle ±0.0004 rad, torque ≈ −0.0005 N·m). The sweep was then rerun from kp 0.15 up, as a per-pair pass/fail with no runaways:

| kd | kp 0.15 … 0.8 behaviour |
|---|---|
| 0.010 | quiet at every kp (angle std 0.0002–0.0004, torque std ≤ 0.0003), settles 0.06–0.23 s, overshoot ≤ 3% |
| 0.020 | quiet at every kp (angle std 0.0003–0.0021, torque std ≤ 0.0011), settles 0.13–0.59 s, overshoot ≤ 2% |
| **0.035 (the default)** | **chatters at every kp** (angle std 0.005–0.013, torque std 0.016–0.047, 4–11 zero crossings after a step), settles 0.24–1.49 s |

So the default `kd` is too high for the free tail (velocity-noise amplification; kd_default is 87.5% of kd_max); `kp` up to 0.8 is fine. Best: **kp 0.8 with kd 0.02**
(settle 0.14 s, overshoot 2%, torque jitter 0.0011 vs 0.0469 at the default) or kd 0.01 (settle 0.07 s). The edge between kd 0.02 and 0.035 was not mapped.
The tail is the lightest joint; heavier joints may tolerate or need more kd and have not been swept. No defaults have been changed.

Gain *units* remain unresolved: the "effective kp ≈ 1.8 (2.2x nominal)" regression from the chattering trace is withdrawn, and the step-onset torque (0.28x of kp·0.1)
cannot decide it either, since the joint settles within 3-7 telemetry samples. A units check needs a held, non-moving error (e.g. a joint clamped by hand or a loaded joint).

#### CONFOUND in the tail-chatter evidence (2026-11-03)

Nick: holding the robot vertical with the gl40.py gains (kp 0.8, kd 0.035) has always been smooth for him. The bench measurement that looked quiet
(servo 7, default gains, angle std 0.00023 rad, torque std 0.0004 N·m) was taken on the **original power manager**; every hanging run that
chattered (angle std 0.0146 rad, torque std 0.052 N·m) ran on `feat/power-allocation` with the **new demand-weighted torque cap** active. So the
chatter may be caused by that change, not by the hanging pose or the gains. To separate them, `scripts/tune_tail_gains.py` now has
`--cap-mode equal|demand`, `--raw` (no power manager, as the old `tune_kp_kd.py`) and `--still-only`. Planned test, hanging, kp 0.8 / kd 0.035:
demand vs equal vs raw. The earlier conclusion "the default kd is the culprit" is not established until that is done (the kd comparison in the sweep
was within the demand mode only).

#### Confound test result (hanging tail, kp 0.8 / kd 0.035 = gl40.py gains, 3 s hold-still, 2026-11-03, motors 43–45 °C)

| power path | angle std (rad) | torque std (N·m) |
|---|---|---|
| demand (new allocation) | 0.0002 | 0.0003 |
| equal (original behaviour) | 0.0002 | 0.0003 |
| raw backend, no power manager | 0.0003 | 0.0005 |
| demand, repeat | 0.0002 | 0.0003 |

All quiet, ~50x below the chattering runs (0.0146 rad / 0.052 N·m, motors 33–42 °C). So: **the new demand allocation is not the cause, and the default gains are
fine here**, matching Nick's experience. The earlier chatter (three hang runs and the kd 0.035 rows of the sweep) is **unexplained**. Candidates, none tested: motor
temperature (all chatter runs were at 33–42 °C, all quiet runs at 43–45 °C; cold grease/friction), the preceding motion (the sweep stepped the tail before each hold),
the chain swinging in Nick's hand, or where the tail happened to rest inside its backlash. The statement that "the default kd 0.035 is too high" is **not
established**; the sweep's kd comparison was real for that session but did not reproduce.

#### Slow-excitation run for the tail (built 2026-11-03; not yet run on the robot)

`scripts/excite_tail.py` + `petctl/tail_fit.py`. Smooth rate-limited steps (±0.3, ±0.4 rad at 3 rad/s) and gentle sweeps (0.2–1.4 Hz, ±0.15 rad) that the 50 Hz telemetry
can resolve; every motor frame is logged at its true arrival time (`RobotBackend.set_feedback_listener`) with the commands as sent. The fit simulates the whole
closed loop at 1 kHz and fits seven parameters at once to the measured **position and torque**: inertia J, viscous B and Coulomb C friction, a stiffness K of
either sign, the *effective* kp and kd (so gain units are measured, not assumed) and the command delay. An earlier integral-regression fit was rejected: at 50 Hz it
overestimated J by 20-40% in simulation. Validation: on simulated data it recovers kp/kd/J/C/delay (kp to 15%, J to 25%, delay to 15 ms) and detects a 2x kp-units
discrepancy from a bad starting guess (ratio 2.4 vs 2.0); end-to-end on the mock it recovers J 0.0145 (true 0.015), B 0.032 (0.030), effective kp 0.298 (commanded 0.30),
position error < 1% of travel. Runs at the gl40.py-quiet gains by default (kp 0.3, kd 0.02) and stops on bus current, temperature, power-manager intervention or a
runaway (angle leaves ±0.6 rad, then parks at soft gains and goes on).

#### RESULT: tail slow-excitation run (servo 7 hanging from the head, 2026-11-03, data/excite/excite_s7_a.json)

6 trials (steps ±0.3, ±0.4 rad; sweeps 0.2–1.4 Hz), commanded kp 0.3 / kd 0.02, motors 43–46 °C, bus peak 0.80 A, no aborts. The tail reached 0.286 / 0.396 rad on the
0.3 / 0.4 rad steps. Closed-loop simulation fit to measured position and torque (position rmse 0.017 rad = 3–8% of travel; torque rmse 0.003 N·m):

| quantity | result | confidence |
|---|---|---|
| effective kp | 0.285 N·m/rad (commanded 0.3, ratio 0.95) | spread 36%: **the gain units are N·m/rad as assumed** (the earlier "2.2x" claim was wrong) |
| effective kd | 0.0198 N·m·s/rad (commanded 0.02, ratio 0.99) | spread 27%: units confirmed |
| Coulomb friction C | 0.0057 N·m | spread 34%; **agrees with the independent torque-pulse breakaway bracket 0.0037–0.0075** |
| stiffness K | −0.008 N·m/rad (tiny, slightly negative) | consistent with the predicted ~0 gravity lever for the tail; too small to explain the runaway at kp 0.1 |
| command delay | ~20 ms | poorly pinned (spread 83%) |
| viscous friction B | ~0.0004–0.0007 N·m·s/rad (small) | weakly determined |
| **inertia J** | **not identified; upper bound ≈ 1e-3 kg·m², most likely 1–5e-4** | profile of the fit loss at fixed J: ×1.00 (1e-4), ×1.02 (3e-4), ×1.07 (6e-4), ×1.23 (1.2e-3), ×1.64 (2.4e-3, which would also need kp 0.73 / kd 0.08) |

The mock's 0.015 kg·m² is at least ~15x too large for the tail. A real inertia estimate needs faster excitation than 50 Hz telemetry resolves (or a held-joint measurement), which this run cannot provide.

#### RESULT: hanging test, servo 4 (robot hung from the head, 4 modules below, default gains, 2026-11-03, data/hang/hang_s4_a.jsonl)

26 static holds, angle swept to about +0.09 / −0.17 rad from both directions. Motors 45–47 °C, bus peak 0.92 A, no aborts, max |tau| 0.111 N·m.
- **Quiet:** holding still, angle jitter 0.0005 rad, torque jitter 0.0004 N·m at the gl40.py gains (kp 0.8, kd 0.035).
- **Gravity stiffness:** the straight-line fit gives K = 0.645 ± 0.054 N·m/rad (predicted 0.478 with 0.275 kg per module, ratio 1.35); this is inflated by the hysteresis
  (the secant τ/θ at the largest negative angle is 0.48). Whatever the exact value, the three modules below carry more weight at the far end than a uniform split: the fit implies
  m5 + 2·m6 + 3·m7 ≈ 2.2 kg·modules (uniform 0.275 kg gives 1.65), e.g. a tail module of ~0.5 kg. Module masses still need weighing.
- **Joint friction is large and asymmetric:** Coulomb c = 0.020 ± 0.004 N·m on average (the tail: 0.0057), but the up/down gap at the same angle is ~0.09 N·m on the positive side
  and ~0.03 N·m on the negative side, and the angle at zero command differs by 0.03 rad between the start and end of the sweep. The residual 0.020 N·m shows a constant-c
  model is a simplification (stick-slip/backlash).
- **Current model, clean static test:** delta_bus = 13.2 A/(N·m)² · tau² (±6%; individual holds 11–15) against 60.1 in the power manager's model at 14.5 V: **the model over-predicts
  the bus current ~4.5x** (measured/model 0.22). This matches the earlier moving-joint result (0.19–0.24), so it is repeatable. So the torque cap and the 2 A "model peak" are far more
  conservative than the measured bus current. Heating depends on the phase current (tau/Kt), not bus current, so this does NOT show that higher torques are thermally safe. Nothing has been changed.

#### RESULT: hanging test, servo 3 (5 modules below, default gains, 2026-11-03, data/hang/hang_s3_a.jsonl)

28 static holds, motors 44–47 °C, bus peak 0.95 A, max |tau| 0.061 N·m (the 0.30 rad command range ended before the 0.10 N·m stop), no aborts. Quiet when still (angle 0.0008 rad, torque 0.0006 N·m).
- **Current model: reproduced.** delta_bus = 13.4 A/(N·m)² · tau² (±18%) vs 60.1 in the model: measured/model = 0.22, the same as servo 4 (13.2, 0.22) and the earlier moving-joint run (0.19–0.24). Two joints, two loads, static and
  moving agree: **the power manager's current model over-predicts bus current by ~4.5x in this setup.**
- **Friction:** Coulomb c = 0.0082 ± 0.0005 N·m with a tight linear fit (residual 0.0025 N·m): clean, unlike servo 4 (c ≈ 0.020, residual 0.020, strong asymmetric hysteresis). Joint friction varies ~3.5x between joints: tail 0.0057, servo 3 0.0082, servo 4 0.020.
- **Gravity stiffness does not behave like an isolated joint:** K(servo 3) = 0.268 ± 0.004 vs K(servo 4) = 0.645 ± 0.054 N·m/rad, i.e. measured/predicted 0.34 and 1.35. For a hanging chain the stiffness must grow toward the head (servo 3 has more
  mass below it than servo 4), so a single mass scaling cannot explain both. Suspected (not verified): the other joints, held by kp 0.8 springs, deflect as the tested joint rotates the chain (axes alternate, so rotating one joint loads its
  neighbours), so the measured stiffness is a coupled-system quantity. The masses therefore cannot be inferred from these sweeps; weigh the modules, or fit all seven joints together.

#### Coupled gravity model validated; one-run characterisation built (2026-11-03)

`petctl/gravity_model.py`: exact static model of the hanging robot from the CAD geometry (joint axes and module positions), 0.275 kg per module with the centre of mass on the motor axis (Nick), every joint held by
`tau = kp·(cmd − phi)` springs (kp 0.8, confirmed in N·m/rad). It reproduces the simple closed form for an isolated joint (1.70, 1.22, 0.81, 0.50, 0.25, 0.09, 0 N·m/rad) but shows that with the neighbours held
the stiffness seen at a joint is lower and falls monotonically (servo 1…7: 0.86, 0.47, 0.36, 0.27, 0.18, 0.08, 0), which resolves the earlier "non-monotonic" servo 3 vs 4 puzzle (it was coupling through the held joints).
With **no free parameters** it predicts the torques of the *other* joints in the existing runs: servo 3 test: joint 2 slope 0.91 (corr 0.99), joint 3 0.81 (0.98), joint 4 0.98 (0.99), joint 5 0.55, joint 6 0.32; servo 4 test:
joint 2 0.98 (0.84), joint 3 0.70 (0.96), joint 5 0.51, joint 6 0.35. The tested joint's own torque is inflated by its friction (servo 4: 1.8x). So the 275 g / motor-axis model is right to about the accuracy of the friction.

`scripts/capture_all.py`: one automated run that sweeps all seven joints in succession (hysteresis sweeps, then steps and a gentle chirp each) and then in groups (adjacent pairs, a travelling wave over all seven, six static whole-body poses), logging every
joint's angle, torque and velocity, the commands, bus current/voltage, temperatures, power-manager caps and every motor feedback frame at its true arrival time to one `.npz`. Offline analysis follows from the raw data. Dry-run on the mock
(78 s at quick scale); the real run is ~8–9 min with cool-down waits and the usual stops (4.5 A bus, 52 °C, 0.6 rad per joint, power-manager intervention).

