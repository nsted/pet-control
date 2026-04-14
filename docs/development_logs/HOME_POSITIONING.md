# Home Positioning — Development Log

**Scope:** How the real robot and the 3D visualization establish and agree on "home."
**Period covered:** Feb 20 – Mar 2, 2026 (36 commits)
**Current status:** Functionally resolved, with one known caveat (multi-turn).

---

## What "Home" Means in This System

"Home" has two coupled meanings:

| Layer | Definition | Lives in |
|---|---|---|
| **Real robot** | The physical pose where every servo reports `position_center` raw ticks | EEPROM offset register per servo + `config.py` |
| **Visualization** | The 3D mesh arrangement when all joint angles are 0° | `robot_assembly.json` rotation fields |

These two are **tightly coupled**: when a servo is at its physical home, it must report `position_center` ticks, and the `rotation` values in `robot_assembly.json` must encode the world-frame pose of each module at that same physical home. If either side shifts, the viz and the real robot diverge.

---

## The Hardware Home: EEPROM Offset Register

### What the servo reports

The Feetech HLS servo has two position concepts:
- **Raw encoder** — the absolute multi-turn counter (15-bit, ±32767 ticks). Resets to 0 on power cycle.
- **Reported position** — `raw_encoder - stored_offset`. This is what `ReadPos` and `GroupSyncRead` return. `stored_offset` persists through power cycles in EEPROM.

Home calibration means: choose a physical pose, set the EEPROM offset so that pose reports exactly `position_center` (2048) on every boot.

### `write_home_offsets` — three iterations

The procedure was implemented and fixed three times across Phase 6, 7, and 9.

**Attempt 1** (`5a4dc27`, Feb 26) — Read existing offset first, then compute delta.
- **Problem:** The "read existing offset" step read stale EEPROM values, and the delta math compounded errors. Positions didn't land at `position_center`.

**Attempt 2** (`9803dc9`, Feb 27) — Zero offset first, read raw encoder, write raw as new offset.
- **Problem:** This made the servo report `0` at home (raw encoder − offset = raw − raw = 0), not `position_center`. Worked in-session but broke behavior engine math expecting 2048 as center.

**Attempt 3** (`81437c7`, Mar 1) — Current implementation. Procedure per servo:
```
1. unLockEprom → write offset = 0,0 → LockEprom
2. ReadPos → this is raw_encoder (no offset applied)
3. new_offset = (raw_encoder % 4096) - position_center
4. Encode as sign-magnitude (scs_toscs), write to EEPROM
```

The `% 4096` is key: it strips the multi-turn accumulated count so only the within-revolution encoder value (0–4095) is stored. This is the only part that survives a power cycle. After reboot:
```
reported = (raw_encoder_at_home % 4096) - stored_offset
         = (raw_encoder_at_home % 4096) - ((raw_encoder_at_home % 4096) - 2048)
         = 2048  ✓
```

**Known caveat** (documented in code): This procedure must be run immediately after power-on, before any movement. If the robot has moved (e.g., accumulated ±3 turns = raw_pos ≈ 14000), `raw_pos % 4096 ≈ 1600`, which produces the wrong offset for a non-zero physical displacement. The stored offset will be wrong after the next power cycle by the accumulated rotation.

### Resulting behavior

After a successful `write_home_offsets` and power cycle:
- All servos report `2048` at their calibrated physical home.
- `raw_to_angle(2048) = 0.0°` — they read as zero-angle.
- The viz renders the pose from `robot_assembly.json` rotations with zero joint contribution — the correct home pose appears.

---

## The Visualization Home: `robot_assembly.json` Rotations

Each module in `robot_assembly.json` has a `rotation: [H, P, R]` field in Panda3D HPR convention (Heading=Z, Pitch=X, Roll=Y). This encodes the module's mesh orientation in world-frame **when the joint angle is zero**.

Current values:
```json
module 0 (head):   [180, 0, 180]
module 1:          [180, 0, 180]
modules 2–7:       [0, 90, 180]
```

These were tuned empirically on Feb 22 (`5a4dc27`) to match the physical robot's resting pose.

### The coupling constraint

If `position_center` is changed (e.g., from 2048 to 0), `raw_to_angle` shifts its zero point. A servo physically at home now reports a non-zero angle, which the viz adds as a joint rotation on top of the HPR rotation — the mesh rotates away from its intended pose. The fix is to subtract the equivalent angle from every module's Heading in `robot_assembly.json`.

This is why the Feb 28 refactor to `center=0` was **reverted on Mar 1**: it changed the HPR convention throughout without a complete audit of downstream effects. `center=2048` is the current single source of truth.

---

## The Viz Direction Fix

On Feb 26 (`2cdad9a`), the visualization was found to be a **mirror** of the real robot: when a servo moved clockwise, the 3D mesh rotated counter-clockwise.

**Cause:** Feetech HLS uses the Dynamixel CCW-positive convention (positive raw tick = counter-clockwise when viewed from the motor face). Rerun's right-hand-rule Z rotation is the opposite for how the joint axes are oriented in `robot_assembly.json`.

**Fix:** Negate the servo angle before applying it as a joint rotation in `_log_3d_pose`:
```python
# rerun_viz.py:335
angle_rad = -self._servo_angle_rad(mod_id, state)
```

This is a pragmatic sign correction applied once at the viz boundary. The real robot and control schemes remain unaffected — all internal representations use raw ticks relative to `position_center`.

---

## The `center=0` Refactor and Revert

### What was attempted (Feb 28, `119a512`)

A major refactor changed `position_center` from 2048 to 0, converting all internal representations to signed angles centered at zero. The goal was to eliminate the awkward "subtract 2048 everywhere" pattern in angle math.

**Files changed:** 8 files across config, types, backends, schemes, visualizer, assembly JSON.

### Why it failed

1. **Assembly JSON rotations** — the `rotation` values in `robot_assembly.json` encode the home pose for `center=2048`. Switching to `center=0` requires subtracting 180° from each module's Heading, but the refactor didn't update all of them consistently.
2. **Sign-magnitude encoding** — the servo wire protocol uses Feetech sign-magnitude (bit 15 = sign, bits 0–14 = magnitude), not two's complement. The `position_center=0` version conflated signed Python ints with signed wire values in `_write_pos_ex` and `_sync_write_all`.
3. **EEPROM offset logic** — `write_home_offsets` was written for `center=2048`. Switching would require rewriting the offset formula.

### Revert (`431833f`, Mar 1)

Reverted all 7 changed files to restore `position_center=2048` as the single source of truth. The commit message: "the center=0 refactor from Feb 28 caused downstream issues."

**Current decision:** `center=2048` is fixed. All angle conversions go through `config.angle_to_raw` / `config.raw_to_angle`. The `% 4096` in `write_home_offsets` handles the multi-turn/EEPROM boundary correctly only for this value.

---

## The Save-Home Key: 4 Commits in 6 Minutes

The keyboard binding for triggering `write_home_offsets` went through four iterations (Feb 27) before settling:

| Key | Problem |
|---|---|
| `'s'` | Too easy to hit accidentally while typing |
| `Ctrl+S` | Sends XOFF signal in some terminals, freezes stdout |
| backtick | Also too easy to hit accidentally |
| `Cmd+\`` | Final binding — unusual chord, low accidental trigger risk |

---

## Current Resolved State

| Aspect | Status | Implementation |
|---|---|---|
| EEPROM home calibration | Working (third attempt) | `robot.py:write_home_offsets` |
| Viz-robot direction match | Working | Negation in `rerun_viz.py:335` |
| Coordinate system | Settled on `center=2048` | `config.py:SERVO_LIMITS.position_center` |
| Assembly JSON tuned | Yes (Feb 22) | `robot_assembly.json` all 8 modules |
| Mock backend home | `position_center` ticks | `mock.py:90` |

---

## Open Issues

### 1. Multi-turn offset corruption (known, documented)

If `write_home_offsets` is called after the robot has moved significantly (>1 turn), the `% 4096` modulo produces an offset that is correct for today's session but wrong after the next power cycle. There is no runtime guard against this.

**Mitigation today:** User documentation says to run in limp mode immediately after power-on. An actual guard would require reading the multi-turn counter and refusing if it's non-zero.

### 2. In-session position mismatch after EEPROM write

After calling `write_home_offsets`, positions do **not** immediately read `position_center` in the current session — the EEPROM reload updates the register but the multi-turn counter keeps its current value. Positions only snap to `position_center` after a power cycle.

This means the viz will show a slight offset from the true home until the robot is rebooted. The code comment notes this as an accepted trade-off.

### 3. MEMORY.md stale value

`MEMORY.md` states `center = 0, negative values allowed` — this is leftover from the Feb 28 refactor attempt and is incorrect. Actual value is `center = 2048`.

---

## Key Files

| File | Role |
|---|---|
| `petctl/config.py:26` | `position_center = 2048` — single source of truth |
| `petctl/config.py:110-117` | `angle_to_raw` / `raw_to_angle` helpers |
| `petctl/backends/robot.py:886` | `write_home_offsets` implementation |
| `petctl/visualizers/rerun_viz.py:335` | Viz direction negation |
| `petctl/assets/robot_assembly.json` | Module HPR rotations for home pose |
| `petctl/backends/mock.py:90` | Mock backend home = `position_center` |
