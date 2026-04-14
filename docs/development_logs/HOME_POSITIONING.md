# Home positioning — PET / petctl

**Scope:** How the real robot and the 3D visualization agree on “home.”

---

## Current stack (CubeMars MIT)

| Layer | Definition | Where it lives |
|--------|------------|----------------|
| **Control / schemes** | Joint angles in **radians**; `0` is the desired home pose after calibration. | `ServoCommand`, `RobotState.servo_positions`, `petctl/config.py` (`MOTOR_LIMITS`) |
| **Robot backend** | **Save home** (`write_home_offsets` / `set_home`) stores each motor’s current reported position (rad) in `_angle_offsets`. Outgoing MIT position commands use `cmd.position + offset` so commanding `0` holds the pose that was saved. | `petctl/backends/robot.py` |
| **Visualization** | Meshes are arranged for **zero** joint rotation at home. | `petctl/assets/robot_assembly.json` HPR `rotation` fields |

**Limp + save home (user flow):** See project `README.md` — physically pose the robot, run with `--limp`, press **Cmd+`**, then power-cycle or reconnect as appropriate for your firmware.

**Viz vs hardware joint sign:** The Rerun joint rotation applies a **negation** so the mesh matches the physical joint sense for the axes defined in `robot_assembly.json`. See comment near servo angle logging in `petctl/visualizers/rerun_viz.py`.

---

## Historical note (pre–CubeMars migration)

Earlier PET stacks used **Feetech HLS** servos with **raw tick** positions (12-bit within-revolution range, EEPROM offset, `% 4096` offset math, `position_center` ≈ 2048, sign-magnitude wire encoding). That model is **not** used by current `petctl`: there are no tick fields in `petctl/config.py`, and `RobotBackend` talks **CubeMars MIT** over SLCAN/WebSocket.

Commit history around **Feb 20 – Mar 2, 2026** (e.g. home offset attempts, `center=2048` vs `center=0` revert, viz mirror fix) describes that era. If you need the old step-by-step EEPROM narrative for archaeology, use `git show` on the commits referenced in `MASTER_DEVELOPMENT_LOG.md` from that window, or retrieve an older revision of this file.

---

## Keyboard save-home chord

The binding for save-home went through several iterations before settling on **Cmd+`** (low accidental trigger risk). See `petctl/schemes/keyboard.py`.
