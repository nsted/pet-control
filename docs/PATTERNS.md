# Motion Patterns

All patterns are selected via `petctl run --control <name>`.  
Speed-sensitive patterns respond to `--vel 0.0–1.0`.

| Name | Class | Motion character | Touch reactive | Speed param | `--vel` effect |
|---|---|---|---|---|---|
| `snuggle` | `SnuggleMotion` | Two travelling wave crests across the body simultaneously — shorter spatial wavelength makes the body squirm | No | `hz` (default 0.4) | Scales `hz` |
| `walk` | `WalkMotion` | Alias for `snuggle` — used as the LLM's "walk" label | No | `hz` | Scales `hz` |
| `pulse` | `PulseMotion` | All joints in phase — whole-body contraction and release | No | `hz` (default 0.25) | Scales `hz` |
| `cascade` | `CascadeMotion` | Travelling wave with amplitude growing head→tail (crack-the-whip); head barely moves, tail whips | No | `hz` (default 0.2) | Scales `hz` |
| `wiggle` | `SlalomMotion` | Odd/even joints get opposing phase — persistent S-shape that rocks side to side | No | `hz` (default 0.2) | Scales `hz` |
| `twitch` | `TwitchMotion` | Per-joint smoothed Brownian noise — organic, jittery, each joint wanders independently | No | `smoothing` (EMA alpha, default 0.06) | No effect (smoothing param, not speed) |
| `freeze` | `FreezeMotion` | Command all joints to 0° and hold — rigid stop | No | None | No effect |
| `idle` | `IdleMotion` | Motors stay in MIT mode (green light on) but freespin — kp=kd=0, no torque | No | None | No effect |
| `coil` | `CoilMotion` | Quadratic spatial phase — curl accumulates toward the tail, producing a tightening coil effect | No | `hz` (default 0.15) | Scales `hz` |
| `curl` | `CurlMotion` | Ramp all joints to 70° over 8s with alternating signs (slalom pattern), then hold — loops the snake | No | None | No effect |
| `stroke` | `StrokeReactMotion` | Each module spins toward the touching hand; lateral stroke on left face → spin one way, right face → other | Yes — lateral stroke | None | No effect |
| `stroke-curl` | `StrokeCurlMotion` | Each module curls toward its touched face; holds position for 2s after release, then returns home | Yes — per-module touch | None | No effect |
| `touch-curl` | `TouchCurlMotion` | Idle until touched; curl toward the hand with full torque; 3s torque fade on release then freespin | Yes — per-module touch | None | No effect |
| `curl-towards` | `CurlTowardsMotion` | Identical to `stroke-curl` — provided as a clearer alias | Yes — per-module touch | None | No effect |
| `engage` | `CurlTowardsNeighborAssistMotion` | `curl-towards` + neighbor-assist stall recovery: when a module stalls, adjacent neighbors briefly push opposite to relieve mechanical strain | Yes — per-module touch | None | No effect |
| `withdraw` | `CurlAwayMotion` | Mirror of `curl-towards` — curls away from the touched face instead of toward it | Yes — per-module touch | None | No effect |
| `nuzzle` | `StrokeSnuggleMotion` | Starts as `stroke-curl`; after 15s of continuous stroking transitions to a full-body snuggle wave for 15s, then goes home | Yes — sustained stroke | None | No effect |
| `explore` | `ExploreMotion` | Each joint turns at a fixed speed in a random direction, reversing when it stalls (position stuck + torque high) or hits the position ceiling | No | `speed_deg_per_s` (default 45°/s) | Scales `_speed_rad_s` |
| `seek-touch` | `SeekTouchMotion` | Like `explore`, but each module freezes (zero-torque hold) as soon as it detects contact; resumes seeking when the hand leaves | Yes — per-module touch stops movement | `_speed_rad_s` (45°/s) | Scales `_speed_rad_s` |
| `avoid-touch` | `AvoidTouchMotion` | Idle until touched; when contact detected, move away from the touching face; direction chosen by left/right face balance | Yes — per-module touch drives movement | `_speed_rad_s` (45°/s) | Scales `_speed_rad_s` |
| `contort` | `DriftMotion` | All joints share a single speed that varies sinusoidally between 15–90°/s over 12s; stall detection and reversal same as `explore` | No | Speed oscillates via sine — no fixed param | No effect (dynamic speed) |
| `struggle` | `StruggleMotion` | Copy of `explore` with separate class constants for tuning; velocity-based stall detection instead of position-based | No | `speed_deg_per_s` (default 45°/s) | Scales `_speed_rad_s` |
| `writhe` | `NeighborAssistDriftMotion` | `contort` kinematics + neighbor-assist stall recovery; same oscillating speed as `contort`, but stalled motors get neighboring assist nudges | No | Speed oscillates — no fixed param | No effect (dynamic speed) |
| `yield` | `YieldStiffMotion` | Setpoint drifts toward actual position when motor torque is high — joint yields to force but holds wherever it ends up on release | No (torque-driven) | None | No effect |
| `pose` | `PoseMotion` | Track actual position while a hand is present and moving the joint; lock on release — lets a human sculpt the robot's pose | Yes — hand presence + displacement gate | None | No effect |
| `balanced-torque` | `BalancedTorqueMotion` | Seek a pose where all motors carry equal load at a 1A combined target; under-loaded joints creep outward, over-loaded retreat toward home | No (torque-driven) | None | No effect |
| `purr` | `PurrRippleMotion` | kd-vibration wave propagating head→tail; elevated kd resists perturbations, producing a purring texture; position stays at 0° | No | `speed` (ripple hz multiplier, default 1.0) | No effect (uses `speed` not `hz`) |
