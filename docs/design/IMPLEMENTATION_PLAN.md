# PET Touch Interactivity — Implementation Plan

## Architecture Overview

```
Robot sensors (20Hz)
    ↓
petctl (normalize, stream)
    ↓
Perception: SensorStream → TouchClassifier → TouchEvent
    ↓
Valence: TouchEvent × valence_map → signed float (-1 to +1)
    ↓                                         ↑
Reflex: detect transitions, inject burst      │ valence_map
    ↓                                         │
BehaviorEngine: valence modulates all     Agent (Claude API)
active behaviors continuously                 ↑
    ↓                                    TouchEvent history,
ServoCommands → Robot                    valence history,
                                         current mood
```

Key design principles:
- Valence is not binary. The agent decides what the robot finds pleasant or aversive, and this changes with mood.
- Reflexes (flinch, startle) fire instantly on touch-type transitions, before mood-mediated responses.
- Behaviors run continuously and are modulated by valence, not switched on/off discretely.
- The agent updates mood every few seconds. Between updates, a local valence function provides instant reactions.

---

## Stage 1 — Behavior Engine + Core Behaviors

**Goal:** Get parametric behaviors running on mock backend with Rerun visualization. No classifier, no agent — just behaviors driven by raw sensor data.

**Test:** `python examples/behavior_demo.py --behavior combo` shows the robot undulating and nestling toward mock sine-wave touch in Rerun.

### Prompt 1.1 — Behavior engine scaffold

```
Read the design docs in docs/behaviors/. Implement a new petctl/behaviors/ 
package with:

1. engine.py — BehaviorEngine class that implements ControlScheme. 
   - Behaviors produce per-module angle CONTRIBUTIONS (degrees), not absolute 
     positions. The engine sums weighted contributions from all active behaviors.
   - Handles fade-in/fade-out via weight interpolation.
   - Applies exponential smoothing on final output.
   - Clamps to ±45° per module.
   - servo_id == module_id (offset 0) in our robot.

2. Base Behavior ABC with:
   - update(state, params, dt) → dict[int, float]  (module_id → angle degrees)
   - reset()
   - BehaviorParams dataclass: intensity, speed, focus_modules, contact_face, extras dict

Keep it simple. Follow the existing code patterns in petctl/ (type hints, 
docstrings, dataclasses). Don't implement any specific behaviors yet.

Test: import and instantiate BehaviorEngine, call update() with an empty 
RobotState, verify it returns an empty command list.
```

### Prompt 1.2 — Nestle behavior

```
Implement petctl/behaviors/nestle.py — a reactive nestle behavior.

Core logic: each module reads its touch sensors and bends TOWARD the 
touched face.
- Left touch → positive joint angle (with pressure weighting)
- Right touch → negative joint angle
- Middle touch (back face) → neighbors bend toward each other, 
  cupping around the touch point
- Neighbor influence: touch on module N affects N-1 and N+1 with 
  configurable spread (default 2) and falloff (default 0.5 per step)

All joints are single-axis Z revolute. Positive = one direction, 
negative = the other.

Constructor params: gain (default 30°), spread (default 2), 
falloff (default 0.5), middle_curl_gain (default 15°).

If params.focus_modules is set, only affect those modules 
(plus their neighbors within spread).

Test with mock backend in sine mode — the robot should visibly 
curve toward the simulated touch in Rerun.
```

### Prompt 1.3 — Recoil, undulate, posture behaviors

```
Implement three more behaviors following the same Behavior ABC pattern:

1. petctl/behaviors/recoil.py — opposite of nestle, bend AWAY from touch.
   Add a "startle" component: on sudden touch onset, overshoot briefly 
   (1.8x for ~300ms) then settle. Track per-module previous touch to 
   detect onset.

2. petctl/behaviors/undulate.py — traveling sine wave along the body chain.
   This is the robot's idle "breathing." Phase offset per module creates 
   traveling wave. Speed param maps to frequency (0.2→very slow, 1.0→fast 
   agitated). Extras: wavelength (modules per cycle), asymmetry (lean bias).

3. petctl/behaviors/posture.py — two simple behaviors:
   - GoLimpBehavior: drive all joints toward 0° at controlled speed
   - StiffenBehavior: capture current pose on activation, output those 
     angles (resists other behaviors when blended)

Register all behaviors in a build_engine() helper function.
```

### Prompt 1.4 — Behavior demo + CLI integration

```
1. Create examples/behavior_demo.py that runs BehaviorEngine with 
   MockBackend(mode="sine") and RerunVisualizer. Accept --behavior 
   and --intensity args. Include a "combo" mode that layers 
   undulate + nestle.

2. Add --control behavior option to petctl CLI (cli.py) alongside 
   keyboard and passthrough. When selected, create a BehaviorEngine 
   with all core behaviors registered and activate undulate + nestle 
   as defaults.

Test: petctl run --control behavior --backend mock --mode sine
Should show the robot undulating with nestle response to simulated touch.
```

---

## Stage 2 — Perception Layer (Touch Classification)

**Goal:** Classify raw sensor streams into structured TouchEvent objects. Start with heuristic rules, design the interface for a future trained model.

**Test:** Run with mock sine sensors and see classified events printed to console.

### Prompt 2.1 — Sensor stream and touch event types

```
Create petctl/perception/ package with touch_events.py containing:

1. TouchType enum: IDLE, STROKE, PAT, STRIKE, HOLD, SQUEEZE, RELEASE

2. TouchEvent dataclass:
   - touch_type, confidence (0-1)
   - primary_modules (list[int]), primary_face (str or None)
   - intensity (0-1), velocity (float, positive=head→tail)
   - timestamp, duration
   - module_activations: dict[int, float] — per-module total touch

3. SensorStream class:
   - Maintains a rolling deque of sensor frames (window_length=50)
   - push(state: RobotState) extracts 48 features per frame 
     (8 modules × 6 channels: touch_middle, touch_left, touch_right, 
     pressure_middle, pressure_left, pressure_right)
   - get_window() returns 2D list or None if not full
   - Design so IMU channels can be appended later (just extend the 
     feature vector, change FEATURES_PER_FRAME from 48 to 58)

This is pure data structures, no classification logic yet.
```

### Prompt 2.2 — Heuristic touch classifier

```
Implement HeuristicTouchClassifier in petctl/perception/touch_events.py.

This is a rule-based classifier for bootstrapping — no ML needed. 
It will be replaced by a trained model later but the interface 
(takes sensor stream + state, returns TouchEvent) stays the same.

Classification rules:
- STRIKE: sudden onset (large delta in total touch over 2-3 frames), 
  high intensity, short duration (<0.3s)
- STROKE: touch center-of-mass moving along the body over time. 
  Compute weighted centroid of touch across modules, track velocity. 
  Velocity > threshold = stroke.
- HOLD: sustained static contact > 1 second
- PAT: brief (<0.5s), moderate intensity, no movement
- RELEASE: transition from any active touch to idle
- SQUEEZE: increasing pressure without touch movement (stretch goal)
- IDLE: no meaningful contact

Track state transitions for RELEASE detection. 
Output confidence based on how well the signal matches the pattern.
```

### Prompt 2.3 — Wire perception into the control loop

```
Create petctl/perception/perception_layer.py with a PerceptionLayer class 
that:
- Owns a SensorStream and a TouchClassifier (heuristic for now)
- Has an update(state: RobotState) → TouchEvent method
- Can be plugged into the Controller loop (either as a component of 
  BehaviorEngine or as a standalone step)

Modify BehaviorEngine to optionally accept a PerceptionLayer. When present, 
each tick it:
1. Pushes state to perception
2. Gets TouchEvent back
3. Makes the TouchEvent available to behaviors and to external consumers 
   (the agent layer will need it)

Add a --perception flag to the CLI that enables perception layer and 
prints classified TouchEvents to console every 0.5s.

Test: petctl run --control behavior --backend mock --mode sine --perception
Console should show touch events being classified as the sine wave 
activates different modules.
```

---

## Stage 3 — Valence System + Reflexes

**Goal:** Touch events drive behavior through a valence function. Reflexes handle sudden transitions. No agent yet — use a static valence map.

**Test:** Mock sine sensors produce smooth nestling. Injecting a simulated strike (via state file) causes immediate flinch then valence-driven recoil.

### Prompt 3.1 — Valence function

```
Create petctl/affect/valence.py with:

1. ValenceMap dataclass — maps TouchType → float (-1 to +1):
   Default: stroke=+0.6, pat=+0.4, hold=+0.2, strike=-0.8, 
   squeeze=-0.4, idle=0.0, release=0.0

2. ValenceFunction class:
   - Takes a TouchEvent + current ValenceMap
   - Returns a float: signed valence (-1 to +1)
   - Valence = map[touch_type] × event.intensity
   - Smooths over time (exponential moving average, ~0.5s window) 
     so valence doesn't flicker on noisy classifications

3. ValenceToMotor class — maps continuous valence to behavior modulation:
   - nestle_weight = smooth_clamp(valence, 0, 1)
   - recoil_weight = smooth_clamp(-valence, 0, 1)  
   - undulate speed increases with abs(valence) (more aroused = faster)
   - undulate amplitude decreases with abs(valence) (strong reaction 
     suppresses idle motion)
   - Calls BehaviorEngine.request() with updated params each tick

The ValenceMap is what the agent will eventually update. For now it's static.
```

### Prompt 3.2 — Reflex layer

```
Create petctl/affect/reflexes.py with a ReflexLayer class:

- Watches for TRANSITIONS in TouchType (not steady state)
- When a transition fires (e.g. idle→strike, stroke→strike, hold→release):
  - Injects a time-limited behavior override into BehaviorEngine
  - Override has high weight and fast fade (duration ~200-500ms)
  - Bypasses the smoothing in BehaviorEngine for the initial burst

Reflex table (configurable):
  idle → strike:    sharp recoil burst, intensity 1.0, 300ms
  idle → hold:      mild stiffen, intensity 0.3, 200ms (startle)
  stroke → strike:  sharp recoil, intensity 1.0, 400ms (interrupted)
  hold → release:   brief undulate burst (relaxation), 300ms
  any → squeeze:    stiffen burst, intensity 0.6, 250ms

Not all transitions need reflexes. Stroke→hold is gentle, no reflex.

The reflex fades naturally via the BehaviorEngine's weight system, 
then the valence-driven sustained response takes over.
```

### Prompt 3.3 — Wire valence + reflexes into BehaviorEngine

```
Update BehaviorEngine to integrate valence and reflexes:

1. BehaviorEngine now owns: PerceptionLayer, ValenceFunction, 
   ValenceToMotor, ReflexLayer

2. Each tick:
   a. perception.update(state) → TouchEvent
   b. reflexes.check(touch_event) → optional reflex burst injected
   c. valence_function.compute(touch_event) → valence float
   d. valence_to_motor.apply(valence, touch_event) → updates behavior 
      weights and params
   e. All behaviors run, blend, output servo commands

3. Expose current_valence, current_touch_event, and recent_events 
   (last 30 seconds) as properties — the agent layer will read these.

4. Add a set_valence_map(map) method that the agent will call to 
   update the valence mapping.

Test with mock backend: steady sine touch → smooth nestle. 
Edit the mock state JSON file to inject a sudden spike → should see 
immediate flinch then sustained recoil.
```

---

## Stage 4 — Agent Integration (Claude API)

**Goal:** Claude API calls on a slow cycle update mood and valence map. The robot's disposition changes over time based on interaction history.

**Test:** Extended mock interaction where the agent shifts mood and the robot's responses visibly change character.

### Prompt 4.1 — Agent context builder

```
Create petctl/agent/context.py with an AgentContext class that builds 
the text prompt for the Claude API call.

It reads from BehaviorEngine's exposed state:
- recent_events (last 30s of TouchEvents)
- current valence and mood
- Robot physical state (if available: battery, temperature)
- Interaction summary: duration of current session, number of touch 
  events, predominant touch types

Output: a concise natural language summary, NOT raw data.
Example: "Sustained gentle stroking on modules 2-4 for 12 seconds 
(positive valence, +0.6). No other contact. Robot is in a calm, 
receptive mood. Session duration: 3 minutes."

Keep token count low — this runs every few seconds. Target < 500 tokens 
for the context prompt.
```

### Prompt 4.2 — Agent mood model

```
Create petctl/agent/mood.py with:

1. MoodState dataclass:
   - arousal: float (0=calm, 1=agitated)
   - receptiveness: float (0=aversive, 1=welcoming)
   - energy: float (0=lethargic, 1=vigorous)
   - playfulness: float (0=serious, 1=playful)
   - valence_map: ValenceMap
   - behavior_preferences: dict with behavior-specific params

2. MoodAgent class:
   - async update(context: AgentContext) → MoodState
   - Makes a Claude API call (claude-sonnet-4-20250514) with:
     - System prompt defining the robot's personality and the mood 
       dimensions
     - The context summary
     - Instruction to output JSON with updated mood dimensions and 
       valence map
   - Parses response into MoodState
   - Handles API errors gracefully (keep last mood on failure)

3. System prompt for the agent should explain:
   - You are the emotional core of a robot called PET
   - You experience touch through sensor events
   - Your mood dimensions and what they mean
   - Stroking is not always good — your reaction depends on your 
     current state
   - You should be complex and unpredictable like a real creature
   - Output format (JSON)

Include a MockMoodAgent that returns slowly shifting random moods 
for testing without API calls.
```

### Prompt 4.3 — Agent loop integration

```
Create petctl/agent/agent_loop.py with an AgentLoop class:

- Runs as an asyncio task alongside the main control loop
- Every N seconds (configurable, default 5):
  1. Build context from BehaviorEngine state
  2. Call MoodAgent.update(context)
  3. Apply new MoodState:
     - Update ValenceMap via BehaviorEngine.set_valence_map()
     - Update behavior preferences (undulate base intensity, etc.)
  4. Interpolate smoothly — don't snap to new mood, blend over 2-3s

- The agent loop NEVER blocks the control loop. It runs fully async.
- If the API call takes longer than the update interval, skip that cycle.

Wire into Controller or BehaviorEngine. Add CLI flags:
  --agent          Enable agent loop (default: off)
  --agent-mock     Use MockMoodAgent instead of real API
  --agent-hz 0.2   Agent update frequency (default: every 5 seconds)

Test with --agent-mock: mood should drift and robot's touch responses 
should gradually change character over 30-60 seconds.
```

### Prompt 4.4 — Real agent testing

```
Test the full stack with the real Claude API:

petctl run --control behavior --backend mock --mode sine \
  --perception --agent

Watch for:
1. Agent receives touch event summaries
2. Agent outputs mood changes as JSON
3. Valence map updates affect robot response
4. Mood persists after touch ends (robot stays agitated/calm)
5. Agent makes contextually interesting decisions (e.g. becomes 
   less receptive after prolonged contact)

Iterate on the agent system prompt if responses are too predictable 
or not grounded in the sensor data. The goal is a creature with 
genuine-seeming moods, not a chatbot deciding to be happy or sad.
```

---

## Stage 5 — Real Robot Testing + Tuning

**Goal:** Everything running on the physical robot. Tune gains by feel.

### Prompt 5.1 — Real robot integration test

```
Test the full behavior stack on the real robot:

petctl run --control behavior --backend robot --perception

Expected issues to debug:
- Gain values will be wrong — nestle gain of 30° may be way too 
  much or too little on the real hardware. Start at 10° and increase.
- Sensor calibration may need adjustment for the classifier — 
  thresholds tuned on mock sine waves won't match real touch.
- Servo speed/acceleration params may need tuning for smooth motion.
- The control loop may be too slow at 20Hz with all layers running — 
  profile and optimize if needed.

Add a --gain-scale CLI flag (float multiplier on all behavior gains) 
for quick tuning without editing code.
```

### Prompt 5.2 — Data collection mode

```
Add a data collection mode for training the touch classifier later:

petctl run --backend robot --record touch_session_001.jsonl

Each line is a JSON object:
{
  "timestamp": 1234567.89,
  "sensors": { ... raw RobotState.sensors as dict ... },
  "servo_positions": { ... },
  "label": null
}

After recording, a separate labeling script lets you scrub through 
the timeline and annotate segments with TouchType labels. These 
labeled recordings become training data for a BiLSTM or similar 
to replace the heuristic classifier.
```

---

## File Structure After All Stages

```
petctl/
├── __init__.py
├── types.py
├── protocols.py
├── controller.py
├── cli.py
├── backends/
│   ├── mock.py
│   └── robot.py
├── schemes/
│   ├── keyboard.py
│   └── passthrough.py
├── behaviors/          ← NEW (Stage 1)
│   ├── __init__.py
│   ├── engine.py
│   ├── nestle.py
│   ├── recoil.py
│   ├── undulate.py
│   └── posture.py
├── perception/         ← NEW (Stage 2)
│   ├── __init__.py
│   └── touch_events.py
├── affect/             ← NEW (Stage 3)
│   ├── __init__.py
│   ├── valence.py
│   └── reflexes.py
├── agent/              ← NEW (Stage 4)
│   ├── __init__.py
│   ├── context.py
│   ├── mood.py
│   └── agent_loop.py
├── visualizers/
│   └── rerun_viz.py
└── assets/
    ├── robot_assembly.json
    └── 3d_models/
```

## Design Reference Files

Place the sketch implementations from our design session in `docs/behaviors/` 
for Claude Code to reference. These are design intent, not production code:

- `docs/behaviors/engine_sketch.py`
- `docs/behaviors/nestle_sketch.py`
- `docs/behaviors/recoil_sketch.py`
- `docs/behaviors/undulate_sketch.py`
- `docs/behaviors/posture_sketch.py`
- `docs/behaviors/touch_events_sketch.py`
- `docs/behaviors/behavior_demo_sketch.py`

Claude Code should read these for design intent but implement fresh, 
following the existing petctl code patterns and conventions.

## CLAUDE.md additions

Add to the project's CLAUDE.md:

```
## Behavior System (docs/behaviors/)
- Behaviors produce angle CONTRIBUTIONS, not absolute positions
- BehaviorEngine sums weighted contributions and converts to ServoCommands
- servo_id == module_id (offset 0)
- Odd modules have left/right swapped (handled in RobotBackend, transparent to behaviors)
- All joints are single-axis Z revolute
- Gain/spread/falloff values in sketches are initial guesses — tune on real hardware
- Valence is continuous (-1 to +1), not discrete behavior switching
- Reflexes are short-duration overrides that fade back to valence-driven behavior
```
