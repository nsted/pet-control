# Behavior System — Implementation TODO

Tracks progress through `docs/design/IMPLEMENTATION_PLAN.md`.

---

## Stage 1 — Behavior Engine + Core Behaviors

Goal: Parametric behaviors running on mock backend with Rerun visualization.

- [ ] **1.1** `petctl/behaviors/engine.py` — BehaviorEngine scaffold
  - BehaviorEngine implements ControlScheme
  - Behaviors produce per-module angle contributions (degrees)
  - Weight interpolation for fade in/out
  - Exponential smoothing on output
  - Clamp to ±45° per module

- [ ] **1.2** `petctl/behaviors/nestle.py` — reactive nestle behavior
  - Left touch → positive angle, right touch → negative angle
  - Middle touch → neighbors bend toward each other (cupping)
  - Neighbor spread + falloff configurable
  - Constructor params: gain, spread, falloff, middle_curl_gain

- [ ] **1.3** Core behaviors
  - [ ] `petctl/behaviors/recoil.py` — bend away from touch + startle overshoot
  - [ ] `petctl/behaviors/undulate.py` — traveling sine wave (idle breathing)
  - [ ] `petctl/behaviors/posture.py` — GoLimp + Stiffen
  - [ ] Register all in `build_engine()` helper

- [ ] **1.4** Demo + CLI integration
  - [ ] `examples/behavior_demo.py` with `--behavior` and `--intensity` args, `combo` mode
  - [ ] `petctl run --control behavior` CLI option

---

## Stage 2 — Perception Layer

Goal: Classify raw sensor streams into TouchEvent objects.

- [ ] **2.1** `petctl/perception/touch_events.py` — data structures
  - TouchType enum (IDLE, STROKE, PAT, STRIKE, HOLD, SQUEEZE, RELEASE)
  - TouchEvent dataclass
  - SensorStream with rolling window (50 frames), 48 features/frame

- [ ] **2.2** `HeuristicTouchClassifier` — rule-based classification
  - STRIKE: sudden onset (large delta, short duration)
  - STROKE: touch CoM moving along body
  - HOLD: sustained static >1s
  - PAT: brief moderate, no movement
  - RELEASE: transition from active to idle
  - IDLE: no meaningful contact

- [ ] **2.3** `petctl/perception/perception_layer.py` + CLI flag `--perception`

---

## Stage 3 — Valence System + Reflexes

Goal: Touch events drive behavior through valence. Static valence map for now.

- [ ] **3.1** `petctl/affect/valence.py`
  - ValenceMap (TouchType → float)
  - ValenceFunction with exponential smoothing
  - ValenceToMotor mapping

- [ ] **3.2** `petctl/affect/reflexes.py` — ReflexLayer
  - Watches TouchType transitions
  - Injects time-limited high-weight behavior overrides
  - Configurable reflex table

- [ ] **3.3** Wire valence + reflexes into BehaviorEngine
  - BehaviorEngine owns PerceptionLayer, ValenceFunction, ValenceToMotor, ReflexLayer
  - Expose current_valence, current_touch_event, recent_events as properties
  - Add set_valence_map() method

---

## Stage 4 — Agent Integration

Goal: Claude API updates mood and valence map on a slow cycle.

- [ ] **4.1** `petctl/agent/context.py` — AgentContext builder
  - Concise natural language summary (<500 tokens)

- [ ] **4.2** `petctl/agent/mood.py` — MoodState + MoodAgent
  - MoodState: arousal, receptiveness, energy, playfulness, valence_map
  - MoodAgent calls claude-sonnet-4-6, parses JSON response
  - MockMoodAgent for testing

- [ ] **4.3** `petctl/agent/agent_loop.py` — async loop alongside control loop
  - CLI flags: `--agent`, `--agent-mock`, `--agent-hz`
  - Smooth mood interpolation over 2–3s

- [ ] **4.4** Real agent end-to-end test

---

## Stage 5 — Real Robot Testing + Tuning

- [ ] **5.1** Full stack on real robot
  - [ ] Add `--gain-scale` CLI flag
  - Tune nestle gain (start at 10°, increase)
  - Tune sensor thresholds for real touch vs mock sine

- [ ] **5.2** Data collection mode
  - `petctl run --backend robot --record <file>.jsonl`
  - Post-hoc labeling script for TouchType annotations
