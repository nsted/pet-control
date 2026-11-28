# What PET is for (the north star for decisions)

Read this before choosing what to build, train or measure. If a piece of work cannot be tied back to this page, ask why we are doing it. This page is the owner's own statement of purpose (verbatim below) plus a decision guide derived from it. The derived parts are marked as interpretation and can be corrected by the owner; the verbatim statement is authoritative.

Source and history: owner statement pasted 2026-11-28. Related: `docs/plans/AUTONOMOUS_TRAINER.md` (the standing brief for the trainer), `docs/plans/CURRICULUM_STATE.md` (current state).

---

## 1. The owner's statement (verbatim)

PET is a robot that exhibits lifelike behavior and companionship on par with a pet animal. Inspired by enactivism, PET is optimized for moving its body as a means of both sensing and acting in its environment, and to affect the people within. This approach stands in stark contrast to conventional design of social robots that mimics human features without the underlying physiology that grounds the features in instrumental value to the robot.

In the Embodied Mind, Varela laid the foundation to understand cognition as formed through enactivism, meaning that embodied interaction shapes the mind to make sense of the world and produce effective actions within it. Following, Rafiee and Sutton argue that enactive artificial intelligence is possible if it implements four principles: embodiment, action-perception inseparability, experience, and autonomy. These principles are guiding factors in PET's design.

PET's body is optimized to sense and act through embodied interaction. It is formed as a chain of modules each containing a proprioceptive actuator that senses as it moves, giving insight into the characteristics of the environment it is in contact with. Linking the modules together in a chain allows PET to conform to the shape of both the environment as it locomotes, or objects within by wrapping around them. The body is furthermore covered in capacitive sensors that allow the robot to distinguish between a human body and inert materials. It also contains an inertial measurement unit, to sense its orientation and direction of movement.

With this set of physical affordances, PET uses a model-free reinforcement learning framework in order to develop behaviors adapted to the environment. Currently I am developing movement primitives, such as gaits and object navigation. These primitives will then be scaffolded into a network that can blend between them to enact the optimal movement for different conditions.

PET will adopt an overarching goal of maximizing its time in contact with a human over its lifespan. This will incentivize it to use its motion primitives to seek out contact with people and adapt its behavior to the user's interests in the moment, as interpreted through tactile engagement.

To illustrate, imagine PET seeking human contact, it locomotes around one's home, navigating around obstacles. Sensing skin, it rubs up against the user's foot until it is picked up. PET then wraps around their arm, and the user reciprocates by stroking it. Interest waning, the user stops engaging. This time, PET wants attention, so it nudges the user, once, twice to no effect. It then starts playfully thrashing about until the user is provoked into contact. Next time, however, PET is looking for calm, so when the user settles, so does the robot, burrowing further into their arms until it is maximally snuggled. Doesn't that sound more interesting than looking into a robot's "face" and talking to it?

---

## 2. In one paragraph (interpretation)

PET is a body, not a face. It earns lifelike behaviour by using movement to feel its world (proprioceptive actuators, IMU, capacitive skin) and to change the people around it. Its behaviours are learned by model-free reinforcement learning, first as movement primitives and then as a network that blends them. Its one lifelong goal is more time in contact with a human, which makes it seek people, read how they respond to touch, and adapt: playful when attention fades, calm and snuggled when the person settles.

## 3. The four principles and what each demands of our work (interpretation)

| Principle (Rafiee and Sutton, as cited by the owner) | What it asks of design and training here |
|---|---|
| Embodiment | The body is the instrument. Prefer solutions that use the real mechanics (the twist-chain geometry, wrapping, rolling, compliance) over ones that bypass them. Body-specific facts (length 11, the 0.5 N.m cap, the GIM3505-8 motors) are part of the design, not noise. |
| Action-perception inseparability | Sensing should come from acting, and acting should be shaped by sensing. Open-loop behaviour with a separate "perception module" is the wrong shape; feedback from proprioception and touch should be inside the behaviour. A gait that cannot feel the floor is incomplete. |
| Experience | Behaviour should be shaped by what the robot has actually undergone, over its lifespan, not only by hand-set rules. Prefer learning from interaction over fixed programs; keep the ability to keep learning (and adapting to one user) after deployment in mind. |
| Autonomy | The robot decides what to do next, driven by its own goal (contact with a human), not by commands or scripts. Scripted sequences are acceptable as scaffolding or tests, not as the end behaviour. |

## 4. Decision guide

Use these as questions when choosing between options. They are in priority order.

1. **Does it serve contact with a human?** The overarching reward is time in contact over a lifespan. Seeking, holding, wrapping, being picked up, yielding to a stroke and reading when interest fades all count. Speed, distance covered, or beating a benchmark do not count by themselves.
2. **Is the behaviour learned by model-free RL, or is it scaffolding?** The framework is model-free RL for movement primitives, scaffolded into a blending network. Other tools (search over gait parameters, hand-written reflexes, supervised estimators, rule-based skill selectors) are allowed as scaffolding, baselines and benchmarks. A primitive is not finished until it is learned by RL, unless the owner decides otherwise. Always say which of the two a result is.
3. **Does sensing come from the body's own movement and contact?** Proprioceptive actuators (torque and position feedback), the IMU and the capacitive skin are the senses. Do not add external perception (cameras, a face, speech) to solve a problem the body should solve. Capacitive sensing distinguishes a human body from inert material; floors, walls and obstacles must be sensed through proprioception (motor feedback).
4. **Would a person touching it find it alive?** The test is lifelike responsiveness and companionship on par with a pet, not metrics. When a metric and that test disagree, the test wins. Prefer expressive, responsive, adaptable behaviour over optimal but mechanical behaviour.
5. **Does it conform to what it meets?** The chain body wraps around objects and arms and conforms to terrain. Behaviours that exploit this (wrap, hold, burrow, yield) are first-class, not side demos.
6. **Can it adapt to a person and to the moment?** The same robot should be playful on one occasion and calm on another, depending on how the user engages. Prefer designs where such adaptation can be learned over designs that hard-code modes.
7. **Is it safe and gentle to touch?** (Implied by the scenario, not stated by the owner; to be confirmed.) The torque cap, current budgets and no-hurt-to-people limits come first. Safety rules in `CLAUDE.md` and `petctl/config.py` apply.

## 5. How current work maps to the purpose (interpretation)

| Work so far | Role in the purpose | Status and caveat |
|---|---|---|
| Gait library (CPG search), selector | Movement primitives for getting around a home | Not RL: scaffolding and a benchmark. Open loop (no feedback inside a gait). |
| Self-righting | Essential: the robot will be dropped, picked up and flipped | Works (hand-written trigger plus searched motion). Should eventually be learned. |
| Edge avoidance (platforms) | Furniture edges, stairs, being carried: staying safe while moving | Not solved. Needs proprioceptive sensing and fast reflexes; learned detector is weak. |
| Pose holding under disturbance | Wrapping around a moving arm is holding a pose against a compliant, moving, living object | Planned; baseline script exists, not run. Strong fit with the purpose. |
| Compliance (stiff vs yielding) | Yielding to a stroke, holding against gravity or being carried | Discussed; not built. |
| Touch (capacitive) | The reward signal and the sense for contact with a person | In simulation the pads only respond to a "post" stand-in, not a person; no touch-driven behaviour yet. |
| Gap crossing, climbing, poses | Obstacle negotiation in a home | Lower priority than contact-seeking behaviour; climbing 12 cm not found. |
| Body length 11 | Chosen by the owner (preference and tie-break); also long enough to wrap an arm | Decided. |

## 6. Anti-goals (what PET is not)

- Not a humanoid or face-based social robot: no mimicry of human features, no talking-face interaction.
- Not a speed or distance benchmark: a slower, more responsive creature beats a faster mechanical one.
- Not scripted: sequences of canned behaviours are test scaffolding, not the product.
- Not dependent on a perfect model: the real robot's structure and motors are only partly known, so learning from its own feedback is preferred to reliance on the CAD.

## 7. Open tensions and questions for the owner

1. The framework is model-free RL for primitives, but much of the recent work used search (CMA-ES, MAP-Elites) and hand-written skills because the first RL attempts in CPG-parameter space failed. Should search-based primitives be treated as acceptable scaffolding to be replaced by RL later, or should RL be the only route from now on? (The owner has said RL layers are of interest.)
2. How is "time in contact with a human" measured, in simulation and on the real robot, and how is it kept from being gamed (for example, clamping onto anything touchable, or never letting go)?
3. What counts as "the user's interest in the moment" as interpreted through tactile engagement: pressure, stroking rhythm, duration, movement of the person? This defines the reward for adaptation (playful versus calm).
4. Simulation touch currently covers only a post as a human stand-in. A better human model (a limb that moves, strokes, picks up and lets go) is needed to train the contact-seeking behaviours.
5. How much learning happens after deployment (lifelong, per user) versus before (simulation)? This sets how safe and sample-efficient the learning must be on the real robot, which is also constrained by the rule against moving the robot without permission.
