# Learning petctl: study checklist

Two paradigms: **A** = LLM picks a hand-coded motion (original, frozen). **B** = RL policy trained in MuJoCo (current).
Per session: read the files in order, do the exercise, then explain it back in your own words.

Note: the motion ABC is `Motion` (`petctl/protocols.py`), called "ControlScheme" in older docs. `BehaviorEngine` was only ever a design sketch. The real loop runs ~200 Hz.

## Overview (about 1 hr)
- [ ] Read `docs/plans/JOINT_FOUNDATIONS.md` section "Re-scope 2026-11-03" (why A was archived)
- [ ] Read `docs/plans/RL_LOCOMOTION.md`

## Paradigm A: LLM + behaviors
- [ ] **A1** `petctl/types.py`, `protocols.py`, `config.py`. Exercise: explain what goes in/out of `Motion.update`
- [ ] **A2** `schemes/sine.py`, `controller.py` (`_loop`, `_apply_slew_to_commands`), `cli.py` (`run`). Exercise: `petctl run --mode mock-sensor-sine`
- [ ] **A3** `backends/mock.py`, `backends/safety.py`, skim `backends/robot.py`. Exercise: trace one command to a CAN frame
- [ ] **A4** `schemes/ollama_scheme.py`, `prompts/robot_context.md`, `llm/client.py`, `perception/stroke.py`, skim `schemes/patterns.py`. Exercise: run `petctl run --control ollama --ollama-monitor` on the mock backend
- [ ] Optional: `driver.py`, `backends/remote.py`, `wire.py`, `docs/plans/PROCESS_SPLIT.md`

## ML fundamentals (no code)
- [ ] Glossary in your own words: policy, observation, action, reward, episode, environment
- [ ] Glossary: neural network, PPO (clip), entropy coefficient, learning rate, gamma
- [ ] Glossary: reward shaping, reward hacking, telescoping rewards, domain randomisation, sim-to-real, curriculum, held-out evaluation

## Paradigm B: RL
- [ ] **B1** `docs/plans/RL_LOCOMOTION.md`. Exercise: 5-line timeline of what went wrong and how it was fixed
- [ ] **B2** `rl/tasks/base.py`, `rl/tasks/locomote.py`, `rl/config.py`. Exercise: predict what each penalty discourages
- [ ] **B3** `rl/env.py`, `rl/obs.py`. Exercise: draw the step loop on paper
- [ ] **B4** `sim/model_builder.py`, `tests/test_model_builder.py`. Exercise: watch a trained policy (`petctl rl run <skill>`)
- [ ] **B5** `rl/train.py`, `rl/policy.py`, `rl/records.py`. Exercise: open TensorBoard on a run in `data/rl/`, read `episodes.jsonl`
- [ ] **B6** `rl/tasks/{reach_still,jump,goal_reach}.py`, `rl/curriculum/`, `models/skills/README.md`, `tests/test_rl_tasks.py`, `tests/test_curriculum.py`

## Done when you can, without looking
- [ ] Trace one tick of paradigm A from touch to motor frame
- [ ] Explain observation, action, and reward for `reach_still`
- [ ] Say why paradigm A was archived
- [ ] Write a one-page "How PET works" in your own words
