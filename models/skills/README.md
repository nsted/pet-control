# Skill library

One folder per behaviour: `models/skills/<task>/<name>/` holding `policy.npz` (inference-only weights), `config.json` (the
`EnvConfig` it was trained with), `skill.json` (task, **verified** score, conditions, source run, caveats) and `demo.mp4`.
About 0.7 MB of weights per skill. The score is a fresh deterministic replay (5 fixed seeds, nominal physics and DR 0.3), never the
training record. Scores are in metres (shown as cm) of the task's own measure; see `score_label` in `skill.json`.

```bash
petctl rl skills [--task jump]                       # list, best per task marked (*), caveats shown
petctl rl play jump/jump_20m_v1 --seconds 10         # recall by id (a bare unique name also works; --view for the MuJoCo viewer)
petctl rl skill-video jump/jump_20m_v1               # re-record demo.mp4
# after a training run, bank its best policy under a new name (existing skills are never overwritten):
petctl rl skill-add data/rl/<run> --name <name> --policy-file best_policy.npz --note "what it is"
```

Skills trained at a torque cap above the real 0.164 N.m are flagged: they work in the sim only until retrained at the real cap.
