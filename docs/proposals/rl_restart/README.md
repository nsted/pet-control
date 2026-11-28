# RL restart: agents to install (owner action)

The trainer may not edit `.claude/` (standing rule), so the agent definitions are proposed here. Install by copying them yourself, in the same place as the phase-1 agents (`.claude/agents/` in the project, or `~/.claude/agents/`):

```bash
cp docs/proposals/rl_restart/agents/rl-robotics-expert.md  .claude/agents/
cp docs/proposals/rl_restart/agents/method-review.md        .claude/agents/     # replaces the phase-1 version (CPG-parameter wording is obsolete)
```

`reviewer.md` and `explore.md` from `docs/proposals/phase1/agents/` stay as they are. After copying, restart Claude Code (or start the new session) so the agents are picked up.

Roles (see `docs/plans/RL_RESTART_PLAN.md` section 4): `rl-robotics-expert` decides (writes `docs/plans/STAGE_n_GATES.md`, go/no-go, design), `reviewer` attacks the expert's designs before every pilot and full run, `method-review` after two failures with different causes or every 10 runs, `explore` searches and summarises. The expert has Write/Edit but is told to touch only `docs/plans/`.

If the expert agent is not installed, the trainer must say so and use the general-purpose agent with the expert's brief (the body of `agents/rl-robotics-expert.md`) pasted as its prompt, with `model: opus`.

The session prompt to paste into a new Claude Code session is `docs/plans/RL_RESTART_PROMPT.md`.
