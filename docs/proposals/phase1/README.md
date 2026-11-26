# Phase 1 proposals (owner action needed)

Nothing here is active. The trainer may not edit `.claude/` or hook scripts (approval rule 8), so you install these.

## 1. Hooks (PreToolUse guard)
`hooks/petctl_guard.py` blocks (exit 2): full-run launches without a passed pilot + a reviewed previous run, `git reset --hard`, force pushes, `git branch -D`, and writes to
any `.claude/` directory or the hook directory. Tested by `tests/test_guard.py`. Install OUTSIDE the repo, in your user settings:

```bash
mkdir -p ~/.claude/petctl-hooks
cp docs/proposals/phase1/hooks/petctl_guard.py ~/.claude/petctl-hooks/
chmod a-w ~/.claude/petctl-hooks/petctl_guard.py      # optional: read-only (hard protection: chown root + chflags uchg)
```
Then merge `hooks/settings_snippet.json` into `~/.claude/settings.json` (user level, so the repo's `.claude/settings.local.json` cannot override it). Restart the session.
The guard defines how runs start: `scripts/rl/launch_run.py pilot|full ...` (checks `PILOT_PASSED`, written by `scripts/rl/mark_pilot.py`, and the previous run's `review.md`).
Limit: a hook is a seatbelt, not a sandbox. An indirect command (a script that itself writes to `.claude/`) is only caught if the command text shows it.

## 2. Subagents
Copy `agents/*.md` to `.claude/agents/` (project) or `~/.claude/agents/`: `reviewer` (opus), `method-review` (opus), `explore` (haiku). None has Edit/Write; the reviewer writes its
report with a Bash redirect into the pilot directory.
