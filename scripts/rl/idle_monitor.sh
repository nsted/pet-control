#!/bin/sh
# Zero-token idle watcher for the Monitor tool (CLAUDE.md: use Monitor, not a polling cron). Polls scripts/rl/idle_status.sh and emits ONE line each time the machine has been idle for
# IDLE_AFTER_POLLS consecutive polls while docs/plans/TASK_QUEUE.md says "autonomy: ON". It stays silent while busy, while autonomy is PAUSED / OFF, and after it has reported until work has started and finished again.
# Env: POLL_S (default 60), IDLE_AFTER_POLLS (default 5 = about 5 minutes).
cd "$(dirname "$0")/../.." || exit 2
POLL_S=${POLL_S:-60}; N=${IDLE_AFTER_POLLS:-5}; idle=0; reported=0
while true; do
  if scripts/rl/idle_status.sh >/dev/null 2>&1; then idle=0; reported=0; else idle=$((idle + 1)); fi
  if [ "$idle" -ge "$N" ] && [ "$reported" -eq 0 ]; then
    if grep -qi "^autonomy: *ON" docs/plans/TASK_QUEUE.md 2>/dev/null; then
      echo "IDLE $((idle * POLL_S / 60)) min, autonomy ON: run the idle-check agent (docs/plans/RL_RESTART_PLAN.md section 4a)"
      reported=1
    fi
  fi
  sleep "$POLL_S"
done
