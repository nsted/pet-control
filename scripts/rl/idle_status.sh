#!/bin/sh
# Read-only: is any RL job running? Prints the running jobs and the age of the newest log file. Exit code 0 = BUSY, 1 = IDLE.
# Usage: scripts/rl/idle_status.sh
cd "$(dirname "$0")/../.." || exit 2
JOBS=$(ps -axo pid=,etime=,command= | grep -E "python[0-9.]* .*(scripts/rl/|petctl[./]rl[./])" | grep -v -E "idle_status|idle_monitor|progress_videos|render_|grep" | cut -c1-140)
if [ -n "$JOBS" ]; then
  echo "BUSY"
  echo "$JOBS"
  exit 0
fi
NEWEST=$(find data/rl -maxdepth 2 -name '*.log' -type f -print0 2>/dev/null | xargs -0 ls -t 2>/dev/null | head -1)
echo "IDLE"
[ -n "$NEWEST" ] && echo "newest log: $NEWEST ($(( ($(date +%s) - $(stat -f %m "$NEWEST" 2>/dev/null || stat -c %Y "$NEWEST")) / 60 )) min old)"
exit 1
