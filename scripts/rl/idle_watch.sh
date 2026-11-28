#!/bin/sh
# Zero-token idle detector. Run it through the Monitor tool; each stdout line wakes the session.
#
#   Monitor: scripts/rl/idle_watch.sh
#
# Silent (prints nothing) unless docs/plans/AUTONOMY.md contains a line `mode: on` (or FORCE_ON=1 for testing).
# BUSY = a python process of scripts/rl/ or petctl.rl. is running, OR a file under data/rl or docs/media changed within IDLE_MIN minutes.
# When it has been idle for IDLE_MIN minutes it prints one `IDLE ...` line, then repeats every REPEAT_MIN minutes while still idle, so a missed wake-up is not lost.
# It never starts anything. The session reacts by running the `idle-check` agent (see docs/proposals/rl_restart/README.md).
cd "$(dirname "$0")/../.." || exit 1
IDLE_MIN=${IDLE_MIN:-10}
REPEAT_MIN=${REPEAT_MIN:-30}
POLL_S=${POLL_S:-60}
idle_since=""
last_emit=0
while true; do
  now=$(date +%s)
  if [ "${FORCE_ON:-0}" = "1" ] || grep -q '^mode: on' docs/plans/AUTONOMY.md 2>/dev/null; then
    jobs=$(ps -eo pid,command | grep -E 'python.*(scripts/rl/|petctl\.rl\.)' | grep -v -E 'grep|idle_watch|progress_videos' | head -1)
    recent=$(find data/rl docs/media -type f -mmin -"$IDLE_MIN" 2>/dev/null | head -1)
    if [ -n "$jobs" ] || [ -n "$recent" ]; then
      idle_since=""
    else
      [ -z "$idle_since" ] && idle_since=$now
      if [ $((now - idle_since)) -ge $((IDLE_MIN * 60)) ] && [ $((now - last_emit)) -ge $((REPEAT_MIN * 60)) ]; then
        echo "IDLE $(date '+%H:%M'): no training/job process and no file activity for $(( (now - idle_since) / 60 )) min. Run the idle-check agent."
        last_emit=$now
      fi
    fi
  else
    idle_since=""
  fi
  sleep "$POLL_S"
done
