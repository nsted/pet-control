#!/bin/bash
# Zero-token event watcher. Prints ONE line only when something needs Claude; silent otherwise.
# Use with the Claude Code Monitor tool (each stdout line = a wake-up), or run in a terminal.
#   scripts/rl/event_watch.sh [poll_s=60] [idle_alert_s=900]
# Events (each emitted once until its condition clears): a STOP_*/DONE marker appears; no training
# driver running for idle_alert_s. Also restarts the (free) video watchdog if a driver runs without it.
# Portable to macOS bash 3.2 (no associative arrays).
cd "$(dirname "$0")/../.." || exit 1
POLL=${1:-60}
IDLE_ALERT=${2:-900}
STATE=$(mktemp -d "${TMPDIR:-/tmp}/event_watch.XXXXXX")
trap 'rm -rf "$STATE"' EXIT
DRIVER_RE='scripts/rl/[a-z0-9_]*driver|petctl\.rl.*train'
idle_since=0
seen_driver=0
# markers that already exist at startup are known (documented in CURRICULUM_STATE.md); only NEW ones are events
ls data/rl/*/STOP_* data/rl/*/DONE 2>/dev/null | tr '/' '_' > "$STATE/known"

emit() { # emit KEY MESSAGE: once per key until clear_key KEY
  [ -e "$STATE/$1" ] && return
  : > "$STATE/$1"
  echo "[$(date +%H:%M:%S)] EVENT $2"
}
clear_key() { rm -f "$STATE/$1"; }

while true; do
  markers=$(ls data/rl/*/STOP_* data/rl/*/DONE 2>/dev/null)
  for m in $markers; do
    k=$(echo "$m" | tr '/' '_')
    grep -qx "$k" "$STATE/known" && continue
    emit "marker_$k" "marker $m: $(head -c 200 "$m" | tr '\n' ' ')"
  done

  if pgrep -f "$DRIVER_RE" >/dev/null 2>&1; then
    idle_since=0; seen_driver=1; clear_key idle
    pgrep -f progress_videos >/dev/null 2>&1 || {
      nohup scripts/rl/progress_videos_watchdog.sh >> data/rl/progress_videos.log 2>&1 &
      echo "[$(date +%H:%M:%S)] restarted video watchdog" >> data/rl/event_watch.log
    }
  else
    [ "$idle_since" -eq 0 ] && idle_since=$(date +%s)
    if [ "$seen_driver" -eq 1 ] && [ $(( $(date +%s) - idle_since )) -ge "$IDLE_ALERT" ]; then
      if [ -n "$markers" ]; then
        emit idle "no training driver for >= ${IDLE_ALERT}s; stopped by marker(s) above: decide the next job"
      else
        emit idle "no training driver for >= ${IDLE_ALERT}s and no STOP/DONE marker: driver died or nothing launched"
      fi
    fi
  fi
  sleep "$POLL"
done
