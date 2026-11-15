#!/bin/bash
# Restarts the progress-video watcher whenever it exits (idle window, crash). Run from the repo root: nohup scripts/rl/progress_videos_watchdog.sh > data/rl/progress_videos.log 2>&1 &
while true; do
  python scripts/rl/progress_videos.py "$@"
  echo "[$(date +%H:%M:%S)] watcher exited ($?); restarting in 30 s"
  sleep 30
done
