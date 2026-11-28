# AUTONOMY: switch for the idle loop

mode: on

(Switched on by the owner's request, 2026-11-28.)

How it works. `scripts/rl/idle_watch.sh` (started through the Monitor tool) is silent unless the line above says `mode: on`. With it on, the watcher prints an `IDLE` line when no training/job process has run and no file under `data/rl` or `docs/media` has changed for 10 minutes (and again every 30 minutes while still idle). The session then runs the `idle-check` agent (Haiku, read-only), which reports the next runnable task from `docs/plans/TASK_QUEUE.md`, or NONE. If there is a task, the session does it (respecting the gates and approvals); if there is none, the session asks the `rl-robotics-expert` to write new tasks into the queue from the accepted plan and the stage gate files. Tasks that need an owner approval are marked BLOCKED and skipped, never nagged, never left as a reason to sit idle: unblocked work (baselines, tests, videos, docs, the next stage's preparation) comes first.

Rules.
- Only the owner turns this on: change `off` to `on` (or tell the session "go autonomous"). The session may not switch it on by itself.
- The owner always has priority. When the owner sends a message, the session answers it and does not start new autonomous work from an idle line until the owner says to continue (or has left the mode on and gone away; if unsure, ask once).
- One training job at a time (10 cores). Pilots through `scripts/rl/launch_run.py`; the reviewer before every pilot and full run.
- Idle events are advisory: never use them to skip the gates, the approvals list, or a pending owner question.
- To stop: set `mode: off` (the watcher goes silent within a minute) and stop the Monitor task.
