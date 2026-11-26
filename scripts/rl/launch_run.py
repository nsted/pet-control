"""The only sanctioned way to start an RL run (the PreToolUse hook blocks driver / trainer commands that bypass it).

    python scripts/rl/launch_run.py pilot --steps 1000000 -- <driver command...>
    python scripts/rl/launch_run.py full --pilot-dir data/rl/<pilot> --prev-run data/rl/<previous full run> -- <driver command...>

``pilot``: exports PETCTL_RUN_KIND=pilot and refuses more than 1.5 M steps (--steps is only checked here; the driver enforces its own budget).
``full``: refuses unless ``<pilot-dir>/PILOT_PASSED`` exists and ``<prev-run>/review.md`` exists (pass ``--prev-run none`` for the very first run, which is logged).
The command after ``--`` runs with PETCTL_WATCHDOG_BASELINE=<pilot-dir> (the watchdog uses the pilot's reward-term shares as its baseline).
"""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

MAX_PILOT_STEPS = 1_500_000


def main() -> int:
    """CLI entry point."""
    argv = sys.argv[1:]
    cmd = argv[argv.index("--") + 1:] if "--" in argv else []
    ap = argparse.ArgumentParser()
    ap.add_argument("kind", choices=("pilot", "full"))
    ap.add_argument("--steps", type=int, default=0)
    ap.add_argument("--pilot-dir", type=Path)
    ap.add_argument("--prev-run", default=None)
    a = ap.parse_args(argv[:argv.index("--")] if "--" in argv else argv)
    if not cmd:
        print("no command after '--'")
        return 2
    env = dict(os.environ, PETCTL_RUN_KIND=a.kind, PETCTL_WATCHDOG="1")
    if a.kind == "pilot":
        if a.steps <= 0 or a.steps > MAX_PILOT_STEPS:
            print(f"a pilot needs --steps in 1..{MAX_PILOT_STEPS}")
            return 2
    else:
        if a.pilot_dir is None or not (a.pilot_dir / "PILOT_PASSED").exists():
            print("full run refused: --pilot-dir with PILOT_PASSED required (scripts/rl/mark_pilot.py)")
            return 2
        if a.prev_run is None or (a.prev_run != "none" and not (Path(a.prev_run) / "review.md").exists()):
            print("full run refused: --prev-run <dir with review.md> required (or 'none' for the very first run)")
            return 2
        env["PETCTL_WATCHDOG_BASELINE"] = str(a.pilot_dir)
    os.execvpe(cmd[0], cmd, env)


if __name__ == "__main__":
    sys.exit(main())
