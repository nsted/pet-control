"""Mark a pilot run as passed (writes ``PILOT_PASSED``): the prerequisite for launching a full run (hook + ``launch_run.py``).

    python scripts/rl/mark_pilot.py data/rl/<pilot> --reviewer data/rl/<pilot>/reviewer.md --viewed-contact-sheet

Requirements (all checked): the pilot finished (``meta.json``), no ``STOP_watchdog``, ``reward_terms.json`` exists, ``contact_sheet*.png`` exists, the adversarial reviewer's
report exists and is non-empty, and ``--viewed-contact-sheet`` attests that the contact sheet was looked at (the trainer reads it; the flag is an explicit statement).
Records git hash, body length and seed so every later table can cite them.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path


def main() -> int:
    """CLI entry point; exit 1 and print the reasons when the pilot does not qualify."""
    ap = argparse.ArgumentParser()
    ap.add_argument("pilot", type=Path)
    ap.add_argument("--reviewer", type=Path, required=True)
    ap.add_argument("--viewed-contact-sheet", action="store_true")
    a = ap.parse_args()
    p, bad = a.pilot, []
    if not (p / "meta.json").exists():
        bad.append("pilot not finished (no meta.json)")
    if (p / "STOP_watchdog").exists():
        bad.append(f"watchdog stopped the pilot: {(p / 'STOP_watchdog').read_text().splitlines()[0]}")
    if not (p / "reward_terms.json").exists():
        bad.append("no reward_terms.json (watchdog off?)")
    if not list(p.glob("contact_sheet*.png")):
        bad.append("no contact_sheet*.png (scripts/rl/contact_sheet.py)")
    if not a.reviewer.exists() or not a.reviewer.read_text().strip():
        bad.append("adversarial reviewer report missing or empty")
    if not a.viewed_contact_sheet:
        bad.append("pass --viewed-contact-sheet after looking at the contact sheet")
    if bad:
        print("PILOT NOT PASSED:\n  " + "\n  ".join(bad))
        return 1
    meta = json.loads((p / "meta.json").read_text())
    (p / "PILOT_PASSED").write_text(json.dumps({"time": time.strftime("%Y-%m-%d %H:%M:%S"), "git": meta.get("git"), "n_modules": meta.get("n_modules"), "seed": meta.get("seed"),
                                                "steps": meta.get("steps"), "reviewer": str(a.reviewer)}, indent=1))
    print(f"PILOT_PASSED written in {p}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
