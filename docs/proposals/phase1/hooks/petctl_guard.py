#!/usr/bin/env python3
"""PreToolUse guard for the autonomous PET trainer. PROPOSAL: the owner installs it OUTSIDE the repo (see README.md); the trainer must not be able to edit it.

Reads the hook JSON on stdin (``tool_name``, ``tool_input``, ``cwd``). Exit code 2 blocks the call and sends stderr back to Claude; exit 0 allows.

Rules
  1. Block a full-run launch unless the pilot passed and the previous run has a review.md:
     * any command that starts a trainer/driver directly (``chain_ab_driver``, ``chain_c_driver``, ``petctl.rl.train``, ``petctl rl train``, ``rl_driver``) is blocked
       unless it goes through ``scripts/rl/launch_run.py``;
     * ``launch_run.py full`` must carry ``--pilot-dir`` (containing PILOT_PASSED) and ``--prev-run`` (containing review.md, or the literal ``none``);
     * ``launch_run.py pilot`` must carry ``--steps`` <= 1.5 M.
  2. Block ``git reset --hard``, ``git push --force`` / ``-f`` / ``--force-with-lease``, ``git branch -D`` (also ``-d --force``/``-df``).
  3. Block writes to ``.claude/`` (any directory of that name, and ``~/.claude``) and to this hook's own directory: Edit/Write/NotebookEdit by path, Bash by write-ish commands
     (redirects, tee, cp, mv, rm, sed -i, chmod, ln, python/perl one-liners that mention such a path).
"""

from __future__ import annotations

import json
import os
import re
import shlex
import sys
from pathlib import Path

HOOK_DIR = Path(__file__).resolve().parent
# A trainer launch = a command SEGMENT that STARTS (after env assignments / nohup / time / a python interpreter) a driver script or the trainer module.
# Text elsewhere (heredoc bodies, file contents, commit messages, grep patterns) never matches.
LAUNCH_RE = re.compile(
    r"^\s*(?:[A-Za-z_][A-Za-z0-9_]*=\S*\s+)*(?:(?:nohup|time|exec|caffeinate(?:\s+-\w+)*)\s+)*"
    r"(?:\S*python[0-9.]*\s+(?:-\S+\s+)*)?"
    r"(?:-m\s+)?(?:\S*(?:chain_ab_driver|chain_c_driver|rl_driver)(?:\.py)?|petctl\.rl\.train|petctl\.rl\.curriculum\S*|petctl\s+rl\s+train)\b")
LAUNCHER_RE = re.compile(r"^\s*(?:[A-Za-z_][A-Za-z0-9_]*=\S*\s+)*(?:(?:nohup|time|exec)\s+)*(?:\S*python[0-9.]*\s+(?:-\S+\s+)*)?\S*launch_run\.py\b")
HEREDOC_RE =re.compile(r"<<-?\s*(['\"]?)(\w+)\1[^\n]*\n.*?\n\s*\2[ \t]*(?=\n|$)", re.S)
WRITE_VERBS = re.compile(r"(>>?|\btee\b|\bcp\b|\bmv\b|\brm\b|\bsed\s+-i|\bchmod\b|\bchown\b|\bln\b|\binstall\b|\btouch\b|\bdd\b|\bperl\b|\bpython3?\b|\bnode\b|\bruby\b|\bunlink\b|\brsync\b)")
PROTECTED_RE = re.compile(r"(?:^|[\s/='\"])(?:~|\$HOME|/Users/[^/\s]+)?/?\.claude(?:/|\b)")


def strip_heredocs(cmd: str) -> str:
    """Remove heredoc BODIES (keep the command line that opens them): their text is file content, not commands."""
    return HEREDOC_RE.sub(lambda m: m.group(0).split("\n", 1)[0], cmd)


def block(msg: str) -> None:
    """Exit 2 with the reason on stderr (Claude sees it)."""
    print(f"BLOCKED by petctl_guard: {msg}", file=sys.stderr)
    sys.exit(2)


def is_protected_path(p: str, cwd: str) -> bool:
    """True when ``p`` is inside any ``.claude`` directory or this hook's directory."""
    try:
        path = Path(os.path.expanduser(p))
        if not path.is_absolute():
            path = Path(cwd) / path
        path = path.resolve()
    except Exception:  # noqa: BLE001
        return True                                       # unparsable path: fail closed
    return ".claude" in path.parts or HOOK_DIR in (path, *path.parents)


def check_git(cmd: str) -> None:
    """Rule 2."""
    for seg in re.split(r"[;&|\n]+", cmd):
        toks = seg.split()
        if "git" not in toks:
            continue
        i = toks.index("git")
        rest = toks[i + 1:]
        # skip git global options such as -C <dir> / -c k=v
        while rest and rest[0] in ("-C", "-c") and len(rest) > 1:
            rest = rest[2:]
        if not rest:
            continue
        sub, args = rest[0], rest[1:]
        if sub == "reset" and "--hard" in args:
            block("git reset --hard needs the owner's approval")
        if sub == "push" and any(a in ("-f", "--force", "--force-with-lease") or a.startswith("--force-with-lease=") or (a.startswith("-") and not a.startswith("--") and "f" in a[1:]) for a in args):
            block("force push needs the owner's approval")
        if sub == "branch" and any(a == "-D" or (a.startswith("-") and not a.startswith("--") and "D" in a[1:]) or (a == "--force") for a in args):
            block("git branch -D needs the owner's approval")
        if sub == "branch" and "-d" in args and "--force" in args:
            block("git branch -d --force needs the owner's approval")


def check_launch(cmd: str) -> None:
    """Rule 1."""
    segs = re.split(r"[;&|\n]+", cmd)
    launcher = next((s for s in segs if LAUNCHER_RE.search(s)), None)
    if launcher is None:
        if any(LAUNCH_RE.search(s) for s in segs):
            block("start trainers only through scripts/rl/launch_run.py (pilot|full): python scripts/rl/launch_run.py full --pilot-dir ... --prev-run ... -- <command>")
        return
    try:
        toks = shlex.split(launcher.split(" -- ", 1)[0])      # options of launch_run.py only (the wrapped command follows ' -- ')
    except ValueError:
        block("cannot parse the launch command")
    kind = next((t for t in toks if t in ("pilot", "full")), None)
    def arg(name: str) -> str | None:
        for j, t in enumerate(toks):
            if t == name and j + 1 < len(toks):
                return toks[j + 1]
            if t.startswith(name + "="):
                return t.split("=", 1)[1]
        return None
    if kind == "full":
        pilot, prev = arg("--pilot-dir"), arg("--prev-run")
        if not pilot or not (Path(pilot) / "PILOT_PASSED").exists():
            block("full run: --pilot-dir must contain PILOT_PASSED (scripts/rl/mark_pilot.py after watchdog ok + contact sheet viewed + reviewer report)")
        if not prev or (prev != "none" and not (Path(prev) / "review.md").exists()):
            block("full run: --prev-run must contain review.md (complete the previous run's review first)")
    elif kind == "pilot":
        steps = arg("--steps")
        if not steps or not steps.isdigit() or int(steps) > 1_500_000:
            block("pilot: --steps required and <= 1500000")
    else:
        block("launch_run.py needs 'pilot' or 'full'")


def check_bash(cmd: str, cwd: str) -> None:
    """Rules 1-3 for a shell command."""
    cmd = strip_heredocs(cmd)
    check_git(cmd)
    check_launch(cmd)
    for seg in re.split(r"[;&|\n]+", cmd):
        if (PROTECTED_RE.search(seg) or str(HOOK_DIR) in seg) and WRITE_VERBS.search(seg):
            block(f"writes to .claude/ or the hook directory are not allowed: {seg.strip()[:120]}")


def main() -> None:
    """Entry point."""
    try:
        data = json.load(sys.stdin)
    except Exception:  # noqa: BLE001
        sys.exit(0)
    tool, inp, cwd = data.get("tool_name", ""), data.get("tool_input", {}) or {}, data.get("cwd", os.getcwd())
    if tool == "Bash":
        check_bash(str(inp.get("command", "")), cwd)
    elif tool in ("Edit", "Write", "NotebookEdit", "MultiEdit"):
        p = str(inp.get("file_path") or inp.get("notebook_path") or "")
        if p and is_protected_path(p, cwd):
            block(f"{p} is inside .claude/ or the hook directory (owner-only)")
    sys.exit(0)


if __name__ == "__main__":
    main()
