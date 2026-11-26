"""The proposed PreToolUse guard (docs/proposals/phase1/hooks/petctl_guard.py): blocked and allowed calls."""

from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

import pytest

GUARD = Path(__file__).resolve().parents[1] / "docs/proposals/phase1/hooks/petctl_guard.py"


def run(tool: str, **inp: str) -> int:
    """Exit code of the guard for one call."""
    r = subprocess.run([sys.executable, str(GUARD)], input=json.dumps({"tool_name": tool, "tool_input": inp, "cwd": "/Users/x/proj"}), text=True, capture_output=True)
    return r.returncode


@pytest.mark.parametrize("cmd", [
    "git reset --hard HEAD~1", "git push --force origin main", "git push -f", "git push origin main --force-with-lease", "git branch -D old", "git -C . branch -D old",
    "python scripts/rl/chain_ab_driver.py --stage x", "python -m petctl.rl.train", "python scripts/rl/launch_run.py full -- python x.py",
    "python scripts/rl/launch_run.py full --pilot-dir /nonexistent --prev-run none -- python x.py", "python scripts/rl/launch_run.py pilot --steps 9000000 -- python x.py",
    "nohup python scripts/rl/chain_c_driver.py --stage x &", "PETCTL_X=1 python -m petctl.rl.train --steps 5", ".venv/bin/python scripts/rl/chain_ab_driver.py",
    "cat > .claude/x.md <<'EOF'\nhello\nEOF",
    "echo hi > .claude/settings.json", "sed -i '' s/a/b/ ~/.claude/settings.json", "cp x /Users/me/.claude/petctl-hooks/petctl_guard.py", "rm -rf .claude/agents",
])
def test_blocked(cmd: str) -> None:
    assert run("Bash", command=cmd) == 2


@pytest.mark.parametrize("cmd", [
    "cat > f.py <<'EOF'\nfrom petctl.rl.train import make_env_fn\nimport os  # python .claude/x > y; launch_run.py\nEOF\npython f.py",
    "grep -rn petctl.rl.train petctl", "git commit -m 'run chain_ab_driver.py via launcher'", "python scripts/rl/bench_envs.py --n-envs 10", "python scripts/rl/eval_suite.py check",
    "git status", "git push origin feat/x", "git branch -d merged", "git reset --soft HEAD~1", "ls .claude", "cat .claude/settings.local.json",
    "python scripts/rl/launch_run.py pilot --steps 1000000 -- python x.py", "pytest tests", "git commit -m 'x'",
])
def test_allowed(cmd: str) -> None:
    assert run("Bash", command=cmd) == 0


def test_file_tools() -> None:
    assert run("Write", file_path="/Users/x/proj/.claude/agents/a.md") == 2
    assert run("Edit", file_path="/Users/x/.claude/settings.json") == 2
    assert run("Write", file_path="/Users/x/proj/docs/a.md") == 0
