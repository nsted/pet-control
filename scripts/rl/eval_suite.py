"""Frozen eval suites: ``build`` (first time only), ``check`` (exit 1 if any file changed), ``list``."""

from __future__ import annotations

import sys

from petctl.rl.evalsuite import SKILLS, SPLITS, SUITE_DIR, check_manifest, write_suite


def main() -> int:
    """CLI entry point."""
    cmd = sys.argv[1] if len(sys.argv) > 1 else "check"
    if cmd == "build":
        m = write_suite(overwrite="--overwrite-approved" in sys.argv)
        print(f"wrote {len(m['files'])} files, suite sha256 {m['suite_sha256']}")
        return 0
    if cmd == "list":
        for s in SPLITS:
            print(s, ", ".join(SKILLS))
        return 0
    bad = check_manifest()
    print("eval suite OK" if not bad else "EVAL SUITE CHANGED:\n  " + "\n  ".join(bad), f"({SUITE_DIR})")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
