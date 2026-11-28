"""S2 frozen suites: SHA set A intact, yield draws hash into A and carry no press fields, section B is write-once (STAGE_2_GATES, Two-phase freeze)."""
from __future__ import annotations

import json
import shutil
from pathlib import Path

import pytest

from petctl.rl.s2 import suite

pytestmark = pytest.mark.skipif(not (suite.SUITE_DIR / suite.MANIFEST).exists(), reason="S2 suites not frozen yet")


def test_set_a_is_intact() -> None:
    assert suite.check_set_a() == []


def test_yield_draws_are_in_set_a_and_have_no_press() -> None:
    man = json.loads((suite.SUITE_DIR / suite.MANIFEST).read_text())
    for split in suite.SPLITS:
        for lv in suite.LEVEL_NAMES:
            name = f"{split}_{lv}_yield_draws.json"
            assert name in man["files"]
            recs = json.loads((suite.SUITE_DIR / name).read_text())
            assert recs and all("press_n" not in r and "loaded_joints" not in r for r in recs)


def test_tamper_is_detected_and_b_is_write_once(tmp_path: Path) -> None:
    shutil.copytree(suite.SUITE_DIR, tmp_path / "s")
    root = tmp_path / "s"
    (root / suite.YIELD_B).unlink(missing_ok=True)
    assert suite.check_set_a(root) == []
    cells = json.loads((root / "cells.json").read_text())["cells"]
    b = {f"{p}|{o}": {"press_n": 3.0, "loaded_joints": [1, 2, 3]} for p, o in cells}
    sha1 = suite.write_yield_b(b, root)
    assert suite.check_set_a(root) == []                                   # writing B does not change set A
    with pytest.raises(FileExistsError):
        suite.write_yield_b(b, root)
    assert sha1
    f = root / "test_L2_nonyield.json"
    f.write_text(f.read_text().replace("push", "pusH", 1))
    assert any("changed test_L2_nonyield.json" in x for x in suite.check_set_a(root))
    with pytest.raises(FileExistsError):
        suite.write_set_a([("head lift", "upright")], {}, root)
