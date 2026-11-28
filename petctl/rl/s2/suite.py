"""S2 frozen suites with a SHA manifest (STAGE_2_GATES, Two-phase freeze). SHA set A (written at T6) = the NON-yield decision / test suites for every level, the yield DRAWS (cell, onset, body, direction,
multiplier, seeds) for every level, the cell set, poses and ladder definitions. Section B (press magnitudes and loaded joints per cell) is written ONCE at T14, with its own SHA; a second write fails.
Changing anything in set A needs the owner's approval."""

from __future__ import annotations

import hashlib
import json
import time
from pathlib import Path
from typing import Any

from petctl.rl.s2.ladder import LEVELS, level_dict
from petctl.rl.s2.scenario import S2Scenario, build_suite

SUITE_DIR = Path("data/rl/s2/suite")
SPLITS = ("decision", "test")
LEVEL_NAMES = tuple(LEVELS)
MANIFEST = "manifest.json"
YIELD_B = "yield_B.json"


def _sha(p: Path) -> str:
    return hashlib.sha256(p.read_bytes()).hexdigest()


def _dump(obj: Any, p: Path) -> None:
    p.write_text(json.dumps(obj, indent=1, sort_keys=True))


def write_set_a(cells: list[tuple[str, str]], meta: dict[str, Any], root: Path = SUITE_DIR) -> dict[str, Any]:
    """Generate and freeze SHA set A. Refuses to overwrite an existing manifest."""
    if (root / MANIFEST).exists():
        raise FileExistsError(f"{root / MANIFEST} exists: S2 suites are frozen (changing them needs the owner's approval)")
    root.mkdir(parents=True, exist_ok=True)
    files: dict[str, str] = {}
    counts: dict[str, int] = {}
    for split in SPLITS:
        for lv in LEVEL_NAMES:
            ny, yd = build_suite(split, lv, cells)
            for kind, sc in (("nonyield", ny), ("yield_draws", yd)):
                name = f"{split}_{lv}_{kind}.json"
                _dump([s.to_dict() for s in sc], root / name)
                files[name] = _sha(root / name)
                counts[name] = len(sc)
    _dump({"cells": cells}, root / "cells.json")
    files["cells.json"] = _sha(root / "cells.json")
    man = {"created": time.strftime("%Y-%m-%d %H:%M:%S"), "files": files, "counts": counts, "levels": {k: level_dict(k) for k in LEVEL_NAMES}, "meta": meta,
           "friction_note": "friction draws below 1.0 are NOT REALISED until the owner approves the friction fix (T12): the floor's 1.0 wins in MuJoCo"}
    _dump(man, root / MANIFEST)
    return man


def check_set_a(root: Path = SUITE_DIR) -> list[str]:
    """Problems with set A (empty = intact): missing or changed files, and any yield-draw record carrying press fields (B must hold those)."""
    bad = []
    man = json.loads((root / MANIFEST).read_text())
    for name, sha in man["files"].items():
        if not (root / name).exists():
            bad.append(f"missing {name}")
        elif _sha(root / name) != sha:
            bad.append(f"changed {name}")
    for name in man["files"]:
        if name.endswith("yield_draws.json") and (root / name).exists():
            for rec in json.loads((root / name).read_text()):
                if "press_n" in rec or "loaded_joints" in rec:
                    bad.append(f"{name}: draw carries press / loaded-joint fields (they belong in section B)")
                    break
    return bad


def write_yield_b(presses: dict[str, dict[str, Any]], root: Path = SUITE_DIR) -> str:
    """Write section B ONCE: ``presses[cell_key] = {"press_n": float, "loaded_joints": [int, int, int]}``. A second write raises."""
    p = root / YIELD_B
    if p.exists():
        raise FileExistsError("yield section B is write-once (STAGE_2_GATES, Two-phase freeze)")
    for k, v in presses.items():
        assert set(v) == {"press_n", "loaded_joints"}, f"section B holds press magnitudes and loaded joints only ({k})"
    _dump(presses, p)
    man = json.loads((root / MANIFEST).read_text())
    man["yield_B"] = {"file": YIELD_B, "sha": _sha(p), "written": time.strftime("%Y-%m-%d %H:%M:%S")}
    _dump(man, root / MANIFEST)
    return man["yield_B"]["sha"]


def load(split: str, level: str, kind: str = "nonyield", root: Path = SUITE_DIR) -> list[S2Scenario]:
    """Load a frozen list (``nonyield`` or ``yield_draws``) after checking set A; yield scenarios get their press from section B when it exists."""
    bad = check_set_a(root)
    if bad:
        raise RuntimeError("S2 suite tampered: " + "; ".join(bad))
    recs = json.loads((root / f"{split}_{level}_{kind}.json").read_text())
    b = json.loads((root / YIELD_B).read_text()) if (kind == "yield_draws" and (root / YIELD_B).exists()) else {}
    out = []
    for r in recs:
        if kind == "yield_draws" and b:
            r = {**r, "press_n": b[f"{r['pose']}|{r['orient']}"]["press_n"]}
        out.append(S2Scenario(**r))
    return out
