"""Decision rules of the odd-length (L = 9) study driver (``scripts/rl/hug_curr_odd_driver.py``): flat pilot go / no-go and the automatic hour-by-hour hug continuation rule.

Pure functions over episode rows / screen results, so they are testable with synthetic data. The 9-module body is the GENERATED URDF variant (``petctl.sim.urdf_gen``, ``tail_flip`` auto);
whether the physical tail module can be mounted in the flipped orientation is an unconfirmed owner assumption (docs/plans/URDF_GENERATOR.md) and is stated in every result caption / log entry.
"""

from __future__ import annotations

from typing import Any

from petctl.rl.curriculum import m2v7
from petctl.rl.curriculum.hug_curr2 import SCREEN_DONE
from petctl.rl.target_shapes import held_out_set

FLAT_GO_SUCCESS = 0.15               # flat P1 pilot: rolling success over the last 100 episodes at the end of the run
FLAT_BUDGET_MIN, FLAT_CAP = 25.0, 6_000_000
RETRY_BUDGET_MIN, RETRY_CAP = 45.0, 12_000_000          # one retry (2x the steps, wall budget capped at 45 min by the 1 h rule)
HUG_BUDGET_MIN, HUG_CAP = 45.0, 4_000_000
HOUR2_MIN_SCREEN = 0.15              # hour 3 runs only if the hour-2 screen >= this ...
HOUR2_MIN_CURVED_FRAC = 2.0 / 6.0    # ... or any curved gate shape scored >= 2 of 6 screen episodes
MAX_HOURS = 3


def body_note(n_modules: int) -> str:
    """Caption / log note about the body of length ``n_modules`` (empty for the owner's 8-module export)."""
    return "" if int(n_modules) == 8 else f"{n_modules}-module body = generated URDF variant (tail_flip auto; physical tail orientation unconfirmed by the owner)"


def flat_pilot_go(rows: list[dict[str, Any]], steps: int) -> tuple[bool, dict[str, float]]:
    """Flat pilot rule: rolling success over the last 100 episodes finished by ``steps`` >= 0.15 (the median-ratio signal is recorded, not required)."""
    sig = m2v7.flat_signals(rows, steps)
    return bool(sig["n"] > 0 and sig["success"] >= FLAT_GO_SUCCESS), sig


def curved_shape_names(n_modules: int) -> tuple[str, ...]:
    """Names of the curved (cylinder / rounded box) shapes of the length's held-out set."""
    return tuple(h[0] for h in held_out_set(n_modules) if h[1] in ("cyl", "rbox"))


def hour_decision(hour: int, screen_success: float, per_shape: dict[str, float], n_modules: int) -> str:
    """What follows the n=30 screen of hug hour ``hour`` (1-3): ``final`` (screen >= 0.55) | ``next`` (run the next hour) | ``stop`` (after hour 2: screen < 0.15 and no curved gate shape >= 2/6;
    marker STOP_L<n>_hug) | ``stop_after3`` (hour 3 is the last automatic hour; marker STOP_L<n>_after3)."""
    if screen_success >= SCREEN_DONE:
        return "final"
    if hour >= MAX_HOURS:
        return "stop_after3"
    if hour == 1:
        return "next"
    curved_ok = any(per_shape.get(n, 0.0) >= HOUR2_MIN_CURVED_FRAC - 1e-9 for n in curved_shape_names(n_modules))
    return "next" if screen_success >= HOUR2_MIN_SCREEN - 1e-9 or curved_ok else "stop"
