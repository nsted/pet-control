"""Gate D1 of the shape-agnostic hug (stage ``m2_hug5``): the frozen held-out shape set, evaluated through ``run_gate2``.

Frozen: aggregate held-out success Wilson lower bound >= 0.70 (the stage gate threshold), every held-out shape >= 0.50, capacitive_off drop (paired bootstrap CI lower bound) >= 0.15,
current budget (median mean bus current <= 4 A, <= 5 % of episodes over the 10 A peak), glitch 0. Metric: hug metric v2 (normal span), see ``petctl.rl.hug``.
"""

from __future__ import annotations

from functools import partial

from petctl.rl.curriculum.gate2 import Ablation, Gate2Spec
from petctl.rl.target_shapes import heldout_group

MIN_SHAPE_SUCCESS = 0.50
CAP_OFF_MIN_DROP = 0.15


def gate2_spec_hug5(n_modules: int = 10) -> Gate2Spec:
    """The gate2 spec of ``m2_hug5``: 100 held-out episodes (5 shapes x 20, seeds 10000+, dr 0.6), per-shape floor, capacitive_off ablation. ``n_modules`` picks the length's held-out set
    (``target_shapes.held_out_set``: 8: cyl r3 / r7, square 8x8, rect 5x14, rounded 8x8 r2; 9: r3 / r8 ...; 11: r3 / r9 ...; otherwise the L = 10 set)."""
    return Gate2Spec(n=100, dr_scale=0.6, nominal=False, ablations=(Ablation("capacitive_off", {"capacitive_off": 1.0}, min_drop=CAP_OFF_MIN_DROP),),
                     group_of=partial(heldout_group, n_modules=n_modules), min_group_success=MIN_SHAPE_SUCCESS)
