"""Gate D1 of the shape-agnostic hug (stage ``m2_hug5``): the frozen held-out shape set, evaluated through ``run_gate2``.

Frozen: aggregate held-out success Wilson lower bound >= 0.70 (the stage gate threshold), every held-out shape >= 0.50, capacitive_off drop (paired bootstrap CI lower bound) >= 0.15,
current budget (median mean bus current <= 4 A, <= 5 % of episodes over the 10 A peak), glitch 0. Metric: hug metric v2 (normal span), see ``petctl.rl.hug``.
"""

from __future__ import annotations

from petctl.rl.curriculum.gate2 import Ablation, Gate2Spec
from petctl.rl.target_shapes import heldout_group

MIN_SHAPE_SUCCESS = 0.50
CAP_OFF_MIN_DROP = 0.15


def gate2_spec_hug5() -> Gate2Spec:
    """The gate2 spec of ``m2_hug5``: 100 held-out episodes (5 shapes x 20, seeds 10000+, dr 0.6), per-shape floor, capacitive_off ablation."""
    return Gate2Spec(n=100, dr_scale=0.6, nominal=False, ablations=(Ablation("capacitive_off", {"capacitive_off": 1.0}, min_drop=CAP_OFF_MIN_DROP),),
                     group_of=heldout_group, min_group_success=MIN_SHAPE_SUCCESS)
