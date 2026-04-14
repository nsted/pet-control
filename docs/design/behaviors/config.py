"""
Design sketch — behavior-layer limits.

Canonical hardware and loop limits live in the installed package:
`petctl/config.py` (`MOTOR_LIMITS`, `LOOP_LIMITS`, `BEHAVIOR_LIMITS`).

This file is not imported by `petctl`; it exists so behavior-engine sketches can
discuss angles and timing alongside the same names as production config.
Implement behaviors against `petctl.config`, not by copying values from here.
"""

from __future__ import annotations
