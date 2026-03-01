"""
Behavior engine demo — test behaviors with mock backend + Rerun.

This shows the full stack:
  MockBackend (sine sensors) → BehaviorEngine → RerunVisualizer

Run:
    python examples/behavior_demo.py
    python examples/behavior_demo.py --behavior nestle
    python examples/behavior_demo.py --behavior recoil

The mock backend generates traveling sine-wave touch data across
the modules, so you can see nestle/recoil reacting to simulated
contact without a real robot.
"""

from __future__ import annotations

import asyncio
import argparse

from petctl import Controller
from petctl.backends.mock import MockBackend
from petctl.visualizers.rerun_viz import RerunVisualizer

from petctl.behaviors.engine import BehaviorEngine, BehaviorParams
from petctl.behaviors.nestle import NestleBehavior
from petctl.behaviors.recoil import RecoilBehavior
from petctl.behaviors.undulate import UndulateBehavior
from petctl.behaviors.posture import GoLimpBehavior, StiffenBehavior


def build_engine() -> BehaviorEngine:
    """Create a BehaviorEngine with all core behaviors registered."""
    engine = BehaviorEngine()
    engine.register(NestleBehavior())
    engine.register(RecoilBehavior())
    engine.register(UndulateBehavior())
    engine.register(GoLimpBehavior())
    engine.register(StiffenBehavior())
    return engine


def main() -> None:
    parser = argparse.ArgumentParser(description="Behavior engine demo")
    parser.add_argument(
        "--behavior", default="nestle",
        choices=["nestle", "recoil", "undulate", "limp", "stiffen", "combo"],
        help="Which behavior to demonstrate",
    )
    parser.add_argument("--intensity", type=float, default=0.7)
    parser.add_argument("--speed", type=float, default=0.5)
    args = parser.parse_args()

    # Mock backend with sine-wave sensors — simulates touch traveling
    # along the body so reactive behaviors have something to respond to.
    backend = MockBackend(mode="sine", num_modules=8, sine_hz=0.15)
    engine = build_engine()

    # Activate the requested behavior
    params = BehaviorParams(intensity=args.intensity, speed=args.speed)

    if args.behavior == "combo":
        # Layer undulate (base) + nestle (reactive) — the typical runtime config
        engine.request("undulate", BehaviorParams(intensity=0.3, speed=0.3))
        engine.request("nestle", params)
    else:
        engine.request(args.behavior, params)

    ctrl = Controller(
        backend=backend,
        scheme=engine,
        visualizers=[RerunVisualizer()],
    )

    asyncio.run(ctrl.run())


if __name__ == "__main__":
    main()
