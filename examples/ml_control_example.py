"""
ML Control Scheme Example
=========================
Shows how to wrap a PyTorch model (the BiLSTM touch classifier from
grapple_ai) into a petctl ControlScheme.

Run with mock backend (no robot needed):
    python examples/ml_control_example.py

Run with real robot:
    python examples/ml_control_example.py --real
"""

from __future__ import annotations

import asyncio
import argparse
import math
import os
import sys
from collections import deque

import numpy as np

# Add grapple_ai to path so we can import the existing model.
# Override with GRAPPLE_AI_PATH env var for non-standard directory layouts.
_GRAPPLE_AI = os.environ.get(
    "GRAPPLE_AI_PATH",
    os.path.normpath(os.path.join(os.path.dirname(__file__), "../../python/grapple_ai")),
)
if os.path.isdir(_GRAPPLE_AI) and _GRAPPLE_AI not in sys.path:
    sys.path.insert(0, _GRAPPLE_AI)

from petctl import Controller, ControlScheme, RobotState, ServoCommand  # noqa: E402
from petctl.backends.mock import MockBackend
from petctl.backends.robot import RobotBackend
from petctl.visualizers.rerun_viz import RerunVisualizer

# Sensor fields in the order the model expects
_SENSOR_FIELDS = (
    "touch_middle", "touch_left", "touch_right",
    "pressure_middle", "pressure_left", "pressure_right",
)


class TouchReactiveScheme(ControlScheme):
    """
    Uses the BiLSTM touch classifier from grapple_ai to drive servos
    based on detected touch type and valence.

    Positive valence → extend all joints to +20°
    Negative valence → retract all joints to -20°
    Neutral          → return to 0°

    Replace the valence-to-servo mapping with your own logic.
    """

    name = "touch_reactive"

    def __init__(
        self,
        model_path: str,
        sequence_length: int = 50,
        num_modules: int = 10,
        num_servos: int = 8,
    ) -> None:
        self.model_path = model_path
        self.sequence_length = sequence_length
        self.num_modules = num_modules
        self.num_servos = num_servos

        # Rolling buffer of sensor frames
        self._buffer: deque = deque(maxlen=sequence_length)
        self._model = None
        self._torch = None  # set in on_start() if torch is available

    def on_start(self, controller) -> None:
        try:
            import torch
            from models.touch_classifier import GrappleTouchClassifier

            self._torch = torch
            self._model = GrappleTouchClassifier()
            state_dict = torch.load(self.model_path, map_location="cpu")
            self._model.load_state_dict(state_dict)
            self._model.eval()
            print(f"[TouchReactiveScheme] Loaded model from {self.model_path}")
        except ImportError as e:
            print(f"[TouchReactiveScheme] Import error: {e}")
            print("  Install torch: pip install torch")
        except FileNotFoundError:
            print(f"[TouchReactiveScheme] Model file not found: {self.model_path}")
            print("  Train the model first using grapple_ai/training/train_touch_classifier.py")

    def update(self, state: RobotState) -> list[ServoCommand]:
        # Build a feature frame: num_modules * 6 sensors
        frame: list[float] = []
        for mod_id in range(self.num_modules):
            sensors = state.sensors.get(mod_id)
            if sensors:
                frame.extend([getattr(sensors, f, 0.0) for f in _SENSOR_FIELDS])
            else:
                frame.extend([0.0] * 6)

        self._buffer.append(frame)

        if self._model is None or self._torch is None or len(self._buffer) < self.sequence_length:
            return []  # Not ready yet

        torch = self._torch

        # Shape: (1, sequence_length, num_modules * 6)
        x = torch.tensor(list(self._buffer), dtype=torch.float32).unsqueeze(0)

        with torch.no_grad():
            touch_logits, valence_logits = self._model(x)

        valence_class = valence_logits.argmax().item()
        # 0 = negative, 1 = neutral, 2 = positive
        if valence_class == 2:
            target_angle = 20.0
        elif valence_class == 0:
            target_angle = -20.0
        else:
            target_angle = 0.0

        return [
            ServoCommand.from_angle(servo_id=i + 1, angle_deg=target_angle)
            for i in range(self.num_servos)
        ]


class SineWaveScheme(ControlScheme):
    """
    Simple demonstration scheme: drives each servo with a sine wave.
    No ML required.  Useful for testing the visualizer.
    """

    name = "sine_wave"

    def __init__(self, amplitude_deg: float = 30.0, hz: float = 0.3) -> None:
        self.amplitude = amplitude_deg
        self.hz = hz
        self._t = 0.0

    def update(self, state: RobotState) -> list[ServoCommand]:
        self._t += state.dt
        commands = []
        for i in range(8):
            phase = (i / 8) * 2 * math.pi  # stagger phases across servos
            angle = self.amplitude * math.sin(2 * math.pi * self.hz * self._t + phase)
            commands.append(ServoCommand.from_angle(servo_id=i + 1, angle_deg=angle))
        return commands


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(description="petctl ML control example")
    parser.add_argument("--real", action="store_true", help="Use real robot (RobotBackend)")
    parser.add_argument("--host", default="pet-robot.local", help="Robot hostname")
    parser.add_argument(
        "--scheme", choices=["touch", "sine"], default="sine",
        help="Control scheme to use (default: sine — no model needed)"
    )
    parser.add_argument(
        "--model", default="grapple_ai/models/best_grapple_model.pth",
        help="Path to trained model weights (for --scheme touch)"
    )
    args = parser.parse_args()

    # Backend
    if args.real:
        backend = RobotBackend(host=args.host)
    else:
        backend = MockBackend(mode="sine", num_modules=4)

    # Scheme
    if args.scheme == "touch":
        scheme = TouchReactiveScheme(model_path=args.model)
    else:
        scheme = SineWaveScheme()

    ctrl = Controller(
        backend=backend,
        scheme=scheme,
        visualizers=[RerunVisualizer()],
        dry_run=not args.real,  # safe by default when using mock
    )

    asyncio.run(ctrl.run())


if __name__ == "__main__":
    main()
