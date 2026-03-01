"""
petctl.perception.touch_events — Touch event classification.

Sits between petctl's RobotState and the behavior/agent layers.
Consumes the raw sensor stream, maintains a rolling window, and
emits structured TouchEvent objects.

Sensor frame layout (per tick):
    The classifier sees a 2D array:  (window_length, features_per_frame)

    Current features per frame (touch + pressure only):
        For each of 8 modules, 6 sensor channels:
            touch_middle, touch_left, touch_right,
            pressure_middle, pressure_left, pressure_right

        Total: 8 modules × 6 channels = 48 features per frame

    Future features when IMU is added:
        Append to the same frame vector:
            accel_x, accel_y, accel_z,        (linear acceleration, m/s²)
            gyro_x, gyro_y, gyro_z,           (angular velocity, rad/s)
            orient_w, orient_x, orient_y, orient_z  (quaternion)

        New total: 48 + 10 = 58 features per frame

    The classifier input shape is (1, window_length, num_features).
    When you add the IMU, you just change num_features and retrain.
    No structural changes needed to the pipeline.

Touch event types (initial vocabulary — expand as needed):
    idle        — no meaningful contact
    stroke      — sustained moving contact along the body
    pat         — brief gentle contact
    strike      — sudden hard contact
    hold        — sustained static contact (cradling, gripping)
    squeeze     — increasing pressure without movement
    release     — contact ending (transition from any → idle)

These are what the agent sees.  The agent never looks at raw
sensor values — it reasons about "I'm being stroked on my
midsection" not "touch_left on module 3 is 0.7."
"""

from __future__ import annotations

import time
from collections import deque
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional

from petctl.types import RobotState


# ---------------------------------------------------------------------------
# Touch event types
# ---------------------------------------------------------------------------

class TouchType(Enum):
    IDLE = "idle"
    STROKE = "stroke"
    PAT = "pat"
    STRIKE = "strike"
    HOLD = "hold"
    SQUEEZE = "squeeze"
    RELEASE = "release"


@dataclass
class TouchEvent:
    """A classified touch interaction.

    This is what flows UP to the agent layer.  It describes WHAT is
    happening in human-interpretable terms, not raw sensor values.
    """
    touch_type: TouchType
    confidence: float               # 0.0–1.0, classifier confidence

    # Where on the body
    primary_modules: list[int]      # modules most involved (e.g. [2, 3, 4])
    primary_face: Optional[str]     # "left", "right", "middle", or None

    # Intensity and dynamics
    intensity: float                # 0.0–1.0, overall strength
    velocity: float                 # how fast the touch is moving along body
                                    # (positive = head→tail, negative = tail→head)

    # Timing
    timestamp: float = field(default_factory=time.monotonic)
    duration: float = 0.0           # how long this event has been ongoing

    # Raw context (for the agent to use if it wants richer detail)
    module_activations: dict[int, float] = field(default_factory=dict)
    # Maps module_id → total touch activation (0-1)


# ---------------------------------------------------------------------------
# Sensor stream — the rolling window the classifier consumes
# ---------------------------------------------------------------------------

# Sensor channels per module, in order
_TOUCH_CHANNELS = (
    "touch_middle", "touch_left", "touch_right",
    "pressure_middle", "pressure_left", "pressure_right",
)

_NUM_MODULES = 8
_CHANNELS_PER_MODULE = len(_TOUCH_CHANNELS)
# Will become 58 when IMU is added
FEATURES_PER_FRAME = _NUM_MODULES * _CHANNELS_PER_MODULE  # 48


@dataclass
class SensorFrame:
    """A single timestep of sensor data, flattened for the classifier."""
    timestamp: float
    features: list[float]           # length = FEATURES_PER_FRAME
    # Placeholder for IMU — currently empty, will be populated later.
    # When added, these get appended to features and FEATURES_PER_FRAME
    # increases accordingly.
    imu_accel: Optional[tuple[float, float, float]] = None
    imu_gyro: Optional[tuple[float, float, float]] = None
    imu_orient: Optional[tuple[float, float, float, float]] = None


class SensorStream:
    """Maintains a rolling window of sensor frames for the classifier.

    Usage:
        stream = SensorStream(window_length=50)

        # In your control loop (every tick):
        stream.push(robot_state)
        window = stream.get_window()   # shape: (window_length, FEATURES_PER_FRAME)
        if window is not None:
            events = classifier.classify(window)
    """

    def __init__(self, window_length: int = 50) -> None:
        self.window_length = window_length
        self._buffer: deque[SensorFrame] = deque(maxlen=window_length)

    def push(self, state: RobotState) -> None:
        """Extract sensor features from RobotState and append to buffer."""
        features: list[float] = []
        for mod_id in range(_NUM_MODULES):
            sensors = state.sensors.get(mod_id)
            if sensors:
                for channel in _TOUCH_CHANNELS:
                    features.append(getattr(sensors, channel, 0.0))
            else:
                features.extend([0.0] * _CHANNELS_PER_MODULE)

        # IMU placeholder — when RobotState gets IMU fields, extract here:
        # imu_accel = (state.imu_accel_x, state.imu_accel_y, state.imu_accel_z)
        # imu_gyro  = (state.imu_gyro_x, ...)
        # features.extend([*imu_accel, *imu_gyro, *imu_orient])

        self._buffer.append(SensorFrame(
            timestamp=state.timestamp,
            features=features,
        ))

    def get_window(self) -> Optional[list[list[float]]]:
        """Return the current window as a 2D list, or None if not full yet.

        Returns:
            List of shape (window_length, FEATURES_PER_FRAME), suitable
            for conversion to a tensor: torch.tensor(window).unsqueeze(0)
        """
        if len(self._buffer) < self.window_length:
            return None
        return [frame.features for frame in self._buffer]

    def get_partial_window(self) -> list[list[float]]:
        """Return whatever frames we have (may be shorter than window_length).

        Useful for heuristic classifiers that don't need a full window.
        """
        return [frame.features for frame in self._buffer]

    @property
    def is_ready(self) -> bool:
        return len(self._buffer) >= self.window_length

    def clear(self) -> None:
        self._buffer.clear()


# ---------------------------------------------------------------------------
# Heuristic classifier — no ML needed, works immediately
# ---------------------------------------------------------------------------

class HeuristicTouchClassifier:
    """Simple rule-based touch classifier for bootstrapping.

    Good enough to get the behavior engine and agent loop working
    before you train a real model.  Uses thresholds and temporal
    patterns on the raw sensor stream.

    Replace with a trained BiLSTM (or whatever) when you have
    labeled data.  The interface is the same: takes a window,
    returns a TouchEvent.
    """

    def __init__(
        self,
        strike_threshold: float = 0.6,
        stroke_velocity_threshold: float = 0.3,
        hold_duration_threshold: float = 1.0,
    ) -> None:
        self.strike_threshold = strike_threshold
        self.stroke_velocity_threshold = stroke_velocity_threshold
        self.hold_duration_threshold = hold_duration_threshold
        self._event_start: Optional[float] = None
        self._last_active_modules: list[int] = []

    def classify(self, stream: SensorStream, state: RobotState) -> TouchEvent:
        """Classify the current touch state from the sensor stream.

        This runs every tick.  It examines the most recent frames
        to determine what's happening right now.
        """
        frames = stream.get_partial_window()
        if not frames:
            return TouchEvent(
                touch_type=TouchType.IDLE,
                confidence=1.0,
                primary_modules=[],
                primary_face=None,
                intensity=0.0,
                velocity=0.0,
            )

        # Analyze the most recent frame
        latest = frames[-1]
        module_totals: dict[int, float] = {}
        module_faces: dict[int, str] = {}

        for mod_id in range(_NUM_MODULES):
            base = mod_id * _CHANNELS_PER_MODULE
            t_mid = latest[base + 0]
            t_left = latest[base + 1]
            t_right = latest[base + 2]
            p_mid = latest[base + 3]
            p_left = latest[base + 4]
            p_right = latest[base + 5]

            total = t_mid + t_left + t_right + p_mid + p_left + p_right
            module_totals[mod_id] = total

            # Determine primary face for this module
            face_vals = {"middle": t_mid + p_mid, "left": t_left + p_left, "right": t_right + p_right}
            if max(face_vals.values()) > 0.05:
                module_faces[mod_id] = max(face_vals, key=face_vals.get)

        # Active modules: those with meaningful touch
        active = [m for m, t in module_totals.items() if t > 0.1]
        overall_intensity = max(module_totals.values()) if module_totals else 0.0

        # No touch
        if not active:
            if self._event_start is not None:
                self._event_start = None
                return TouchEvent(
                    touch_type=TouchType.RELEASE,
                    confidence=0.8,
                    primary_modules=self._last_active_modules,
                    primary_face=None,
                    intensity=0.0,
                    velocity=0.0,
                )
            return TouchEvent(
                touch_type=TouchType.IDLE,
                confidence=1.0,
                primary_modules=[],
                primary_face=None,
                intensity=0.0,
                velocity=0.0,
            )

        # Track event timing
        now = time.monotonic()
        if self._event_start is None:
            self._event_start = now
        duration = now - self._event_start

        # Primary face: most common face among active modules
        face_counts: dict[str, int] = {}
        for m in active:
            f = module_faces.get(m)
            if f:
                face_counts[f] = face_counts.get(f, 0) + 1
        primary_face = max(face_counts, key=face_counts.get) if face_counts else None

        # --- Classify by temporal pattern ---

        # Strike: sudden onset, high intensity
        if len(frames) >= 3 and duration < 0.3:
            prev = frames[-3]
            prev_max = 0.0
            for mod_id in range(_NUM_MODULES):
                base = mod_id * _CHANNELS_PER_MODULE
                prev_max = max(prev_max, sum(prev[base:base + _CHANNELS_PER_MODULE]))
            onset_delta = overall_intensity - prev_max
            if onset_delta > self.strike_threshold:
                self._last_active_modules = active
                return TouchEvent(
                    touch_type=TouchType.STRIKE,
                    confidence=min(1.0, onset_delta / self.strike_threshold),
                    primary_modules=active,
                    primary_face=primary_face,
                    intensity=overall_intensity,
                    velocity=0.0,
                    duration=duration,
                    module_activations=module_totals,
                )

        # Stroke: contact moving along the body over time
        if len(frames) >= 10:
            # Compare center of mass of touch across recent frames
            recent_coms = []
            lookback = min(20, len(frames))
            for i in range(-lookback, 0):
                frame = frames[i]
                weighted_sum = 0.0
                total_weight = 0.0
                for mod_id in range(_NUM_MODULES):
                    base = mod_id * _CHANNELS_PER_MODULE
                    t = sum(frame[base:base + _CHANNELS_PER_MODULE])
                    weighted_sum += mod_id * t
                    total_weight += t
                if total_weight > 0.1:
                    recent_coms.append(weighted_sum / total_weight)

            if len(recent_coms) >= 5:
                # Velocity = how fast the center of mass is moving
                velocity = (recent_coms[-1] - recent_coms[0]) / (len(recent_coms) * 0.05)
                if abs(velocity) > self.stroke_velocity_threshold:
                    self._last_active_modules = active
                    return TouchEvent(
                        touch_type=TouchType.STROKE,
                        confidence=min(1.0, abs(velocity) / 2.0),
                        primary_modules=active,
                        primary_face=primary_face,
                        intensity=overall_intensity,
                        velocity=velocity,
                        duration=duration,
                        module_activations=module_totals,
                    )

        # Hold: sustained static contact
        if duration > self.hold_duration_threshold and len(active) > 0:
            self._last_active_modules = active
            return TouchEvent(
                touch_type=TouchType.HOLD,
                confidence=min(1.0, duration / 3.0),
                primary_modules=active,
                primary_face=primary_face,
                intensity=overall_intensity,
                velocity=0.0,
                duration=duration,
                module_activations=module_totals,
            )

        # Pat: brief gentle contact (short duration, moderate intensity)
        if duration < 0.5 and overall_intensity < 0.5:
            self._last_active_modules = active
            return TouchEvent(
                touch_type=TouchType.PAT,
                confidence=0.6,
                primary_modules=active,
                primary_face=primary_face,
                intensity=overall_intensity,
                velocity=0.0,
                duration=duration,
                module_activations=module_totals,
            )

        # Default: hold (contact exists but doesn't match other patterns)
        self._last_active_modules = active
        return TouchEvent(
            touch_type=TouchType.HOLD,
            confidence=0.4,
            primary_modules=active,
            primary_face=primary_face,
            intensity=overall_intensity,
            velocity=0.0,
            duration=duration,
            module_activations=module_totals,
        )
