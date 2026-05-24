"""
OllamaControlScheme — touch-reactive control driven by a local LLM.

The scheme reads touch events from the controller's `touch_events` queue
(AsyncIO, emits on type transitions only) and asks a gemma3 model via Ollama
which movement to perform.  LLM calls happen in a background thread so the
30 Hz control loop stays non-blocking.  Between calls the last commanded
motion continues uninterrupted.

A persistent conversation is maintained for the session: the combined system
prompt (robot_context.md + behavior_guide.md) is sent once on start, then
each touch event is appended as a user turn so the model retains context.

Motion is delegated to the same ControlScheme classes used by standalone
patterns (patterns.py).  Intensity and speed from the LLM response are
used to scale amplitude and frequency when instantiating each pattern.

Behaviour is configured by petctl/prompts/robot_context.md — edit the
Character, Rules, and Principles sections freely; changes take effect on restart.
"""

from __future__ import annotations

import asyncio
import logging
import threading
from pathlib import Path
from typing import TYPE_CHECKING

from petctl.llm.client import OllamaClient
from petctl.protocols import ControlScheme
from petctl.schemes.patterns import (
    BreatheControlScheme,
    CascadeControlScheme,
    CoilControlScheme,
    CurlControlScheme,
    DriftControlScheme,
    FreezeControlScheme,
    PoseScheme,
    PulseControlScheme,
    RippleControlScheme,
    SlalomControlScheme,
    StrokeCurlScheme,
    StrokeReactControlScheme,
    StrokeRippleScheme,
    SwayControlScheme,
    TwitchControlScheme,
    WanderControlScheme,
    YieldStiffScheme,
)
from petctl.types import RobotState, ServoCommand, TouchSummary

if TYPE_CHECKING:
    from petctl.controller import Controller

logger = logging.getLogger(__name__)

_PROMPTS_DIR = Path(__file__).parent.parent / "prompts"

# Minimum seconds between LLM calls regardless of event rate.
_LLM_MIN_INTERVAL_S = 1.0

# Max amplitude per movement (degrees). intensity (0–1) scales this linearly.
# Patterns with no amplitude param use 0.0 — intensity is ignored for those.
_AMP_MAX: dict[str, float] = {
    "freeze":        0.0,
    "home":          0.0,
    "breathe":      12.0,
    "pulse":        50.0,
    "ripple":       40.0,
    "sway":         60.0,
    "cascade":      60.0,
    "slalom":       45.0,
    "twitch":       30.0,
    "coil":         55.0,
    "curl_right":   70.0,
    "curl_left":    70.0,
    "wander":        0.0,
    "drift":         0.0,
    "stroke":        0.0,  # touch-reactive — senses internally
    "stroke-curl":   0.0,
    "stroke-ripple": 0.0,
    "yield-stiff":   0.0,
    "pose":          0.0,
}

_VALID_MOVEMENTS = set(_AMP_MAX)


class _CurlLeft(CurlControlScheme):
    """CurlControlScheme with all joint signs negated → curl to the left."""

    name = "curl_left"

    def update(self, state: RobotState) -> list[ServoCommand]:
        cmds = super().update(state)
        for cmd in cmds:
            if cmd.position is not None:
                cmd.position = -cmd.position
        return cmds


def _make_pattern(motion: str, intensity: float, speed: float) -> ControlScheme:
    """Instantiate a ControlScheme for the given motion, scaled by intensity and speed."""
    amp = _AMP_MAX.get(motion, 0.0) * intensity

    if motion in ("freeze", "home"):
        return FreezeControlScheme()
    if motion == "breathe":
        return BreatheControlScheme(amplitude_deg=amp, hz=0.05 + speed * 0.06)
    if motion == "pulse":
        return PulseControlScheme(amplitude_deg=amp, hz=0.15 + speed * 0.35)
    if motion == "ripple":
        return RippleControlScheme(amplitude_deg=amp, hz=0.2 + speed * 0.4)
    if motion == "sway":
        return SwayControlScheme(amplitude_deg=amp, hz=0.08 + speed * 0.15)
    if motion == "cascade":
        return CascadeControlScheme(amplitude_deg=amp, hz=0.15 + speed * 0.25)
    if motion == "slalom":
        return SlalomControlScheme(amplitude_deg=amp, hz=0.1 + speed * 0.3)
    if motion == "twitch":
        return TwitchControlScheme(amplitude_deg=amp, smoothing=0.03 + speed * 0.08)
    if motion == "coil":
        return CoilControlScheme(amplitude_deg=amp, hz=0.1 + speed * 0.2)
    if motion == "curl_right":
        return CurlControlScheme(target_deg=amp)
    if motion == "curl_left":
        return _CurlLeft(target_deg=amp)
    if motion == "wander":
        return WanderControlScheme(speed_deg_per_s=20.0 + speed * 60.0)
    if motion == "drift":
        return DriftControlScheme()
    if motion == "stroke":
        return StrokeReactControlScheme()
    if motion == "stroke-curl":
        return StrokeCurlScheme()
    if motion == "stroke-ripple":
        return StrokeRippleScheme()
    if motion == "yield-stiff":
        return YieldStiffScheme()
    if motion == "pose":
        return PoseScheme()
    return FreezeControlScheme()


class OllamaControlScheme(ControlScheme):
    """Control scheme that uses a local Ollama LLM to map touch→movement.

    The controller populates state.touch each tick and emits TouchSummary
    events on the touch_events queue on type transitions.  This scheme drains
    that queue each tick and spawns a background LLM call for each new event.

    Args:
        model:    Ollama model tag (default: gemma3:1b).
        base_url: Ollama server URL (default: localhost:11434).
        timeout:  LLM HTTP timeout in seconds.
    """

    name = "ollama"

    def __init__(
        self,
        model: str = "gemma3:1b",
        base_url: str = "http://localhost:11434",
        timeout: float = 12.0,
        log_input: bool = False,
    ) -> None:
        self._client = OllamaClient(model=model, base_url=base_url, timeout=timeout, log_input=log_input)

        # Injected in on_start() — the controller's shared touch event queue.
        self._touch_queue: asyncio.Queue[TouchSummary] | None = None

        # Active delegated pattern — written by background thread (LLM response)
        # and by update() (touch-end home), read by update(). Protected by _lock.
        self._lock = threading.Lock()
        self._active_pattern: ControlScheme = FreezeControlScheme()

        # Stored so background thread can call on_start() on new patterns.
        self._controller: Controller | None = None

        self._last_call_t: float = 0.0
        self._pending: threading.Thread | None = None
        self._system_prompt: str = ""

    # ------------------------------------------------------------------
    # ControlScheme interface
    # ------------------------------------------------------------------

    def on_start(self, controller: Controller) -> None:
        self._controller = controller
        self._system_prompt = _load_system_prompt()
        initial = FreezeControlScheme()
        initial.on_start(controller)
        with self._lock:
            self._active_pattern = initial
        self._touch_queue = controller.touch_events

        if not self._client.is_available():
            logger.warning(
                "[Ollama] server not reachable at %s — "
                "start Ollama with 'ollama serve' then restart PET.",
                self._client._url,
            )
        else:
            self._client.start(self._system_prompt)
            logger.info(
                "[Ollama] connected, model=%s.",
                self._client.model,
            )

    def update(self, state: RobotState) -> list[ServoCommand]:
        if self._touch_queue is not None:
            self._drain_touch_queue()

        with self._lock:
            pattern = self._active_pattern

        return pattern.update(state)

    def take_slew_resets(self) -> dict[int, float]:
        """Forward slew reset requests from the active pattern to the controller."""
        with self._lock:
            pattern = self._active_pattern
        fn = getattr(pattern, "take_slew_resets", None)
        return fn() if fn is not None else {}

    def on_stop(self) -> None:
        if self._pending and self._pending.is_alive():
            self._pending.join(timeout=0.5)

    # ------------------------------------------------------------------
    # Touch queue draining
    # ------------------------------------------------------------------

    def _drain_touch_queue(self) -> None:
        assert self._touch_queue is not None
        while True:
            try:
                summary: TouchSummary = self._touch_queue.get_nowait()
            except asyncio.QueueEmpty:
                break

            now = summary.timestamp
            pending = self._pending

            # Contact ended — return to home.
            if summary.touch_type == "none":
                self._switch_pattern("home", 0.5, 0.3)
                continue

            # Rate-limit LLM calls; skip if one is already in flight.
            thread_free = pending is None or not pending.is_alive()
            if not thread_free or (now - self._last_call_t) < _LLM_MIN_INTERVAL_S:
                continue

            self._last_call_t = now
            description = summary.describe()
            t = threading.Thread(
                target=self._llm_call,
                args=(description,),
                daemon=True,
            )
            self._pending = t
            t.start()

    # ------------------------------------------------------------------
    # Pattern switching
    # ------------------------------------------------------------------

    def _switch_pattern(self, motion: str, intensity: float, speed: float) -> None:
        """Instantiate and activate a new pattern. Safe to call from any thread."""
        pattern = _make_pattern(motion, intensity, speed)
        controller = self._controller
        if controller is not None:
            pattern.on_start(controller)
        with self._lock:
            self._active_pattern = pattern

    # ------------------------------------------------------------------
    # LLM interaction (background thread)
    # ------------------------------------------------------------------

    def _llm_call(self, touch_description: str) -> None:
        logger.debug("[Ollama] calling LLM: %s", touch_description)
        result = self._client.chat(touch_description)
        if result is None:
            return
        self._apply_llm_response(result)

    def _apply_llm_response(self, response: dict) -> None:
        motion = str(response.get("movement", "")).strip().lower()
        if motion not in _VALID_MOVEMENTS:
            logger.warning(
                "[Ollama] unknown movement %r — ignoring. Valid: %s",
                motion,
                sorted(_VALID_MOVEMENTS),
            )
            return

        intensity = max(0.0, min(1.0, float(response.get("intensity", 0.5))))
        speed = max(0.05, min(1.0, float(response.get("speed", 0.4))))

        logger.info("[Ollama] → %s (intensity=%.2f, speed=%.2f)", motion, intensity, speed)
        self._switch_pattern(motion, intensity, speed)


# ------------------------------------------------------------------
# Prompt loading
# ------------------------------------------------------------------

def _load_system_prompt() -> str:
    """Load the system prompt from robot_context.md."""
    path = _PROMPTS_DIR / "robot_context.md"
    try:
        return path.read_text(encoding="utf-8").strip()
    except FileNotFoundError:
        logger.warning("[Ollama] prompt file not found: %s", path)
        return ""
