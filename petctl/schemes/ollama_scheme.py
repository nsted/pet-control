"""
OllamaControlScheme — touch-reactive control driven by a local LLM.

The scheme reads touch events from the controller's `touch_events` queue
(AsyncIO, emits on type transitions only) and asks a gemma3 model via Ollama
which movement to perform.  LLM calls happen in a background thread so the
30 Hz control loop stays non-blocking.  Between calls the last commanded
motion continues uninterrupted.

Behaviour is configured by two markdown files in petctl/prompts/:
    robot_context.md   — technical reference (auto-loaded, do not edit)
    behavior_guide.md  — user-editable personality and touch→movement rules
"""

from __future__ import annotations

import asyncio
import logging
import math
import random
import threading
import time
from pathlib import Path
from typing import TYPE_CHECKING

from petctl.llm.client import OllamaClient
from petctl.protocols import ControlScheme
from petctl.types import RobotState, ServoCommand, TouchSummary

if TYPE_CHECKING:
    from petctl.controller import Controller

logger = logging.getLogger(__name__)

_PROMPTS_DIR = Path(__file__).parent.parent / "prompts"

# Minimum seconds between LLM calls regardless of event rate.
_LLM_MIN_INTERVAL_S = 1.0

# Max angle magnitude per motion mode (degrees). intensity (0-1) scales this.
_MAX_AMP: dict[str, float] = {
    "freeze":     0.0,
    "home":       0.0,
    "pulse":     50.0,
    "ripple":    40.0,
    "sway":      25.0,
    "curl_right": 45.0,
    "curl_left":  45.0,
    "twitch":    12.0,
}

_VALID_MOVEMENTS = set(_MAX_AMP)


class OllamaControlScheme(ControlScheme):
    """Control scheme that uses a local Ollama LLM to map touch→movement.

    The controller populates state.touch each tick and emits TouchSummary
    events on the touch_events queue on type transitions.  This scheme drains
    that queue each tick and spawns a background LLM call for each new event.

    Args:
        model:    Ollama model tag (default: gemma3:4b).
        base_url: Ollama server URL (default: localhost:11434).
        timeout:  LLM HTTP timeout in seconds.
    """

    name = "ollama"

    def __init__(
        self,
        model: str = "gemma3:1b",
        base_url: str = "http://localhost:11434",
        timeout: float = 12.0,
    ) -> None:
        self._client = OllamaClient(model=model, base_url=base_url, timeout=timeout)

        # Injected in on_start() — the controller's shared touch event queue.
        self._touch_queue: asyncio.Queue[TouchSummary] | None = None

        # Current motion state — written by background thread, read by update().
        self._lock = threading.Lock()
        self._motion: str = "freeze"
        self._intensity: float = 0.5
        self._speed: float = 0.4
        self._motion_start_t: float = 0.0

        self._last_call_t: float = 0.0
        self._pending: threading.Thread | None = None

        self._system_prompt: str = ""
        self._active_ids: list[int] = []

        # Per-servo random targets for twitch mode.
        self._twitch_targets: dict[int, float] = {}
        self._twitch_next_t: float = 0.0

    # ------------------------------------------------------------------
    # ControlScheme interface
    # ------------------------------------------------------------------

    def on_start(self, controller: Controller) -> None:
        self._system_prompt = _load_system_prompt()
        self._active_ids = sorted(controller.state.active_servo_ids)
        self._motion_start_t = time.monotonic()
        # Read from the controller's shared transition-only touch event queue.
        self._touch_queue = controller.touch_events

        if not self._client.is_available():
            logger.warning(
                "[Ollama] server not reachable at %s — "
                "start Ollama with 'ollama serve' then restart PET.",
                self._client._url,
            )
        else:
            logger.info(
                "[Ollama] connected, model=%s, %d active servos.",
                self._client.model,
                len(self._active_ids),
            )

    def update(self, state: RobotState) -> list[ServoCommand]:
        self._active_ids = sorted(state.active_servo_ids)
        if not self._active_ids:
            return []

        # Drain touch transition events and trigger LLM calls.
        # The controller emits one event per type change (onset, upgrade, end),
        # so no further debouncing is needed beyond the minimum interval guard.
        if self._touch_queue is not None:
            self._drain_touch_queue()

        return self._generate_commands(state, time.monotonic())

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

            now = time.monotonic()
            pending = self._pending

            # Contact ended — return to home.
            if summary.touch_type == "none":
                with self._lock:
                    if self._motion not in ("freeze", "home"):
                        self._motion = "home"
                        self._speed = 0.3
                        self._motion_start_t = now
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
    # LLM interaction (background thread)
    # ------------------------------------------------------------------

    def _llm_call(self, touch_description: str) -> None:
        logger.debug("[Ollama] calling LLM: %s", touch_description)
        result = self._client.chat(self._system_prompt, touch_description)
        if result is None:
            return
        self._apply_llm_response(result)

    def _apply_llm_response(self, response: dict) -> None:
        movement = str(response.get("movement", "")).strip().lower()
        if movement not in _VALID_MOVEMENTS:
            logger.warning(
                "[Ollama] unknown movement %r — ignoring. Valid: %s",
                movement,
                sorted(_VALID_MOVEMENTS),
            )
            return

        intensity = float(response.get("intensity", 0.5))
        speed = float(response.get("speed", 0.4))
        intensity = max(0.0, min(1.0, intensity))
        speed = max(0.05, min(1.0, speed))

        with self._lock:
            prev = self._motion
            self._motion = movement
            self._intensity = intensity
            self._speed = speed
            if movement != prev:
                self._motion_start_t = time.monotonic()

        logger.info("[Ollama] → %s (intensity=%.2f, speed=%.2f)", movement, intensity, speed)

    # ------------------------------------------------------------------
    # Motion generation
    # ------------------------------------------------------------------

    def _generate_commands(self, state: RobotState, now: float) -> list[ServoCommand]:
        with self._lock:
            motion = self._motion
            intensity = self._intensity
            speed = self._speed
            motion_t = now - self._motion_start_t

        ids = self._active_ids
        n = len(ids)
        amp = _MAX_AMP.get(motion, 0.0) * intensity

        if motion in ("freeze", "home"):
            return [ServoCommand.from_angle(sid, 0.0) for sid in ids]

        if motion == "pulse":
            hz = 0.15 + speed * 0.35
            angle = amp * math.sin(2 * math.pi * hz * motion_t)
            return [ServoCommand.from_angle(sid, angle) for sid in ids]

        if motion == "ripple":
            hz = 0.2 + speed * 0.4
            return [
                ServoCommand.from_angle(
                    sid,
                    amp * math.sin(2 * math.pi * hz * motion_t + (i / max(n - 1, 1)) * 2 * math.pi),
                )
                for i, sid in enumerate(ids)
            ]

        if motion == "sway":
            hz = 0.08 + speed * 0.15
            angle = amp * math.sin(2 * math.pi * hz * motion_t)
            return [ServoCommand.from_angle(sid, angle) for sid in ids]

        if motion == "curl_right":
            return [
                ServoCommand.from_angle(sid, amp * (i + 1) / n)
                for i, sid in enumerate(ids)
            ]

        if motion == "curl_left":
            return [
                ServoCommand.from_angle(sid, -amp * (i + 1) / n)
                for i, sid in enumerate(ids)
            ]

        if motion == "twitch":
            update_interval = 0.4 - speed * 0.3
            if now >= self._twitch_next_t:
                for sid in ids:
                    self._twitch_targets[sid] = random.uniform(-amp, amp)
                self._twitch_next_t = now + update_interval
            return [
                ServoCommand.from_angle(sid, self._twitch_targets.get(sid, 0.0))
                for sid in ids
            ]

        return [ServoCommand.from_angle(sid, 0.0) for sid in ids]


# ------------------------------------------------------------------
# Prompt loading
# ------------------------------------------------------------------

def _load_system_prompt() -> str:
    """Load and concatenate robot_context.md and behavior_guide.md."""
    parts: list[str] = []
    for name in ("robot_context.md", "behavior_guide.md"):
        path = _PROMPTS_DIR / name
        try:
            parts.append(path.read_text(encoding="utf-8").strip())
        except FileNotFoundError:
            logger.warning("[Ollama] prompt file not found: %s", path)
    return "\n\n---\n\n".join(parts)
