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
patterns (patterns.py).  Speed from the LLM response scales frequency;
amplitude always runs at the full _AMP_MAX value for each motion.

Behaviour is configured by petctl/prompts/robot_context.md — edit the
Character, Rules, and Principles sections freely; changes take effect on restart.
"""

from __future__ import annotations

import asyncio
import logging
import math
import threading
import time
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
    IdleControlScheme,
    PoseScheme,
    PulseControlScheme,
    SnuggleControlScheme,
    SlalomControlScheme,
    StrokeCurlScheme,
    StrokeReactControlScheme,
    StrokeSnuggleScheme,
    SwayControlScheme,
    TwitchControlScheme,
    ExploreControlScheme,
    YieldStiffScheme,
)
from petctl.types import RobotState, ServoCommand, TouchSummary

if TYPE_CHECKING:
    from petctl.controller import Controller

logger = logging.getLogger(__name__)

_PROMPTS_DIR = Path(__file__).parent.parent / "prompts"

# Accumulate touch events for this many seconds before sending a batch to the LLM.
_BATCH_WINDOW_S = 5.0

# Max amplitude per movement (degrees). Always used at full value — no intensity scaling.
# Patterns with no amplitude param use 0.0.
_AMP_MAX: dict[str, float] = {
    "freeze":        0.0,
    "idle":          0.0,
    "home":          0.0,
    "breathe":      15.0,
    "pulse":        60.0,
    "snuggle":       55.0,
    "sway":         70.0,
    "cascade":      70.0,
    "slalom":       60.0,
    "twitch":       40.0,
    "coil":         65.0,
    "curl_right":   80.0,
    "curl_left":    80.0,
    "explore":        0.0,
    "drift":         0.0,
    "stroke":        0.0,  # touch-reactive — senses internally
    "stroke-curl":   0.0,
    "stroke-snuggle": 0.0,
    "yield-stiff":   0.0,
    "pose":          0.0,
}

_VALID_MOVEMENTS = set(_AMP_MAX)

# Movement to use on startup and after idle revert (no touch for _TOUCH_IDLE_S).
_DEFAULT_MOTION = "stroke-curl"


class _CurlLeft(CurlControlScheme):
    """CurlControlScheme with all joint signs negated → curl to the left."""

    name = "curl_left"

    def update(self, state: RobotState) -> list[ServoCommand]:
        cmds = super().update(state)
        for cmd in cmds:
            if cmd.position is not None:
                cmd.position = -cmd.position
        return cmds


def _make_pattern(motion: str, speed: float) -> ControlScheme:
    """Instantiate a ControlScheme for the given motion, scaled by speed."""
    amp = _AMP_MAX.get(motion, 0.0)

    if motion in ("freeze", "home"):
        return FreezeControlScheme()
    if motion == "idle":
        return IdleControlScheme()
    if motion == "breathe":
        return BreatheControlScheme(amplitude_deg=amp, hz=0.05 + speed * 0.06)
    if motion == "pulse":
        return PulseControlScheme(amplitude_deg=amp, hz=0.15 + speed * 0.35)
    if motion == "snuggle":
        return SnuggleControlScheme(amplitude_deg=amp, hz=0.2 + speed * 0.4)
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
    if motion == "explore":
        return ExploreControlScheme(speed_deg_per_s=20.0 + speed * 60.0)
    if motion == "drift":
        return DriftControlScheme()
    if motion == "stroke":
        return StrokeReactControlScheme()
    if motion == "stroke-curl":
        return StrokeCurlScheme()
    if motion == "stroke-snuggle":
        return StrokeSnuggleScheme()
    if motion == "yield-stiff":
        return YieldStiffScheme()
    if motion == "pose":
        return PoseScheme()
    return FreezeControlScheme()


def _update_pattern_params(pattern: ControlScheme, motion: str, speed: float) -> None:
    """Update a running pattern's amplitude/speed without resetting its phase."""
    amp = _AMP_MAX.get(motion, 0.0)

    if motion == "breathe":
        pattern.amplitude_deg = amp  # type: ignore[attr-defined]
        pattern.hz = 0.05 + speed * 0.06  # type: ignore[attr-defined]
    elif motion == "pulse":
        pattern.amplitude_deg = amp  # type: ignore[attr-defined]
        pattern.hz = 0.15 + speed * 0.35  # type: ignore[attr-defined]
    elif motion == "snuggle":
        pattern.amplitude_deg = amp  # type: ignore[attr-defined]
        pattern.hz = 0.2 + speed * 0.4  # type: ignore[attr-defined]
    elif motion == "sway":
        pattern.amplitude_deg = amp  # type: ignore[attr-defined]
        pattern.hz = 0.08 + speed * 0.15  # type: ignore[attr-defined]
    elif motion == "cascade":
        pattern.amplitude_deg = amp  # type: ignore[attr-defined]
        pattern.hz = 0.15 + speed * 0.25  # type: ignore[attr-defined]
    elif motion == "slalom":
        pattern.amplitude_deg = amp  # type: ignore[attr-defined]
        pattern.hz = 0.1 + speed * 0.3  # type: ignore[attr-defined]
    elif motion == "twitch":
        pattern.amplitude_deg = amp  # type: ignore[attr-defined]
        pattern.smoothing = 0.03 + speed * 0.08  # type: ignore[attr-defined]
    elif motion == "coil":
        pattern.amplitude_deg = amp  # type: ignore[attr-defined]
        pattern.hz = 0.1 + speed * 0.2  # type: ignore[attr-defined]
    elif motion in ("curl_right", "curl_left"):
        pattern.target_deg = amp  # type: ignore[attr-defined]
    elif motion == "explore":
        pattern._speed_rad_s = math.radians(20.0 + speed * 60.0)  # type: ignore[attr-defined]
    # freeze, home, drift, stroke, stroke-curl, stroke-ripple, yield-stiff, pose:
    # no tunable params — nothing to update


def _format_batch(batch: list[TouchSummary]) -> str:
    t0 = batch[0].timestamp
    duration = batch[-1].timestamp - t0
    lines = [f"Touch sequence over {duration:.1f}s:"]
    for s in batch:
        lines.append(f"+{s.timestamp - t0:.1f}s: {s.describe()}")
    return "\n".join(lines)


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
        model: str = "gemma3:4b",
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

        self._batch: list[TouchSummary] = []
        self._batch_start_t: float = 0.0
        self._pending: threading.Thread | None = None
        self._system_prompt: str = ""
        self._active_motion: str = ""
        self._was_connected: bool = False
        self._touch_ended_t: float | None = None  # wall time of last "none" event

    # ------------------------------------------------------------------
    # ControlScheme interface
    # ------------------------------------------------------------------

    def on_start(self, controller: Controller) -> None:
        self._controller = controller
        self._system_prompt = _load_system_prompt()
        self._touch_queue = controller.touch_events
        self._switch_pattern(_DEFAULT_MOTION, 0.0)

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
        if self._was_connected and not state.connected:
            logger.info("[System] WebSocket disconnected — resetting conversation history and pattern.")
            self._client.start(self._system_prompt)
            self._batch = []
            self._touch_ended_t = None
            self._switch_pattern(_DEFAULT_MOTION, 0.0)
        elif not self._was_connected and state.connected:
            logger.info("[System] WebSocket reconnected — reverting to %s.", _DEFAULT_MOTION)
            self._switch_pattern(_DEFAULT_MOTION, 0.0)
        self._was_connected = state.connected

        if self._touch_queue is not None:
            self._drain_touch_queue()

        ended_t = self._touch_ended_t
        if (
            ended_t is not None
            and state.timestamp - ended_t >= 5.0
            and self._active_motion != _DEFAULT_MOTION
        ):
            logger.info("[System] no touch for 5s — reverting to %s.", _DEFAULT_MOTION)
            self._touch_ended_t = None
            self._switch_pattern(_DEFAULT_MOTION, 0.0)

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
        last_ts: float = 0.0
        while True:
            try:
                summary: TouchSummary = self._touch_queue.get_nowait()
            except asyncio.QueueEmpty:
                break

            now = summary.timestamp
            last_ts = now

            # "none" events (touch ended) — record the time so update() can
            # revert to idle after 5 s of inactivity.
            if summary.touch_type == "none":
                self._touch_ended_t = now
                continue

            # Clear any pending idle-revert on new touch.
            self._touch_ended_t = None

            if not self._batch:
                self._batch_start_t = now
            self._batch.append(summary)

        if not self._batch or last_ts == 0.0:
            return

        elapsed = last_ts - self._batch_start_t
        thread_free = self._pending is None or not self._pending.is_alive()
        if elapsed < _BATCH_WINDOW_S or not thread_free:
            return

        batch, self._batch = self._batch, []
        t = threading.Thread(target=self._llm_call, args=(_format_batch(batch),), daemon=True)
        self._pending = t
        t.start()

    # ------------------------------------------------------------------
    # Pattern switching
    # ------------------------------------------------------------------

    def _switch_pattern(self, motion: str, speed: float) -> None:
        """Instantiate and activate a new pattern. Safe to call from any thread."""
        pattern = _make_pattern(motion, speed)
        controller = self._controller
        if controller is not None:
            pattern.on_start(controller)
        with self._lock:
            self._active_pattern = pattern
            self._active_motion = motion

    # ------------------------------------------------------------------
    # LLM interaction (background thread)
    # ------------------------------------------------------------------

    def _llm_call(self, touch_description: str) -> None:
        logger.debug("[Ollama] calling LLM: %s", touch_description)
        t0 = time.monotonic()
        result = self._client.chat(touch_description)
        rtt = time.monotonic() - t0
        if result is None:
            return
        self._apply_llm_response(result, rtt)

    def _apply_llm_response(self, response: dict, rtt: float = 0.0) -> None:
        motion = str(response.get("movement", "")).strip().lower()
        if motion not in _VALID_MOVEMENTS:
            logger.warning(
                "[Ollama] unknown movement %r — ignoring. Valid: %s",
                motion,
                sorted(_VALID_MOVEMENTS),
            )
            return

        speed = max(0.05, min(1.0, float(response.get("speed", 0.4))))
        explanation = str(response.get("explanation", "")).strip()

        with self._lock:
            same_motion = self._active_motion == motion
            pattern = self._active_pattern

        pt = self._client.last_prompt_tokens
        et = self._client.last_eval_tokens
        ld = self._client.last_load_ms
        pf = self._client.last_prefill_ms
        gn = self._client.last_gen_ms
        if same_motion:
            logger.info(
                "[Ollama] → %s (speed=%.2f) — %s [params updated]\n  rtt=%.2fs  p=%d e=%d  ld=%d pf=%d gn=%dms",
                motion, speed, explanation, rtt, pt, et, ld, pf, gn,
            )
            _update_pattern_params(pattern, motion, speed)
        else:
            logger.info(
                "[Ollama] → %s (speed=%.2f) — %s\n  rtt=%.2fs  p=%d e=%d  ld=%d pf=%d gn=%dms",
                motion, speed, explanation, rtt, pt, et, ld, pf, gn,
            )
            self._switch_pattern(motion, speed)


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
