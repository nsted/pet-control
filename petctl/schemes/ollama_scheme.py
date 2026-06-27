"""
OllamaMotion — touch-reactive motion driven by a local LLM.

Reads gesture events from the controller's `touch_events` queue
(AsyncIO, emits on type transitions only) and asks a gemma3 model via Ollama
which movement to perform.  LLM calls happen in a background thread so the
30 Hz control loop stays non-blocking.  Between calls the last commanded
motion continues uninterrupted.

Each gesture batch is a new conversation turn. After each LLM reply the
history is trimmed to the most recent `history_turns` exchange pairs (default 4)
so the model has recent context without unbounded token growth.

Motion is delegated to the same Motion classes used by standalone
patterns (patterns.py).  Speed from the LLM response scales frequency;
amplitude always runs at the full _AMP_MAX value for each motion.

Behaviour is configured by petctl/prompts/robot_context.md — edit the
Character, Rules, and Principles sections freely; changes take effect on restart.
"""

from __future__ import annotations

import asyncio
import logging
import threading
import time
from pathlib import Path
from typing import TYPE_CHECKING

from petctl.llm.client import OllamaClient
from petctl.protocols import Motion
from petctl.schemes.patterns import (
    AvoidTouchMotion,
    CurlAwayMotion,
    CurlMotion,
    CurlTowardsNeighborAssistMotion,
    DriftMotion,
    ExploreMotion,
    FreezeMotion,
    IdleMotion,
    NeighborAssistDriftMotion,
    PurrRippleMotion,
    SeekTouchMotion,
    SlalomMotion,
    SnuggleMotion,
    StrokeSnuggleMotion,
    StruggleMotion,
    TwitchMotion,
    YieldStiffMotion,
)
from petctl.types import GestureEvent, RobotState, ServoCommand

if TYPE_CHECKING:
    from petctl.controller import Controller

logger = logging.getLogger(__name__)

_PROMPTS_DIR = Path(__file__).parent.parent / "prompts"

# Accumulate touch events for this many seconds before sending a batch to the LLM.
_BATCH_WINDOW_S = 5.0

_VALID_MOVEMENTS: set[str] = {
    "idle", "snuggle", "walk", "nuzzle", "wiggle", "purr",
    "explore", "contort", "twitch", "struggle", "writhe",
    "engage", "withdraw", "seek-touch", "avoid-touch", "yield", "curl",
}

# Movement to use on startup and after idle revert (no touch for _TOUCH_IDLE_S).
_DEFAULT_MOTION = "engage"


def _make_pattern(motion: str) -> Motion:
    """Instantiate a Motion for the given motion name using class defaults."""
    if motion == "idle":
        return IdleMotion()
    if motion in ("snuggle", "walk"):
        return SnuggleMotion()
    if motion == "nuzzle":
        return StrokeSnuggleMotion()
    if motion == "wiggle":
        return SlalomMotion()
    if motion == "purr":
        return PurrRippleMotion()
    if motion == "explore":
        return ExploreMotion()
    if motion == "contort":
        return DriftMotion()
    if motion == "twitch":
        return TwitchMotion()
    if motion == "struggle":
        return StruggleMotion()
    if motion == "writhe":
        return NeighborAssistDriftMotion()
    if motion == "engage":
        return CurlTowardsNeighborAssistMotion()
    if motion == "withdraw":
        return CurlAwayMotion()
    if motion == "seek-touch":
        return SeekTouchMotion()
    if motion == "avoid-touch":
        return AvoidTouchMotion()
    if motion == "yield":
        return YieldStiffMotion()
    if motion == "curl":
        return CurlMotion()
    return FreezeMotion()


def _vitals_phrase(state: RobotState) -> str | None:
    """Natural-language description of thermal and power state, or None if no data."""
    temps = list(state.motor_temperatures.values()) + list(state.motor_winding_temperatures.values())
    if not temps:
        return None

    peak_temp = max(temps)
    if peak_temp >= 65:
        temp_adj = "very hot"
    elif peak_temp >= 55:
        temp_adj = "hot"
    elif peak_temp >= 40:
        temp_adj = "warm"
    else:
        temp_adj = "cool"

    if state.battery_voltage_raw == 0:
        return f"Running {temp_adj}"

    voltage = state.battery_voltage_v
    if voltage >= 12.0:
        energy_adj = "full of energy"
    elif voltage >= 11.0:
        energy_adj = "getting tired"
    else:
        energy_adj = "low on energy"

    return f"Running {temp_adj} and {energy_adj}"


def _format_batch(batch: list[GestureEvent], vitals: str | None = None) -> str:
    lines = []
    if vitals:
        lines.append(vitals)
    for i, event in enumerate(batch):
        phrase = event.describe()
        if i == 0:
            lines.append(phrase)
        else:
            delta = event.timestamp - batch[i - 1].timestamp
            lines.append(f"{delta:.1f} seconds later — {phrase}")
    return "\n".join(lines)


class OllamaMotion(Motion):
    """Motion source that uses a local Ollama LLM to map touch→movement.

    The controller populates state.gesture each tick and emits GestureEvent
    events on the touch_events queue on type transitions.  This motion drains
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
        llm_enabled: bool = True,
        monitor_only: bool = False,
        history_turns: int = 4,
        default_speed: float = 1.0,
    ) -> None:
        self._llm_enabled = llm_enabled
        self._monitor_only = monitor_only
        self._history_turns = history_turns
        self._default_speed = max(0.05, min(1.0, default_speed))
        self._client = OllamaClient(model=model, base_url=base_url, timeout=timeout, log_input=log_input)

        # Injected in on_start() — the controller's shared gesture event queue.
        self._touch_queue: asyncio.Queue[GestureEvent] | None = None

        # Active delegated pattern — written by background thread (LLM response)
        # and by update() (touch-end home), read by update(). Protected by _lock.
        self._lock = threading.Lock()
        self._active_pattern: Motion = FreezeMotion()

        # Stored so background thread can call on_start() on new patterns.
        self._controller: Controller | None = None

        self._batch: list[GestureEvent] = []
        self._batch_start_t: float = 0.0
        self._pending: threading.Thread | None = None
        self._system_prompt: str = ""
        self._active_motion: str = ""
        self._was_connected: bool = False
        self._touch_ended_t: float | None = None  # wall time of last "none" event
        self._revert_gen: int = 0  # incremented on every forced revert; used to discard stale LLM responses

    # ------------------------------------------------------------------
    # Motion interface
    # ------------------------------------------------------------------

    def is_active(self) -> bool:
        with self._lock:
            return self._active_pattern.is_active()

    def on_start(self, controller: Controller) -> None:
        self._controller = controller
        self._controller.speed_gain = self._default_speed
        self._system_prompt = _load_system_prompt()
        self._touch_queue = controller.touch_events
        self._switch_pattern(_DEFAULT_MOTION)

        if not self._llm_enabled:
            logger.info("[Ollama] LLM disabled (dev-ui mode).")
        elif not self._client.is_available():
            logger.warning(
                "[Ollama] server not reachable at %s — "
                "start Ollama with 'ollama serve' then restart PET.",
                self._client._url,
            )
        else:
            self._client.start(self._system_prompt)
            if self._monitor_only:
                logger.info(
                    "[Ollama] connected, model=%s — monitor mode (responses logged, not applied).",
                    self._client.model,
                )
            else:
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
            with self._lock:
                self._revert_gen += 1
            self._controller.speed_gain = self._default_speed
            self._switch_pattern(_DEFAULT_MOTION)
        elif not self._was_connected and state.connected:
            logger.info("[System] WebSocket reconnected — reverting to %s.", _DEFAULT_MOTION)
            self._batch = []
            with self._lock:
                self._revert_gen += 1
            self._controller.speed_gain = self._default_speed
            self._switch_pattern(_DEFAULT_MOTION)
        self._was_connected = state.connected

        if self._touch_queue is not None:
            self._drain_touch_queue(state)

        ended_t = self._touch_ended_t
        if (
            ended_t is not None
            and state.timestamp - ended_t >= 5.0
            and self._active_motion != _DEFAULT_MOTION
        ):
            logger.info("[System] no touch for 5s — reverting to %s.", _DEFAULT_MOTION)
            self._touch_ended_t = None
            self._batch = []
            with self._lock:
                self._revert_gen += 1
            self._controller.speed_gain = self._default_speed
            self._switch_pattern(_DEFAULT_MOTION)

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

    def _drain_touch_queue(self, state: RobotState) -> None:
        assert self._touch_queue is not None
        now = state.timestamp
        while True:
            try:
                summary: GestureEvent = self._touch_queue.get_nowait()
            except asyncio.QueueEmpty:
                break

            ts = summary.timestamp

            # "none" events (touch ended) — record the time so update() can
            # revert to idle after 5 s of inactivity.
            if summary.touch_type == "none":
                self._touch_ended_t = ts
                continue

            # Clear any pending idle-revert on new touch.
            self._touch_ended_t = None

            if summary.status != "complete":
                continue

            if not self._batch:
                self._batch_start_t = ts
            self._batch.append(summary)

        if not self._batch:
            return

        if not self._llm_enabled:
            return

        elapsed = now - self._batch_start_t
        thread_free = self._pending is None or not self._pending.is_alive()
        if elapsed < _BATCH_WINDOW_S or not thread_free:
            return

        batch, self._batch = self._batch, []
        with self._lock:
            gen = self._revert_gen
        vitals = _vitals_phrase(state)
        t = threading.Thread(target=self._llm_call, args=(_format_batch(batch, vitals), gen), daemon=True)
        self._pending = t
        t.start()

    # ------------------------------------------------------------------
    # Pattern switching
    # ------------------------------------------------------------------

    def _switch_pattern(self, motion: str) -> None:
        """Instantiate and activate a new pattern. Safe to call from any thread."""
        pattern = _make_pattern(motion)
        controller = self._controller
        if controller is not None:
            pattern.on_start(controller)
        with self._lock:
            self._active_pattern = pattern
            self._active_motion = motion

    # ------------------------------------------------------------------
    # LLM interaction (background thread)
    # ------------------------------------------------------------------

    def _llm_call(self, touch_description: str, gen: int) -> None:
        logger.info("[Ollama] sending prompt.")
        logger.debug("[Ollama] calling LLM: %s", touch_description)
        t0 = time.monotonic()
        result = self._client.chat(touch_description)
        rtt = time.monotonic() - t0
        try:
            if result is not None:
                self._apply_llm_response(result, rtt, gen)
        except Exception as exc:
            logger.warning("[Ollama] could not apply response: %s — raw: %s", exc, result)
        finally:
            self._client.trim_history(self._history_turns)

    def _apply_llm_response(self, response: dict, rtt: float = 0.0, gen: int = 0) -> None:
        with self._lock:
            if gen != self._revert_gen:
                return
        motion = str(response.get("movement", "")).strip().lower()
        if motion not in _VALID_MOVEMENTS:
            logger.warning(
                "[Ollama] unknown movement %r — ignoring. Valid: %s",
                motion,
                sorted(_VALID_MOVEMENTS),
            )
            return

        try:
            intensity = max(0.05, min(1.0, float(response.get("intensity", 1.0))))
        except (TypeError, ValueError):
            logger.warning("[Ollama] invalid intensity field %r — using default 1.0", response.get("intensity"))
            intensity = 1.0
        feel = str(response.get("explanation", response.get("feel", ""))).strip()

        pt = self._client.last_prompt_tokens
        et = self._client.last_eval_tokens
        ld = self._client.last_load_ms
        pf = self._client.last_prefill_ms
        gn = self._client.last_gen_ms
        _mv = f"*** {motion} (intensity={intensity:.2f}) ***"
        if self._monitor_only:
            logger.info("\n[Ollama] rtt=%.2fs → %s — %s [monitor]\n", rtt, _mv, feel)
            logger.debug("[Ollama] p=%d e=%d  ld=%d pf=%d gn=%dms", pt, et, ld, pf, gn)
            return

        with self._lock:
            same_motion = self._active_motion == motion
        if self._controller is not None:
            self._controller.speed_gain = intensity
        if same_motion:
            logger.info("\n[Ollama] rtt=%.2fs → %s — %s [speed updated]\n", rtt, _mv, feel)
            logger.debug("[Ollama] p=%d e=%d  ld=%d pf=%d gn=%dms", pt, et, ld, pf, gn)
        else:
            logger.info("\n[Ollama] rtt=%.2fs → %s — %s\n", rtt, _mv, feel)
            logger.debug("[Ollama] p=%d e=%d  ld=%d pf=%d gn=%dms", pt, et, ld, pf, gn)
            self._switch_pattern(motion)


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
