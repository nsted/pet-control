"""
Thin HTTP client for the Ollama chat API.

Calls are blocking — intended to be run in a background thread, not the
async control loop.
"""

from __future__ import annotations

import json
import logging
import time
import urllib.error
import urllib.request
from typing import Any

logger = logging.getLogger(__name__)

_MAX_CONV_LINES = 10    # max lines in conversation history (excluding system prompt)
_MAX_AGE_S      = 60.0  # drop turns older than this


class OllamaClient:
    """Blocking client for Ollama's /api/chat endpoint.

    Maintains a persistent conversation: call start() once with the system
    prompt, then chat() for each user turn. History accumulates across calls.

    Args:
        model:    Ollama model tag, e.g. "gemma3:1b".
        base_url: Ollama server URL (default: localhost:11434).
        timeout:  HTTP timeout in seconds.
    """

    def __init__(
        self,
        model: str = "gemma3:4b",
        base_url: str = "http://localhost:11434",
        timeout: float = 12.0,
        log_input: bool = False,
    ) -> None:
        self.model = model
        self._url = f"{base_url.rstrip('/')}/api/chat"
        self._timeout = timeout
        self._log_input = log_input
        self._messages: list[dict[str, str]] = []
        self._turn_times: list[float] = []  # monotonic time when each turn completed
        self._gen: int = 0
        self.last_prompt_tokens: int = 0
        self.last_eval_tokens: int = 0
        self.last_load_ms: int = 0       # model load time (>0 means model was evicted)
        self.last_prefill_ms: int = 0    # prompt evaluation time
        self.last_gen_ms: int = 0        # token generation time

    def start(self, system: str) -> None:
        """Begin a new conversation with the given system prompt."""
        self._gen += 1
        self._messages = [{"role": "system", "content": system}]
        self._turn_times = []

    def _prune(self) -> None:
        """Drop turns that are too old or push conversation past _MAX_CONV_LINES."""
        now = time.monotonic()
        drop = 0
        for i, t in enumerate(self._turn_times):
            if now - t > _MAX_AGE_S:
                drop = i + 1
            else:
                break
        # Count lines in the remaining conversation; drop oldest turns until fits.
        total = sum(
            len(m["content"].splitlines())
            for m in self._messages[1 + drop * 2 :]
        )
        while total > _MAX_CONV_LINES and drop < len(self._turn_times):
            pair = self._messages[1 + drop * 2 : 1 + (drop + 1) * 2]
            total -= sum(len(m["content"].splitlines()) for m in pair)
            drop += 1
        if drop:
            logger.debug("[Ollama] pruning %d turn(s) from history", drop)
            del self._messages[1 : 1 + drop * 2]
            del self._turn_times[:drop]

    def chat(self, user: str) -> dict[str, Any] | None:
        """Append a user turn, call the API, and return the parsed JSON response.

        The assistant reply is appended to history on success. On failure the
        user message is removed so history stays consistent.

        Returns:
            Parsed dict from the LLM, or None if the call fails or the
            response cannot be parsed as JSON.
        """
        self._prune()
        gen = self._gen
        self._messages.append({"role": "user", "content": user})
        payload = json.dumps({
            "model": self.model,
            "stream": False,
            "format": "json",
            "keep_alive": -1,
            "messages": self._messages,
        }).encode()

        prompt_tokens = sum(len(m["content"].split()) for m in self._messages)
        if self._log_input:
            logger.info("[Ollama] sending: %s", payload.decode())
        req = urllib.request.Request(
            self._url,
            data=payload,
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        t0 = time.monotonic()
        try:
            with urllib.request.urlopen(req, timeout=self._timeout) as resp:
                body = resp.read().decode()
        except urllib.error.URLError as exc:
            logger.warning("[Ollama] connection error after %.1fs: %s", time.monotonic() - t0, exc)
            if self._gen == gen:
                self._messages.pop()
            return None
        except TimeoutError:
            logger.warning(
                "[Ollama] request timed out after %.1fs (prompt ~%d words, %d turns)",
                self._timeout, prompt_tokens, (len(self._messages) - 1) // 2,
            )
            if self._gen == gen:
                self._messages.pop()
            return None
        rtt = time.monotonic() - t0

        if self._gen != gen:
            return None

        try:
            envelope = json.loads(body)
            content = envelope["message"]["content"]
            result = json.loads(content)
            self._messages.append({"role": "assistant", "content": content})
            self._turn_times.append(time.monotonic())
            self.last_prompt_tokens = envelope.get("prompt_eval_count", 0) or 0
            self.last_eval_tokens = envelope.get("eval_count", 0) or 0
            self.last_load_ms = int((envelope.get("load_duration") or 0) / 1e6)
            self.last_prefill_ms = int((envelope.get("prompt_eval_duration") or 0) / 1e6)
            self.last_gen_ms = int((envelope.get("eval_duration") or 0) / 1e6)
            if self.last_load_ms > 500:
                logger.warning("[Ollama] model reloaded — load took %dms (was evicted from memory)", self.last_load_ms)
            return result
        except (KeyError, json.JSONDecodeError, ValueError) as exc:
            logger.warning("[Ollama] could not parse response after %.2fs: %s — raw: %.200s", rtt, exc, body)
            self._messages.pop()
            return None

    def clear_history(self) -> None:
        """Reset conversation to just the system prompt, keeping the model loaded."""
        system = self._messages[0] if self._messages else None
        self._messages = [system] if system else []
        self._turn_times = []

    def is_available(self) -> bool:
        """Return True if the Ollama server responds to a health check."""
        try:
            url = self._url.rsplit("/api/", 1)[0]
            urllib.request.urlopen(url, timeout=2.0)
            return True
        except Exception:
            return False
