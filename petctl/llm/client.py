"""
Thin HTTP client for the Ollama chat API.

Calls are blocking — intended to be run in a background thread, not the
async control loop.
"""

from __future__ import annotations

import json
import logging
import urllib.error
import urllib.request
from typing import Any

logger = logging.getLogger(__name__)


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
        model: str = "gemma3:1b",
        base_url: str = "http://localhost:11434",
        timeout: float = 12.0,
        log_input: bool = False,
    ) -> None:
        self.model = model
        self._url = f"{base_url.rstrip('/')}/api/chat"
        self._timeout = timeout
        self._log_input = log_input
        self._messages: list[dict[str, str]] = []
        self._gen: int = 0

    def start(self, system: str) -> None:
        """Begin a new conversation with the given system prompt."""
        self._gen += 1
        self._messages = [{"role": "system", "content": system}]

    def chat(self, user: str) -> dict[str, Any] | None:
        """Append a user turn, call the API, and return the parsed JSON response.

        The assistant reply is appended to history on success. On failure the
        user message is removed so history stays consistent.

        Returns:
            Parsed dict from the LLM, or None if the call fails or the
            response cannot be parsed as JSON.
        """
        gen = self._gen
        self._messages.append({"role": "user", "content": user})
        payload = json.dumps({
            "model": self.model,
            "stream": False,
            "format": "json",
            "messages": self._messages,
        }).encode()

        if self._log_input:
            logger.info("[Ollama] sending: %s", payload.decode())
        req = urllib.request.Request(
            self._url,
            data=payload,
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        try:
            with urllib.request.urlopen(req, timeout=self._timeout) as resp:
                body = resp.read().decode()
        except urllib.error.URLError as exc:
            logger.warning("[Ollama] connection error: %s", exc)
            if self._gen == gen:
                self._messages.pop()
            return None
        except TimeoutError:
            logger.warning("[Ollama] request timed out after %.1fs", self._timeout)
            if self._gen == gen:
                self._messages.pop()
            return None

        if self._gen != gen:
            return None

        try:
            envelope = json.loads(body)
            content = envelope["message"]["content"]
            result = json.loads(content)
            self._messages.append({"role": "assistant", "content": content})
            return result
        except (KeyError, json.JSONDecodeError, ValueError) as exc:
            logger.warning("[Ollama] could not parse response: %s — raw: %.200s", exc, body)
            self._messages.pop()
            return None

    def is_available(self) -> bool:
        """Return True if the Ollama server responds to a health check."""
        try:
            url = self._url.rsplit("/api/", 1)[0]
            urllib.request.urlopen(url, timeout=2.0)
            return True
        except Exception:
            return False
