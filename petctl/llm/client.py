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

_DEFAULT_URL = "http://localhost:11434/api/chat"


class OllamaClient:
    """Blocking client for Ollama's /api/chat endpoint.

    Args:
        model:   Ollama model tag, e.g. "gemma3:4b".
        base_url: Ollama server URL (default: localhost:11434).
        timeout: HTTP timeout in seconds.
    """

    def __init__(
        self,
        model: str = "gemma3:4b",
        base_url: str = "http://localhost:11434",
        timeout: float = 12.0,
    ) -> None:
        self.model = model
        self._url = f"{base_url.rstrip('/')}/api/chat"
        self._timeout = timeout

    def chat(self, system: str, user: str) -> dict[str, Any] | None:
        """Send a system+user message pair and return the parsed JSON response.

        Returns:
            Parsed dict from the LLM, or None if the call fails or the
            response cannot be parsed as JSON.
        """
        payload = json.dumps({
            "model": self.model,
            "stream": False,
            "format": "json",
            "messages": [
                {"role": "system", "content": system},
                {"role": "user", "content": user},
            ],
        }).encode()

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
            return None
        except TimeoutError:
            logger.warning("[Ollama] request timed out after %.1fs", self._timeout)
            return None

        try:
            envelope = json.loads(body)
            content = envelope["message"]["content"]
            return json.loads(content)
        except (KeyError, json.JSONDecodeError, ValueError) as exc:
            logger.warning("[Ollama] could not parse response: %s — raw: %.200s", exc, body)
            return None

    def is_available(self) -> bool:
        """Return True if the Ollama server responds to a health check."""
        try:
            url = self._url.rsplit("/api/", 1)[0]
            urllib.request.urlopen(url, timeout=2.0)
            return True
        except Exception:
            return False
