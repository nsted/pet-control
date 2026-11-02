"""
Message-bus abstraction for the driver ↔ brain split.

``Bus`` moves opaque ``bytes`` (see ``petctl.wire`` for what they contain):

  publish(key, data)           fire-and-forget
  subscribe(key, callback)     callback(data) runs on a bus thread — hand off to
                               asyncio with ``loop.call_soon_threadsafe``
  serve(key, handler)          request/reply; handler(data) -> bytes, runs on a bus thread
  request(key, data, timeout)  blocking request/reply (call from an executor)

Two implementations:

  ZenohBus      real transport. Peer-to-peer over loopback TCP; multicast and
                gossip scouting are disabled so nothing leaves this machine
                unless an endpoint says so. ``zenoh`` is imported lazily, so
                petctl works without the optional ``[split]`` extra.
  LoopbackBus   in-memory, for tests. Callbacks run synchronously.

This is the only module that imports ``zenoh``.
"""

from __future__ import annotations

import json
import logging
import threading
from abc import ABC, abstractmethod
from typing import Callable, Optional

logger = logging.getLogger(__name__)

DEFAULT_ENDPOINT = "tcp/127.0.0.1:7447"

Callback = Callable[[bytes], None]
Handler = Callable[[bytes], bytes]


class Bus(ABC):
    """Minimal pub/sub + request/reply transport."""

    @abstractmethod
    def publish(self, key: str, data: bytes) -> None: ...

    @abstractmethod
    def subscribe(self, key: str, callback: Callback) -> None: ...

    @abstractmethod
    def serve(self, key: str, handler: Handler) -> None: ...

    @abstractmethod
    def request(self, key: str, data: bytes, timeout: float = 2.0) -> bytes:
        """Blocking request. Raises TimeoutError if no reply, RuntimeError on a transport error."""

    @abstractmethod
    def close(self) -> None: ...


# ---------------------------------------------------------------------------
# Zenoh
# ---------------------------------------------------------------------------

class ZenohBus(Bus):
    """Zenoh peer session. The driver *listens* on an endpoint; the brain *connects* to it."""

    def __init__(
        self,
        *,
        listen: Optional[str] = None,
        connect: Optional[str] = None,
    ) -> None:
        try:
            import zenoh
        except ImportError as e:  # pragma: no cover - depends on environment
            raise RuntimeError(
                "The driver/brain split needs eclipse-zenoh. Install with: pip install -e '.[split]'"
            ) from e
        if (listen is None) == (connect is None):
            raise ValueError("ZenohBus needs exactly one of listen= or connect=")
        conf: dict = {
            "mode": "peer",
            "scouting": {"multicast": {"enabled": False}, "gossip": {"enabled": False}},
        }
        if listen is not None:
            conf["listen"] = {"endpoints": [listen]}
        else:
            conf["connect"] = {"endpoints": [connect]}
        self._zenoh = zenoh
        self._session = zenoh.open(zenoh.Config.from_json5(json.dumps(conf)))
        self._publishers: dict[str, object] = {}
        self._keepalive: list[object] = []   # subscribers / queryables must stay referenced
        self._lock = threading.Lock()

    def publish(self, key: str, data: bytes) -> None:
        with self._lock:
            pub = self._publishers.get(key)
            if pub is None:
                pub = self._session.declare_publisher(key)
                self._publishers[key] = pub
        pub.put(data)  # type: ignore[attr-defined]

    def subscribe(self, key: str, callback: Callback) -> None:
        def _on_sample(sample) -> None:  # zenoh thread
            try:
                callback(sample.payload.to_bytes())
            except Exception:
                logger.exception("[Bus] subscriber callback for %s failed", key)

        self._keepalive.append(self._session.declare_subscriber(key, _on_sample))

    def serve(self, key: str, handler: Handler) -> None:
        def _on_query(query) -> None:  # zenoh thread
            try:
                payload = query.payload.to_bytes() if query.payload is not None else b""
                query.reply(query.key_expr, handler(payload))
            except Exception as e:
                logger.exception("[Bus] service handler for %s failed", key)
                query.reply_err(str(e).encode("utf-8"))

        self._keepalive.append(self._session.declare_queryable(key, _on_query))

    def request(self, key: str, data: bytes, timeout: float = 2.0) -> bytes:
        for reply in self._session.get(key, payload=data, timeout=timeout):
            if reply.ok is not None:
                return reply.ok.payload.to_bytes()
            raise RuntimeError(reply.err.payload.to_string())
        raise TimeoutError(f"no reply from {key} within {timeout}s")

    def close(self) -> None:
        self._keepalive.clear()
        self._publishers.clear()
        self._session.close()


# ---------------------------------------------------------------------------
# In-memory (tests)
# ---------------------------------------------------------------------------

class LoopbackBus(Bus):
    """Single shared in-memory bus: both ends hold the same instance."""

    def __init__(self) -> None:
        self._subs: dict[str, list[Callback]] = {}
        self._services: dict[str, Handler] = {}
        self.published: list[tuple[str, bytes]] = []

    def publish(self, key: str, data: bytes) -> None:
        self.published.append((key, data))
        for cb in list(self._subs.get(key, [])):
            cb(data)

    def subscribe(self, key: str, callback: Callback) -> None:
        self._subs.setdefault(key, []).append(callback)

    def serve(self, key: str, handler: Handler) -> None:
        self._services[key] = handler

    def request(self, key: str, data: bytes, timeout: float = 2.0) -> bytes:
        handler = self._services.get(key)
        if handler is None:
            raise TimeoutError(f"no service at {key}")
        return handler(data)

    def close(self) -> None:
        self._subs.clear()
        self._services.clear()
