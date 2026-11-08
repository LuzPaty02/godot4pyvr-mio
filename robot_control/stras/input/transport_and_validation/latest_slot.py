"""Thread-safe single-slot "latest value" holder.

Shared by the input server (writer) and the latest-frame consumer.
Knows nothing about networking or controller semantics.
"""
from __future__ import annotations

import threading
from typing import Generic, Optional, TypeVar

T = TypeVar("T")


class Mailbox(Generic[T]):
    """Single slot, latest wins, thread-safe.

    ``get`` does NOT consume: a reader may see the same item on several ticks,
    and a dead producer is detected from the item's age (e.g. ``now -
    received_at``), not from the slot being empty. Not a queue on purpose: a
    queue would let stale items pile up between a fast writer and a slower
    reader.
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._item: Optional[T] = None

    def put(self, item: T) -> None:
        with self._lock:
            self._item = item

    def get(self) -> Optional[T]:
        with self._lock:
            return self._item