"""Sequence classification for latest-value controller datagrams."""
from __future__ import annotations

from typing import Optional

SESSION_RESTART_GAP = 1000
ACCEPT, STALE, RESTART = "accept", "stale", "restart"


class SeqFilter:
    """Drops duplicates/reordered packets and notices client restarts."""

    def __init__(self, restart_gap: int = SESSION_RESTART_GAP):
        self._restart_gap = restart_gap
        self.last: Optional[int] = None

    def classify(self, seq: int) -> tuple[str, int]:
        """Return the verdict and estimated packets lost before this one."""
        if self.last is None:
            self.last = seq
            return ACCEPT, 0
        if seq > self.last:
            lost = seq - self.last - 1
            self.last = seq
            return ACCEPT, lost
        if self.last - seq > self._restart_gap:
            self.last = seq
            return RESTART, 0
        return STALE, 0