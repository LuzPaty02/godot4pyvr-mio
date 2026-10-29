from __future__ import annotations

from typing import Any, Protocol


class VisualizerLike(Protocol):
    """Minimal interface expected by the follower package."""

    def render(self, q: Any, output: Any) -> Any:
        ...
