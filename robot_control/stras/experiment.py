from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class ExperimentState:
    """Mutable experiment flags used by the follower implementation.

    These flags are intentionally lightweight: the follower API reads and writes
    them as simple attributes, so a dataclass with default values is enough for
    import compatibility and tests.
    """

    flag_virtual_limits: bool = False
    limit_flag_virtual: bool = False
    timeout_error: bool = False
    send_error: bool = False
    last_error: str | None = None
    extra: dict = field(default_factory=dict)
