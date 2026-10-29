"""Safety gate between the VR link and the robot. Pure logic, injected clock.

Question answered every control tick: *may the robot move, and which hands
may drive it?* The watchdog can only ever say "stop"/"don't move"; it never
produces motion.

Link faults (all hands disabled)
    NO_MESSAGE  nothing has ever arrived
    STALE       newest message older than ``timeout_s`` (client hung, link
                dropped, Godot crashed). Age is ``now - msg.received_at`` using
                this process's monotonic clock.

Per-hand deadman (grip by default, with hysteresis)
    A hand is *enabled* only when its controller is connected AND it is
    *armed* AND the deadman is held. Armed means "has been seen released since
    the last fault". So after startup, a link fault, or a disconnect, the
    operator must release and re-press before the robot moves again; a link
    that recovers while the grip is still squeezed does NOT resume motion.
    Pressing needs value >= ``press``; releasing needs value < ``release``
    (``release`` < ``press`` prevents flicker around one threshold).

Unreadable values (missing, None, NaN, bool, strings) count as "released".
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum
from typing import Any, Optional

HANDS = ("left", "right")


class Reason(str, Enum):
    OK = "ok"
    NO_MESSAGE = "no_message"
    STALE = "stale"


@dataclass(frozen=True)
class WatchdogConfig:
    timeout_s: float = 0.2
    deadman: str = "grip"            # "grip" or "trigger"
    press: float = 0.6
    release: float = 0.4
    hands: tuple = HANDS

    def __post_init__(self) -> None:
        if self.timeout_s <= 0:
            raise ValueError("timeout_s must be > 0")
        if self.deadman not in ("grip", "trigger"):
            raise ValueError("deadman must be 'grip' or 'trigger'")
        if not 0.0 <= self.release < self.press <= 1.0:
            raise ValueError("need 0 <= release < press <= 1")
        if not self.hands or any(h not in HANDS for h in self.hands):
            raise ValueError(f"hands must be a non-empty subset of {HANDS}")


@dataclass(frozen=True)
class Verdict:
    reason: Reason
    age_s: Optional[float]
    enabled_hands: frozenset

    @property
    def link_ok(self) -> bool:
        return self.reason is Reason.OK

    @property
    def may_move(self) -> bool:
        return bool(self.enabled_hands)


def _number(x: Any) -> float:
    if isinstance(x, bool) or not isinstance(x, (int, float)):
        return 0.0
    x = float(x)
    return x if math.isfinite(x) else 0.0


class Watchdog:
    def __init__(self, config: Optional[WatchdogConfig] = None):
        self._cfg = config or WatchdogConfig()
        self.reset()

    def reset(self) -> None:
        """Back to the safe start state: nothing armed, nothing held."""
        self._armed = {h: False for h in self._cfg.hands}
        self._held = {h: False for h in self._cfg.hands}

    def check(self, msg: Any, now: float) -> Verdict:
        """``msg``: newest InputMessage (or None). ``now``: time.monotonic()."""
        if msg is None:
            self.reset()
            return Verdict(Reason.NO_MESSAGE, None, frozenset())
        age = max(0.0, now - msg.received_at)
        if age > self._cfg.timeout_s:
            self.reset()
            return Verdict(Reason.STALE, age, frozenset())

        enabled = set()
        for hand in self._cfg.hands:
            state = msg.payload.get(hand)
            if not isinstance(state, dict) or state.get("connected") is not True:
                self._armed[hand] = False
                self._held[hand] = False
                continue
            value = _number(state.get(self._cfg.deadman))
            if value < self._cfg.release:
                self._armed[hand] = True
                self._held[hand] = False
            elif value >= self._cfg.press and self._armed[hand]:
                self._held[hand] = True
            # between thresholds: keep the current held state
            if self._armed[hand] and self._held[hand]:
                enabled.add(hand)
        return Verdict(Reason.OK, age, frozenset(enabled))