"""Controller input -> normalized robot command. Pure functions, no I/O.

Output is dimensionless on purpose; the limits layer turns it into units:

    velocity[joint]  in [-1, 1]  fraction of that joint's max speed
    position[joint]  in [ 0, 1]  fraction of that joint's range
                                 (scale to real units with map_norm2range /
                                  the *_cs limits, then clamp and rate-limit)

Joints are addressed by NAME ('BR', 'RR', 'TR', 'BL', 'RL', 'TL', 'EBV',
'EBH', 'ER', 'ET'), never by index, so nothing here depends on vector order.

Safety contract: only hands in ``enabled_hands`` (from Watchdog) produce
motion. Every joint that has a velocity binding is ALWAYS present in
``velocity`` (0.0 when its hand is not enabled), so sending the command
stops anything that was moving.

Input assumption: thumbstick axes arrive SIGNED in [-1, 1] (trigger/grip in
[0, 1]). The current ControllerInputReader.normalize_axis squashes axes to
[0, 1] (rest = 0.5); until you change it, construct the mapper with
``axes_normalized=True`` so rest maps back to 0.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Iterable, Mapping, Optional

HANDS = ("left", "right")
ANALOG_SOURCES = ("trigger", "grip")                       # range [0, 1]
AXIS_SOURCES = ("primary.x", "primary.y", "secondary.x", "secondary.y")  # [-1, 1]
SOURCES = ANALOG_SOURCES + AXIS_SOURCES
MODES = ("velocity", "position")


@dataclass(frozen=True)
class Binding:
    """One input -> one joint.

    velocity mode: source (axis or analog) -> deadzone -> * scale. Several
        bindings on one joint add up; the sum is clamped to [-1, 1].
    position mode: source must be 'trigger' or 'grip'; the 0..1 value is the
        target; negative ``scale`` inverts it (1 - v).
    """
    hand: str
    source: str
    joint: str
    mode: str = "velocity"
    scale: float = 1.0
    deadzone: float = 0.1

    def __post_init__(self) -> None:
        if self.hand not in HANDS:
            raise ValueError(f"hand must be one of {HANDS}, got {self.hand!r}")
        if self.source not in SOURCES:
            raise ValueError(f"unknown source {self.source!r}")
        if self.mode not in MODES:
            raise ValueError(f"mode must be one of {MODES}, got {self.mode!r}")
        if self.mode == "position" and self.source not in ANALOG_SOURCES:
            raise ValueError("position mode needs source 'trigger' or 'grip'")
        if not 0.0 <= self.deadzone < 1.0:
            raise ValueError("deadzone must be in [0, 1)")
        if not self.joint:
            raise ValueError("joint name required")


# Endoscope on the thumbsticks (Quest 3). Stick up / right = positive.
# Spring-return sticks are naturally RATE controls, so these are velocity
# bindings: hold the stick and the joint keeps moving; release and it stops.
# If a direction feels backwards, flip it with scale=-1.0.
#
#   right stick  y -> ET  (insertion, up = positive)
#                x -> ER  (rotation, right = positive)
#   left stick   y -> EBV (vertical bending)
#                x -> EBH (horizontal bending)
#
# NOTE: 'primary' is the thumbstick in Godot's default OpenXR action map; check
# yours if the values stay at zero.
ENDOSCOPE_BINDINGS = (
    Binding("right", "primary.y", "ET"),
    Binding("right", "primary.x", "ER"),
    Binding("left", "primary.y", "EBV"),
    Binding("left", "primary.x", "EBH"),
)
DEFAULT_BINDINGS = ENDOSCOPE_BINDINGS


@dataclass(frozen=True)
class TeleopCommand:
    velocity: Mapping[str, float] = field(default_factory=dict)
    position: Mapping[str, float] = field(default_factory=dict)


def _number(x: Any) -> float:
    if isinstance(x, bool) or not isinstance(x, (int, float)):
        return 0.0
    x = float(x)
    return x if math.isfinite(x) else 0.0


def apply_deadzone(v: float, deadzone: float) -> float:
    """Zero inside the deadzone, rescaled so output still reaches +-1."""
    a = abs(v)
    if a <= deadzone:
        return 0.0
    return math.copysign((min(a, 1.0) - deadzone) / (1.0 - deadzone), v)


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


class TeleopMapper:
    def __init__(
        self,
        bindings: Iterable[Binding] = DEFAULT_BINDINGS,
        axes_normalized: bool = False,
        known_joints: Optional[Iterable[str]] = None,
    ):
        self._bindings = tuple(bindings)
        self._axes_normalized = axes_normalized
        if known_joints is not None:
            known = set(known_joints)
            bad = sorted({b.joint for b in self._bindings} - known)
            if bad:
                raise ValueError(f"bindings use unknown joints: {bad}")
        self._velocity_joints = tuple(dict.fromkeys(
            b.joint for b in self._bindings if b.mode == "velocity"))

    @property
    def driven_joints(self) -> tuple:
        """Joints with a velocity binding (always present in command.velocity)."""
        return self._velocity_joints

    def stop_command(self) -> TeleopCommand:
        return TeleopCommand(velocity={j: 0.0 for j in self._velocity_joints}, position={})

    def _read(self, state: Mapping, source: str) -> float:
        if source in ANALOG_SOURCES:
            return _clamp(_number(state.get(source)), 0.0, 1.0)
        stick, axis = source.split(".")
        pair = state.get(stick)
        v = _number(pair.get(axis)) if isinstance(pair, dict) else 0.0
        if self._axes_normalized:
            v = v * 2.0 - 1.0
        return _clamp(v, -1.0, 1.0)

    def map(self, payload: Mapping, enabled_hands: Iterable[str]) -> TeleopCommand:
        enabled = set(enabled_hands)
        velocity = {j: 0.0 for j in self._velocity_joints}
        position: dict = {}
        for b in self._bindings:
            if b.hand not in enabled:
                continue
            state = payload.get(b.hand)
            if not isinstance(state, dict):
                continue
            raw = self._read(state, b.source)
            if b.mode == "velocity":
                velocity[b.joint] += apply_deadzone(raw, b.deadzone) * b.scale
            else:
                position[b.joint] = _clamp(raw if b.scale >= 0 else 1.0 - raw, 0.0, 1.0)
        velocity = {j: _clamp(v, -1.0, 1.0) for j, v in velocity.items()}
        return TeleopCommand(velocity=velocity, position=position)