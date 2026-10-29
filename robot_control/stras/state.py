"""Robot state: immutable snapshots, axis ranges and the byte parsers.

``parse_state`` / ``parse_axis_infos`` return values instead of appending to
lists on ``self``, which removes the old "keep only the last 12 entries"
trimming and the ever-growing ``axisMins`` / ``axisMaxs``.
"""
from __future__ import annotations

import struct
from dataclasses import dataclass, field
from typing import Sequence, Tuple

import numpy as np

from .constants import INVERTED_AXES, SLAVE_AXIS_LABELS
from .errors import ProtocolError, StrasError

STATE_HEADER_END = 41      # first axis record starts here
AXIS_RECORD_SIZE = 27      # 3 flags + measured + reference + torque (3 doubles)
AXIS_INFO_START = 26
AXIS_INFO_RECORD_SIZE = 17  # active flag + min + max


@dataclass(frozen=True)
class AxisState:
    enabled: bool
    calibrated: bool
    used: bool
    measured: float
    reference: float
    torque: float


@dataclass(frozen=True)
class RobotState:
    tick: int
    controller_mode: int
    controller_error: int
    trace_on: int
    calibrated: int
    security_on: int
    power_on: int
    dc_on: int
    number_axis: int
    active_axis: int
    axes: Tuple[AxisState, ...]
    buttons: Tuple[int, ...] = ()
    digital_io: Tuple[int, ...] = ()

    @property
    def measured(self) -> np.ndarray:
        return np.array([a.measured for a in self.axes], dtype=np.float64)

    @property
    def reference(self) -> np.ndarray:
        return np.array([a.reference for a in self.axes], dtype=np.float64)


@dataclass
class AxisRanges:
    """Per-axis [min, max] used to normalise measured values to [0, 1]."""
    mins: np.ndarray = field(default_factory=lambda: np.array([], dtype=np.float64))
    maxs: np.ndarray = field(default_factory=lambda: np.array([], dtype=np.float64))

    @classmethod
    def empty(cls) -> "AxisRanges":
        return cls()

    def __len__(self) -> int:
        return len(self.mins)

    def normalize(self, measured: Sequence[float],
                  labels: Sequence[str] = SLAVE_AXIS_LABELS) -> np.ndarray:
        if len(self) < len(measured):
            raise StrasError(
                "axis ranges not loaded; call get_axis_info() first")
        out = np.empty(len(measured), dtype=np.float64)
        for i, value in enumerate(measured):
            inverted = i < len(labels) and labels[i] in INVERTED_AXES
            out[i] = np.interp(value, (self.mins[i], self.maxs[i]),
                               (1, 0) if inverted else (0, 1))
        return out

    def expand(self, measured: Sequence[float], upto: int = 7) -> None:
        """Widen the ranges so that ``measured`` fits (first ``upto`` axes)."""
        for i in range(min(upto, len(measured), len(self))):
            if measured[i] > self.maxs[i]:
                self.maxs[i] = measured[i]
            if measured[i] < self.mins[i]:
                self.mins[i] = measured[i]


@dataclass(frozen=True)
class AxisInfos:
    number_axis: int
    active_axis: int
    active: Tuple[bool, ...]
    ranges: AxisRanges


def _need(raw: bytes, size: int, what: str) -> None:
    if len(raw) < size:
        raise ProtocolError(f"{what}: need {size} bytes, got {len(raw)}")


def parse_axis_infos(raw: bytes) -> AxisInfos:
    _need(raw, AXIS_INFO_START, "axis infos header")
    number_axis = raw[24]
    active_axis = raw[25]
    _need(raw, AXIS_INFO_START + number_axis * AXIS_INFO_RECORD_SIZE,
          "axis infos body")
    active, mins, maxs = [], [], []
    for i in range(number_axis):
        o = AXIS_INFO_START + i * AXIS_INFO_RECORD_SIZE
        active.append(raw[o] > 0)
        lo, hi = struct.unpack_from("<dd", raw, o + 1)
        mins.append(lo)
        maxs.append(hi)
    return AxisInfos(number_axis, active_axis, tuple(active),
                     AxisRanges(np.array(mins), np.array(maxs)))


def parse_state(raw: bytes) -> RobotState:
    _need(raw, STATE_HEADER_END, "state header")
    tick = int.from_bytes(raw[24:32], "little", signed=False)
    (mode, err, trace, cal, sec, pwr, dc, n_axes, n_active) = raw[32:41]

    idx = STATE_HEADER_END + n_axes * AXIS_RECORD_SIZE
    _need(raw, idx + 1, "state axes")
    axes = []
    for i in range(n_axes):
        o = STATE_HEADER_END + i * AXIS_RECORD_SIZE
        measured, reference, torque = struct.unpack_from("<ddd", raw, o + 3)
        axes.append(AxisState(
            enabled=raw[o] != 0, calibrated=raw[o + 1] != 0,
            used=raw[o + 2] != 0,
            measured=measured, reference=reference, torque=torque))

    n_buttons = raw[idx]
    idx += 1
    _need(raw, idx + n_buttons, "state buttons")
    buttons = tuple(int(b != 0) for b in raw[idx:idx + n_buttons])
    idx += n_buttons

    digital_io: Tuple[int, ...] = ()
    if idx < len(raw):
        # NOTE: the original read digital I/O from the wrong offset
        # (init_axis_info + i*n); the value was never used.  Read properly here.
        n_dio = raw[idx]
        idx += 1
        digital_io = tuple(int(b != 0) for b in raw[idx:idx + n_dio])

    return RobotState(
        tick=tick, controller_mode=mode, controller_error=err,
        trace_on=trace, calibrated=cal, security_on=sec, power_on=pwr,
        dc_on=dc, number_axis=n_axes, active_axis=n_active,
        axes=tuple(axes), buttons=buttons, digital_io=digital_io)