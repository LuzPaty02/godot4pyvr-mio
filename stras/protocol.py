"""Wire protocol for the STRAS slave: pure functions, no sockets, no state.

Frame layout (all integers little-endian)::

    0:4    sync            b'!**!'
    4:8    total length    header + body + checksum   (u32)
    8:12   message type    (u32)   0 = CMD
    12:16  version         (u32)   1
    16:20  reference       (u32)   0
    20:24  reply mode      (u32)   0 = no reply, 1 = reply requested
    24:-1  body            command-specific
    -1     checksum        sum(all previous bytes) % 256

The length field always equals ``len(frame)``. The legacy code hard-coded 33
for plain commands (correct) but computed ``33 + 10n`` for velocity messages
(wrong for n != 1; the real size is ``34 + 9n``).
"""
from __future__ import annotations

import struct
from dataclasses import dataclass
from typing import Iterable, Sequence

from .constants import Command
from .errors import ProtocolError

SYNC = b"!**!"
HEADER_SIZE = 24
CHECKSUM_SIZE = 1
MIN_FRAME_SIZE = HEADER_SIZE + CHECKSUM_SIZE
MAX_FRAME_SIZE = 1 << 16  # sanity bound used when resynchronising
MSG_TYPE_CMD = 0
PROTOCOL_VERSION = 1

_HEADER = struct.Struct("<4sIIIII")
assert _HEADER.size == HEADER_SIZE


# --------------------------------------------------------------------------
# building
# --------------------------------------------------------------------------
def checksum(data: bytes) -> int:
    return sum(data) & 0xFF


def _frame(body: bytes, reply_mode: int, msg_type: int = MSG_TYPE_CMD) -> bytes:
    total = HEADER_SIZE + len(body) + CHECKSUM_SIZE
    head = _HEADER.pack(SYNC, total, msg_type, PROTOCOL_VERSION, 0, reply_mode)
    msg = head + body
    return msg + bytes([checksum(msg)])


def build_cmd(command: int, reply_mode: int = 1, extra: int | None = None) -> bytes:
    """Plain command (ON, OFF, GET_STATE, SEND_AXIS_INFOS, ...).

    ``extra`` is the optional 4-byte argument the old code called ``xtraV``.
    Unlike the legacy builder, the length field accounts for it (37, not 33).
    """
    body = struct.pack("<Q", int(command))
    if extra is not None:
        body += struct.pack("<I", int(extra))
    return _frame(body, reply_mode)


def _check_axis_count(n: int) -> None:
    if not 1 <= n <= 255:
        raise ProtocolError(f"axis count must be 1..255, got {n}")


def _axis_byte(ax: int) -> int:
    ax = int(ax)
    if not 0 <= ax <= 255:
        raise ProtocolError(f"axis index out of byte range: {ax}")
    return ax


def build_vel_idx(axes_vals: Sequence[tuple[int, float]], reply_mode: int = 0) -> bytes:
    """Constant velocity per axis: ``[(axis_idx, velocity), ...]``."""
    n = len(axes_vals)
    _check_axis_count(n)
    body = struct.pack("<IIB", Command.IDX_VELOCITY, 9 * n + 1, n)
    for ax, vel in axes_vals:
        body += struct.pack("<Bd", _axis_byte(ax), float(vel))
    return _frame(body, reply_mode)


def build_goto(
    entries: Sequence[tuple[int, float, float]],
    command: int = Command.IDX_GO_TO_VEL,
    reply_mode: int = 0,
) -> bytes:
    """Go-to-position per axis: ``[(axis_idx, position, max_rel_vel), ...]``.

    ``command`` defaults to 620 (the one the legacy code comments say to use).
    The legacy ``send_GoToPosPerAxis`` sent 42, which is the *message type*
    AXISINFOS, not a command; pass ``command=42`` only to reproduce it.
    """
    n = len(entries)
    _check_axis_count(n)
    body = struct.pack("<IIB", int(command), 17 * n + 1, n)
    for ax, pos, max_vel in entries:
        body += struct.pack("<Bdd", _axis_byte(ax), float(pos), float(max_vel))
    return _frame(body, reply_mode)


# convenience wrappers (one line each, so call sites read well)
def build_on() -> bytes: return build_cmd(Command.ON)
def build_off() -> bytes: return build_cmd(Command.OFF)
def build_stop() -> bytes: return build_cmd(Command.STOP)
def build_reset() -> bytes: return build_cmd(Command.RESET)
def build_get_state() -> bytes: return build_cmd(Command.GET_STATE)
def build_get_axis_infos() -> bytes: return build_cmd(Command.SEND_AXIS_INFOS)


# --------------------------------------------------------------------------
# parsing
# --------------------------------------------------------------------------
@dataclass(frozen=True)
class Header:
    length: int
    msg_type: int
    version: int
    reference: int
    reply_mode: int


def parse_header(frame: bytes) -> Header:
    if len(frame) < HEADER_SIZE:
        raise ProtocolError(f"frame shorter than header: {len(frame)} bytes")
    sync, length, msg_type, version, ref, reply = _HEADER.unpack_from(frame)
    if sync != SYNC:
        raise ProtocolError(f"bad sync bytes: {sync!r}")
    return Header(length, msg_type, version, ref, reply)


def verify_checksum(frame: bytes) -> bool:
    return len(frame) >= MIN_FRAME_SIZE and checksum(frame[:-1]) == frame[-1]


def body_of(frame: bytes) -> bytes:
    """Payload between header and checksum (what parse_state/parse_axis_infos
    should index from 0; legacy code indexed the raw frame from 24)."""
    return frame[HEADER_SIZE:-CHECKSUM_SIZE]


def split_frames(buffer: bytes) -> tuple[list[bytes], bytes]:
    """Cut a receive buffer into complete frames using the length field.

    Returns ``(frames, remainder)``; keep ``remainder`` and prepend it to the
    next ``recv``. Handles merged frames, partial frames and leading garbage
    (resynchronises on the sync bytes).
    """
    frames: list[bytes] = []
    buf = bytes(buffer)
    while True:
        start = buf.find(SYNC)
        if start < 0:
            # keep a possible partial sync at the tail
            return frames, buf[-(len(SYNC) - 1):] if buf else b""
        buf = buf[start:]
        if len(buf) < 8:
            return frames, buf
        (length,) = struct.unpack_from("<I", buf, 4)
        if not MIN_FRAME_SIZE <= length <= MAX_FRAME_SIZE:
            buf = buf[1:]  # false sync, keep scanning
            continue
        if len(buf) < length:
            return frames, buf
        frames.append(buf[:length])
        buf = buf[length:]