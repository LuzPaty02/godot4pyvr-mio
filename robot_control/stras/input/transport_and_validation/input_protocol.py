"""Validation and decoding for Godot-to-Python input datagrams."""
from __future__ import annotations

import json
from dataclasses import dataclass

PROTOCOL_VERSION = 1
_HANDS = ("left", "right")


class PacketError(ValueError):
    """Datagram is not a valid input packet."""


@dataclass(frozen=True)
class InputMessage:
    seq: int
    payload: dict          # Controller payload with protocol fields removed.
    received_at: float     # Local time.monotonic() at arrival.


def decode_packet(data: bytes, received_at: float) -> InputMessage:
    """Decode and validate one JSON datagram."""
    try:
        obj = json.loads(data.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise PacketError(f"not valid UTF-8 JSON: {exc}") from None
    if not isinstance(obj, dict):
        raise PacketError("top level is not an object")
    if obj.get("v") != PROTOCOL_VERSION:
        raise PacketError(f"unsupported protocol version: {obj.get('v')!r}")
    seq = obj.get("seq")
    if not isinstance(seq, int) or isinstance(seq, bool) or seq < 0:
        raise PacketError(f"bad seq: {seq!r}")
    for hand in _HANDS:
        if not isinstance(obj.get(hand), dict):
            raise PacketError(f"missing or invalid '{hand}'")
    payload = {key: value for key, value in obj.items() if key not in ("v", "seq")}
    return InputMessage(seq=seq, payload=payload, received_at=received_at)