"""Server side of the Godot -> controller link (counterpart of input_client.py).

    UDP socket -> decode_packet -> SeqFilter -> Mailbox (latest wins)

Runs in the controller process. Knows nothing about the robot, the watchdog
or the teleop mapping: it only turns datagrams into validated InputMessages
and keeps the newest one.

Packet format (JSON, UTF-8), as produced by input_client.py::

    {"v": 1, "seq": 42,
     "left":  {"connected": true, "position": {"x": 0.0, "y": 0.0, "z": 0.0},
               "trigger": 0.0, "grip": 0.0,
               "primary": {"x": 0.0, "y": 0.0}, "secondary": {...},
               "buttons": {...}},
     "right": {...},
     "dofs": {"ET": 0.5, "ER": 0.5, "EBV": 0.5, "EBH": 0.5}}

Timing: ``received_at`` is stamped with this process's monotonic clock on
arrival. Never compare it with a clock from the Godot process.

Run ``python input_server.py`` to print what arrives (use tools/fake_client.py).
"""
from __future__ import annotations

import json
import socket
import threading
import time
from dataclasses import dataclass, replace
from typing import Callable, Optional

from robot_control.latest_slot import Mailbox

PROTOCOL_VERSION = 1
MAX_PACKET = 65535
# A seq this far *below* the last accepted one means the client restarted
# (Godot reloaded; its counter went back to 0), not that a packet arrived late.
SESSION_RESTART_GAP = 1000
_HANDS = ("left", "right")


class PacketError(ValueError):
    """Datagram is not a valid input packet."""


# --------------------------------------------------------------------------
# decoding
# --------------------------------------------------------------------------
@dataclass(frozen=True)
class InputMessage:
    seq: int
    payload: dict          # {"left": {...}, "right": {...}} as sent
    received_at: float     # local time.monotonic() at arrival


def decode_packet(data: bytes, received_at: float) -> InputMessage:
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
    payload = {k: v for k, v in obj.items() if k not in ("v", "seq")}
    return InputMessage(seq=seq, payload=payload, received_at=received_at)


# --------------------------------------------------------------------------
# sequence filtering
# --------------------------------------------------------------------------
ACCEPT, STALE, RESTART = "accept", "stale", "restart"


class SeqFilter:
    """Drops old/duplicate packets, counts gaps, notices client restarts."""

    def __init__(self, restart_gap: int = SESSION_RESTART_GAP):
        self._restart_gap = restart_gap
        self.last: Optional[int] = None

    def classify(self, seq: int) -> tuple[str, int]:
        """Returns (verdict, packets_lost_before_this_one)."""
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
        return STALE, 0            # duplicate or reordered


# --------------------------------------------------------------------------
# server
# --------------------------------------------------------------------------
@dataclass(frozen=True)
class ServerStats:
    received: int = 0     # datagrams seen
    accepted: int = 0     # valid and newer than the last
    stale: int = 0        # valid but old/duplicate
    lost: int = 0         # gaps in seq (estimated packet loss)
    malformed: int = 0
    restarts: int = 0     # client session restarts detected


class InputServer:
    def __init__(
        self,
        mailbox: Mailbox[InputMessage],
        host: str = "127.0.0.1",
        port: int = 9000,
        clock: Callable[[], float] = time.monotonic,
    ):
        self._mailbox = mailbox
        self._host, self._port = host, port
        self._clock = clock
        self._filter = SeqFilter()
        self._stats = ServerStats()
        self._stats_lock = threading.Lock()
        self._sock: Optional[socket.socket] = None
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()

    # -- lifecycle ---------------------------------------------------------
    def start(self) -> None:
        if self._thread is not None:
            raise RuntimeError("server already started")
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((self._host, self._port))
        sock.settimeout(0.1)            # lets stop() take effect promptly
        self._sock = sock
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="InputServer", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        if self._sock is not None:
            self._sock.close()
            self._sock = None

    def __enter__(self) -> "InputServer":
        self.start()
        return self

    def __exit__(self, *exc) -> None:
        self.stop()

    @property
    def address(self) -> tuple[str, int]:
        """Bound (host, port); useful when constructed with port=0."""
        if self._sock is None:
            raise RuntimeError("server not started")
        return self._sock.getsockname()

    @property
    def stats(self) -> ServerStats:
        with self._stats_lock:
            return self._stats

    # -- internals ---------------------------------------------------------
    def _bump(self, **inc: int) -> None:
        with self._stats_lock:
            self._stats = replace(
                self._stats, **{k: getattr(self._stats, k) + v for k, v in inc.items()})

    def _run(self) -> None:
        sock = self._sock
        while not self._stop.is_set():
            try:
                data, _addr = sock.recvfrom(MAX_PACKET)
            except socket.timeout:
                continue
            except OSError:
                break                   # socket closed under us
            self.handle_datagram(data, self._clock())

    def handle_datagram(self, data: bytes, now: float) -> None:
        """Process one datagram. Public so tests can drive it without a socket."""
        self._bump(received=1)
        try:
            msg = decode_packet(data, now)
        except PacketError:
            self._bump(malformed=1)
            return
        verdict, lost = self._filter.classify(msg.seq)
        if verdict == STALE:
            self._bump(stale=1)
            return
        if verdict == RESTART:
            self._bump(restarts=1)
        self._bump(accepted=1, lost=lost)
        self._mailbox.put(msg)


# --------------------------------------------------------------------------
# demo: python input_server.py [port]
# --------------------------------------------------------------------------
if __name__ == "__main__":
    import sys

    port = int(sys.argv[1]) if len(sys.argv) > 1 else 9000
    box: Mailbox[InputMessage] = Mailbox()
    with InputServer(box, port=port) as lst:
        print(f"listening on {lst.address}; Ctrl-C to quit")
        try:
            while True:
                time.sleep(0.5)
                m = box.get()
                if m is None:
                    print("no packets yet", lst.stats)
                else:
                    age_ms = (time.monotonic() - m.received_at) * 1000
                    r = m.payload["right"]
                    print(f"seq={m.seq} age={age_ms:6.1f}ms "
                          f"R.trigger={r.get('trigger')} R.grip={r.get('grip')} {lst.stats}")
        except KeyboardInterrupt:
            pass