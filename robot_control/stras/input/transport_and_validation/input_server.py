"""Server side of the Godot -> controller link (counterpart of input_client.py).

    UDP socket -> decode_packet -> SeqFilter -> Mailbox (latest wins)

Runs alongside the FK visualization process. It only turns datagrams into
validated InputMessages and keeps the newest one.

Packet format (JSON, UTF-8), as produced by input_client.py::

Controller ``position`` is Godot's global position in metres; ``orientation``
is the global rotation quaternion in xyzw order. The server preserves these
fields as input data; the FK runner maps controller offsets and local wrist
twist into joint configuration unless a packet supplies normalized ``joints``.
The runner does not perform pose registration or IK.

    {"v": 1, "seq": 42,
    "left":  {"connected": true,
               "position": {"x": 0.0, "y": 0.0, "z": 0.0},
               "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
               "trigger": 0.0, "grip": 0.0,
               "primary": {"x": 0.0, "y": 0.0}, "secondary": {...},
               "buttons": {...}},
    "right": {...}}

Timing: ``received_at`` is stamped with this process's monotonic clock on
arrival. Never compare it with a clock from the Godot process.

Run ``python input_server.py`` to print what arrives (use tools/fake_client.py).
"""
import socket
import threading
import time
from dataclasses import dataclass, replace
from typing import Callable, Optional

from robot_control.stras.input.transport_and_validation.input_protocol import (
    PROTOCOL_VERSION,
    InputMessage,
    PacketError,
    decode_packet,
)
from robot_control.stras.input.transport_and_validation.input_sequence import (
    ACCEPT,
    RESTART,
    STALE,
    SESSION_RESTART_GAP,
    SeqFilter,
)
from robot_control.stras.input.transport_and_validation.latest_slot import Mailbox
from robot_control.network_constants import (
    INPUT_PORT,
    INPUT_SOCKET_TIMEOUT_SECONDS,
    LOOPBACK_HOST,
    MAX_UDP_PACKET_BYTES,
    SERVER_THREAD_JOIN_TIMEOUT_SECONDS,
)

MAX_PACKET = MAX_UDP_PACKET_BYTES

__all__ = [
    "ACCEPT",
    "InputMessage",
    "InputServer",
    "MAX_PACKET",
    "PROTOCOL_VERSION",
    "PacketError",
    "RESTART",
    "SESSION_RESTART_GAP",
    "STALE",
    "SeqFilter",
    "ServerStats",
    "decode_packet",
]


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
        host: str = LOOPBACK_HOST,
        port: int = INPUT_PORT,
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
        sock.settimeout(INPUT_SOCKET_TIMEOUT_SECONDS)  # lets stop() take effect promptly
        self._sock = sock
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="InputServer", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=SERVER_THREAD_JOIN_TIMEOUT_SECONDS)
            self._thread = None
        if self._sock is not None:
            self._sock.close()
            self._sock = None

    def __enter__(self) -> "InputServer":
        self.start()
        return self

    def __exit__(self, *_: object) -> None:
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
                data, _ = sock.recvfrom(MAX_PACKET)
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

    port = int(sys.argv[1]) if len(sys.argv) > 1 else INPUT_PORT
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