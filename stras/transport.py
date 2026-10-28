"""Thin socket wrapper.

Failures are raised as exceptions (see ``errors.py``) instead of being stored
in flags.  Tests can pass a fake ``sock`` or replace the whole transport.
"""
from __future__ import annotations

import socket
from typing import Optional, Union

from .errors import ConnectionFailed, TransportError, TransportTimeout

RECV_SIZE = 4096


class StrasTransport:
    def __init__(self, host: str, port: Union[int, str],
                 timeout: Optional[float] = None,
                 sock: Optional[socket.socket] = None):
        self.host = host
        self.port = int(port)   # the original mixed '6666' and 6666
        self._sock = sock or socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        if timeout is not None:
            self.set_timeout(timeout)

    def set_timeout(self, seconds: float) -> None:
        self._sock.settimeout(float(seconds))

    def connect(self) -> None:
        if self._sock.connect_ex((self.host, self.port)) != 0:
            raise ConnectionFailed(f"could not connect to {self.host}:{self.port}")

    def send(self, msg: bytes) -> None:
        try:
            self._sock.sendall(msg)
        except socket.timeout as exc:       # subclass of OSError: keep first
            raise TransportTimeout("timeout while sending") from exc
        except OSError as exc:
            raise TransportError(f"send failed: {exc}") from exc

    def recv(self) -> bytes:
        try:
            return self._sock.recv(RECV_SIZE)
        except socket.timeout as exc:
            raise TransportTimeout("timeout while receiving") from exc
        except OSError as exc:
            raise TransportError(f"receive failed: {exc}") from exc

    def request(self, msg: bytes) -> bytes:
        """Send a message and return the raw reply."""
        self.send(msg)
        return self.recv()

    def close(self) -> None:
        self._sock.close()

    def __enter__(self) -> "StrasTransport":
        self.connect()
        return self

    def __exit__(self, *exc) -> None:
        self.close()