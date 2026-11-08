# input_sender.py
import json
import socket

from robot_control.stras.input.transport_and_validation.input_protocol import PROTOCOL_VERSION
from robot_control.network_constants import INPUT_PORT, LOOPBACK_HOST


class InputClient:
    def __init__(self, addr=(LOOPBACK_HOST, INPUT_PORT)):
        self._addr = addr
        self._seq = 0
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) #AF_INET = IPV4, SOCK_DGRAM = UDP
        self._sock.setblocking(False)#

    def send(self, payload: dict) -> None:
        if self._sock is None:
            return
        msg = {**payload, "v": PROTOCOL_VERSION, "seq": self._seq}
        self._seq += 1
        try:
            self._sock.sendto(json.dumps(msg).encode("utf-8"), self._addr)
        except OSError:
            pass  # UDP: dropping a packet is fine, never block the frame

    def close(self) -> None:
        if self._sock is not None:
            self._sock.close()
            self._sock = None