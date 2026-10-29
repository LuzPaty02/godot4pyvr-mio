"""Exception hierarchy.

These replace the ad-hoc boolean flags of the original class
(errorSendMsg, errorRecvMsg, errorPairing, timeOutError).
"""


class StrasError(Exception):
    """Base class for every error raised by this package."""


class TransportError(StrasError):
    """The socket failed to send or receive."""


class ConnectionFailed(TransportError):
    """connect_ex() returned a non-zero status."""


class TransportTimeout(TransportError):
    """The socket timed out waiting for a reply."""


class ProtocolError(StrasError):
    """A message was malformed or too short to decode."""