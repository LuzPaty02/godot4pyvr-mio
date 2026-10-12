"""Persistent UDP service for STRAS forward-kinematics shape requests."""

from __future__ import annotations

import socket
import struct
import sys
from pathlib import Path

import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parent.parent
MODULE_ROOT = PROJECT_ROOT / "module_main"
# The bundled py4godot distribution places its pure-Python liegroups module in
# this directory. Standalone environments may instead install it normally.
BUNDLED_PYTHON_BIN = (
    PROJECT_ROOT
    / "addons"
    / "py4godot"
    / "cpython-3.14.3-linux64"
    / "python"
    / "bin"
)
sys.path.insert(0, str(MODULE_ROOT))
if BUNDLED_PYTHON_BIN.is_dir():
    sys.path.append(str(BUNDLED_PYTHON_BIN))

from EASE_STRAS_follower_xr import XRFollowerAdapter  # noqa: E402


HOST = "127.0.0.1"
PORT = 4242
MAGIC = b"STRS"
POINT_COUNT = 32
REQUEST = struct.Struct("<4sI10f")
RESPONSE_HEADER = struct.Struct("<4sIB")
RESPONSE_SIZE = RESPONSE_HEADER.size + 2 * POINT_COUNT * 3 * 4


def resample_polyline(sections: list, point_count: int = POINT_COUNT) -> np.ndarray:
    """Join section samples and resample them uniformly by arc length."""
    nonempty = [np.asarray(section, dtype=np.float64).reshape(-1, 3) for section in sections if len(section)]
    if not nonempty:
        return np.zeros((point_count, 3), dtype="<f4")

    points = np.concatenate(nonempty, axis=0)
    keep = np.ones(points.shape[0], dtype=bool)
    if points.shape[0] > 1:
        keep[1:] = np.linalg.norm(np.diff(points, axis=0), axis=1) > 1e-9
    points = points[keep]
    if points.shape[0] == 1:
        return np.repeat(points, point_count, axis=0).astype("<f4")

    lengths = np.concatenate(([0.0], np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))))
    if lengths[-1] <= 1e-12:
        return np.repeat(points[:1], point_count, axis=0).astype("<f4")

    targets = np.linspace(0.0, lengths[-1], point_count)
    sampled = np.column_stack(
        [np.interp(targets, lengths, points[:, axis]) for axis in range(3)]
    )
    return sampled.astype("<f4")


def make_response(sequence: int, status: int, right=None, left=None) -> bytes:
    if right is None:
        right = np.zeros((POINT_COUNT, 3), dtype="<f4")
    if left is None:
        left = np.zeros((POINT_COUNT, 3), dtype="<f4")
    return (
        RESPONSE_HEADER.pack(MAGIC, sequence, status)
        + np.asarray(right, dtype="<f4").reshape(POINT_COUNT, 3).tobytes()
        + np.asarray(left, dtype="<f4").reshape(POINT_COUNT, 3).tobytes()
    )


def main() -> None:
    adapter = XRFollowerAdapter()
    adapter.initializeModels()
    print(f"STRAS model initialized; listening on {HOST}:{PORT}", flush=True)

    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as server:
        server.bind((HOST, PORT))
        while True:
            packet, client = server.recvfrom(2048)
            if len(packet) != REQUEST.size:
                continue

            magic, sequence, *configuration = REQUEST.unpack(packet)
            if magic != MAGIC or not np.isfinite(configuration).all():
                continue

            try:
                print(f"Received request {sequence}: {np.asarray(configuration, dtype=np.float32)}", flush=True)
                right_sections, left_sections, _endoscope_tip = adapter.robot_stras.computeShape(
                    np.asarray(configuration, dtype=np.float32)
                )
                right_points = resample_polyline(right_sections)
                left_points = resample_polyline(left_sections)
                print(f"Sending response for request {sequence}; right={right_points.shape}, left={left_points.shape}", flush=True)
                response = make_response(sequence, 0, right_points, left_points)
            except Exception as error:  # Keep the service alive after a bad request.
                print(f"Request {sequence} failed: {error!r}", flush=True)
                response = make_response(sequence, 1)

            server.sendto(response, client)


if __name__ == "__main__":
    main()
