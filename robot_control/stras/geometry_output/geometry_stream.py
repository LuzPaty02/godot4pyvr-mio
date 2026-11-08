"""Serialize and send STRAS geometry frames to the Godot visualization."""
from __future__ import annotations

import json
import socket

import numpy as np

from robot_control.network_constants import (
    GEOMETRY_PORT,
    GEOMETRY_PROTOCOL_VERSION,
    LOOPBACK_HOST,
)
from robot_control.stras.model.kinematics import ShapeOutput


def build_geometry_packet(frame: int, shape: ShapeOutput,
                          camera_calibration=None) -> dict:
    """Build the JSON-compatible frame expected by ``stras_receiver.gd``."""
    right_sections = [np.asarray(section).tolist() for section in shape.right_sections]
    left_sections = [np.asarray(section).tolist() for section in shape.left_sections]
    endoscope_sections = shape.endoscope_sections or ()
    endoscope = (np.concatenate(endoscope_sections, axis=0)
                 if endoscope_sections else np.empty((0, 3)))
    camera = {"pose": np.asarray(shape.tip_e).tolist(), "intrinsics": None}
    if camera_calibration is not None:
        camera["intrinsics"] = {
            "fx": float(camera_calibration.f[0]),
            "fy": float(camera_calibration.f[1]),
            "cx": float(camera_calibration.c[0]),
            "cy": float(camera_calibration.c[1]),
            "width": int(camera_calibration.imgSize[0]),
            "height": int(camera_calibration.imgSize[1]),
        }
    return {
        "v": GEOMETRY_PROTOCOL_VERSION,
        "frame": int(frame),
        "right": np.asarray(shape.right).tolist(),
        "left": np.asarray(shape.left).tolist(),
        "endoscope": endoscope.tolist(),
        "right_sections": right_sections,
        "left_sections": left_sections,
        "endoscope_sections": [np.asarray(section).tolist()
                               for section in endoscope_sections],
        "camera": camera,
    }


class GodotGeometrySender:
    """Send latest-only STRAS geometry frames to the Godot UDP receiver."""

    def __init__(self, host: str = LOOPBACK_HOST, port: int = GEOMETRY_PORT):
        self._address = (host, port)
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send(self, frame: int, shape: ShapeOutput,
             camera_calibration=None) -> None:
        packet = build_geometry_packet(frame, shape, camera_calibration)
        self._socket.sendto(
            json.dumps(packet, separators=(",", ":")).encode("utf-8"), self._address)

    def close(self) -> None:
        self._socket.close()