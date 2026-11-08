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
from robot_control.stras.input.mapping_and_limits.limits import RobotLimits


_HUD_JOINTS = ("BL", "RL", "TL", "BR", "RR", "TR")


def _normalized_hud_values(q, limits: RobotLimits) -> dict[str, float]:
    joint_limits = limits.virtual_cs
    values = np.asarray(q, dtype=np.float64)
    if values.shape != joint_limits.lower.shape:
        raise ValueError("q must match the virtual joint limits")
    indices = {label: joint_limits.labels.index(label) for label in _HUD_JOINTS}
    hud = {}
    for label in _HUD_JOINTS:
        index = indices[label]
        span = joint_limits.upper[index] - joint_limits.lower[index]
        normalized = (values[index] - joint_limits.lower[index]) / span
        hud[label] = float(np.clip(normalized, 0.0, 1.0))
    return hud


def build_geometry_packet(frame: int, shape: ShapeOutput,
                          camera_calibration=None, q=None,
                          limits: RobotLimits | None = None) -> dict:
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
    packet = {
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
    if (q is None) != (limits is None):
        raise ValueError("q and limits must be provided together")
    if q is not None and limits is not None:
        packet["hud"] = _normalized_hud_values(q, limits)
    return packet


class GodotGeometrySender:
    """Send latest-only STRAS geometry frames to the Godot UDP receiver."""

    def __init__(self, host: str = LOOPBACK_HOST, port: int = GEOMETRY_PORT):
        self._address = (host, port)
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send(self, frame: int, shape: ShapeOutput,
             camera_calibration=None, q=None,
             limits: RobotLimits | None = None) -> None:
        packet = build_geometry_packet(
            frame, shape, camera_calibration, q=q, limits=limits)
        self._socket.sendto(
            json.dumps(packet, separators=(",", ":")).encode("utf-8"), self._address)

    def close(self) -> None:
        self._socket.close()