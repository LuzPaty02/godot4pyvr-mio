"""Map head/camera pose deltas to endoscope joints."""
from __future__ import annotations

import math
from typing import Mapping

from ...constants import QP_ORDER
from .limits import JointLimits
from .mapping_math import (
    CAMERA_POSITION_FULL_SCALE_M,
    YAW_FULL_SCALE_RAD,
    read_vector,
    scaled_offset,
)


class CameraInputMapper:
    """Capture the initial camera pose and map subsequent deltas."""

    def __init__(self, limits: JointLimits):
        self._limits = limits
        self._position_neutral: tuple[float, float, float] | None = None
        self._yaw_neutral: float | None = None

    @property
    def neutral_yaw(self) -> float:
        if self._yaw_neutral is None:
            raise RuntimeError("camera neutral pose has not been captured")
        return self._yaw_neutral

    def update(self, camera: Mapping) -> dict[str, float]:
        if not isinstance(camera, Mapping):
            raise ValueError("camera must be an object")
        position = read_vector(
            camera.get("position", {"x": 0.0, "y": 0.0, "z": 0.0}),
            ("x", "y", "z"), "camera.position")
        yaw = camera.get("yaw", 0.0)
        if isinstance(yaw, bool) or not isinstance(yaw, (int, float)):
            raise ValueError("camera.yaw must be a finite number")
        yaw = float(yaw)
        if not math.isfinite(yaw):
            raise ValueError("camera.yaw must be a finite number")
        if self._position_neutral is None:
            self._position_neutral = position
            self._yaw_neutral = yaw

        position_delta = tuple(current - base for current, base in zip(
            position, self._position_neutral))
        yaw_delta = (yaw - self.neutral_yaw + math.pi) % (2.0 * math.pi) - math.pi
        values: dict[str, float] = {}
        for joint, offset in (("EBH", position_delta[0]),
                              ("EBV", position_delta[1]),
                              # Godot looks along -Z; forward head motion extends ET.
                              ("ET", -position_delta[2])):
            index = QP_ORDER.index(joint)
            values[joint] = scaled_offset(
                offset, CAMERA_POSITION_FULL_SCALE_M,
                float(self._limits.lower[index]),
                float(self._limits.upper[index]))

        index = QP_ORDER.index("ER")
        values["ER"] = scaled_offset(
            yaw_delta, YAW_FULL_SCALE_RAD,
            float(self._limits.lower[index]),
            float(self._limits.upper[index]))
        return values