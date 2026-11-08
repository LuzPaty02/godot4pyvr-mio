"""Compose camera and hand mapping into a clamped STRAS configuration."""
from __future__ import annotations

from typing import Mapping

import numpy as np

from .constants import QP_ORDER
from .limits import JointLimits
from .arm_input_mapping import ArmInputMapper
from .camera_input_mapping import CameraInputMapper
from .mapping_math import (
    ARM_BEND_FULL_SCALE_M,
    ARM_INSERTION_FULL_SCALE_M,
    CAMERA_POSITION_FULL_SCALE_M,
    YAW_FULL_SCALE_RAD,
)

__all__ = [
    "ARM_BEND_FULL_SCALE_M",
    "ARM_INSERTION_FULL_SCALE_M",
    "CAMERA_POSITION_FULL_SCALE_M",
    "ControllerToConfiguration",
    "YAW_FULL_SCALE_RAD",
]


class ControllerToConfiguration:
    """Convert global pose deltas from a captured neutral frame into q.

    The mapping is absolute, not velocity-integrated: each connected hand and
    the camera capture a neutral pose on their first frame. The hands' lateral
    and vertical offsets jointly determine bend magnitude and direction; a
    15 cm radial offset reaches the curvature bound. Arm insertion uses a
    25 cm full scale, while camera translation retains a 10 cm full scale.
    A 90 degree yaw delta reaches ER's bound.
    """

    def __init__(self, limits: JointLimits):
        if tuple(limits.labels) != QP_ORDER:
            raise ValueError(f"limits must use QP order {QP_ORDER}")
        self._limits = limits
        self._arm_mapper = ArmInputMapper(limits)
        self._camera_mapper = CameraInputMapper(limits)

    def update(self, payload: Mapping) -> np.ndarray:
        """Return a fresh, finite q vector in the project's QP order."""
        if not isinstance(payload, Mapping):
            raise ValueError("input payload must be an object")

        camera_values = self._camera_mapper.update(payload.get("camera", {}))
        arm_values = self._arm_mapper.update(
            payload, camera_yaw=self._camera_mapper.neutral_yaw)
        values = {label: 0.0 for label in QP_ORDER}
        values.update(arm_values)
        values.update(camera_values)
        q = np.array([values[joint] for joint in QP_ORDER], dtype=np.float64)
        return np.clip(q, self._limits.lower, self._limits.upper)