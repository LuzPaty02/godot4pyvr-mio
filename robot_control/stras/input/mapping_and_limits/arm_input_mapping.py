"""Map controller hand poses to the six arm joints."""
from __future__ import annotations

import math
from typing import Mapping

from ...constants import QP_ORDER
from .limits import JointLimits
from .mapping_math import (
    ARM_BEND_FULL_SCALE_M,
    ARM_INSERTION_FULL_SCALE_M,
    map_bend,
    read_vector,
    scaled_offset,
)


class ArmInputMapper:
    """Track hand neutral poses and convert offsets into arm joint values."""

    def __init__(self, limits: JointLimits):
        self._limits = limits
        self._hand_neutral: dict[str, tuple[float, float, float] | None] = {
            "left": None,
            "right": None,
        }

    def update(self, payload: Mapping, camera_yaw: float) -> dict[str, float]:
        values: dict[str, float] = {}
        hand_joints = {
            "right": ("BR", "RR", "TR"),
            "left": ("BL", "RL", "TL"),
        }
        right_x = math.cos(camera_yaw)
        right_z = -math.sin(camera_yaw)
        forward_x = -math.sin(camera_yaw)
        forward_z = -math.cos(camera_yaw)
        for hand, joints in hand_joints.items():
            state = payload.get(hand)
            if not isinstance(state, Mapping):
                raise ValueError(f"missing or invalid '{hand}' controller")
            if not state.get("connected", False):
                self._hand_neutral[hand] = None
                continue
            position = read_vector(state.get("position"), ("x", "y", "z"),
                                   f"{hand}.position")
            neutral = self._hand_neutral[hand]
            if neutral is None:
                self._hand_neutral[hand] = position
                neutral = position

            # Godot's +Y is up, while the calibrated OpenCV view's +Y is down.
            dx = position[0] - neutral[0]
            dz = position[2] - neutral[2]
            horizontal = dx * right_x + dz * right_z
            vertical = neutral[1] - position[1]
            bend_index = QP_ORDER.index(joints[0])
            bend, rotation = map_bend(
                horizontal, vertical, ARM_BEND_FULL_SCALE_M,
                float(self._limits.lower[bend_index]),
                float(self._limits.upper[bend_index]))
            values[joints[0]] = bend
            values[joints[1]] = rotation

            # Away from the torso extends the instrument along the initial view axis.
            insertion_index = QP_ORDER.index(joints[2])
            insertion_offset = dx * forward_x + dz * forward_z
            values[joints[2]] = scaled_offset(
                insertion_offset, ARM_INSERTION_FULL_SCALE_M,
                float(self._limits.lower[insertion_index]),
                float(self._limits.upper[insertion_index]))
        return values