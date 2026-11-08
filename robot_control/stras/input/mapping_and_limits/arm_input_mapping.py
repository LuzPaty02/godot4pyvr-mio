"""Map controller hand poses to the six arm joints."""
from __future__ import annotations

import math
from typing import Mapping

from ...simulation.constants import QP_ORDER
from .limits import JointLimits
from .mapping_math import (
    ARM_BEND_FULL_SCALE_M,
    ARM_INSERTION_FULL_SCALE_M,
    ARM_INSERTION_HOME_MM,
    ARM_INSERTION_TRAVEL_MM,
    map_bend,
    read_vector,
    read_quaternion,
    signed_twist_angle,
    wrap_angle,
)

HAND_ROLL_AXIS = (0.0, 0.0, -1.0)  # OpenXR controller-local forward axis.
HAND_ROLL_GAIN = 1.0  # Equal radians of wrist twist produce equal RR/RL change.


class ArmInputMapper:
    """Track hand neutral poses and convert offsets into arm joint values."""

    def __init__(self, limits: JointLimits):
        self._limits = limits
        self._hand_neutral: dict[str, tuple[float, float, float] | None] = {
            "left": None,
            "right": None,
        }
        self._hand_neutral_orientation: dict[
            str, tuple[float, float, float, float] | None
        ] = {"left": None, "right": None}
        self._bend_branch: dict[str, int | None] = {"left": None, "right": None}

    def update(self, payload: Mapping, camera_yaw: float) -> dict[str, float]:
        if (isinstance(camera_yaw, bool)
                or not isinstance(camera_yaw, (int, float))):
            raise ValueError("camera_yaw must be a finite number")
        try:
            camera_yaw = float(camera_yaw)
        except OverflowError as error:
            raise ValueError("camera_yaw must be a finite number") from error
        if not math.isfinite(camera_yaw):
            raise ValueError("camera_yaw must be a finite number")
        if not isinstance(payload, Mapping):
            raise ValueError("input payload must be an object")

        values: dict[str, float] = {}
        pending_neutral = self._hand_neutral.copy()
        pending_neutral_orientation = self._hand_neutral_orientation.copy()
        pending_branch = self._bend_branch.copy()
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
                pending_neutral[hand] = None
                pending_neutral_orientation[hand] = None
                pending_branch[hand] = None
                continue
            position = read_vector(state.get("position"), ("x", "y", "z"),
                                   f"{hand}.position")
            orientation = None
            if "orientation" in state:
                orientation = read_quaternion(
                    state.get("orientation"), f"{hand}.orientation")
            neutral = pending_neutral[hand]
            if neutral is None:
                pending_neutral[hand] = position
                neutral = position
            neutral_orientation = pending_neutral_orientation[hand]
            if orientation is not None and neutral_orientation is None:
                pending_neutral_orientation[hand] = orientation
                neutral_orientation = orientation

            # Godot's +Y is up, while the calibrated OpenCV view's +Y is down.
            dx = position[0] - neutral[0]
            dz = position[2] - neutral[2]
            horizontal = dx * right_x + dz * right_z
            vertical = neutral[1] - position[1]
            bend_index = QP_ORDER.index(joints[0])
            bend, rotation = map_bend(
                horizontal, vertical, ARM_BEND_FULL_SCALE_M,
                float(self._limits.lower[bend_index]),
                float(self._limits.upper[bend_index]),
                previous_branch=pending_branch[hand])
            if bend != 0.0:
                pending_branch[hand] = 1 if bend > 0.0 else -1
            roll = 0.0
            if orientation is not None and neutral_orientation is not None:
                roll = HAND_ROLL_GAIN * signed_twist_angle(
                    neutral_orientation, orientation, HAND_ROLL_AXIS)
            values[joints[0]] = bend
            values[joints[1]] = wrap_angle(rotation + roll)

            # Away from the torso extends the instrument along the initial view axis.
            insertion_index = QP_ORDER.index(joints[2])
            insertion_offset = dx * forward_x + dz * forward_z
            insertion_min = max(0.0, float(self._limits.lower[insertion_index]))
            insertion_max = float(self._limits.upper[insertion_index])
            insertion_home = min(max(ARM_INSERTION_HOME_MM, insertion_min), insertion_max)
            insertion_travel = min(
                ARM_INSERTION_TRAVEL_MM,
                insertion_home - insertion_min,
                insertion_max - insertion_home,
            )
            insertion_ratio = max(
                -1.0, min(1.0, insertion_offset / ARM_INSERTION_FULL_SCALE_M))
            values[joints[2]] = insertion_home + insertion_ratio * insertion_travel
        self._hand_neutral.update(pending_neutral)
        self._hand_neutral_orientation.update(pending_neutral_orientation)
        self._bend_branch.update(pending_branch)
        return values