"""Map tracked VR pose offsets to a clamped, simulation-only STRAS q."""
from __future__ import annotations

import math
from typing import Mapping

import numpy as np

from .constants import QP_ORDER
from .limits import JointLimits

POSITION_FULL_SCALE_M = 0.1
ARM_BEND_FULL_SCALE_M = 0.25
YAW_FULL_SCALE_RAD = math.pi / 2.0


def _read_vector(values: Mapping, fields: tuple[str, ...], label: str) -> tuple[float, ...]:
    if not isinstance(values, Mapping):
        raise ValueError(f"{label} must be an object")
    result = []
    for field in fields:
        value = values.get(field)
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError(f"{label}.{field} must be a finite number")
        value = float(value)
        if not math.isfinite(value):
            raise ValueError(f"{label}.{field} must be a finite number")
        result.append(value)
    return tuple(result)


def _scaled_offset(offset: float, full_scale: float, lower: float,
                   upper: float) -> float:
    """Map a signed full-scale offset to the corresponding signed q bound."""
    ratio = max(-1.0, min(1.0, offset / full_scale))
    return ratio * (upper if ratio >= 0.0 else abs(lower))


def _map_bend(horizontal: float, vertical: float, full_scale: float,
              lower: float, upper: float) -> tuple[float, float]:
    """Convert screen-plane hand movement to signed curvature and bend angle."""
    distance = math.hypot(horizontal, vertical)
    if distance < 1e-9:
        return 0.0, 0.0

    angle = math.atan2(vertical, horizontal)
    curvature_sign = 1.0
    if angle > math.pi / 2.0:
        angle -= math.pi
        curvature_sign = -1.0
    elif angle < -math.pi / 2.0:
        angle += math.pi
        curvature_sign = -1.0

    magnitude = min(1.0, distance / full_scale)
    curvature_limit = upper if curvature_sign > 0.0 else abs(lower)
    return curvature_sign * magnitude * curvature_limit, angle


class ControllerToConfiguration:
    """Convert global pose deltas from a captured neutral frame into q.

    The mapping is absolute, not velocity-integrated: each connected hand and
    the camera capture a neutral pose on their first frame. The hands' lateral
    and vertical offsets jointly determine bend magnitude and direction; a
    25 cm radial offset reaches the curvature bound. Hand insertion and camera
    translation use a 10 cm full scale. A 90 degree yaw delta reaches ER's bound.
    """

    def __init__(self, limits: JointLimits):
        if tuple(limits.labels) != QP_ORDER:
            raise ValueError(f"limits must use QP order {QP_ORDER}")
        self._limits = limits
        self._hand_neutral: dict[str, tuple[float, float, float] | None] = {
            "left": None,
            "right": None,
        }
        self._camera_position_neutral: tuple[float, float, float] | None = None
        self._camera_yaw_neutral: float | None = None

    def update(self, payload: Mapping) -> np.ndarray:
        """Return a fresh, finite q vector in the project's QP order."""
        if not isinstance(payload, Mapping):
            raise ValueError("input payload must be an object")

        camera = payload.get("camera", {})
        if not isinstance(camera, Mapping):
            raise ValueError("camera must be an object")
        camera_position = _read_vector(
            camera.get("position", {"x": 0.0, "y": 0.0, "z": 0.0}),
            ("x", "y", "z"), "camera.position")
        yaw = camera.get("yaw", 0.0)
        if isinstance(yaw, bool) or not isinstance(yaw, (int, float)):
            raise ValueError("camera.yaw must be a finite number")
        yaw = float(yaw)
        if not math.isfinite(yaw):
            raise ValueError("camera.yaw must be a finite number")
        if self._camera_position_neutral is None:
            self._camera_position_neutral = camera_position
            self._camera_yaw_neutral = yaw

        values = {label: 0.0 for label in QP_ORDER}
        hand_joints = {
            "right": ("BR", "RR", "TR"),
            "left": ("BL", "RL", "TL"),
        }
        right_x = math.cos(self._camera_yaw_neutral)
        right_z = -math.sin(self._camera_yaw_neutral)
        forward_x = -math.sin(self._camera_yaw_neutral)
        forward_z = -math.cos(self._camera_yaw_neutral)
        for hand, joints in hand_joints.items():
            state = payload.get(hand)
            if not isinstance(state, Mapping):
                raise ValueError(f"missing or invalid '{hand}' controller")
            if not state.get("connected", False):
                self._hand_neutral[hand] = None
                continue
            position = _read_vector(state.get("position"), ("x", "y", "z"),
                                    f"{hand}.position")
            neutral = self._hand_neutral[hand]
            if neutral is None:
                self._hand_neutral[hand] = position
                neutral = position
            # The model represents a bend in polar form (curvature + plane
            # angle). Convert lateral/upward controller motion together;
            # assigning Y to plane angle alone makes vertical motion invisible
            # whenever curvature is zero. The calibrated OpenCV view has +Y
            # downward, so negate Godot's +Y-up controller offset.
            dx = position[0] - neutral[0]
            dz = position[2] - neutral[2]
            horizontal = dx * right_x + dz * right_z
            vertical = neutral[1] - position[1]
            bend_index = QP_ORDER.index(joints[0])
            rotation_index = QP_ORDER.index(joints[1])
            bend, rotation = _map_bend(
                horizontal, vertical, ARM_BEND_FULL_SCALE_M,
                float(self._limits.lower[bend_index]),
                float(self._limits.upper[bend_index]))
            values[joints[0]] = bend
            values[joints[1]] = rotation

            # Away from the torso extends the instrument; toward the torso
            # retracts it, relative to the initial facing axis.
            insertion_index = QP_ORDER.index(joints[2])
            insertion_offset = dx * forward_x + dz * forward_z
            values[joints[2]] = _scaled_offset(
                insertion_offset, POSITION_FULL_SCALE_M,
                float(self._limits.lower[insertion_index]),
                float(self._limits.upper[insertion_index]))

        camera_delta = tuple(current - base for current, base in zip(
            camera_position, self._camera_position_neutral))
        yaw_delta = (yaw - self._camera_yaw_neutral + math.pi) % (2.0 * math.pi) - math.pi
        for joint, offset in (("EBH", camera_delta[0]),
                              ("EBV", camera_delta[1]),
                              # Godot looks along -Z, so forward head motion
                              # increases the model's ET insertion value.
                              ("ET", -camera_delta[2])):
            index = QP_ORDER.index(joint)
            values[joint] = _scaled_offset(
                offset, POSITION_FULL_SCALE_M,
                float(self._limits.lower[index]),
                float(self._limits.upper[index]))
        index = QP_ORDER.index("ER")
        values["ER"] = _scaled_offset(
            yaw_delta, YAW_FULL_SCALE_RAD,
            float(self._limits.lower[index]),
            float(self._limits.upper[index]))

        q = np.array([values[joint] for joint in QP_ORDER], dtype=np.float64)
        return np.clip(q, self._limits.lower, self._limits.upper)