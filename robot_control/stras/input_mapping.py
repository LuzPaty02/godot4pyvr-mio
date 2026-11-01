"""Map tracked VR pose offsets to a clamped, simulation-only STRAS q."""
from __future__ import annotations

import math
from typing import Mapping

import numpy as np

from .constants import QP_ORDER
from .limits import JointLimits

POSITION_FULL_SCALE_M = 0.1
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


class ControllerToConfiguration:
    """Convert global pose deltas from a captured neutral frame into q.

    The mapping is absolute, not velocity-integrated: each connected hand and
    the camera capture a neutral pose on their first frame. 10 cm reaches the
    associated position-joint bound; a 90 degree yaw delta reaches the ER bound.
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

        values = {label: 0.0 for label in QP_ORDER}
        hand_joints = {
            "right": ("BR", "RR", "TR"),
            "left": ("BL", "RL", "TL"),
        }
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
            for joint, current, baseline in zip(joints, position, neutral):
                index = QP_ORDER.index(joint)
                values[joint] = _scaled_offset(
                    current - baseline, POSITION_FULL_SCALE_M,
                    float(self._limits.lower[index]),
                    float(self._limits.upper[index]))

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