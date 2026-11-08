"""Shared validation and scaling math for VR-to-STRAS input mapping."""
from __future__ import annotations

import math
from typing import Mapping

CAMERA_POSITION_FULL_SCALE_M = 0.1
ARM_INSERTION_FULL_SCALE_M = 0.25
ARM_BEND_FULL_SCALE_M = 0.15
YAW_FULL_SCALE_RAD = math.pi / 2.0


def read_vector(values: Mapping, fields: tuple[str, ...], label: str) -> tuple[float, ...]:
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


def scaled_offset(offset: float, full_scale: float,
                  lower: float, upper: float) -> float:
    """Map a signed full-scale offset to the corresponding signed q bound."""
    ratio = max(-1.0, min(1.0, offset / full_scale))
    return ratio * (upper if ratio >= 0.0 else abs(lower))


def map_bend(horizontal: float, vertical: float, full_scale: float,
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