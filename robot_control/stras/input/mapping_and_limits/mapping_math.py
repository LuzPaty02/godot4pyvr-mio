"""Shared validation and scaling math for VR-to-STRAS input mapping."""
from __future__ import annotations

import math
from typing import Mapping

CAMERA_POSITION_FULL_SCALE_M = 0.1
ARM_INSERTION_FULL_SCALE_M = 0.25
ARM_BEND_FULL_SCALE_M = 0.15
# At the current endoscope calibration, straight-arm insertions from 30–70 mm
# keep both distal arms inside the floating viewport.
ARM_INSERTION_HOME_MM = 50.0
ARM_INSERTION_TRAVEL_MM = 20.0
DEADZONE_M = 0.005
YAW_FULL_SCALE_RAD = math.pi / 2.0


def _validate_full_scale(full_scale: float, minimum: float) -> float:
    if isinstance(full_scale, bool) or not isinstance(full_scale, (int, float)):
        raise ValueError("full_scale must be a finite positive number")
    try:
        value = float(full_scale)
    except OverflowError as error:
        raise ValueError("full_scale must be a finite positive number") from error
    if not math.isfinite(value) or value <= minimum:
        raise ValueError("full_scale must be a finite positive number")
    return value


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


def read_quaternion(values: Mapping, label: str) -> tuple[float, float, float, float]:
    """Read and normalize an xyzw quaternion from a controller payload."""
    x, y, z, w = read_vector(values, ("x", "y", "z", "w"), label)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-12:
        raise ValueError(f"{label} quaternion must be non-zero")
    return x / norm, y / norm, z / norm, w / norm


def signed_twist_angle(neutral: tuple[float, float, float, float],
                       current: tuple[float, float, float, float],
                       axis: tuple[float, float, float] = (0.0, 0.0, -1.0)) -> float:
    """Return shortest signed twist from neutral around a controller-local axis.

    Quaternions use xyzw order. Swing away from the selected axis is discarded;
    choosing the negative Z axis matches the OpenXR controller forward axis.
    """
    axis_norm = math.sqrt(sum(component * component for component in axis))
    if axis_norm < 1e-12:
        raise ValueError("twist axis must be non-zero")
    axis = tuple(component / axis_norm for component in axis)

    # q_delta = inverse(neutral) * current, expressed in the neutral hand frame.
    nx, ny, nz, nw = neutral
    cx, cy, cz, cw = current
    dot = nx * cx + ny * cy + nz * cz + nw * cw
    if dot < 0.0:  # q and -q encode the same rotation; choose the short arc.
        cx, cy, cz, cw = -cx, -cy, -cz, -cw
    dx = nw * cx - nx * cw - ny * cz + nz * cy
    dy = nw * cy + nx * cz - ny * cw - nz * cx
    dz = nw * cz - nx * cy + ny * cx - nz * cw
    dw = nw * cw + nx * cx + ny * cy + nz * cz

    projected = dx * axis[0] + dy * axis[1] + dz * axis[2]
    twist_norm = math.hypot(projected, dw)
    if twist_norm < 1e-12:
        return 0.0  # twist is undefined for a pure 180-degree swing.
    angle = 2.0 * math.atan2(projected / twist_norm, dw / twist_norm)
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def wrap_angle(angle: float) -> float:
    """Wrap an angle to [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def scaled_offset(offset: float, full_scale: float,
                  lower: float, upper: float) -> float:
    """Map a signed full-scale offset to the corresponding signed q bound."""
    full_scale = _validate_full_scale(full_scale, 0.0)
    ratio = max(-1.0, min(1.0, offset / full_scale))
    return ratio * (upper if ratio >= 0.0 else abs(lower))


def map_bend(horizontal: float, vertical: float, full_scale: float,
              lower: float, upper: float,
              previous_branch: int | None = None) -> tuple[float, float]:
    """Convert screen-plane hand movement to signed curvature and bend angle.

    At angles of +/- pi/2 the equivalent representation changes branch: the
    angle jumps by about pi and curvature changes sign, despite the same
    physical pose (with a different RR/RL rotation). ``previous_branch`` lets
    the caller retain its prior sign/angle branch until it crosses a 10-degree
    hysteresis margin. The mapping remains pure; branch state belongs to the
    per-hand mapper.
    """
    try:
        full_scale = _validate_full_scale(full_scale, DEADZONE_M)
    except ValueError as error:
        raise ValueError("full_scale must be greater than DEADZONE_M") from error

    distance = math.hypot(horizontal, vertical)
    if distance < 1e-9:
        return 0.0, 0.0

    raw_angle = math.atan2(vertical, horizontal)
    margin = math.radians(10.0)
    if previous_branch == 1:
        curvature_sign = (1.0 if -math.pi / 2.0 - margin <= raw_angle
                          <= math.pi / 2.0 + margin else -1.0)
    elif previous_branch == -1:
        curvature_sign = (-1.0 if raw_angle >= math.pi / 2.0 - margin
                          or raw_angle <= -math.pi / 2.0 + margin else 1.0)
    else:
        curvature_sign = 1.0 if -math.pi / 2.0 <= raw_angle <= math.pi / 2.0 else -1.0

    if curvature_sign < 0.0:
        angle = raw_angle - math.pi if raw_angle > 0.0 else raw_angle + math.pi
    else:
        angle = raw_angle

    magnitude = min(1.0, max(0.0, (distance - DEADZONE_M) /
                             (full_scale - DEADZONE_M)))
    curvature_limit = upper if curvature_sign > 0.0 else abs(lower)
    return curvature_sign * magnitude * curvature_limit, angle