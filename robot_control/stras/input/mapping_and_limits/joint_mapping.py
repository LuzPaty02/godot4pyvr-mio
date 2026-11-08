"""Conversions between normalized joint inputs and STRAS configurations."""
from __future__ import annotations

import math
from typing import Mapping

import numpy as np

from ...simulation.constants import QP_ORDER
from .limits import RobotLimits


def normalized_joints_to_q(values: Mapping, limits: RobotLimits) -> np.ndarray:
    """Convert a complete label-to-[0, 1] mapping to STRAS q order."""
    if not isinstance(values, Mapping):
        raise ValueError("joints must be an object mapping every joint label to [0, 1]")
    missing = set(QP_ORDER) - set(values)
    extra = set(values) - set(QP_ORDER)
    if missing or extra:
        raise ValueError(
            f"joints labels mismatch (missing={sorted(missing)}, extra={sorted(extra)})")

    normalized = []
    for joint in QP_ORDER:
        value = values[joint]
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError(f"normalized {joint} must be a finite number in [0, 1]")
        value = float(value)
        if not math.isfinite(value) or not 0.0 <= value <= 1.0:
            raise ValueError(f"normalized {joint} must be a finite number in [0, 1]")
        normalized.append(value)

    joint_limits = limits.virtual_cs
    lower = np.asarray(joint_limits.lower, dtype=np.float64)
    upper = np.asarray(joint_limits.upper, dtype=np.float64)
    return lower + np.asarray(normalized, dtype=np.float64) * (upper - lower)