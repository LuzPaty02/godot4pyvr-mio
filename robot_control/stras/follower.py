"""In-process follower state for the VR-driven STRAS simulation."""
from __future__ import annotations

import numpy as np

from .constants import QP_ORDER
from .limits import RobotLimits


class VirtualFollower:
    """Store validated simulated joint positions; never communicates with hardware."""

    def __init__(self, limits: RobotLimits | None = None):
        self.limits = limits or RobotLimits.default()
        self.q_config_space = np.zeros(len(QP_ORDER), dtype=np.float64)

    def set_q(self, q) -> None:
        """Set a finite, in-range configuration in ``QP_ORDER``."""
        values = np.asarray(q, dtype=np.float64)
        if values.shape != (len(QP_ORDER),):
            raise ValueError(f"q must have shape ({len(QP_ORDER)},), got {values.shape}")
        if not np.all(np.isfinite(values)):
            raise ValueError("q must contain only finite values")

        limits = self.limits.virtual_cs
        if np.any(values < limits.lower) or np.any(values > limits.upper):
            raise ValueError("q is outside the virtual simulation limits")
        self.q_config_space = values.copy()
