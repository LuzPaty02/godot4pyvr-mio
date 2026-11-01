"""Joint limits used by the VR-driven STRAS simulation."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping, Sequence

import numpy as np

from .constants import QP_ORDER

_ARM_BEND_LIMIT = np.pi / (2 * 18.5)
_ENDOSCOPE_BEND_LIMIT = np.pi / (2 * 185)

VIRTUAL_LOWER = {
    "BR": -_ARM_BEND_LIMIT,
    "RR": np.radians(-160),
    "TR": -10,
    "BL": -_ARM_BEND_LIMIT,
    "RL": np.radians(-160),
    "TL": -10,
    "EBV": -_ENDOSCOPE_BEND_LIMIT,
    "EBH": -_ENDOSCOPE_BEND_LIMIT,
    "ER": np.radians(-90),
    "ET": -100,
}
VIRTUAL_UPPER = {
    "BR": _ARM_BEND_LIMIT,
    "RR": np.radians(160),
    "TR": 72,
    "BL": _ARM_BEND_LIMIT,
    "RL": np.radians(160),
    "TL": 72,
    "EBV": _ENDOSCOPE_BEND_LIMIT,
    "EBH": _ENDOSCOPE_BEND_LIMIT,
    "ER": np.radians(90),
    "ET": 95,
}


@dataclass(frozen=True, eq=False)
class JointLimits:
    """Lower and upper bounds ordered by the corresponding joint labels."""

    labels: Sequence[str]
    lower: np.ndarray
    upper: np.ndarray

    def __post_init__(self) -> None:
        lower = np.asarray(self.lower, dtype=np.float64)
        upper = np.asarray(self.upper, dtype=np.float64)
        if len(self.labels) != len(lower) or len(self.labels) != len(upper):
            raise ValueError("labels, lower and upper must have equal length")
        if not np.all(np.isfinite(lower)) or not np.all(np.isfinite(upper)):
            raise ValueError("joint limits must be finite")
        if np.any(lower > upper):
            raise ValueError("each lower joint limit must be <= its upper limit")
        object.__setattr__(self, "labels", tuple(self.labels))
        object.__setattr__(self, "lower", lower)
        object.__setattr__(self, "upper", upper)

    @classmethod
    def from_dicts(
        cls,
        lower: Mapping[str, float],
        upper: Mapping[str, float],
        order: Sequence[str] = QP_ORDER,
    ) -> JointLimits:
        labels = tuple(order)
        return cls(
            labels,
            np.array([lower[label] for label in labels], dtype=np.float64),
            np.array([upper[label] for label in labels], dtype=np.float64),
        )


@dataclass(frozen=True, eq=False)
class RobotLimits:
    """Virtual STRAS joint ranges; contains no physical-controller limits."""

    virtual_cs: JointLimits

    @classmethod
    def default(cls) -> RobotLimits:
        return cls(JointLimits.from_dicts(VIRTUAL_LOWER, VIRTUAL_UPPER))
