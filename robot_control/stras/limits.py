"""All joint / velocity limit tables in one place.

The original constructor held ~10 overlapping limit tables as separate
attributes.  They are grouped here as named ``JointLimits`` objects inside one
``RobotLimits``.
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Dict, NamedTuple, Optional, Sequence

import numpy as np

from .config import QP_RANGE_FILE, PathLike, module_main_dir, read_key_value_file
from .constants import QP_ORDER, ROTATION_JOINTS

# --- Raw tables (dictionary order matches the original source) ---------------
_BEND_ARM = np.pi / (2 * 18.5)
_BEND_ENDO = np.pi / (2 * 185)

VIRTUAL_LOWER: Dict[str, float] = {
    'BR': -_BEND_ARM, 'RR': np.radians(-160), 'TR': -10,
    'BL': -_BEND_ARM, 'RL': np.radians(-160), 'TL': -10,
    'EBH': -_BEND_ENDO, 'EBV': -_BEND_ENDO,
    'ER': np.radians(-90), 'ET': -100}
VIRTUAL_UPPER: Dict[str, float] = {
    'BR': _BEND_ARM, 'RR': np.radians(160), 'TR': 72,
    'BL': _BEND_ARM, 'RL': np.radians(160), 'TL': 72,
    'EBH': _BEND_ENDO, 'EBV': _BEND_ENDO,
    'ER': np.radians(90), 'ET': 95}

CS_LOWER: Dict[str, float] = {
    'BR': -_BEND_ARM, 'RR': np.radians(-160), 'TR': -200,
    'BL': -_BEND_ARM, 'RL': np.radians(-160), 'TL': -200,
    'EBH': -_BEND_ENDO, 'EBV': -_BEND_ENDO,
    'ER': np.radians(-90), 'ET': -200}
CS_UPPER: Dict[str, float] = {
    'BR': _BEND_ARM, 'RR': np.radians(160), 'TR': 200,
    'BL': _BEND_ARM, 'RL': np.radians(160), 'TL': 200,
    'EBH': _BEND_ENDO, 'EBV': _BEND_ENDO,
    'ER': np.radians(90), 'ET': 200}


class LimitCheck(NamedTuple):
    """Result of a range check.

    ``out_of_range`` is the flag returned by ``module_main.useful.checkInRange``;
    the original ``visualizeFrame`` treats True as "limit reached".
    """
    out_of_range: bool
    per_axis: list


@dataclass(frozen=True, eq=False)
class JointLimits:
    labels: Sequence[str]
    lower: np.ndarray
    upper: np.ndarray

    def __post_init__(self):
        if not (len(self.labels) == len(self.lower) == len(self.upper)):
            raise ValueError("labels, lower and upper must have equal length")

    def __len__(self) -> int:
        return len(self.labels)

    @classmethod
    def from_dicts(cls, lower: Dict[str, float], upper: Dict[str, float],
                   order: Sequence[str] = QP_ORDER) -> "JointLimits":
        return cls(tuple(order),
                   np.array([lower[k] for k in order], dtype=np.float32),
                   np.array([upper[k] for k in order], dtype=np.float32))

    def check(self, q) -> LimitCheck:
        """Range check delegated to the project's ``checkInRange`` helper."""
        if len(q) != len(self):
            raise ValueError(
                f"q has {len(q)} entries, expected {len(self)}")
        from module_main.useful import checkInRange  # lazy: keeps this module light
        per_axis, flag = checkInRange(q, self.lower, self.upper)
        return LimitCheck(flag, per_axis)

    def clip(self, q, skip: Sequence[str] = ROTATION_JOINTS) -> np.ndarray:
        """Return a *copy* of q clipped to the limits (rotations left alone).

        The original ``filterConfigurationState`` modified its argument in place
        and also returned it; this version never mutates the input.
        """
        if len(q) != len(self):
            raise ValueError(
                f"q has {len(q)} entries, expected {len(self)}")
        out = np.array(q, dtype=np.float64, copy=True)
        for i, label in enumerate(self.labels):
            if label in skip:
                continue
            out[i] = min(max(out[i], self.lower[i]), self.upper[i])
        return out


@dataclass
class SegmentVelocityLimits:
    """Per-step velocity limits indexed by configuration-variable position."""
    base: np.ndarray
    factor: float = 1.1          # was q_dot_lim_segFactor
    step_fraction: float = 0.01  # was max_step_q
    safety_factor: float = 0.8

    @property
    def limit(self) -> np.ndarray:
        return self.base * self.factor

    @property
    def max_step(self) -> np.ndarray:
        return self.base * self.step_fraction


@dataclass(frozen=True, eq=False)
class RobotLimits:
    joint_velocity: JointLimits      # lims_down/up_joint
    virtual_cs: JointLimits          # lims_down/up_joint_cs
    cs: JointLimits                  # lowerCSlims / upperCSlims
    q_bound: JointLimits             # q_bound_min / q_bound_max
    segment_velocity: SegmentVelocityLimits
    console: Optional[JointLimits] = None  # from dofDictRange-qp.txt, if present

    @classmethod
    def default(cls, module_main: Optional[PathLike] = None) -> "RobotLimits":
        arm = (np.pi / (18.5 * 2) + np.pi / (18.5 * 2)) * 0.5
        endo = (np.pi / (185 * 2) + np.pi / (185 * 2)) * 0.5
        max_trans_arm, max_trans_endo = 5, 10
        rad_arm, rad_endo = 10, 1
        vel = np.array([
            arm, np.radians(rad_arm), max_trans_arm,    # right arm
            arm, np.radians(rad_arm), max_trans_arm,    # left arm
            endo, endo, np.radians(rad_endo), max_trans_endo])  # endoscope
        # NOTE: the original used the same array for "down" and "up" (the
        # comments say "min ... vel (neg max)").  Preserved here; if a negative
        # lower bound was intended, pass ``-vel`` as the lower array.
        joint_velocity = JointLimits(QP_ORDER, vel.astype(np.float32),
                                     vel.astype(np.float32))

        q_bound = JointLimits(
            QP_ORDER,
            np.array([(-np.pi / 37) * 0.83, -163, 24,
                      -np.pi / 37, -163, 24,
                      -np.pi / 370, -np.pi / 370, -163, -500]),
            np.array([np.pi / 37, 163, 65,
                      np.pi / 37, 163, 65,
                      np.pi / 370, np.pi / 370, 163, 500]))

        seg = SegmentVelocityLimits(np.array([
            0.0085 * 2 * 2, np.radians(200), 54,
            0.0085 * 2 * 2, np.radians(200), 54,
            0.085 * 0.08, -0.085 * 0.08, np.radians(30), 100]))

        return cls(
            joint_velocity=joint_velocity,
            virtual_cs=JointLimits.from_dicts(VIRTUAL_LOWER, VIRTUAL_UPPER),
            cs=JointLimits.from_dicts(CS_LOWER, CS_UPPER),
            q_bound=q_bound,
            segment_velocity=seg,
            console=cls._load_console(module_main))

    @staticmethod
    def _load_console(module_main: Optional[PathLike]) -> Optional[JointLimits]:
        path = Path(module_main_dir(module_main)) / QP_RANGE_FILE
        if not path.exists():
            return None
        raw = read_key_value_file(path)
        try:
            lower = {k: float(raw[k + 'min']) for k in QP_ORDER}
            upper = {k: float(raw[k + 'max']) for k in QP_ORDER}
        except KeyError as exc:
            raise KeyError(f"{path} is missing entry {exc}") from None
        return JointLimits.from_dicts(lower, upper)