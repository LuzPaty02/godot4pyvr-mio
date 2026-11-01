"""Typed, alias-safe wrapper around ``StrasModel``.

The legacy model classes mutate the arrays they are given
(``robot_model.setJointValues`` clamps ``q[2]`` in place, and
``robot_twobending.computeJacobian_new`` does ``self.q[i] += dq`` on the
caller's array). Everything here therefore works on a private float64 copy and
returns read-only arrays, so no caller can alias model internals.

q is the 10-vector in *configuration-space order*::
q is the 10-vector in *configuration-space / QP order*::

    [BR, RR, TR,  BL, RL, TL,  EBV, EBH, ER, ET]
     right arm    left arm     endoscope

Limits are NOT enforced here; that is ``limits.py``'s job. Note the model
itself still clamps TR/TL translation to ``0..lengths[0]`` internally.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Protocol

import numpy as np
from .constants import QP_ORDER

# The legacy StrasModel consumes q in the same order used by the QP and
# follower. Keep this as an alias rather than a second independently-maintained
# joint-order declaration.
KINEMATICS_ORDER = QP_ORDER
N_JOINTS = len(KINEMATICS_ORDER)


class StrasLike(Protocol):
    """What Kinematics needs from the legacy ``StrasModel`` (duck-typed so
    tests can substitute a fake)."""

    def computemodels(self, q: Any) -> tuple: ...
    def computeShape(self, q: Any) -> tuple: ...


def _ro(a: Any) -> np.ndarray:
    out = np.array(a, dtype=np.float64, copy=True)
    out.setflags(write=False)
    return out


@dataclass(frozen=True)
class ModelOutput:
    """Poses (4x4) and position Jacobians (3xn) of each instrument.

    Suffix ``_e``: expressed in the endoscope frame. Suffix ``_0``: fixed frame.
    ``Te``/``Je``: the endoscope itself in the fixed frame.
    ``zr_e``: right-arm tip tangent in the endoscope frame.

    Dropped on purpose: legacy ``Jzr_e`` (tip-tangent Jacobian). The arm model
    never computes it (TODO upstream), so it was always zeros.
    """

    Tr_e: np.ndarray
    Jr_e: np.ndarray
    Tl_e: np.ndarray
    Jl_e: np.ndarray
    Tr_0: np.ndarray
    Jr_0: np.ndarray
    Tl_0: np.ndarray
    Jl_0: np.ndarray
    Te: np.ndarray
    Je: np.ndarray
    zr_e: np.ndarray

    FIELDS = ("Tr_e", "Jr_e", "Tl_e", "Jl_e", "Tr_0", "Jr_0",
              "Tl_0", "Jl_0", "Te", "Je", "zr_e")

    def as_legacy_tuple(self) -> tuple:
        """First ten entries, in the order ``StrasModel.computemodels`` returns."""
        return tuple(getattr(self, f) for f in self.FIELDS[:10])


@dataclass(frozen=True)
class ShapeOutput:
    """Polyline geometry for rendering.

    ``*_sections``: one (10, 3) array per model section, in the endoscope frame
    for the arms. ``tip_e``: 4x4 endoscope tip pose. ``endoscope_sections`` is
    only filled when the underlying model has ``endoscopeShapeFlag`` on.
    """

    right_sections: tuple[np.ndarray, ...]
    left_sections: tuple[np.ndarray, ...]
    tip_e: np.ndarray
    endoscope_sections: tuple[np.ndarray, ...] | None = None

    @property
    def right(self) -> np.ndarray:
        return np.concatenate(self.right_sections, axis=0)

    @property
    def left(self) -> np.ndarray:
        return np.concatenate(self.left_sections, axis=0)


def _sections(shape: Any) -> tuple[np.ndarray, ...]:
    return tuple(_ro(np.asarray(sec, dtype=np.float64).reshape(-1, 3)) for sec in shape)


def _as_matrix(T: Any) -> np.ndarray:
    return _ro(T.as_matrix() if hasattr(T, "as_matrix") else T)


class Kinematics:
    def __init__(self, model: StrasLike):
        self._model = model

    @staticmethod
    def prepare_q(q: Any) -> np.ndarray:
        """Validated private copy. Raises ValueError on wrong size / NaN."""
        arr = np.array(q, dtype=np.float64, copy=True)
        if arr.shape != (N_JOINTS,):
            raise ValueError(f"q must have shape ({N_JOINTS},), got {arr.shape}")
        if not np.all(np.isfinite(arr)):
            raise ValueError("q contains NaN or inf")
        return arr

    def compute(self, q: Any) -> ModelOutput:
        out = self._model.computemodels(self.prepare_q(q))
        (Tr_e, Jr_e, Tl_e, Jl_e, Tr_0, Jr_0, Tl_0, Jl_0, Te, Je, _jzr_e, zr_e) = out
        return ModelOutput(*(_ro(x) for x in
                             (Tr_e, Jr_e, Tl_e, Jl_e, Tr_0, Jr_0, Tl_0, Jl_0, Te, Je, zr_e)))

    def shape(self, q: Any) -> ShapeOutput:
        # Fresh copy: computemodels may have clamped/perturbed its own copy.
        out = self._model.computeShape(self.prepare_q(q))
        endo = _sections(out[3]) if len(out) > 3 else None
        return ShapeOutput(_sections(out[0]), _sections(out[1]), _as_matrix(out[2]), endo)