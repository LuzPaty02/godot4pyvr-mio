"""VR-driven STRAS simulation components.

The active pipeline maps XR poses to virtual joint positions, computes model
kinematics, and sends geometry to Godot. It contains no robot transport or
physical STRAS command interface.
"""

from .constants import QP_ORDER
from .follower import VirtualFollower
from .input_mapping import ControllerToConfiguration
from .kinematics import Kinematics, ModelOutput, ShapeOutput
from .limits import JointLimits, RobotLimits

__all__ = [
    "ControllerToConfiguration",
    "JointLimits",
    "Kinematics",
    "ModelOutput",
    "QP_ORDER",
    "RobotLimits",
    "ShapeOutput",
    "VirtualFollower",
]
