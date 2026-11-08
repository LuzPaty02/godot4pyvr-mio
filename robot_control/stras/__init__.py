"""VR-driven STRAS simulation components.

The active pipeline maps XR poses to virtual joint positions, computes model
kinematics, and sends geometry to Godot. It contains no robot transport or
physical STRAS command interface.
"""

from .simulation.constants import QP_ORDER
from .simulation.virtual_follower import VirtualFollower
from .input.mapping_and_limits.input_mapping import ControllerToConfiguration
from .model.kinematics import Kinematics, ModelOutput, ShapeOutput
from .input.mapping_and_limits.limits import JointLimits, RobotLimits

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
