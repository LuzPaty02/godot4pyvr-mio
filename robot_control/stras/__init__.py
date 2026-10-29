"""EASE / STRAS follower package.

Layout (each module has one job):

    constants.py      protocol enums, axis labels, qp ordering
    errors.py         exception hierarchy
    config.py         locating and reading the config text files
    limits.py         JointLimits / RobotLimits (all limit tables in one place)
    protocol.py       pure functions: build and parse wire messages
    transport.py      socket wrapper (connect / send / request)
    state.py          RobotState, AxisRanges, parse_state, parse_axis_infos
    kinematics.py     wrapper around StrasModel, returns ModelOutput
    visualizer_base.py    VisualizerLike: the one-method renderer interface
    godot_visualization.py  GodotVisualizer: streams geometry to Godot
    experiment.py     experiment / teleoperation flags (not hardware state)
    follower.py       Follower, RealFollower, VirtualFollower, make_follower
    compat.py         legacy `EASEfollower` facade with the old method names
"""
from .errors import (ConnectionFailed, ProtocolError, StrasError,
                     TransportError, TransportTimeout)
from .follower import Follower, RealFollower, VirtualFollower, make_follower
from .limits import JointLimits, RobotLimits
from .state import AxisRanges, AxisState, RobotState
from .transport import StrasTransport

__all__ = [
    "AxisRanges", "AxisState", "ConnectionFailed", "ExperimentState",
    "Follower", "JointLimits", "ProtocolError", "RealFollower", "RobotLimits",
    "RobotState", "StrasError", "StrasTransport", "TransportError",
    "TransportTimeout", "VirtualFollower", "make_follower",
]