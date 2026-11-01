import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from robot_control.stras.constants import QP_ORDER
from robot_control.stras.kinematics import ShapeOutput
from robot_control.stras.limits import RobotLimits
from robot_control.fk_runner import build_geometry_packet, normalized_joints_to_q


def test_normalized_joint_positions_map_to_virtual_range_in_qp_order():
    limits = RobotLimits.default(ROOT / "module_main")
    values = {joint: index / (len(QP_ORDER) - 1) for index, joint in enumerate(QP_ORDER)}

    q = normalized_joints_to_q(values, limits)

    expected = limits.virtual_cs.lower + np.linspace(
        0.0, 1.0, len(QP_ORDER)
    ) * (limits.virtual_cs.upper - limits.virtual_cs.lower)
    np.testing.assert_allclose(q, expected)


@pytest.mark.parametrize("values", [
    {},
    {**{joint: 0.5 for joint in QP_ORDER}, "EXTRA": 0.0},
    {**{joint: 0.5 for joint in QP_ORDER}, "BR": float("nan")},
    {**{joint: 0.5 for joint in QP_ORDER}, "BR": True},
    {**{joint: 0.5 for joint in QP_ORDER}, "BR": 1.1},
])
def test_invalid_normalized_joint_positions_are_rejected(values):
    with pytest.raises(ValueError):
        normalized_joints_to_q(values, RobotLimits.default(ROOT / "module_main"))


def test_geometry_packet_matches_godot_receiver_protocol():
    shape = ShapeOutput(
        right_sections=(np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]]),),
        left_sections=(np.array([[7.0, 8.0, 9.0], [10.0, 11.0, 12.0]]),),
        tip_e=np.eye(4),
        endoscope_sections=(np.array([[13.0, 14.0, 15.0], [16.0, 17.0, 18.0]]),),
    )

    packet = build_geometry_packet(42, shape)

    assert packet == {
        "v": 1,
        "frame": 42,
        "right": [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]],
        "left": [[7.0, 8.0, 9.0], [10.0, 11.0, 12.0]],
        "endoscope": [[13.0, 14.0, 15.0], [16.0, 17.0, 18.0]],
        "right_sections": [[[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]]],
        "left_sections": [[[7.0, 8.0, 9.0], [10.0, 11.0, 12.0]]],
        "endoscope_sections": [[[13.0, 14.0, 15.0], [16.0, 17.0, 18.0]]],
        "camera": {"pose": np.eye(4).tolist(), "intrinsics": None},
    }


def test_geometry_packet_includes_calibrated_endoscope_camera():
    class Calibration:
        f = np.array([500.0, 510.0])
        c = np.array([320.0, 240.0])
        imgSize = np.array([640, 480])

    shape = ShapeOutput(
        (np.zeros((2, 3)),), (np.zeros((2, 3)),), np.eye(4))

    packet = build_geometry_packet(1, shape, Calibration())

    assert packet["camera"] == {
        "pose": np.eye(4).tolist(),
        "intrinsics": {
            "fx": 500.0, "fy": 510.0, "cx": 320.0, "cy": 240.0,
            "width": 640, "height": 480,
        },
    }