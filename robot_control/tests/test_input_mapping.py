import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from robot_control.stras.constants import QP_ORDER
from robot_control.stras.input_mapping import ControllerToConfiguration
from robot_control.stras.limits import RobotLimits


def controller(position, connected=True):
    return {"connected": connected, "position": dict(zip(("x", "y", "z"), position))}


def frame(left=(0.0, 0.0, 0.0), right=(0.0, 0.0, 0.0),
          camera=(0.0, 0.0, 0.0), yaw=0.0, left_connected=True,
          right_connected=True):
    return {
        "left": controller(left, left_connected),
        "right": controller(right, right_connected),
        "camera": {
            "position": dict(zip(("x", "y", "z"), camera)),
            "yaw": yaw,
        },
    }


def test_pose_offsets_map_to_expected_q_axes_and_units():
    limits = RobotLimits.default(ROOT / "module_main").virtual_cs
    mapper = ControllerToConfiguration(limits)

    neutral = mapper.update(frame())
    q = mapper.update(frame(
        left=(0.05, -0.05, 0.1),
        right=(-0.05, 0.05, -0.05),
        camera=(0.1, -0.05, -0.05),
        yaw=np.pi / 4.0,
    ))

    np.testing.assert_array_equal(neutral, np.zeros(len(QP_ORDER)))
    index = {label: QP_ORDER.index(label) for label in QP_ORDER}
    assert q[index["BR"]] == limits.lower[index["BR"]] / 2
    assert q[index["RR"]] == limits.upper[index["RR"]] / 2
    assert q[index["TR"]] == limits.lower[index["TR"]] / 2
    assert q[index["BL"]] == limits.upper[index["BL"]] / 2
    assert q[index["RL"]] == limits.lower[index["RL"]] / 2
    assert q[index["TL"]] == limits.upper[index["TL"]]
    assert q[index["EBH"]] == limits.upper[index["EBH"]]
    assert q[index["EBV"]] == limits.lower[index["EBV"]] / 2
    assert q[index["ER"]] == limits.upper[index["ER"]] / 2
    assert q[index["ET"]] == limits.upper[index["ET"]] / 2  # camera forward is -Z


def test_mapping_clamps_offsets_and_recalibrates_reconnected_controllers():
    limits = RobotLimits.default(ROOT / "module_main").virtual_cs
    mapper = ControllerToConfiguration(limits)
    mapper.update(frame())

    outside_range = mapper.update(frame(right=(1.0, 1.0, 1.0)))
    for label in ("BR", "RR", "TR"):
        idx = QP_ORDER.index(label)
        assert outside_range[idx] == limits.upper[idx]

    disconnected = mapper.update(frame(right_connected=False))
    assert all(disconnected[QP_ORDER.index(label)] == 0.0
               for label in ("BR", "RR", "TR"))

    # The first frame after reconnect establishes a fresh neutral pose.
    reconnected = mapper.update(frame(right=(0.8, 0.2, -0.3)))
    assert all(reconnected[QP_ORDER.index(label)] == 0.0
               for label in ("BR", "RR", "TR"))