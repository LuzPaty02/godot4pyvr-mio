import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from robot_control.stras.constants import QP_ORDER
from robot_control.stras.input.mapping_and_limits.input_mapping import (
    ARM_BEND_FULL_SCALE_M,
    ControllerToConfiguration,
)
from robot_control.stras.input.mapping_and_limits.limits import RobotLimits


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
    limits = RobotLimits.default().virtual_cs
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
    bend_ratio = np.hypot(0.05, 0.05) / ARM_BEND_FULL_SCALE_M
    assert np.isclose(q[index["BR"]], limits.lower[index["BR"]] * bend_ratio)
    assert np.isclose(q[index["RR"]], np.pi / 4.0)
    # Forward is -Z at yaw 0: moving the right hand forward extends its arm.
    assert np.isclose(q[index["TR"]], limits.upper[index["TR"]] * 0.2)
    assert np.isclose(q[index["BL"]], limits.upper[index["BL"]] * bend_ratio)
    assert np.isclose(q[index["RL"]], np.pi / 4.0)
    # Moving the left hand toward the torso retracts its arm to the lower limit.
    assert np.isclose(q[index["TL"]], limits.lower[index["TL"]] * 0.4)
    assert q[index["EBH"]] == limits.upper[index["EBH"]]
    assert q[index["EBV"]] == limits.lower[index["EBV"]] / 2
    assert q[index["ER"]] == limits.upper[index["ER"]] / 2
    assert q[index["ET"]] == limits.upper[index["ET"]] / 2  # camera forward is -Z


def test_hand_up_and_down_both_create_visible_bends_in_screen_direction():
    limits = RobotLimits.default().virtual_cs
    mapper = ControllerToConfiguration(limits)
    mapper.update(frame())

    up = mapper.update(frame(left=(0.0, 0.05, 0.0), right=(0.0, 0.05, 0.0)))
    down = mapper.update(frame(left=(0.0, -0.05, 0.0), right=(0.0, -0.05, 0.0)))

    for curvature_joint, rotation_joint in (("BR", "RR"), ("BL", "RL")):
        assert up[QP_ORDER.index(curvature_joint)] > 0.0
        assert down[QP_ORDER.index(curvature_joint)] > 0.0
        assert np.isclose(up[QP_ORDER.index(rotation_joint)], -np.pi / 2.0)
        assert np.isclose(down[QP_ORDER.index(rotation_joint)], np.pi / 2.0)


def test_head_yaw_maps_signed_deltas_to_endoscope_rotation_including_wraparound():
    limits = RobotLimits.default().virtual_cs
    mapper = ControllerToConfiguration(limits)
    mapper.update(frame(yaw=np.pi - 0.1))

    turned_left = mapper.update(frame(yaw=-np.pi + 0.1))
    turned_right = mapper.update(frame(yaw=np.pi - 0.3))

    assert np.isclose(turned_left[QP_ORDER.index("ER")], 0.2)
    assert np.isclose(turned_right[QP_ORDER.index("ER")], -0.2)


def test_outward_left_and_right_hand_motion_uses_mirrored_curvature_signs():
    limits = RobotLimits.default().virtual_cs
    mapper = ControllerToConfiguration(limits)
    mapper.update(frame())

    q = mapper.update(frame(left=(-0.05, 0.0, 0.0), right=(0.05, 0.0, 0.0)))

    assert q[QP_ORDER.index("BR")] > 0.0
    assert q[QP_ORDER.index("BL")] < 0.0
    assert np.isclose(
        q[QP_ORDER.index("BR")],
        limits.upper[QP_ORDER.index("BR")] * 0.05 / ARM_BEND_FULL_SCALE_M,
    )
    assert np.isclose(
        q[QP_ORDER.index("BL")],
        limits.lower[QP_ORDER.index("BL")] * 0.05 / ARM_BEND_FULL_SCALE_M,
    )


def test_mapping_clamps_offsets_and_recalibrates_reconnected_controllers():
    limits = RobotLimits.default().virtual_cs
    mapper = ControllerToConfiguration(limits)
    mapper.update(frame())

    outside_range = mapper.update(frame(right=(1.0, 1.0, 1.0)))
    assert outside_range[QP_ORDER.index("BR")] == limits.upper[QP_ORDER.index("BR")]
    assert np.isclose(outside_range[QP_ORDER.index("RR")], -np.pi / 4.0)
    assert outside_range[QP_ORDER.index("TR")] == limits.lower[QP_ORDER.index("TR")]

    disconnected = mapper.update(frame(right_connected=False))
    assert all(disconnected[QP_ORDER.index(label)] == 0.0
               for label in ("BR", "RR", "TR"))

    # The first frame after reconnect establishes a fresh neutral pose.
    reconnected = mapper.update(frame(right=(0.8, 0.2, -0.3)))
    assert all(reconnected[QP_ORDER.index(label)] == 0.0
               for label in ("BR", "RR", "TR"))


def test_arm_insertion_uses_initial_user_forward_direction():
    limits = RobotLimits.default().virtual_cs
    mapper = ControllerToConfiguration(limits)
    mapper.update(frame(camera=(0.0, 0.0, 0.0), yaw=np.pi / 2.0))

    # At yaw=90 degrees, Godot's camera faces -X. Moving hands -X is forward
    # and extends; moving along user-right (-Z) bends in the positive direction.
    q = mapper.update(frame(
        left=(-0.1, 0.0, -0.1), right=(-0.1, 0.0, -0.1),
        camera=(0.0, 0.0, 0.0), yaw=np.pi / 2.0,
    ))

    assert np.isclose(q[QP_ORDER.index("TR")], limits.upper[QP_ORDER.index("TR")] * 0.4)
    assert np.isclose(q[QP_ORDER.index("TL")], limits.upper[QP_ORDER.index("TL")] * 0.4)
    assert np.isclose(
        q[QP_ORDER.index("BR")],
        limits.upper[QP_ORDER.index("BR")] * 0.1 / ARM_BEND_FULL_SCALE_M,
    )
    assert np.isclose(
        q[QP_ORDER.index("BL")],
        limits.upper[QP_ORDER.index("BL")] * 0.1 / ARM_BEND_FULL_SCALE_M,
    )