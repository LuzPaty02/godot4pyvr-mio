import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from robot_control.stras.simulation.constants import QP_ORDER
from robot_control.stras.input.mapping_and_limits.input_mapping import (
    ARM_BEND_FULL_SCALE_M,
    ARM_INSERTION_HOME_MM,
    ARM_INSERTION_TRAVEL_MM,
    ControllerToConfiguration,
    DEADZONE_M,
)
from robot_control.stras.input.mapping_and_limits.arm_input_mapping import (
    ArmInputMapper,
)
from robot_control.stras.input.mapping_and_limits.camera_input_mapping import (
    CameraInputMapper,
)
from robot_control.stras.input.mapping_and_limits.limits import RobotLimits
from robot_control.stras.input.mapping_and_limits.mapping_math import (
    map_bend,
    scaled_offset,
)


def controller(position, connected=True, orientation=None):
    state = {"connected": connected,
             "position": dict(zip(("x", "y", "z"), position))}
    if orientation is not None:
        state["orientation"] = orientation
    return state


def frame(left=(0.0, 0.0, 0.0), right=(0.0, 0.0, 0.0),
          camera=(0.0, 0.0, 0.0), yaw=0.0, left_connected=True,
          right_connected=True, left_orientation=None, right_orientation=None):
    return {
        "left": controller(left, left_connected, left_orientation),
        "right": controller(right, right_connected, right_orientation),
        "camera": {
            "position": dict(zip(("x", "y", "z"), camera)),
            "yaw": yaw,
        },
    }


def controller_roll(angle):
    """Quaternion for a signed roll around the controller-local -Z axis."""
    return {"x": 0.0, "y": 0.0, "z": -np.sin(angle / 2.0),
            "w": np.cos(angle / 2.0)}


def quaternion_multiply(left, right):
    lx, ly, lz, lw = (left[key] for key in ("x", "y", "z", "w"))
    rx, ry, rz, rw = (right[key] for key in ("x", "y", "z", "w"))
    return {
        "x": lw * rx + lx * rw + ly * rz - lz * ry,
        "y": lw * ry - lx * rz + ly * rw + lz * rx,
        "z": lw * rz + lx * ry - ly * rx + lz * rw,
        "w": lw * rw - lx * rx - ly * ry - lz * rz,
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

    expected_neutral = np.zeros(len(QP_ORDER))
    expected_neutral[QP_ORDER.index("TR")] = ARM_INSERTION_HOME_MM
    expected_neutral[QP_ORDER.index("TL")] = ARM_INSERTION_HOME_MM
    np.testing.assert_array_equal(neutral, expected_neutral)
    index = {label: QP_ORDER.index(label) for label in QP_ORDER}
    bend_ratio = (np.hypot(0.05, 0.05) - DEADZONE_M) / (ARM_BEND_FULL_SCALE_M - DEADZONE_M)
    assert np.isclose(q[index["BR"]], limits.lower[index["BR"]] * bend_ratio)
    assert np.isclose(q[index["RR"]], np.pi / 4.0)
    # Forward is -Z at yaw 0: moving the right hand forward extends its arm.
    right_insertion_offset = 0.05
    assert np.isclose(
        q[index["TR"]], ARM_INSERTION_HOME_MM
        + right_insertion_offset / 0.25 * ARM_INSERTION_TRAVEL_MM)
    assert np.isclose(q[index["BL"]], limits.upper[index["BL"]] * bend_ratio)
    assert np.isclose(q[index["RL"]], np.pi / 4.0)
    # Moving the left hand toward the torso retracts its arm to the lower limit.
    left_insertion_offset = -0.1
    assert np.isclose(
        q[index["TL"]], ARM_INSERTION_HOME_MM
        + left_insertion_offset / 0.25 * ARM_INSERTION_TRAVEL_MM)
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


def test_wrist_roll_maps_symmetrically_to_bend_plane_rotation():
    limits = RobotLimits.default().virtual_cs
    mapper = ControllerToConfiguration(limits)
    mapper.update(frame(left_orientation=controller_roll(0.0),
                        right_orientation=controller_roll(0.0)))

    inward = mapper.update(frame(
        left_orientation=controller_roll(0.4),
        right_orientation=controller_roll(0.4),
    ))
    outward = mapper.update(frame(
        left_orientation=controller_roll(-0.4),
        right_orientation=controller_roll(-0.4),
    ))

    for rotation_joint in ("RR", "RL"):
        assert np.isclose(inward[QP_ORDER.index(rotation_joint)], 0.4)
        assert np.isclose(outward[QP_ORDER.index(rotation_joint)], -0.4)
        assert np.isclose(
            abs(inward[QP_ORDER.index(rotation_joint)]),
            abs(outward[QP_ORDER.index(rotation_joint)]),
        )


def test_wrist_roll_adds_to_position_angle_and_quaternion_sign_is_equivalent():
    mapper = ControllerToConfiguration(RobotLimits.default().virtual_cs)
    mapper.update(frame(right_orientation=controller_roll(0.0)))

    rolled_up = mapper.update(frame(
        right=(0.0, 0.05, 0.0),
        right_orientation=controller_roll(0.2),
    ))
    equivalent_quaternion = {
        key: -value for key, value in controller_roll(0.2).items()
    }
    same_pose = mapper.update(frame(
        right=(0.0, 0.05, 0.0),
        right_orientation=equivalent_quaternion,
    ))

    assert np.isclose(rolled_up[QP_ORDER.index("RR")], -np.pi / 2.0 + 0.2)
    assert np.isclose(same_pose[QP_ORDER.index("RR")], rolled_up[QP_ORDER.index("RR")])


def test_wrist_roll_is_relative_to_neutral_and_ignores_swing():
    neutral = {"x": 0.0, "y": np.sin(0.3), "z": 0.0, "w": np.cos(0.3)}
    current = quaternion_multiply(neutral, controller_roll(0.35))
    mapper = ControllerToConfiguration(RobotLimits.default().virtual_cs)
    mapper.update(frame(right_orientation=neutral))

    twisted = mapper.update(frame(right_orientation=current))
    swing = {"x": np.sin(0.2), "y": 0.0, "z": 0.0, "w": np.cos(0.2)}
    swung_only = mapper.update(frame(
        right_orientation=quaternion_multiply(neutral, swing),
    ))

    assert np.isclose(twisted[QP_ORDER.index("RR")], 0.35)
    assert np.isclose(swung_only[QP_ORDER.index("RR")], 0.0)


def test_arms_start_inserted_and_retain_symmetric_insertion_travel():
    mapper = ControllerToConfiguration(RobotLimits.default().virtual_cs)
    neutral = mapper.update(frame())
    forward = mapper.update(frame(left=(0.0, 0.0, -0.25),
                                  right=(0.0, 0.0, -0.25)))
    backward = mapper.update(frame(left=(0.0, 0.0, 0.25),
                                   right=(0.0, 0.0, 0.25)))

    for insertion_joint in ("TR", "TL"):
        index = QP_ORDER.index(insertion_joint)
        assert np.isclose(neutral[index], ARM_INSERTION_HOME_MM)
        assert np.isclose(forward[index],
                          ARM_INSERTION_HOME_MM + ARM_INSERTION_TRAVEL_MM)
        assert np.isclose(backward[index],
                          ARM_INSERTION_HOME_MM - ARM_INSERTION_TRAVEL_MM)


def test_head_yaw_maps_signed_deltas_to_endoscope_rotation_including_wraparound():
    limits = RobotLimits.default().virtual_cs
    mapper = ControllerToConfiguration(limits)
    mapper.update(frame(yaw=np.pi - 0.1))

    turned_left = mapper.update(frame(yaw=-np.pi + 0.1))
    turned_right = mapper.update(frame(yaw=np.pi - 0.3))

    assert np.isclose(turned_left[QP_ORDER.index("ER")], 0.2)
    assert np.isclose(turned_right[QP_ORDER.index("ER")], -0.2)


def test_camera_mapper_recenter_and_reset_capture_neutral_pose():
    mapper = CameraInputMapper(RobotLimits.default().virtual_cs)
    initial = {"position": {"x": 0.0, "y": 0.0, "z": 0.0}, "yaw": 0.0}
    shifted = {"position": {"x": 0.05, "y": 0.0, "z": 0.0}, "yaw": 0.2}

    mapper.update(initial)
    assert mapper.update(shifted)["EBH"] != 0.0
    mapper.recenter(shifted)
    assert mapper.update(shifted) == {"EBH": 0.0, "EBV": 0.0,
                                      "ET": 0.0, "ER": 0.0}
    assert mapper.neutral_yaw == 0.2

    mapper.reset()
    with pytest.raises(RuntimeError, match="neutral pose has not been captured"):
        _ = mapper.neutral_yaw
    assert mapper.update(initial) == {"EBH": 0.0, "EBV": 0.0,
                                      "ET": 0.0, "ER": 0.0}


@pytest.mark.parametrize(
    "camera",
    [
        {"yaw": 0.0},
        {"position": {"x": 0.0, "y": 0.0, "z": 0.0}},
    ],
)
def test_camera_mapper_requires_position_and_yaw(camera):
    mapper = CameraInputMapper(RobotLimits.default().virtual_cs)

    with pytest.raises(ValueError):
        mapper.update(camera)


def test_outward_left_and_right_hand_motion_uses_mirrored_curvature_signs():
    limits = RobotLimits.default().virtual_cs
    mapper = ControllerToConfiguration(limits)
    mapper.update(frame())

    q = mapper.update(frame(left=(-0.05, 0.0, 0.0), right=(0.05, 0.0, 0.0)))

    assert q[QP_ORDER.index("BR")] > 0.0
    assert q[QP_ORDER.index("BL")] < 0.0
    assert np.isclose(
        q[QP_ORDER.index("BR")],
        limits.upper[QP_ORDER.index("BR")] * (0.05 - DEADZONE_M) / (ARM_BEND_FULL_SCALE_M - DEADZONE_M),
    )
    assert np.isclose(
        q[QP_ORDER.index("BL")],
        limits.lower[QP_ORDER.index("BL")] * (0.05 - DEADZONE_M) / (ARM_BEND_FULL_SCALE_M - DEADZONE_M),
    )


def test_mapping_clamps_offsets_and_recalibrates_reconnected_controllers():
    limits = RobotLimits.default().virtual_cs
    mapper = ControllerToConfiguration(limits)
    mapper.update(frame())

    outside_range = mapper.update(frame(right=(1.0, 1.0, 1.0)))
    assert outside_range[QP_ORDER.index("BR")] == limits.upper[QP_ORDER.index("BR")]
    assert np.isclose(outside_range[QP_ORDER.index("RR")], -np.pi / 4.0)
    assert np.isclose(outside_range[QP_ORDER.index("TR")],
                      ARM_INSERTION_HOME_MM - ARM_INSERTION_TRAVEL_MM)

    disconnected = mapper.update(frame(right_connected=False))
    assert all(disconnected[QP_ORDER.index(label)] == 0.0
               for label in ("BR", "RR", "TR"))

    # The first frame after reconnect establishes a fresh neutral pose.
    reconnected = mapper.update(frame(right=(0.8, 0.2, -0.3)))
    assert reconnected[QP_ORDER.index("BR")] == 0.0
    assert reconnected[QP_ORDER.index("RR")] == 0.0
    assert reconnected[QP_ORDER.index("TR")] == ARM_INSERTION_HOME_MM


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

    assert np.isclose(q[QP_ORDER.index("TR")],
                      ARM_INSERTION_HOME_MM + ARM_INSERTION_TRAVEL_MM * 0.4)
    assert np.isclose(q[QP_ORDER.index("TL")],
                      ARM_INSERTION_HOME_MM + ARM_INSERTION_TRAVEL_MM * 0.4)
    assert np.isclose(
        q[QP_ORDER.index("BR")],
        limits.upper[QP_ORDER.index("BR")] * (0.1 - DEADZONE_M) / (ARM_BEND_FULL_SCALE_M - DEADZONE_M),
    )
    assert np.isclose(
        q[QP_ORDER.index("BL")],
        limits.upper[QP_ORDER.index("BL")] * (0.1 - DEADZONE_M) / (ARM_BEND_FULL_SCALE_M - DEADZONE_M),
    )


def test_bend_radial_deadzone_rescales_continuously_to_full_scale():
    limits = RobotLimits.default().virtual_cs
    lower = float(limits.lower[QP_ORDER.index("BR")])
    upper = float(limits.upper[QP_ORDER.index("BR")])

    at_deadzone, _ = map_bend(DEADZONE_M, 0.0, ARM_BEND_FULL_SCALE_M, lower, upper)
    halfway, _ = map_bend(
        (DEADZONE_M + ARM_BEND_FULL_SCALE_M) / 2.0, 0.0,
        ARM_BEND_FULL_SCALE_M, lower, upper)
    at_full_scale, _ = map_bend(
        ARM_BEND_FULL_SCALE_M, 0.0, ARM_BEND_FULL_SCALE_M, lower, upper)

    assert at_deadzone == 0.0
    assert np.isclose(halfway, upper / 2.0)
    assert np.isclose(at_full_scale, upper)


def test_arm_bend_hysteresis_is_independent_for_each_hand():
    limits = RobotLimits.default().virtual_cs
    mapper = ArmInputMapper(limits)

    def pose_at(angle_degrees):
        angle = np.deg2rad(angle_degrees)
        return (0.1 * np.cos(angle), -0.1 * np.sin(angle), 0.0)

    mapper.update(frame(), camera_yaw=0.0)
    mapper.update(frame(left=pose_at(85.0), right=pose_at(85.0)), camera_yaw=0.0)
    q = mapper.update(frame(left=pose_at(95.0), right=pose_at(101.0)), camera_yaw=0.0)

    assert q["BR"] < 0.0  # Right hand crossed the margin and changed branch.
    assert q["RR"] == pytest.approx(np.deg2rad(-79.0))
    assert q["BL"] > 0.0  # Left hand retains the positive branch within the margin.
    assert q["RL"] == pytest.approx(np.deg2rad(95.0))

    q = mapper.update(frame(left=pose_at(79.0), right=pose_at(95.0)), camera_yaw=0.0)
    assert q["BR"] < 0.0  # The right hand keeps its new branch on the return path.
    assert q["BL"] > 0.0  # The left hand switches only after crossing back by 10 degrees.


@pytest.mark.parametrize("full_scale", [0.0, -0.1])
def test_scaling_functions_reject_nonpositive_full_scale(full_scale):
    with pytest.raises(ValueError, match="full_scale"):
        scaled_offset(0.0, full_scale, -1.0, 1.0)
    with pytest.raises(ValueError, match="full_scale"):
        map_bend(0.0, 0.0, full_scale, -1.0, 1.0)


@pytest.mark.parametrize(
    "camera_yaw", [float("nan"), float("inf"), -float("inf"), True, "0"]
)
def test_arm_mapper_rejects_invalid_camera_yaw_without_mutating_state(camera_yaw):
    mapper = ArmInputMapper(RobotLimits.default().virtual_cs)
    before_neutral = mapper._hand_neutral.copy()
    before_branch = mapper._bend_branch.copy()

    with pytest.raises(ValueError, match="camera_yaw"):
        mapper.update(frame(), camera_yaw=camera_yaw)

    assert mapper._hand_neutral == before_neutral
    assert mapper._bend_branch == before_branch


def test_arm_mapper_does_not_commit_right_hand_when_left_hand_is_invalid():
    mapper = ArmInputMapper(RobotLimits.default().virtual_cs)
    mapper.update(frame(), camera_yaw=0.0)
    before_neutral = mapper._hand_neutral.copy()
    before_branch = mapper._bend_branch.copy()
    payload = frame(right=(0.1, 0.0, 0.0))
    payload["left"]["position"] = {"x": "invalid", "y": 0.0, "z": 0.0}

    with pytest.raises(ValueError, match="left.position"):
        mapper.update(payload, camera_yaw=0.0)

    assert mapper._hand_neutral == before_neutral
    assert mapper._bend_branch == before_branch