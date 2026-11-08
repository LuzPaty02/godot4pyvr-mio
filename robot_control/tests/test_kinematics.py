import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from robot_control.stras.simulation.constants import QP_ORDER
from robot_control.stras.model.kinematics import KINEMATICS_ORDER, Kinematics
from robot_control.stras.model.model_factory import (
    build_camera_calibration,
    build_kinematics,
)
from robot_control.stras.input.mapping_and_limits.mapping_math import (
    ARM_INSERTION_HOME_MM,
)
from module_main.model_utilities.robot_model import robot_model, robot_twobending
from module_main.model_utilities.stras_model import StrasModel


class ShapeModel:
    """Small StrasModel substitute that records its configuration input."""

    def __init__(self, include_endoscope_shape=True):
        self.include_endoscope_shape = include_endoscope_shape
        self.received_q = None
        self.right = np.arange(30, dtype=np.float64).reshape(10, 3)
        self.left = self.right + 30
        self.endoscope = self.right + 60
        self.tip = np.eye(4, dtype=np.float64)

    def computemodels(self, _q):
        raise NotImplementedError

    def computeShape(self, q):
        self.received_q = np.array(q, copy=True)
        result = ([self.right], [self.left], self.tip)
        if self.include_endoscope_shape:
            return (*result, [self.endoscope])
        return result


class ModelOutputModel:
    """Fake legacy model with distinct endoscope- and world-frame poses."""

    def computemodels(self, _q):
        right_e = np.eye(4)
        right_e[0, 3] = 1.0
        right_w = np.eye(4)
        right_w[0, 3] = 11.0
        left_e = np.eye(4)
        left_e[1, 3] = 2.0
        left_w = np.eye(4)
        left_w[1, 3] = 12.0
        endoscope = np.eye(4)
        return (
            right_e, np.zeros((3, 3)), left_e, np.zeros((3, 3)),
            right_w, np.ones((3, 3)), left_w, np.ones((3, 3)) * 2,
            endoscope, np.ones((3, 3)) * 3, np.zeros((3, 3)),
            np.array([0.0, 0.0, 1.0]),
        )


def test_compute_preserves_world_frame_arm_poses():
    output = Kinematics(ModelOutputModel()).compute(np.zeros(len(QP_ORDER)))

    np.testing.assert_array_equal(output.Tr_w, output.Tr_0)
    np.testing.assert_array_equal(output.Tl_w, output.Tl_0)
    assert output.Tr_w[0, 3] == 11.0
    assert output.Tl_w[1, 3] == 12.0
    assert not output.Tr_w.flags.writeable
    assert not output.Tl_w.flags.writeable


def test_kinematics_uses_qp_order_for_legacy_model_input():
    assert KINEMATICS_ORDER == QP_ORDER == (
        "BR", "RR", "TR", "BL", "RL", "TL", "EBV", "EBH", "ER", "ET"
    )

    values_by_joint = {joint: float(index + 1)
                       for index, joint in enumerate(KINEMATICS_ORDER)}
    q = np.array([values_by_joint[joint] for joint in QP_ORDER])
    original_q = q.copy()
    model = ShapeModel()

    Kinematics(model).shape(q)

    np.testing.assert_array_equal(model.received_q, original_q)
    np.testing.assert_array_equal(q, original_q)
    assert model.received_q[6] == values_by_joint["EBV"]
    assert model.received_q[7] == values_by_joint["EBH"]


def test_shape_converts_all_legacy_geometry_to_read_only_arrays():
    model = ShapeModel()
    output = Kinematics(model).shape(np.zeros(len(QP_ORDER)))

    np.testing.assert_array_equal(output.right_sections[0], model.right)
    np.testing.assert_array_equal(output.left_sections[0], model.left)
    np.testing.assert_array_equal(output.endoscope_sections[0], model.endoscope)
    np.testing.assert_array_equal(output.tip_e, model.tip)
    np.testing.assert_array_equal(output.right, model.right)
    np.testing.assert_array_equal(output.left, model.left)

    for array in (
        output.right_sections[0], output.left_sections[0],
        output.endoscope_sections[0], output.tip_e,
    ):
        assert not array.flags.writeable


def test_shape_without_endoscope_geometry_returns_none():
    model = ShapeModel(include_endoscope_shape=False)

    output = Kinematics(model).shape(np.zeros(len(QP_ORDER)))

    assert output.endoscope_sections is None


def test_robot_piecewise_arc_sampling_reaches_robot_model_tip():
    model_dir = ROOT / "module_main" / "model_utilities"
    model = robot_model(str(model_dir / "robot_config_right.txt"))
    q = np.array([0.008, 0.45, 35.0])

    sections = model.computeShape(q)
    expected_tip = model.computeModel(q).as_matrix()[:3, 3]

    assert len(sections) == model.nsections
    assert all(section.shape == (10, 3) for section in sections)
    np.testing.assert_allclose(sections[-1][-1], expected_tip, atol=1e-8)


def test_arm_shapes_follow_endoscope_pose_and_match_world_fk():
    model_dir = ROOT / "module_main" / "model_utilities"
    model = StrasModel(
        robot_model(str(model_dir / "robot_config_right.txt")),
        robot_model(str(model_dir / "robot_config_left.txt")),
        robot_twobending(str(model_dir / "robot_config_endoscope.txt")),
    )
    model.defineEndoscopeShapeOn()
    kinematics = Kinematics(model)
    q = np.array([0.008, 0.4, 35.0, -0.006, -0.2, 45.0,
                  0.001, -0.001, 0.2, 20.0])

    shape = kinematics.shape(q)
    poses = kinematics.compute(q)

    np.testing.assert_allclose(
        shape.right_sections[-1][-1], poses.Tr_w[:3, 3], atol=1e-8)
    np.testing.assert_allclose(
        shape.left_sections[-1][-1], poses.Tl_w[:3, 3], atol=1e-8)
    np.testing.assert_allclose(
        shape.endoscope_sections[-1][-1], shape.tip_e[:3, 3], atol=1e-8)


def test_endoscope_rotation_dof_rotates_bend_plane_when_endoscope_is_bent():
    model_dir = ROOT / "module_main" / "model_utilities"
    model = StrasModel(
        robot_model(str(model_dir / "robot_config_right.txt")),
        robot_model(str(model_dir / "robot_config_left.txt")),
        robot_twobending(str(model_dir / "robot_config_endoscope.txt")),
    )
    kinematics = Kinematics(model)
    straight = np.array([0.0, 0.0, 35.0, 0.0, 0.0, 35.0,
                         0.0, 0.0, 0.0, 20.0])
    straight_rotated = straight.copy()
    straight_rotated[8] = 0.3
    straight_pose = kinematics.shape(straight).tip_e
    straight_rotated_pose = kinematics.shape(straight_rotated).tip_e
    np.testing.assert_allclose(straight_pose, straight_rotated_pose, atol=1e-8)

    bent = straight.copy()
    bent[6] = 0.004
    bent_rotated = bent.copy()
    bent_rotated[8] = 0.3
    bent_pose = kinematics.shape(bent).tip_e
    bent_rotated_pose = kinematics.shape(bent_rotated).tip_e

    assert not np.allclose(bent_pose, bent_rotated_pose)


def test_visible_home_pose_projects_both_distal_arms_inside_floating_screen():
    kinematics = build_kinematics()
    calibration = build_camera_calibration(ROOT / "module_main")
    q = np.zeros(len(QP_ORDER))
    q[QP_ORDER.index("TR")] = ARM_INSERTION_HOME_MM
    q[QP_ORDER.index("TL")] = ARM_INSERTION_HOME_MM
    shape = kinematics.shape(q)

    camera_rotation = shape.tip_e[:3, :3]
    camera_origin = shape.tip_e[:3, 3]
    width, height = map(int, calibration.imgSize)
    scale = 1.5  # StrasReceiver's default simulation_view_scale.
    for sections in (shape.right_sections, shape.left_sections):
        points = np.concatenate(sections[1:], axis=0)
        camera_points = (points - camera_origin) @ camera_rotation
        assert np.all(camera_points[:, 2] > 0.01)  # Godot viewport near plane, mm.
        u = (calibration.f[0] * camera_points[:, 0] / camera_points[:, 2]
             + calibration.c[0])
        v = (calibration.f[1] * camera_points[:, 1] / camera_points[:, 2]
             + calibration.c[1])
        u = calibration.c[0] + (u - calibration.c[0]) / scale
        v = calibration.c[1] + (v - calibration.c[1]) / scale
        inside = ((0.0 <= u) & (u < width) & (0.0 <= v) & (v < height))
        assert inside.sum() >= points.shape[0] * 0.8
