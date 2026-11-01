import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from robot_control.stras.constants import QP_ORDER
from robot_control.stras.kinematics import KINEMATICS_ORDER, Kinematics
from module_main.model_utilities.robot_model import robot_model


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
