import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from robot_control.stras.constants import QP_ORDER
from robot_control.stras.follower import VirtualFollower
from robot_control.stras.limits import RobotLimits


def test_virtual_follower_stores_q_in_qp_order_without_aliasing():
    limits = RobotLimits.default()
    follower = VirtualFollower(limits=limits)
    q = (limits.virtual_cs.lower + limits.virtual_cs.upper) / 2.0
    expected = q.copy()

    follower.set_q(q)
    q[0] = -1.0

    np.testing.assert_array_equal(follower.q_config_space, expected)


@pytest.mark.parametrize("q", [
    np.zeros(len(QP_ORDER) - 1),
    np.full(len(QP_ORDER), np.nan),
    np.full(len(QP_ORDER), np.inf),
])
def test_virtual_follower_rejects_malformed_or_non_finite_q(q):
    follower = VirtualFollower()

    with pytest.raises(ValueError):
        follower.set_q(q)


def test_virtual_follower_rejects_q_outside_simulation_limits():
    follower = VirtualFollower()
    q = np.zeros(len(QP_ORDER))
    q[0] = follower.limits.virtual_cs.upper[0] + 1.0

    with pytest.raises(ValueError, match="outside the virtual simulation limits"):
        follower.set_q(q)
