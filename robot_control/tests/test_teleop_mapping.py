import os, sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from robot_control.teleop_mapping import (DEFAULT_BINDINGS, ENDOSCOPE_BINDINGS, Binding,
                            TeleopMapper, apply_deadzone)

QP_ORDER = ("BR", "RR", "TR", "BL", "RL", "TL", "EBV", "EBH", "ER", "ET")
ENDO = {"ET", "ER", "EBV", "EBH"}


def hand(trigger=0.0, grip=0.0, p=(0.0, 0.0), s=(0.0, 0.0)):
    return {"connected": True, "trigger": trigger, "grip": grip,
            "primary": {"x": p[0], "y": p[1]}, "secondary": {"x": s[0], "y": s[1]}}


def payload(left=None, right=None):
    return {"left": left or hand(), "right": right or hand()}


def approx(a, b):
    return abs(a - b) < 1e-9


def mapper(**kw):
    return TeleopMapper(DEFAULT_BINDINGS, known_joints=QP_ORDER, **kw)


def test_deadzone():
    assert apply_deadzone(0.05, 0.1) == 0.0
    assert apply_deadzone(-0.1, 0.1) == 0.0
    assert apply_deadzone(1.0, 0.1) == 1.0 and apply_deadzone(-1.0, 0.1) == -1.0
    assert approx(apply_deadzone(0.55, 0.1), 0.5)
    assert apply_deadzone(2.0, 0.1) == 1.0
    assert apply_deadzone(0.5, 0.0) == 0.5


def test_default_drives_exactly_the_four_endoscope_joints():
    assert set(mapper().driven_joints) == ENDO


def test_right_stick_insertion_and_rotation():
    m = mapper()
    up = m.map(payload(right=hand(p=(0.0, 1.0))), {"right"}).velocity
    assert up["ET"] == 1.0 and up["ER"] == 0.0
    down = m.map(payload(right=hand(p=(0.0, -1.0))), {"right"}).velocity
    assert down["ET"] == -1.0
    right = m.map(payload(right=hand(p=(1.0, 0.0))), {"right"}).velocity
    assert right["ER"] == 1.0 and right["ET"] == 0.0
    left = m.map(payload(right=hand(p=(-1.0, 0.0))), {"right"}).velocity
    assert left["ER"] == -1.0


def test_left_stick_vertical_and_horizontal_bending():
    m = mapper()
    v = m.map(payload(left=hand(p=(0.0, 1.0))), {"left"}).velocity
    assert v["EBV"] == 1.0 and v["EBH"] == 0.0
    v = m.map(payload(left=hand(p=(-1.0, 0.0))), {"left"}).velocity
    assert v["EBH"] == -1.0 and v["EBV"] == 0.0
    v = m.map(payload(left=hand(p=(0.5, -0.5))), {"left"}).velocity       # diagonal: both axes at once
    assert v["EBH"] > 0 and v["EBV"] < 0


def test_full_four_dof_at_once():
    cmd = mapper().map(payload(left=hand(p=(1.0, -1.0)), right=hand(p=(-1.0, 1.0))), {"left", "right"})
    assert cmd.velocity == {"ET": 1.0, "ER": -1.0, "EBV": -1.0, "EBH": 1.0}


def test_each_hand_needs_its_own_deadman():
    m = mapper()
    both = payload(left=hand(p=(1, 1)), right=hand(p=(1, 1)))
    only_right = m.map(both, {"right"}).velocity
    assert only_right["ET"] == 1.0 and only_right["EBV"] == 0.0
    only_left = m.map(both, {"left"}).velocity
    assert only_left["EBV"] == 1.0 and only_left["ET"] == 0.0


def test_disabled_yields_explicit_zeros_for_every_driven_joint():
    m = mapper()
    cmd = m.map(payload(left=hand(p=(1, 1)), right=hand(p=(1, 1))), set())
    assert set(cmd.velocity) == ENDO and all(v == 0.0 for v in cmd.velocity.values())
    assert m.stop_command().velocity == cmd.velocity and cmd.position == {}


def test_stick_drift_inside_deadzone_is_zero():
    cmd = mapper().map(payload(left=hand(p=(0.06, -0.08)), right=hand(p=(0.09, 0.05))), {"left", "right"})
    assert all(v == 0.0 for v in cmd.velocity.values())


def test_axes_normalized_compat_mode_rest_is_zero():
    m = mapper(axes_normalized=True)           # current ControllerInputReader: rest = 0.5
    rest = m.map(payload(left=hand(p=(0.5, 0.5)), right=hand(p=(0.5, 0.5))), {"left", "right"})
    assert all(v == 0.0 for v in rest.velocity.values())
    assert m.map(payload(right=hand(p=(0.5, 1.0))), {"right"}).velocity["ET"] == 1.0
    assert m.map(payload(right=hand(p=(0.5, 0.0))), {"right"}).velocity["ET"] == -1.0


def test_scale_inversion_and_sum_clamp():
    b = [Binding("right", "primary.y", "ET", scale=-1.0)]
    assert TeleopMapper(b).map(payload(right=hand(p=(0, 1.0))), {"right"}).velocity["ET"] == -1.0
    b2 = [Binding("right", "primary.y", "ET"), Binding("right", "secondary.y", "ET")]
    assert TeleopMapper(b2).map(payload(right=hand(p=(0, 1), s=(0, 1))), {"right"}).velocity["ET"] == 1.0
    b3 = [Binding("right", "primary.y", "ET"), Binding("right", "secondary.y", "ET", scale=-1.0)]
    assert TeleopMapper(b3).map(payload(right=hand(p=(0, 1), s=(0, 1))), {"right"}).velocity["ET"] == 0.0


def test_trigger_as_analog_velocity():
    m = TeleopMapper([Binding("left", "trigger", "ET", deadzone=0.0, scale=0.5)])
    assert approx(m.map(payload(left=hand(trigger=0.8)), {"left"}).velocity["ET"], 0.4)


def test_position_mode_and_inversion():
    m = TeleopMapper([Binding("right", "trigger", "ET", mode="position"),
                      Binding("right", "grip", "ER", mode="position", scale=-1.0)])
    cmd = m.map(payload(right=hand(trigger=0.25, grip=0.25)), {"right"})
    assert approx(cmd.position["ET"], 0.25) and approx(cmd.position["ER"], 0.75)
    assert m.driven_joints == ()
    assert m.map(payload(right=hand(trigger=1.0)), set()).position == {}


@pytest.mark.parametrize("bad", [None, float("nan"), "x", True, float("inf")])
def test_garbage_values_are_neutral(bad):
    h = hand()
    h["primary"] = {"x": bad, "y": bad}
    cmd = mapper().map(payload(left=h, right=h), {"left", "right"})
    assert all(v == 0.0 for v in cmd.velocity.values())


def test_missing_fields_are_neutral():
    cmd = mapper().map({"right": {"connected": True}}, {"right"})
    assert all(v == 0.0 for v in cmd.velocity.values())
    assert mapper().map({}, {"left", "right"}).velocity["ET"] == 0.0


def test_out_of_range_input_is_clamped():
    assert mapper().map(payload(right=hand(p=(0, 7.0))), {"right"}).velocity["ET"] == 1.0


@pytest.mark.parametrize("kw", [
    dict(hand="middle", source="grip", joint="ET"),
    dict(hand="left", source="bumper", joint="ET"),
    dict(hand="left", source="grip", joint="ET", mode="teleport"),
    dict(hand="left", source="primary.x", joint="ET", mode="position"),
    dict(hand="left", source="grip", joint="ET", deadzone=1.0),
    dict(hand="left", source="grip", joint=""),
])
def test_binding_validation(kw):
    with pytest.raises(ValueError):
        Binding(**kw)


def test_unknown_joint_rejected_when_known_joints_given():
    with pytest.raises(ValueError):
        TeleopMapper([Binding("left", "grip", "XYZ")], known_joints=QP_ORDER)
    assert all(b.joint in QP_ORDER for b in ENDOSCOPE_BINDINGS)


def test_watchdog_to_mapper_pipeline():
    from input_server import InputMessage
    from robot_control.watchdog import Watchdog
    wd, m = Watchdog(), mapper()
    wd.check(InputMessage(0, payload(), 0.0), 0.0)                       # released -> armed
    live = InputMessage(1, payload(right=hand(grip=1.0, p=(0, 1.0))), 0.1)
    v = wd.check(live, 0.1)
    assert m.map(live.payload, v.enabled_hands).velocity["ET"] == 1.0
    v = wd.check(live, 5.0)                                              # link died
    assert m.map(live.payload, v.enabled_hands).velocity["ET"] == 0.0