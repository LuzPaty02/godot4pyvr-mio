import os, sys, json
import pytest
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from input_server import InputMessage, decode_packet
from watchdog import Reason, Watchdog, WatchdogConfig


def hand(grip=0.0, trigger=0.0, connected=True):
    return {"connected": connected, "trigger": trigger, "grip": grip,
            "primary": {"x": 0.0, "y": 0.0}, "secondary": {"x": 0.0, "y": 0.0}, "buttons": {}}


def msg(left=None, right=None, t=0.0, seq=0):
    return InputMessage(seq=seq, payload={"left": left or hand(), "right": right or hand()},
                        received_at=t)


def arm(wd, t=0.0):
    """Show both hands released so they become armed."""
    return wd.check(msg(t=t), now=t)


def test_no_message():
    v = Watchdog().check(None, now=1.0)
    assert v.reason is Reason.NO_MESSAGE and not v.may_move and v.age_s is None


def test_startup_requires_release_before_enabling():
    wd = Watchdog()
    v = wd.check(msg(right=hand(grip=1.0)), now=0.0)      # squeezed from the start
    assert v.link_ok and v.enabled_hands == frozenset()
    wd.check(msg(right=hand(grip=0.0), t=0.1), now=0.1)   # released -> armed
    v = wd.check(msg(right=hand(grip=1.0), t=0.2), now=0.2)
    assert v.enabled_hands == {"right"}


def test_hysteresis_between_thresholds_keeps_state():
    wd = Watchdog(WatchdogConfig(press=0.6, release=0.4))
    arm(wd)
    assert wd.check(msg(right=hand(grip=0.5), t=0.1), 0.1).enabled_hands == frozenset()  # not pressed yet
    assert wd.check(msg(right=hand(grip=0.7), t=0.2), 0.2).enabled_hands == {"right"}
    assert wd.check(msg(right=hand(grip=0.5), t=0.3), 0.3).enabled_hands == {"right"}    # still held
    assert wd.check(msg(right=hand(grip=0.3), t=0.4), 0.4).enabled_hands == frozenset()  # released


def test_stale_message_disables_and_boundary():
    wd = Watchdog(WatchdogConfig(timeout_s=0.2))
    arm(wd)
    m = msg(right=hand(grip=1.0), t=1.0)
    assert wd.check(m, now=1.0).enabled_hands == {"right"}
    assert wd.check(m, now=1.2).link_ok                    # exactly timeout: still fresh
    v = wd.check(m, now=1.2001)
    assert v.reason is Reason.STALE and not v.may_move and v.age_s > 0.2


def test_recovery_with_grip_still_held_does_not_resume():
    wd = Watchdog(WatchdogConfig(timeout_s=0.2))
    arm(wd)
    wd.check(msg(right=hand(grip=1.0), t=1.0), 1.0)
    assert wd.check(msg(right=hand(grip=1.0), t=1.0), 2.0).reason is Reason.STALE
    v = wd.check(msg(right=hand(grip=1.0), t=2.0), 2.0)   # link back, grip still squeezed
    assert v.link_ok and not v.may_move
    wd.check(msg(right=hand(grip=0.0), t=2.1), 2.1)
    assert wd.check(msg(right=hand(grip=1.0), t=2.2), 2.2).enabled_hands == {"right"}


def test_disconnect_disables_and_requires_rearm():
    wd = Watchdog()
    arm(wd)
    assert wd.check(msg(right=hand(grip=1.0), t=0.1), 0.1).enabled_hands == {"right"}
    assert wd.check(msg(right=hand(grip=1.0, connected=False), t=0.2), 0.2).enabled_hands == frozenset()
    assert wd.check(msg(right=hand(grip=1.0), t=0.3), 0.3).enabled_hands == frozenset()  # reconnected, squeezed
    wd.check(msg(right=hand(grip=0.0), t=0.4), 0.4)
    assert wd.check(msg(right=hand(grip=1.0), t=0.5), 0.5).enabled_hands == {"right"}


def test_hands_are_independent():
    wd = Watchdog()
    arm(wd)
    v = wd.check(msg(left=hand(grip=0.0), right=hand(grip=1.0), t=0.1), 0.1)
    assert v.enabled_hands == {"right"}
    v = wd.check(msg(left=hand(grip=1.0), right=hand(grip=1.0), t=0.2), 0.2)
    assert v.enabled_hands == {"left", "right"}


def test_trigger_as_deadman():
    wd = Watchdog(WatchdogConfig(deadman="trigger"))
    arm(wd)
    v = wd.check(msg(right=hand(grip=1.0, trigger=0.0), t=0.1), 0.1)   # grip ignored
    assert v.enabled_hands == frozenset()
    v = wd.check(msg(right=hand(grip=0.0, trigger=1.0), t=0.2), 0.2)
    assert v.enabled_hands == {"right"}


@pytest.mark.parametrize("bad", [None, float("nan"), "1", True, [1], float("inf")])
def test_unreadable_deadman_counts_as_released(bad):
    wd = Watchdog()
    arm(wd)
    wd.check(msg(right=hand(grip=1.0), t=0.1), 0.1)
    h = hand()
    h["grip"] = bad
    v = wd.check(msg(right=h, t=0.2), 0.2)
    assert v.enabled_hands == frozenset()


def test_missing_hand_dict_is_safe():
    wd = Watchdog()
    arm(wd)
    m = InputMessage(seq=1, payload={"left": hand(grip=1.0)}, received_at=0.1)
    assert wd.check(m, 0.1).enabled_hands == {"left"}     # right missing: just not enabled


@pytest.mark.parametrize("kw", [
    dict(timeout_s=0), dict(deadman="bumper"), dict(press=0.3, release=0.5),
    dict(press=1.5), dict(hands=()), dict(hands=("middle",)),
])
def test_config_validation(kw):
    with pytest.raises(ValueError):
        WatchdogConfig(**kw)


def test_with_real_decoded_packets():
    def pkt(seq, grip):
        d = {"v": 1, "seq": seq, "left": hand(), "right": hand(grip=grip)}
        return json.dumps(d).encode()
    wd = Watchdog()
    wd.check(decode_packet(pkt(0, 0.0), 0.0), 0.0)
    v = wd.check(decode_packet(pkt(1, 1.0), 0.1), 0.1)
    assert v.enabled_hands == {"right"}