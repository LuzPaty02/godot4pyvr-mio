"""Integration: real InputClient -> UDP loopback -> real InputServer -> Mailbox.

The server binds port 0 (a free port chosen by the OS), so these tests never
clash with a running server on 9000.
"""
import os
import sys
import time
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from PythonFiles.input_client import InputClient
from PythonFiles.controller_input import (
    ControllerInputReader,
    ControllerPoseNormalizer,
)
from robot_control.input_server import InputServer
from robot_control.latest_slot import Mailbox


def controller_state(trigger=0.5, connected=True, position=(0.0, 0.0, 0.0)):
    """Same shape ControllerInputReader.read_controller returns."""
    return {
        "connected": connected,
        "position": dict(zip(("x", "y", "z"), position)),
        "trigger": trigger,
        "grip": 0.0,
        "primary": {"x": -0.25, "y": 0.75},
        "secondary": {"x": 0.0, "y": 0.0},
        "buttons": {name: False for name in (
            "trigger_click", "trigger_touch", "primary_click",
            "primary_touch", "secondary_click", "secondary_touch")},
    }


def frame(trigger=0.5):
    return {"left": controller_state(0.1), "right": controller_state(trigger)}


def wait_for(cond, timeout=2.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if cond():
            return True
        time.sleep(0.005)
    return False


@pytest.fixture
def link():
    box = Mailbox()
    server = InputServer(box, port=0)
    server.start()
    client = InputClient(addr=server.address)
    try:
        yield box, server, client
    finally:
        client.close()
        server.stop()


def test_payload_arrives_intact(link):
    box, server, client = link
    sent = {
        "left": controller_state(position=(1.0, 2.0, 3.0)),
        "right": controller_state(trigger=0.875, position=(-1.0, 0.5, 4.0)),
        "dofs": {"ET": 0.75, "ER": 0.25, "EBV": 0.5, "EBH": 1.0},
    }
    client.send(sent)
    assert wait_for(lambda: box.get() is not None)
    msg = box.get()
    assert msg.seq == 0
    assert msg.payload == sent                 # round-trips through JSON unchanged
    assert msg.payload["left"]["primary"] == {"x": -0.25, "y": 0.75}
    assert msg.payload["right"]["position"] == {"x": -1.0, "y": 0.5, "z": 4.0}
    assert msg.payload["dofs"] == sent["dofs"]


def test_controller_reader_includes_position_and_zeros_disconnected_hands():
    class Controller:
        def get_float(self, _action):
            return 0.0

        def get_vector2(self, _action):
            return type("Vector2", (), {"x": 0.0, "y": 0.0})()

        def is_button_pressed(self, _button):
            return False

    active = ControllerInputReader.read_controller(
        Controller(), position={"x": 1.25, "y": -2.0, "z": 0.5})
    disconnected = ControllerInputReader.read_controller(
        Controller(), connected=False, position={"x": 9.0, "y": 8.0, "z": 7.0})

    assert active["position"] == {"x": 1.25, "y": -2.0, "z": 0.5}
    assert disconnected["position"] == {"x": 0.0, "y": 0.0, "z": 0.0}


def test_pose_normalizer_captures_ranges_and_maps_dofs(tmp_path):
    normalizer = ControllerPoseNormalizer(
        str(tmp_path / "controller_pose_calibration.json"))

    def pose(left_x, left_y, right_x, right_y):
        return {
            "left": {
                "connected": True,
                "position": {"x": left_x, "y": left_y, "z": 0.0},
            },
            "right": {
                "connected": True,
                "position": {"x": right_x, "y": right_y, "z": 0.0},
            },
        }

    normalizer.observe(pose(-1.0, -2.0, -3.0, -4.0))
    normalizer.observe(pose(1.0, 2.0, 3.0, 4.0))
    assert normalizer.save()

    loaded = ControllerPoseNormalizer(
        str(tmp_path / "controller_pose_calibration.json"))
    assert loaded.normalize(pose(0.0, 0.0, 1.5, -2.0)) == {
        "ER": 0.75,
        "ET": 0.25,
        "EBV": 0.5,
        "EBH": 0.5,
    }


def test_sequence_increments_and_latest_wins(link):
    box, server, client = link
    for i in range(20):
        client.send(frame(trigger=i / 20))
    assert wait_for(lambda: box.get() is not None and box.get().seq == 19)
    assert box.get().payload["right"]["trigger"] == 19 / 20
    s = server.stats
    assert s.accepted == 20 and s.stale == 0 and s.lost == 0 and s.malformed == 0


def test_gap_in_sequence_counts_as_lost(link):
    box, server, client = link
    client.send(frame())
    assert wait_for(lambda: server.stats.accepted == 1)
    client._seq += 3                           # white-box: pretend 3 packets vanished
    client.send(frame())
    assert wait_for(lambda: server.stats.accepted == 2)
    assert server.stats.lost == 3


def test_client_restart_is_detected_not_treated_as_stale(link):
    box, server, client = link
    client._seq = 5000                         # white-box: a long-running session
    client.send(frame(trigger=0.9))
    assert wait_for(lambda: box.get() is not None and box.get().seq == 5000)

    fresh = InputClient(addr=server.address)   # Godot reloaded: counter back to 0
    try:
        fresh.send(frame(trigger=0.1))
        assert wait_for(lambda: box.get().seq == 0)
        assert box.get().payload["right"]["trigger"] == 0.1
        assert server.stats.restarts == 1
    finally:
        fresh.close()


def test_silent_client_makes_message_age_grow(link):
    box, server, client = link
    client.send(frame())
    assert wait_for(lambda: box.get() is not None)
    first = box.get()
    time.sleep(0.2)
    # nothing new arrived: same message, and it is getting old (watchdog input)
    assert box.get() is first
    assert time.monotonic() - first.received_at >= 0.2


def test_invalid_payload_is_counted_and_does_not_break_the_link(link):
    box, server, client = link
    client.send({"left": controller_state()})  # 'right' missing -> malformed
    assert wait_for(lambda: server.stats.malformed == 1)
    assert box.get() is None
    client.send(frame())                       # next valid packet still gets through
    assert wait_for(lambda: box.get() is not None)
    assert server.stats.accepted == 1


def test_client_send_never_raises_when_nobody_listens():
    # grab a free port, then close it so nothing is listening there
    box = Mailbox()
    server = InputServer(box, port=0)
    server.start()
    addr = server.address
    server.stop()
    client = InputClient(addr=addr)
    try:
        for _ in range(50):
            client.send(frame())               # must not raise or block
    finally:
        client.close()


def test_client_send_after_close_is_a_noop(link):
    box, server, client = link
    client.close()
    client.send(frame())                       # must not raise
    client.close()                             # closing twice is fine
    time.sleep(0.1)
    assert box.get() is None


def test_both_hands_and_disconnected_flag_survive(link):
    box, server, client = link
    client.send({"left": controller_state(connected=False), "right": controller_state()})
    assert wait_for(lambda: box.get() is not None)
    assert box.get().payload["left"]["connected"] is False
    assert box.get().payload["right"]["connected"] is True