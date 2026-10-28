import json, socket, sys, os, time
import pytest
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from input_server import (ACCEPT, RESTART, STALE, InputServer,
                          PacketError, SeqFilter, decode_packet)
from latest_slot import Mailbox


def hand():
    return {"connected": True, "trigger": 0.5, "grip": 0.0,
            "primary": {"x": 0.0, "y": 0.0}, "secondary": {"x": 0.0, "y": 0.0}, "buttons": {}}


def pkt(seq, v=1, **over):
    d = {"v": v, "seq": seq, "left": hand(), "right": hand()}
    d.update(over)
    return json.dumps(d).encode()


def test_decode_valid():
    m = decode_packet(pkt(7), received_at=12.5)
    assert m.seq == 7 and m.received_at == 12.5
    assert set(m.payload) == {"left", "right"}       # v/seq stripped


@pytest.mark.parametrize("bad", [
    b"\xff\xfe", b"not json", b"[1,2]", pkt(1, v=2), pkt(-1), pkt(True),
    json.dumps({"v": 1, "seq": "x", "left": {}, "right": {}}).encode(),
    json.dumps({"v": 1, "seq": 1, "left": {}}).encode(),
    json.dumps({"v": 1, "seq": 1, "left": {}, "right": []}).encode(),
])
def test_decode_rejects(bad):
    with pytest.raises(PacketError):
        decode_packet(bad, 0.0)


def test_seqfilter_verdicts():
    f = SeqFilter(restart_gap=100)
    assert f.classify(5) == (ACCEPT, 0)
    assert f.classify(6) == (ACCEPT, 0)
    assert f.classify(9) == (ACCEPT, 2)        # 7, 8 lost
    assert f.classify(9)[0] == STALE           # duplicate
    assert f.classify(8)[0] == STALE           # reordered
    assert f.classify(3000)[0] == ACCEPT
    assert f.classify(0)[0] == RESTART         # client restarted
    assert f.classify(1) == (ACCEPT, 0)


def test_handle_datagram_latest_wins_and_stats():
    box = Mailbox()
    lst = InputServer(box)
    for seq in (1, 2, 5, 4, 5):                # 3,4 skipped; then late 4 and dup 5
        lst.handle_datagram(pkt(seq), now=float(seq))
    lst.handle_datagram(b"garbage", now=9.0)
    assert box.get().seq == 5 and box.get().received_at == 5.0
    s = lst.stats
    assert (s.received, s.accepted, s.stale, s.lost, s.malformed) == (6, 3, 2, 2, 1)


def _wait(cond, timeout=2.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if cond():
            return True
        time.sleep(0.005)
    return False


def test_real_udp_loopback_and_survives_garbage():
    box = Mailbox()
    with InputServer(box, port=0) as lst:
        tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        tx.sendto(b"garbage", lst.address)
        for seq in range(10):
            tx.sendto(pkt(seq), lst.address)
        assert _wait(lambda: box.get() is not None and box.get().seq == 9)
        assert lst.stats.malformed == 1 and lst.stats.accepted == 10
        tx.close()
    # after stop(): thread gone, socket closed
    assert lst._thread is None and lst._sock is None