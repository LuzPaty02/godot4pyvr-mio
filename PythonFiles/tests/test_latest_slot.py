import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from latest_slot import Mailbox


def test_mailbox_get_does_not_consume():
    box = Mailbox()
    assert box.get() is None
    box.put("a")
    assert box.get() == "a" and box.get() == "a"


def test_latest_wins():
    box = Mailbox()
    box.put(1)
    box.put(2)
    assert box.get() == 2