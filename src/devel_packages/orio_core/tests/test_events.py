import json

import numpy as np

from orio_core.events import EventSink, decode, encode


def test_encode_converts_numpy_and_keeps_schema():
    s = encode("ik", "arm1", {"q": np.arange(3.0), "n": np.int64(2), "ok": True}, 12.5, "sm")
    d = json.loads(s)
    assert d == {"t": 12.5, "src": "sm", "kind": "ik", "name": "arm1",
                 "data": {"q": [0.0, 1.0, 2.0], "n": 2, "ok": True}}
    assert decode(s) == d


def test_sink_sends_and_never_raises():
    sent = []
    sink = EventSink(sent.append, src="sm", clock=lambda: 1.0)
    assert sink.emit("state", "FETCH", x=1)
    assert decode(sent[0])["data"] == {"x": 1}

    def boom(_):
        raise RuntimeError("no master")

    bad = EventSink(boom, src="sm")
    assert bad.emit("state", "FETCH") is False
    assert bad.dropped == 1


def test_disabled_sink_is_noop():
    sent = []
    sink = EventSink(sent.append, src="sm", enabled=False)
    assert sink.emit("state") is False
    assert sent == []


def test_timed_records_duration_and_failure():
    sent = []
    sink = EventSink(sent.append, src="sm")
    with sink.timed("service", "/x") as t:
        t.data["result"] = 1
    d = decode(sent[0])["data"]
    assert d["ok"] is True and d["result"] == 1 and d["dt_s"] >= 0
    try:
        with sink.timed("service", "/y"):
            raise ValueError("x")
    except ValueError:
        pass
    assert decode(sent[1])["data"]["ok"] is False
