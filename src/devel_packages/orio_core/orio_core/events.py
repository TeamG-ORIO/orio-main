"""Structured run events for the rerun recorder: JSON encoding + a sink with an injected sender."""

import json
import time

import numpy as np

TOPIC = "/orio/events"


def _jsonable(v):
    if isinstance(v, np.ndarray):
        return v.tolist()
    if isinstance(v, np.generic):
        return v.item()
    if isinstance(v, dict):
        return {str(k): _jsonable(x) for k, x in v.items()}
    if isinstance(v, (list, tuple)):
        return [_jsonable(x) for x in v]
    if isinstance(v, (str, int, float, bool)) or v is None:
        return v
    return str(v)


def encode(kind, name, data, t, src):
    """One event as a JSON string: {"t", "src", "kind", "name", "data"}."""
    return json.dumps({"t": float(t), "src": str(src), "kind": str(kind),
                       "name": str(name), "data": _jsonable(data or {})},
                      separators=(",", ":"), allow_nan=True)


def decode(s):
    return json.loads(s)


class EventSink:
    """emit(kind, name, **data) -> send(json). Never raises; returns False when dropped."""

    def __init__(self, send, src, enabled=True, clock=time.time):
        self._send = send
        self._src = src
        self.enabled = enabled
        self._clock = clock
        self.dropped = 0

    def emit(self, kind, name="", **data):
        if not self.enabled:
            return False
        try:
            self._send(encode(kind, name, data, self._clock(), self._src))
            return True
        except Exception:
            self.dropped += 1
            return False

    def timed(self, kind, name=""):
        """Context manager: emits kind/name with dt_s (and ok=False on exception)."""
        return _Timed(self, kind, name)


class _Timed:
    def __init__(self, sink, kind, name):
        self._sink, self._kind, self._name = sink, kind, name
        self.data = {}

    def __enter__(self):
        self._t0 = time.perf_counter()
        return self

    def __exit__(self, exc_type, exc, tb):
        self.data.setdefault("ok", exc is None)
        self._sink.emit(self._kind, self._name, dt_s=time.perf_counter() - self._t0, **self.data)
        return False
