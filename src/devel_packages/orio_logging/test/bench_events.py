#!/usr/bin/env python3
"""Microbenchmarks for the two logging calls on the critical path.

    python3 bench_events.py            # needs a roscore

Reports per-call cost of EventSink.emit over a ROS publisher, a log line forwarded to
rosout, and JPEG-compressing a 400x365 RGB crop with rerun.
"""
import logging
import statistics
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[4]
sys.path[:0] = ['/opt/ros/noetic/lib/python3/dist-packages', str(REPO / 'src/devel_packages/orio_core')]

import numpy as np  # noqa: E402
import rospy  # noqa: E402
from std_msgs.msg import String  # noqa: E402
from orio_core.events import TOPIC, EventSink  # noqa: E402


def timeit(fn, n):
    fn()
    ts = []
    for _ in range(n):
        t = time.perf_counter()
        fn()
        ts.append((time.perf_counter() - t) * 1e6)
    return statistics.median(ts), max(ts)


def main():
    rospy.init_node('bench_events')
    pub = rospy.Publisher(TOPIC, String, queue_size=50)
    rospy.Subscriber(TOPIC, String, lambda m: None)  # a live subscriber makes publish do real work
    rospy.sleep(0.5)
    sink = EventSink(lambda s: pub.publish(String(data=s)), src='bench')
    joints = np.random.rand(7)
    med, mx = timeit(lambda: sink.emit('ik', 'arm1', target=[0.4, -0.3, 0.05], pre=joints, final=joints,
                                       attempt=1, err_pre=0.001, err_final=0.002), 2000)
    print(f'events.emit (ik payload, 1 subscriber): median {med:.0f} us  max {mx:.0f} us')

    class ToRosout(logging.Handler):
        def emit(self, record):
            logging.getLogger('rosout').handle(record)
    log = logging.getLogger('bench_fsm')
    log.propagate = False
    log.handlers[:] = [ToRosout()]
    rosout = logging.getLogger('rosout')
    stream = [h for h in rosout.handlers if 'Stream' in type(h).__name__]
    for h in stream:  # keep the terminal quiet during the benchmark
        rosout.removeHandler(h)
    med, mx = timeit(lambda: log.info('[FetchInput] IK pre_joints: %s', np.round(joints, 4)), 2000)
    print(f'log line -> rosout: median {med:.0f} us  max {mx:.0f} us')

    import rerun as rr
    rr.init('bench', recording_id='bench')
    crop = (np.random.rand(365, 400, 3) * 255).astype(np.uint8)
    med, mx = timeit(lambda: rr.Image(crop, color_model='RGB').compress(jpeg_quality=80), 200)
    print(f'JPEG compress 400x365 crop: median {med / 1000:.2f} ms  max {mx / 1000:.2f} ms')
    full = (np.random.rand(1080, 1920, 3) * 255).astype(np.uint8)
    med, mx = timeit(lambda: rr.Image(full, color_model='RGB').compress(jpeg_quality=80), 30)
    print(f'JPEG compress 1920x1080 frame: median {med / 1000:.2f} ms  max {mx / 1000:.2f} ms')


if __name__ == '__main__':
    main()
