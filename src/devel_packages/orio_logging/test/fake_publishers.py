#!/usr/bin/env python3
"""Fake ORIO nodes for exercising the recorder without hardware.

    python3 fake_publishers.py [--duration 10] [--robots 1 2]

Publishes RobotState at 100 Hz per arm, rosout lines, smach status, vacuum flags and
/orio/events (pick, ik, cmd, service) with plausible values.
"""
import argparse
import math
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[4]
sys.path[:0] = ['/opt/ros/noetic/lib/python3/dist-packages', str(REPO / 'devel/lib/python3/dist-packages'),
                str(REPO / 'src/devel_packages/orio_core')]

import numpy as np  # noqa: E402
import rospy  # noqa: E402
from std_msgs.msg import Bool, String  # noqa: E402
from smach_msgs.msg import SmachContainerStatus  # noqa: E402
from franka_interface_msgs.msg import RobotState  # noqa: E402
from orio_core.events import TOPIC, EventSink  # noqa: E402

HOME = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
STATES = ['DECIDE_HUB', 'FETCH_INPUT', 'DROP_TO_ZONE', 'RETRIEVE', 'PLACE_OUTPUT']


def robot_state(n, t):
    m = RobotState()
    m.header.stamp = rospy.Time.now()
    q = [h + 0.3 * math.sin(0.5 * t + i + n) for i, h in enumerate(HOME)]
    m.q = q
    m.dq = [0.15 * math.cos(0.5 * t + i) for i in range(7)]
    m.tau_J = [2.0 * math.sin(t + i) for i in range(7)]
    m.tau_ext_hat_filtered = [0.2 * math.sin(2 * t + i) for i in range(7)]
    m.O_F_ext_hat_K = [math.sin(t), math.cos(t), -2.0 + math.sin(3 * t), 0.1, 0.1, 0.1]
    c, s = math.cos(0.3 * t), math.sin(0.3 * t)
    m.O_T_EE = [c, s, 0, 0, -s, c, 0, 0, 0, 0, 1, 0, 0.4 + 0.1 * c, 0.1 * s, 0.4, 1]  # column-major
    m.robot_mode = 2
    m.control_command_success_rate = 1.0
    return m


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--duration', type=float, default=10.0)
    ap.add_argument('--robots', nargs='*', type=int, default=[1, 2])
    a = ap.parse_args(rospy.myargv(sys.argv)[1:])
    rospy.init_node('fake_orio')
    pubs = {n: rospy.Publisher(f'/robot_state_publisher_node_{n}/robot_state', RobotState, queue_size=1)
            for n in a.robots}
    smach_pub = rospy.Publisher('/orio_visualiser/smach/container_status', SmachContainerStatus, queue_size=5)
    vac = rospy.Publisher('/orio/vacuum/pnp_has_item', Bool, queue_size=5)
    ev_pub = rospy.Publisher(TOPIC, String, queue_size=50)
    events = EventSink(lambda s: ev_pub.publish(String(data=s)), src='fake')
    rospy.sleep(0.5)

    t0 = time.time()
    rate = rospy.Rate(100)
    i = 0
    state_i = -1
    while not rospy.is_shutdown() and time.time() - t0 < a.duration:
        t = time.time() - t0
        for n, p in pubs.items():
            p.publish(robot_state(n, t))
        if i % 100 == 0:
            rospy.loginfo('fake tick %d', i // 100)
            vac.publish(Bool(data=bool((i // 100) % 2)))
        if i % 200 == 0:
            state_i += 1
            s = SmachContainerStatus()
            s.header.stamp = rospy.Time.now()
            s.path = '/ORIO_ROOT/ARM1'
            s.active_states = [STATES[state_i % len(STATES)]]
            smach_pub.publish(s)
            if s.active_states == ['FETCH_INPUT']:
                events.emit('pick', n=state_i // len(STATES) + 1)
                events.emit('ik', 'arm1', target=[0.4, -0.3, 0.05], pre=HOME, final=HOME, attempt=1, err_pre=0.001, err_final=0.002)
                with events.timed('service', '/compute_grasps') as tm:
                    time.sleep(0.05)
                    tm.data['ok'] = True
                with events.timed('cmd', 'arm1') as tm:
                    tm.data.update(joints=HOME, duration=3)
                    time.sleep(0.02)
        i += 1
        rate.sleep()
    rospy.logwarn('fake done after %d ticks', i)


if __name__ == '__main__':
    main()
