#!/usr/bin/env python3
"""
set_head_pose.py - move Stretch's head to a fixed pan/tilt and exit.

Used by launch/record.launch (through scripts/record_wrap.sh) so every
dataset run starts with the D435i in the same, known pose BEFORE the bag opens.
The pose reached is also written to stdout so the wrapper can log it.

    rosrun orio_stretch set_head_pose.py --pan 0.0 --tilt -0.30 [--timeout 30]

Requires stretch_driver (any mode - the trajectory action accepts head joints in
navigation mode too).
"""
import argparse
import sys

import rospy
import actionlib
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint

ACTION = '/stretch_controller/follow_joint_trajectory'
JS_TOPIC = '/stretch/joint_states'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--pan', type=float, default=0.0, help='joint_head_pan target (rad)')
    ap.add_argument('--tilt', type=float, default=-0.30, help='joint_head_tilt target (rad, negative = down)')
    ap.add_argument('--timeout', type=float, default=30.0, help='seconds to wait for driver + motion')
    args, _ = ap.parse_known_args(rospy.myargv(argv=sys.argv)[1:])

    rospy.init_node('set_head_pose', anonymous=True)
    client = actionlib.SimpleActionClient(ACTION, FollowJointTrajectoryAction)
    if not client.wait_for_server(rospy.Duration(args.timeout)):
        rospy.logerr('set_head_pose: no trajectory action server at %s after %.0fs', ACTION, args.timeout)
        return 2

    goal = FollowJointTrajectoryGoal()
    goal.trajectory.joint_names = ['joint_head_pan', 'joint_head_tilt']
    pt = JointTrajectoryPoint()
    pt.positions = [args.pan, args.tilt]
    pt.time_from_start = rospy.Duration(2.0)
    goal.trajectory.points = [pt]
    goal.trajectory.header.stamp = rospy.Time.now()
    goal.goal_time_tolerance = rospy.Duration(2.0)
    client.send_goal(goal)
    if not client.wait_for_result(rospy.Duration(args.timeout)):
        rospy.logerr('set_head_pose: head motion did not finish within %.0fs', args.timeout)
        return 3

    # report where the head actually is (the driver may clip to joint limits)
    try:
        js = rospy.wait_for_message(JS_TOPIC, JointState, timeout=5.0)
        pan = js.position[js.name.index('joint_head_pan')]
        tilt = js.position[js.name.index('joint_head_tilt')]
        print('HEAD_POSE pan=%.4f tilt=%.4f (commanded pan=%.4f tilt=%.4f)' % (pan, tilt, args.pan, args.tilt))
    except Exception as e:  # noqa: BLE001
        rospy.logwarn('set_head_pose: could not read back joint_states: %s', e)
        print('HEAD_POSE pan=? tilt=? (commanded pan=%.4f tilt=%.4f)' % (args.pan, args.tilt))
    return 0


if __name__ == '__main__':
    sys.exit(main())
