#!/usr/bin/env python3
"""
prepare_pose.py - put Stretch into the mapping/navigation body pose and exit.

  1. stow the arm: lift 0.2 m, arm fully retracted, wrist yaw 3.4 rad (pointing inward), gripper closed
  2. point the head camera: pan 0, tilt -0.35 rad (~20 deg down)

Order matters: the arm is stowed FIRST so it is never in the camera's view, then the head moves.
Requires stretch_driver (any mode - the trajectory action accepts these joints in navigation mode).

    rosrun orio_stretch prepare_pose.py [--tilt -0.35] [--lift 0.2] [--arm 0.0] [--wrist-yaw 3.4] [--no-gripper] [--no-arm] [--no-head] [--timeout 30]
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


def send(client, names, positions, duration, timeout, effort=None):
    goal = FollowJointTrajectoryGoal()
    goal.trajectory.joint_names = names
    pt = JointTrajectoryPoint()
    pt.positions = positions
    if effort:
        pt.effort = effort      # the driver reads a contact threshold per joint index; shorter list = the rest default
    pt.time_from_start = rospy.Duration(duration)
    goal.trajectory.points = [pt]
    goal.trajectory.header.stamp = rospy.Time.now()
    goal.goal_time_tolerance = rospy.Duration(2.0)
    client.send_goal(goal)
    if not client.wait_for_result(rospy.Duration(timeout)):
        rospy.logerr('prepare_pose: motion %s did not finish within %.0fs', names, timeout)
        return False
    res = client.get_result()
    if res is not None and res.error_code != 0:
        rospy.logwarn('prepare_pose: %s finished with error_code %d (%s)', names, res.error_code, res.error_string)
    return True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--pan', type=float, default=0.0)
    ap.add_argument('--tilt', type=float, default=-0.35, help='head tilt (rad, negative = down)')
    ap.add_argument('--lift', type=float, default=0.2, help='lift height (m)')
    ap.add_argument('--arm', type=float, default=0.0, help='arm extension (m); 0 = fully retracted')
    ap.add_argument('--lift-effort', type=float, default=0.0,
                    help='contact threshold (effort %%) for the lift when it has to move UP; 0 = driver default (payload on the gripper needs more)')
    ap.add_argument('--split-lift', action='store_true',
                    help='move the lift first as a separate goal, then stow arm + wrist (a lift contact stop then cannot abort the wrist stow)')
    ap.add_argument('--no-gripper', action='store_true', help='do not command the gripper (payload mounted on it)')
    ap.add_argument('--wrist-yaw', type=float, default=3.4, help='wrist yaw (rad); 3.4 = stowed, pointing inward')
    ap.add_argument('--no-arm', action='store_true', help='skip the arm stow')
    ap.add_argument('--no-head', action='store_true', help='skip the head move')
    ap.add_argument('--timeout', type=float, default=30.0)
    args, _ = ap.parse_known_args(rospy.myargv(argv=sys.argv)[1:])

    rospy.init_node('prepare_pose', anonymous=True)
    client = actionlib.SimpleActionClient(ACTION, FollowJointTrajectoryAction)
    if not client.wait_for_server(rospy.Duration(args.timeout)):
        rospy.logerr('prepare_pose: no trajectory action server at %s after %.0fs (is stretch_driver up?)',
                     ACTION, args.timeout)
        return 2

    ok = True
    if not args.no_arm:
        # retract + yaw + close gripper together, lift with them (all slow, 4 s)
        effort = None
        if args.lift_effort > 0:
            js = rospy.wait_for_message(JS_TOPIC, JointState, timeout=5.0)
            if args.lift > js.position[js.name.index('joint_lift')]:
                effort = [args.lift_effort]     # joint_lift is index 0
        names = ['wrist_extension', 'joint_wrist_yaw']
        positions = [args.arm, args.wrist_yaw]
        if not args.no_gripper:
            names.append('gripper_aperture')
            positions.append(0.0)
        if args.split_lift:
            # lift FIRST and on its own: the arm must be up before the wrist swings in (payload on the
            # gripper), and the driver ends the WHOLE goal as soon as one joint reports a guarded contact,
            # so a lift contact stop must not cut the wrist stow short
            ok &= send(client, ['joint_lift'], [args.lift], 4.0, args.timeout, effort)
            ok &= send(client, names, positions, 4.0, args.timeout)
        else:
            ok &= send(client, ['joint_lift'] + names, [args.lift] + positions, 4.0, args.timeout, effort)
    if not args.no_head:
        ok &= send(client, ['joint_head_pan', 'joint_head_tilt'], [args.pan, args.tilt], 2.0, args.timeout)

    try:
        js = rospy.wait_for_message(JS_TOPIC, JointState, timeout=5.0)
        get = lambda n: js.position[js.name.index(n)] if n in js.name else float('nan')
        print('POSE lift=%.3f arm=%.3f wrist_yaw=%.3f head_pan=%.3f head_tilt=%.3f' % (
            get('joint_lift'), get('wrist_extension'), get('joint_wrist_yaw'),
            get('joint_head_pan'), get('joint_head_tilt')))
    except Exception as e:  # noqa: BLE001
        rospy.logwarn('prepare_pose: could not read back joint_states: %s', e)
    return 0 if ok else 3


if __name__ == '__main__':
    sys.exit(main())
