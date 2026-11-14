#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# stretch_twist_arm_teleop.py
#
# ROS 1 keyboard teleop for Stretch that BOTH:
#   * drives the mobile base with geometry_msgs/Twist on /stretch/cmd_vel
#     (works while stretch_driver is in NAVIGATION mode, e.g. during RTAB-Map
#     mapping/localization), with fine-grained, adjustable speed up to a max, and
#   * moves the arm / lift / wrist / gripper / head via the driver's
#     FollowJointTrajectory action (accepted in navigation mode).
#
# It is inspired by stretch_body's stretch_robot_keyboard_teleop.py (key layout /
# which joints), but is fully ROS-native and adds continuous, scalable velocity
# control that the low-level tool lacks.
#
# Run (with the mapping/localization stack already up, so a master + stretch_driver exist):
#   source /opt/ros/noetic/setup.bash
#   source ~/catkin_ws/devel/setup.bash
#   python3 scripts/stretch_twist_arm_teleop.py
#
# Base motion is "hold to move": while you hold a drive key the robot moves; it
# auto-stops shortly after you release (dead-man timeout). SPACE stops instantly.

import sys
import select
import termios
import tty
import threading

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sensor_msgs.msg import JointState
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint


MENU = """
--------------------------------------------------------------------
 S T R E T C H   twist + arm keyboard teleop   (ROS, navigation mode)
--------------------------------------------------------------------
 BASE (hold to move, auto-stops on release; SPACE = stop now)
      w : forward            s : backward
      a : rotate left        d : rotate right
   space: STOP base

 BASE SPEED (fine-grained, clamped to max)
      r / f : linear  speed  + / -    (max {max_lin:.2f} m/s)
      t / g : angular speed  + / -    (max {max_ang:.2f} rad/s)

 ARM / BODY (position steps via trajectory action)
      i / k : lift  up / down
      l / j : arm   extend / retract
      o / u : wrist yaw  + / -
      n / b : gripper close / open
      . / , : head pan   + / -
      ' / ; : head tilt  + / -         {headlock}
      = / - : joint step size  + / -   (current {step:.3f})

      Ctrl-C or capital Q : quit
{marks}--------------------------------------------------------------------
 linear={lin:.2f} m/s   angular={ang:.2f} rad/s   step={step:.3f}
--------------------------------------------------------------------
"""


class TwistArmTeleop:
    def __init__(self):
        # ---- params (override with _param:=value or a launch file) ----
        self.cmd_vel_topic = rospy.get_param('~cmd_vel_topic', '/stretch/cmd_vel')
        self.joint_states_topic = rospy.get_param('~joint_states_topic', '/stretch/joint_states')
        self.traj_action = rospy.get_param('~trajectory_action',
                                           '/stretch_controller/follow_joint_trajectory')

        # base velocity: current setpoint, step, and hard max caps
        self.linear = rospy.get_param('~start_linear', 0.05)     # m/s
        self.angular = rospy.get_param('~start_angular', 0.30)   # rad/s
        self.linear_step = rospy.get_param('~linear_step', 0.02)
        self.angular_step = rospy.get_param('~angular_step', 0.10)
        self.max_linear = rospy.get_param('~max_linear', 0.50)   # RE1 base cap ~0.5 m/s
        self.max_angular = rospy.get_param('~max_angular', 1.20)
        self.min_linear = 0.01
        self.min_angular = 0.05

        # dead-man: base stops if no drive key seen within this many seconds
        self.hold = rospy.get_param('~hold', 1.0)
        self.rate_hz = rospy.get_param('~rate', 20.0)

        # arm/body position step (meters for lift/arm, radians for the rest)
        self.step = rospy.get_param('~joint_step', 0.02)
        self.step_min, self.step_max = 0.005, 0.10
        # lock_head: ignore the head pan/tilt keys. Used by record.launch so a
        # dataset run keeps the camera pose fixed (RTAB-Map runs Reg/Force3DoF and
        # comparisons across runs assume a constant camera-to-base transform).
        self.lock_head = rospy.get_param('~lock_head', False)
        # mark_points: keys 1..9 stop the base and publish "the robot is standing on taped reference point N"
        # on /slam_test/mark (std_msgs/String, JSON). Used by scripts/slam_record.sh so recorded runs can be
        # scored at measured floor points (scripts/marks_report.py). Off by default.
        self.mark_points = rospy.get_param('~mark_points', False)
        self.mark_pub = rospy.Publisher('/slam_test/mark', String, queue_size=10) if self.mark_points else None
        self.marks_done = []
        # lift_up_effort: contact threshold (effort %, 0 = driver default) sent with UPWARD lift steps only.
        # With a payload on the gripper the factory threshold (~47 %) trips as soon as the lift starts
        # to rise ("joint_lift contact detected") and the lift does not move. Downward steps keep the default.
        self.lift_up_effort = rospy.get_param('~lift_up_effort', 0.0)
        # gripper and head steps scale off self.step
        self.head_scale = 2.0

        # ---- ROS I/O ----
        self.cmd_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=1)
        self.joint_state = None
        self.js_lock = threading.Lock()
        rospy.Subscriber(self.joint_states_topic, JointState, self._js_cb)

        self.traj_client = actionlib.SimpleActionClient(self.traj_action,
                                                        FollowJointTrajectoryAction)
        rospy.loginfo('Waiting for trajectory action server %s ...', self.traj_action)
        if not self.traj_client.wait_for_server(rospy.Duration(10.0)):
            rospy.logwarn('Trajectory action server not available - arm keys will be inactive. '
                          '(Base twist will still work.)')
            self.traj_client = None
        else:
            rospy.loginfo('Connected to trajectory action server.')

        # current drive command + timestamp of last drive key
        self.drive = (0.0, 0.0)          # (linear, angular) signs applied
        self.last_drive_key = rospy.Time(0)

    # -------------------------------------------------------------- callbacks
    def _js_cb(self, msg):
        with self.js_lock:
            self.joint_state = msg

    def _joint_pos(self, name):
        with self.js_lock:
            js = self.joint_state
        if js is None or name not in js.name:
            return None
        return js.position[js.name.index(name)]

    # -------------------------------------------------------------- arm cmds
    def send_joint_delta(self, joint, delta, seconds=0.0):
        """Command joint to (current + delta) via a 1-point trajectory."""
        if self.traj_client is None:
            return
        cur = self._joint_pos(joint)
        if cur is None:
            rospy.logwarn_throttle(2.0, 'No joint_state for %s yet.', joint)
            return
        goal = FollowJointTrajectoryGoal()
        goal.goal_time_tolerance = rospy.Duration(1.0)
        goal.trajectory.joint_names = [joint]
        pt = JointTrajectoryPoint()
        pt.positions = [cur + delta]
        if joint == 'joint_lift' and delta > 0 and self.lift_up_effort > 0:
            pt.effort = [self.lift_up_effort]
        pt.time_from_start = rospy.Duration(seconds)
        goal.trajectory.points = [pt]
        goal.trajectory.header.stamp = rospy.Time.now()
        self.traj_client.send_goal(goal)

    # -------------------------------------------------------------- key logic
    def handle_key(self, c):
        redraw = False
        now = rospy.Time.now()

        # ----- base drive (hold to move) -----
        if c == 'w':
            self.drive = (self.linear, 0.0);  self.last_drive_key = now
        elif c == 's':
            self.drive = (-self.linear, 0.0); self.last_drive_key = now
        elif c == 'a':
            self.drive = (0.0, self.angular);  self.last_drive_key = now
        elif c == 'd':
            self.drive = (0.0, -self.angular); self.last_drive_key = now
        elif c == ' ':
            self.drive = (0.0, 0.0)

        # ----- base speed scaling -----
        elif c == 'r':
            self.linear = min(self.max_linear, self.linear + self.linear_step); redraw = True
        elif c == 'f':
            self.linear = max(self.min_linear, self.linear - self.linear_step); redraw = True
        elif c == 't':
            self.angular = min(self.max_angular, self.angular + self.angular_step); redraw = True
        elif c == 'g':
            self.angular = max(self.min_angular, self.angular - self.angular_step); redraw = True

        # ----- arm / body position steps -----
        elif c == 'i':
            self.send_joint_delta('joint_lift', self.step)
        elif c == 'k':
            self.send_joint_delta('joint_lift', -self.step)
        elif c == 'l':
            self.send_joint_delta('wrist_extension', self.step)
        elif c == 'j':
            self.send_joint_delta('wrist_extension', -self.step)
        elif c == 'o':
            self.send_joint_delta('joint_wrist_yaw', self.step)
        elif c == 'u':
            self.send_joint_delta('joint_wrist_yaw', -self.step)
        elif c == 'n':
            self.send_joint_delta('joint_gripper_finger_left', -self.step)   # close
        elif c == 'b':
            self.send_joint_delta('joint_gripper_finger_left', self.step)    # open
        elif c in ('.', ',', '\'', ';') and self.lock_head:
            rospy.logwarn_throttle(2.0, 'Head is LOCKED (~lock_head=true) - head keys ignored during recording.')
        elif c == '.':
            self.send_joint_delta('joint_head_pan', self.step * self.head_scale)
        elif c == ',':
            self.send_joint_delta('joint_head_pan', -self.step * self.head_scale)
        elif c == '\'':
            self.send_joint_delta('joint_head_tilt', self.step * self.head_scale)
        elif c == ';':
            self.send_joint_delta('joint_head_tilt', -self.step * self.head_scale)

        # ----- joint step scaling -----
        elif self.mark_points and c in '1234567890':
            # stop first, then stamp: the pose at this instant is what gets scored ('0' = point 10)
            self.drive = (0.0, 0.0)
            self.cmd_pub.publish(Twist())
            point = 10 if c == '0' else int(c)
            self.mark_pub.publish(String(data='{"point": %d, "stamp": %.3f}' % (point, rospy.Time.now().to_sec())))
            self.marks_done.append(str(point))
            redraw = True
        elif c == '=':
            self.step = min(self.step_max, self.step + 0.005); redraw = True
        elif c == '-':
            self.step = max(self.step_min, self.step - 0.005); redraw = True

        return redraw

    # -------------------------------------------------------------- main loop
    def spin(self):
        settings = termios.tcgetattr(sys.stdin)
        self.print_menu()
        rate = rospy.Rate(self.rate_hz)
        try:
            tty.setcbreak(sys.stdin.fileno())
            while not rospy.is_shutdown():
                # non-blocking read: gather all pending keys this cycle
                got_key = False
                while select.select([sys.stdin], [], [], 0)[0]:
                    c = sys.stdin.read(1)
                    got_key = True
                    if c == '\x03' or c == 'Q':      # Ctrl-C, or capital Q (some terminals / SSH clients swallow Ctrl-C)
                        raise KeyboardInterrupt
                    if self.handle_key(c):
                        self.print_menu()

                # dead-man: auto-stop base if no drive key held recently
                if (rospy.Time.now() - self.last_drive_key).to_sec() > self.hold:
                    self.drive = (0.0, 0.0)

                tw = Twist()
                tw.linear.x = self.drive[0]
                tw.angular.z = self.drive[1]
                self.cmd_pub.publish(tw)
                rate.sleep()
        except KeyboardInterrupt:
            pass
        finally:
            self.cmd_pub.publish(Twist())   # stop the base on exit
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

    def print_menu(self):
        sys.stdout.write(MENU.format(lin=self.linear, ang=self.angular, step=self.step,
                                     max_lin=self.max_linear, max_ang=self.max_angular,
                                     headlock='[HEAD LOCKED]' if self.lock_head else '',
                                     marks=('\n REFERENCE POINTS\n      1 .. 9, 0 : STOP and mark "standing on taped point N" (0 = point 10)\n'
                                            '      marked so far: %s\n' % (' '.join(self.marks_done) or '-'))
                                     if self.mark_points else ''))
        sys.stdout.flush()


def main():
    rospy.init_node('stretch_twist_arm_teleop', anonymous=True)
    TwistArmTeleop().spin()


if __name__ == '__main__':
    main()
