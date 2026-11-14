#!/usr/bin/env python3
"""
lift_motion.py - move the lift (and with it the RFID panel) during an RFID pass, through stretch_driver.

    lift_motion.py goto  0.60                        move once, wait, exit (0 = reached)
    lift_motion.py wave  --lo 0.42 --hi 0.90 --period 11     triangle wave until stopped (SIGINT/SIGTERM)
    lift_motion.py sweeps --lo 0.45 --hi 0.90 --rest 0.60    on every /slam_test/mark (teleop keys 1..9, 0):
                                                              rest -> lo -> hi -> rest, then wait for the next mark

Needs stretch_driver (any mode; the trajectory action accepts the lift in navigation mode, so the base keeps
driving from /stretch/cmd_vel while the lift moves). Every stroke is announced on /rfid_session/lift_state
(std_msgs/String, JSON, latched) so the bag and the sidecar log say what the lift was asked to do.

Safety: targets are clamped to [0.42, 1.00] m (below ~0.39 m the gripper payload hits the base, measured
2026-11-06); nothing moves while the arm is extended more than 5 cm or the
runstop is engaged (it waits). Upward strokes carry the payload contact threshold (--up-effort, effort %%,
default from config/payload.env), because the factory threshold stops the lift at once with the payload on.
A stroke that ends early (contact) is reported; three in a row stop the wave and hold the lift where it is.
Note: on a stop the driver still finishes the stroke it is executing (it does not halt on preemption).
"""
import argparse
import json
import os
import sys
import threading

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String

import orio_paths

ACTION = '/stretch_controller/follow_joint_trajectory'
LO_LIMIT, HI_LIMIT = 0.42, 1.00      # payload touches the base at 0.39 m (2026-11-06)
V_MAX = 0.15          # stretch_body lift 'max' profile
REACHED_TOL = 0.02


def payload_effort():
    try:
        for line in open(orio_paths.PAYLOAD_ENV):
            if line.startswith('LIFT_EFFORT='):
                return float(line.split('=', 1)[1])
    except (OSError, ValueError):
        pass
    return 0.0


def clamp(x):
    return min(HI_LIMIT, max(LO_LIMIT, x))


class Lift:
    def __init__(self, up_effort, accel):
        self.up_effort = up_effort; self.accel = accel
        self.pos = None; self.arm = None; self.runstop = False
        self.marks = []; self.mark_ev = threading.Event()
        self.pub = rospy.Publisher('/rfid_session/lift_state', String, queue_size=50, latch=True)
        rospy.Subscriber('/stretch/joint_states', JointState, self._js)
        rospy.Subscriber('/is_runstopped', Bool, self._rs)
        rospy.Subscriber('/slam_test/mark', String, self._mark)
        self.client = actionlib.SimpleActionClient(ACTION, FollowJointTrajectoryAction)
        if not self.client.wait_for_server(rospy.Duration(30)):
            sys.exit('lift_motion: no %s (is stretch_driver running?)' % ACTION)
        t0 = rospy.get_time()
        while self.pos is None and rospy.get_time() - t0 < 10 and not rospy.is_shutdown():
            rospy.sleep(0.1)
        if self.pos is None:
            sys.exit('lift_motion: no /stretch/joint_states')

    def _js(self, m):
        if 'joint_lift' in m.name:
            self.pos = m.position[m.name.index('joint_lift')]
        if 'wrist_extension' in m.name:
            self.arm = m.position[m.name.index('wrist_extension')]

    def _rs(self, m):
        self.runstop = m.data

    def _mark(self, m):
        try:
            self.marks.append(json.loads(m.data).get('point'))
        except ValueError:
            self.marks.append(None)
        self.mark_ev.set()

    def state(self, event, **kw):
        kw.update(event=event, t=round(rospy.get_time(), 3), lift=None if self.pos is None else round(self.pos, 4))
        self.pub.publish(String(data=json.dumps(kw)))
        rospy.loginfo('lift_motion: %s', json.dumps(kw))

    def safe_to_move(self):
        while not rospy.is_shutdown():
            if self.arm is not None and self.arm > 0.05:
                self.state('blocked', why='arm extended %.3f m' % self.arm); rospy.sleep(2.0); continue
            if self.runstop:
                self.state('blocked', why='runstop'); rospy.sleep(1.0); continue
            return True
        return False

    def move(self, target, v):
        """One stroke to target at speed v; True if it got within 2 cm."""
        target = clamp(target); v = min(max(v, 0.02), V_MAX)
        if not self.safe_to_move():
            return False
        up = target > self.pos
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = ['joint_lift']
        pt = JointTrajectoryPoint(); pt.positions = [target]; pt.velocities = [v]
        if self.accel > 0:
            pt.accelerations = [self.accel]
        if up and self.up_effort > 0:
            pt.effort = [self.up_effort]
        pt.time_from_start = rospy.Duration(abs(target - self.pos) / v)
        goal.trajectory.points = [pt]; goal.trajectory.header.stamp = rospy.Time.now()
        self.state('stroke', target=round(target, 3), v=round(v, 3), dir='up' if up else 'down')
        self.client.send_goal(goal)
        budget = abs(target - self.pos) / v + 8.0
        if not self.client.wait_for_result(rospy.Duration(budget)):
            self.client.cancel_goal(); self.state('timeout', target=round(target, 3)); return False
        rospy.sleep(0.15)       # last joint_states
        ok = abs(self.pos - target) <= REACHED_TOL
        if not ok:
            r = self.client.get_result()
            self.state('short', target=round(target, 3), why=(r.error_string if r else 'no result'))
        return ok

    def stop(self):
        try:
            self.client.cancel_all_goals()
        except Exception:
            pass


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)
    g = sub.add_parser('goto'); g.add_argument('height', type=float); g.add_argument('--v', type=float, default=0.10)
    w = sub.add_parser('wave'); w.add_argument('--lo', type=float, default=0.42); w.add_argument('--hi', type=float, default=0.90)
    w.add_argument('--period', type=float, default=11.0, help='seconds for one full up-and-down')
    s = sub.add_parser('sweeps'); s.add_argument('--lo', type=float, default=0.45); s.add_argument('--hi', type=float, default=0.90)
    s.add_argument('--rest', type=float, default=0.60); s.add_argument('--v', type=float, default=0.10)
    for p in (g, w, s):
        p.add_argument('--up-effort', type=float, default=None, help='upward contact threshold, effort %% (default: LIFT_EFFORT from config/payload.env, written by home_payload.py)')
        p.add_argument('--accel', type=float, default=0.0, help='m/s^2, 0 = driver default (0.15)')
    a = ap.parse_args(rospy.myargv()[1:])
    rospy.init_node('lift_motion', anonymous=True)
    eff = payload_effort() if a.up_effort is None else a.up_effort
    if not eff:
        sys.exit('lift_motion: no upward contact threshold. Home with the payload first (scripts/home_payload.py writes '
                 'config/payload.env), or pass --up-effort; the factory threshold stops the lift with the payload on.')
    lift = Lift(eff, a.accel)
    rospy.on_shutdown(lift.stop)

    if a.cmd == 'goto':
        ok = lift.move(a.height, a.v)
        print('POSE lift=%.3f reached=%s' % (lift.pos, ok))
        sys.exit(0 if ok else 2)

    if a.cmd == 'wave':
        lo, hi = clamp(min(a.lo, a.hi)), clamp(max(a.lo, a.hi))
        if hi - lo < 0.05:
            sys.exit('lift_motion: wave range too small')
        v = 2 * (hi - lo) / a.period
        if v > V_MAX:
            rospy.logwarn('lift_motion: %.2f m in %.1f s needs %.3f m/s; capped at %.2f (period becomes %.1f s)', hi - lo, a.period, v, V_MAX, 2 * (hi - lo) / V_MAX)
            v = V_MAX
        lift.state('wave_start', lo=lo, hi=hi, v=round(v, 3), period=a.period)
        # start at the nearer end
        target = lo if abs(lift.pos - lo) < abs(lift.pos - hi) else hi
        fails = 0; strokes = 0
        while not rospy.is_shutdown():
            if lift.move(target, v):
                fails = 0
            else:
                fails += 1
                if fails >= 3:
                    lift.state('wave_gave_up', why='3 short strokes in a row; holding'); break
            strokes += 1
            target = hi if target == lo else lo
        lift.state('wave_end', strokes=strokes)
        rospy.spin()
        return

    if a.cmd == 'sweeps':
        lo, hi, rest = clamp(min(a.lo, a.hi)), clamp(max(a.lo, a.hi)), clamp(a.rest)
        lift.move(rest, a.v)
        lift.state('sweeps_ready', lo=lo, hi=hi, rest=rest)
        n = 0
        while not rospy.is_shutdown():
            if not lift.mark_ev.wait(0.5):
                continue
            lift.mark_ev.clear()
            point = lift.marks[-1] if lift.marks else None
            n += 1
            lift.state('sweep_start', n=n, point=point)
            ok = lift.move(lo, a.v) and lift.move(hi, a.v)
            ok = lift.move(rest, a.v) and ok
            lift.state('sweep_end', n=n, point=point, ok=ok)
            lift.mark_ev.clear()      # marks pressed during a sweep do not queue another one


if __name__ == '__main__':
    main()
