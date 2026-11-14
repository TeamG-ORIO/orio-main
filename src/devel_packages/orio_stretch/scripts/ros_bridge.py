#!/usr/bin/env python3
"""
ros_bridge.py - the ROS side of demo_agent.py: robot state out, a few commands in, as JSON lines on stdout / stdin.

demo_agent.py has no ROS in it (the ROS master comes and goes with every run), so it starts this node whenever a
master is up and stops reading it when the master is gone. Not meant to be run by hand, but it can be:

    source scripts/orio_env.sh && python3 scripts/ros_bridge.py
    {"drive": [0.05, 0.0]}        <- type commands as JSON lines

stdout, twice a second:
    {"t": ..., "lift_m": ..., "arm_m": ..., "head_pan": ..., "head_tilt": ..., "runstop": bool, "battery_v": ...,
     "mode": "navigation", "localized": bool, "seeded": bool, "driving": bool,
     "jogging": ["lift"], "jog_blocked": {"lift": "contact"}}
stdin:
    {"drive": [v, w]}     base velocity (m/s, rad/s) on /stretch/cmd_vel. Must be repeated: it is held for 0.5 s only,
                          then zero is sent and the topic is left alone (so move_base can drive). Limits 0.15 m/s, 0.5 rad/s.
    {"jog": {"lift": 0.06, "pan": 0, "tilt": -0.3}, "lift_lim": [0.0, 1.099], "effort": 59}
                          hold-to-move for the lift (m/s) and the head camera (rad/s), signed speeds, 0 = let go. Like
                          drive it must be repeated: 0.5 s after the last one the joint is stopped. While held, a goal a
                          short way ahead of the joint (JOG_LOOK_S of travel + its stopping distance) is sent to the
                          driver five times a second; the driver blends a new goal into the running move, so the joint
                          keeps a steady speed, and it can never run further than that if this node dies. Letting go
                          sends the stopping distance only. "effort" is the lift's upward contact threshold with the
                          payload (config/payload.env); "lift_lim" the range. The lift does not move with the arm out
                          more than 5 cm, and after a contact stop it waits until the button is let go.
    {"runstop": true}     /runstop service of stretch_driver: true = stop all motion now and latch, false = release
    {"quit": 1}
"""
import json
import sys
import threading
import time

import actionlib
import rospy
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from geometry_msgs.msg import Twist
from sensor_msgs.msg import BatteryState, JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool
from trajectory_msgs.msg import JointTrajectoryPoint

V_MAX, W_MAX, HOLD_S = 0.15, 0.5, 0.5
ACTION = '/stretch_controller/follow_joint_trajectory'
# hold-to-move joints: state key, driver joint, top speed, acceleration sent with every goal (so the stopping distance
# is known: v^2 / 2a), default range. The lift comes first: the driver reads its contact threshold from effort[0].
JOG = {'lift': ('lift_m', 'joint_lift', 0.10, 0.15, (0.42, 1.00)),
       'pan': ('head_pan', 'joint_head_pan', 0.5, 4.0, (-1.6, 1.6)),
       'tilt': ('head_tilt', 'joint_head_tilt', 0.5, 4.0, (-1.0, 0.3))}
JOG_LOOK_S = 0.6        # how far ahead of the joint the goal is kept, in seconds of travel


def main():
    rospy.init_node('orio_demo_bridge', anonymous=True, disable_signals=False)
    st = {'lift_m': None, 'arm_m': None, 'head_pan': None, 'head_tilt': None, 'runstop': None, 'battery_v': None, 'mode': None, 'localized': None, 'seeded': None}
    cmd = {'v': 0.0, 'w': 0.0, 'until': 0.0, 'zeros': 0}
    jog = {j: {'v': 0.0, 'until': 0.0, 'was': 0.0, 'blocked': None, 'lim': JOG[j][4]} for j in JOG}
    lift_effort = [0.0]
    lock = threading.Lock()
    client = actionlib.SimpleActionClient(ACTION, FollowJointTrajectoryAction)

    def js(m):
        if 'joint_lift' in m.name:
            st['lift_m'] = round(m.position[m.name.index('joint_lift')], 4)
        if 'wrist_extension' in m.name:
            st['arm_m'] = round(m.position[m.name.index('wrist_extension')], 4)
        for key, joint in (('head_pan', 'joint_head_pan'), ('head_tilt', 'joint_head_tilt')):
            if joint in m.name:
                st[key] = round(m.position[m.name.index(joint)], 3)

    rospy.Subscriber('/stretch/joint_states', JointState, js, queue_size=1)
    rospy.Subscriber('/is_runstopped', Bool, lambda m: st.update(runstop=bool(m.data)), queue_size=1)
    rospy.Subscriber('/battery', BatteryState, lambda m: st.update(battery_v=round(m.voltage, 2)), queue_size=1)
    rospy.Subscriber('/mode', String, lambda m: st.update(mode=m.data), queue_size=1)
    rospy.Subscriber('/orbslam/localized', Bool, lambda m: st.update(localized=bool(m.data)), queue_size=1)
    rospy.Subscriber('/orbslam/seeded', Bool, lambda m: st.update(seeded=bool(m.data)), queue_size=1)
    pub = rospy.Publisher('/stretch/cmd_vel', Twist, queue_size=1)

    def jog_done(_state, result):
        # the driver ends a goal with "<joint> contact detected." when the lift's guarded contact stops it
        if result is not None and 'contact' in (result.error_string or ''):
            with lock:
                jog['lift']['was'] = 0.0                     # it has stopped: no stopping goal into the contact
                if jog['lift']['v'] != 0.0:
                    jog['lift']['blocked'] = 'contact: let go, then try again'

    def jog_tick(now, refresh):
        """one goal for all the joints that are held (a goal ahead of them) or were just let go (their stopping distance)"""
        names, pos, vel, acc, effort = [], [], [], [], None
        with lock:
            for j, (key, joint, _vmax, a, _lim) in JOG.items():
                s, p = jog[j], st.get(key)
                if s['v'] != 0.0 and now >= s['until']:        # let go, or the commands stopped coming
                    s['v'] = 0.0
                if s['v'] == 0.0:
                    s['blocked'] = None
                v, lo, hi = s['v'] or s['was'], s['lim'][0], s['lim'][1]
                if v == 0.0 or p is None:
                    s['was'] = 0.0
                    continue
                held = s['v'] != 0.0
                if held and s['blocked']:
                    continue
                if j == 'lift':
                    why = 'the arm is out: POSE AGAIN first' if (st.get('arm_m') or 0.0) > 0.05 else 'runstop is on' if st.get('runstop') else \
                          'no lift effort: HOME first' if v > 0 and lift_effort[0] <= 0 else None
                    if why:
                        s['blocked'], s['was'] = (why if held else None), 0.0
                        continue
                if held and not refresh and s['was'] != 0.0:
                    continue                                    # held: a new goal every other tick is enough
                sgn, stop = (1.0 if v > 0 else -1.0), v * v / (2.0 * a)
                target = min(hi, max(lo, p + sgn * (abs(v) * JOG_LOOK_S + stop if held else stop)))
                if held and abs(target - p) < 1e-3:
                    s['blocked'], s['was'] = 'at the end of its range (%.2f)' % target, 0.0
                    continue
                s['was'] = v if held else 0.0
                names.append(joint); pos.append(target); vel.append(abs(v)); acc.append(a)
                if j == 'lift' and v > 0:
                    effort = [lift_effort[0]]
        if not names:
            return
        if not client.wait_for_server(rospy.Duration(0.01)):
            with lock:
                for j in JOG:
                    if jog[j]['v'] != 0.0:
                        jog[j]['blocked'] = 'the driver is not up'
            return
        goal = FollowJointTrajectoryGoal(); goal.trajectory.joint_names = names
        pt = JointTrajectoryPoint(); pt.positions, pt.velocities, pt.accelerations = pos, vel, acc
        if effort:
            pt.effort = effort
        pt.time_from_start = rospy.Duration(1.0)
        goal.trajectory.points = [pt]; goal.trajectory.header.stamp = rospy.Time.now()
        client.send_goal(goal, done_cb=jog_done)

    def runstop(on):
        try:
            rospy.wait_for_service('/runstop', timeout=2.0)
            rospy.ServiceProxy('/runstop', SetBool)(bool(on))
            reply('runstop', ok=True, on=bool(on))
        except (rospy.ROSException, rospy.ServiceException) as e:
            reply('runstop', ok=False, error=str(e))

    def reply(what, **kw):
        sys.stdout.write(json.dumps(dict(reply=what, t=round(time.time(), 3), **kw)) + '\n'); sys.stdout.flush()

    def stdin_loop():
        for line in sys.stdin:
            try:
                c = json.loads(line)
            except ValueError:
                continue
            if 'drive' in c:
                v, w = float(c['drive'][0]), float(c['drive'][1])
                with lock:
                    cmd.update(v=max(-V_MAX, min(V_MAX, v)), w=max(-W_MAX, min(W_MAX, w)), until=time.time() + HOLD_S, zeros=3)
            if 'jog' in c:
                with lock:
                    if 'lift_lim' in c:
                        jog['lift']['lim'] = (float(min(c['lift_lim'])), float(max(c['lift_lim'])))
                    lift_effort[0] = float(c.get('effort', lift_effort[0]))
                    for j, v in c['jog'].items():
                        if j in jog:
                            vmax = JOG[j][2]; v = max(-vmax, min(vmax, float(v)))
                            if v * jog[j]['v'] < 0:
                                jog[j]['blocked'] = None            # the other way: a contact or end stop does not block that
                            jog[j].update(v=v, until=time.time() + HOLD_S)
            if 'runstop' in c:
                with lock:
                    cmd.update(v=0.0, w=0.0, until=0.0)
                    for s in jog.values():
                        s.update(v=0.0, was=0.0)     # the runstop stops the joints; no stopping goal after it
                threading.Thread(target=runstop, args=(c['runstop'],), daemon=True).start()
            if 'quit' in c:
                break
        rospy.signal_shutdown('stdin closed')      # the agent went away: do not keep driving

    threading.Thread(target=stdin_loop, daemon=True).start()
    rate, k = rospy.Rate(10.0), 0
    while not rospy.is_shutdown():
        with lock:
            live = time.time() < cmd['until']
            if live:
                tw = Twist(); tw.linear.x, tw.angular.z = cmd['v'], cmd['w']; pub.publish(tw)
            elif cmd['zeros'] > 0:                  # a few zeros after the last command, then hands off the topic
                pub.publish(Twist()); cmd['zeros'] -= 1
        k += 1
        try:
            jog_tick(time.time(), k % 2 == 0)
        except rospy.ROSException:
            pass
        if k % 5 == 0:
            with lock:
                jogging = [j for j, s in jog.items() if s['v'] != 0.0 and not s['blocked']]
                blocked = {j: s['blocked'] for j, s in jog.items() if s['blocked']}
            try:
                sys.stdout.write(json.dumps(dict(st, t=round(time.time(), 3), driving=live, jogging=jogging, jog_blocked=blocked)) + '\n'); sys.stdout.flush()
            except (BrokenPipeError, ValueError):
                break
        try:
            rate.sleep()
        except rospy.ROSInterruptException:
            break
    try:
        pub.publish(Twist())
    except Exception:
        pass


if __name__ == '__main__':
    main()
