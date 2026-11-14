#!/usr/bin/env python3
"""
pass_sidecar.py - small text logs next to the bag, so a pass can be checked in seconds (no 25 GB bag scan).

    pass_sidecar.py <run_dir>          runs until SIGINT/SIGTERM

Writes into <run_dir>:
    lift.csv      t, lift_m, lift_vel, lift_effort_pct, arm_m          every /stretch/joint_states message (~30 Hz)
    odom.csv      t, x, y, yaw, v_lin, v_ang                          /odom, decimated to 10 Hz
    events.jsonl  one JSON per line: marks (/slam_test/mark), lift strokes (/rfid_session/lift_state), runstop changes
    sidecar_summary.json   at exit: message counts, lidar scans and their share of finite ranges
All times are the robot clock (same as the bag and rfid_log.py's host_time_s).
"""
import json
import math
import os
import sys

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, LaserScan
from std_msgs.msg import Bool, String


def main():
    run = rospy.myargv()[1] if len(rospy.myargv()) > 1 else sys.exit(__doc__)
    rospy.init_node('pass_sidecar', anonymous=True)
    lift = open(os.path.join(run, 'lift.csv'), 'w', buffering=1); lift.write('t,lift_m,lift_vel,lift_effort_pct,arm_m\n')
    odom = open(os.path.join(run, 'odom.csv'), 'w', buffering=1); odom.write('t,x,y,yaw,v_lin,v_ang\n')
    ev = open(os.path.join(run, 'events.jsonl'), 'w', buffering=1)
    st = {'joint_states': 0, 'odom': 0, 'scans': 0, 'scan_finite_frac_sum': 0.0, 'scans_all_inf': 0, 'last_odom_t': 0.0, 'runstop': None, 'closed': False}

    def js(m):
        if st['closed'] or 'joint_lift' not in m.name:
            return
        i = m.name.index('joint_lift'); st['joint_states'] += 1
        arm = m.position[m.name.index('wrist_extension')] if 'wrist_extension' in m.name else float('nan')
        vel = m.velocity[i] if len(m.velocity) > i else float('nan'); eff = m.effort[i] if len(m.effort) > i else float('nan')
        lift.write('%.4f,%.4f,%.4f,%.2f,%.4f\n' % (m.header.stamp.to_sec(), m.position[i], vel, eff, arm))

    def od(m):
        t = m.header.stamp.to_sec(); st['odom'] += 1
        if st['closed'] or t - st['last_odom_t'] < 0.1:
            return
        st['last_odom_t'] = t; q = m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        odom.write('%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n' % (t, m.pose.pose.position.x, m.pose.pose.position.y, yaw, m.twist.twist.linear.x, m.twist.twist.angular.z))

    def sc(m):
        n = len(m.ranges); fin = sum(1 for r in m.ranges if math.isfinite(r)) if n else 0
        st['scans'] += 1; st['scan_finite_frac_sum'] += fin / n if n else 0.0; st['scans_all_inf'] += (fin == 0)

    def event(kind):
        def cb(m):
            if st['closed']:
                return
            try:
                d = json.loads(m.data)
            except ValueError:
                d = {'raw': m.data}
            d['kind'] = kind; d.setdefault('t', rospy.get_time()); ev.write(json.dumps(d) + '\n')
        return cb

    def rs(m):
        if not st['closed'] and m.data != st['runstop']:
            st['runstop'] = m.data; ev.write(json.dumps({'kind': 'runstop', 'on': m.data, 't': rospy.get_time()}) + '\n')

    rospy.Subscriber('/stretch/joint_states', JointState, js, queue_size=200)
    rospy.Subscriber('/odom', Odometry, od, queue_size=200)
    rospy.Subscriber('/scan', LaserScan, sc, queue_size=20)
    rospy.Subscriber('/slam_test/mark', String, event('mark'))
    rospy.Subscriber('/rfid_session/lift_state', String, event('lift'))
    rospy.Subscriber('/is_runstopped', Bool, rs)
    t0 = rospy.get_time()

    def done():
        st['closed'] = True
        s = {k: v for k, v in st.items() if k not in ('last_odom_t', 'scan_finite_frac_sum', 'closed')}
        s['scan_finite_frac_mean'] = st['scan_finite_frac_sum'] / st['scans'] if st['scans'] else None
        s['t_start'] = t0; s['t_end'] = rospy.get_time()
        json.dump(s, open(os.path.join(run, 'sidecar_summary.json'), 'w'), indent=1)
        for f in (lift, odom, ev):
            f.close()
    rospy.on_shutdown(done)
    rospy.spin()


if __name__ == '__main__':
    main()
