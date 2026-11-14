#!/usr/bin/env python3
"""
nav_waypoints.py - drive a list of waypoints with move_base and score each arrival (T6).

    scripts/nav_waypoints.py config/waypoints_basement.yaml [--tolerance 0.25] [--timeout 120] [--pause]
                             [--dry-run] [--out datasets/nav_<date>.csv] [--events FILE] [--control FILE]

The waypoint file (scripts/nav_points.py writes one) lists goals in the map frame:
    frame: map
    waypoints:
      - {name: "1", x: 0.0, y: 0.0, yaw: 0.0}
      - {name: "2", x: 2.43, y: -0.30, yaw: 0.0}

For every waypoint: send it to move_base, wait for the result, read the robot's pose from TF (map -> base_link,
whatever localiser publishes it: AMCL or orbslam_tf_bridge), and record the distance to the goal and the time
taken. --pause waits for Enter before each goal (so you can measure the real position with the tape).
--dry-run sends nothing and only prints where the robot is. Writes a CSV and prints a pass/fail table, plus
<log>_path.csv with the driven path at 5 Hz (~/orio/scripts/nav_plot.py draws runs on the map).

Retries (--retries, default 1): a goal that ends ABORTED or TIMEOUT is retried after clearing move_base's
costmaps (/move_base/clear_costmaps). The camera only forgets an obstacle when it looks at that spot again, so
a person who stood in the corridor and walked away otherwise blocks the route for good (corridor v2 lap 2,
2026-11-07). Before each retry the robot's view is saved as <out>_fail_<waypoint>_<n>.npz: local and global
costmap, robot pose, latest obstacle cloud (/camera/points_nav, base_link), for diagnosis.

Live progress and pause (used by demo_agent.py and the demo GUI): --events FILE gets one JSON line per step (start,
goal, retry, result, paused, resumed, done). --control FILE is read five times a second: while its content is
"pause" the current goal is cancelled and the robot holds; when it says anything else (or is removed) the same
goal is sent again. A pause is not counted as a retry; the time spent paused is in the paused_s column.

Note: the "error" here is the LOCALISER's own opinion of where it stopped relative to the goal. T6's
25 cm criterion is measured with the tape; this script tells you what the robot believed.
"""
import argparse
import csv
import json
import math
import os
import sys
import threading
import time

import rospy
import yaml
import actionlib
import tf2_ros
import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import Empty
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler, euler_from_quaternion

import orio_paths

STATUS = {0: 'PENDING', 1: 'ACTIVE', 2: 'PREEMPTED', 3: 'SUCCEEDED', 4: 'ABORTED', 5: 'REJECTED', 8: 'RECALLED'}


def robot_pose(buf, frame):
    tr = buf.lookup_transform(frame, 'base_link', rospy.Time(0), rospy.Duration(1.0))
    t, q = tr.transform.translation, tr.transform.rotation
    return t.x, t.y, euler_from_quaternion((q.x, q.y, q.z, q.w))[2]


def save_failure(out, name, n, buf, frame):
    """the robot's view at a failure: costmaps, pose, latest obstacle cloud -> <out>_fail_<name>_<n>.npz"""
    path = out.replace('.csv', '_fail_%s_%d.npz' % (name, n))
    d = {}
    for key, topic in (('local', '/move_base/local_costmap/costmap'), ('global', '/move_base/global_costmap/costmap')):
        try:
            g = rospy.wait_for_message(topic, OccupancyGrid, timeout=3.0)
            d[key] = np.array(g.data, np.int8).reshape(g.info.height, g.info.width)
            d[key + '_meta'] = np.array([g.info.resolution, g.info.origin.position.x, g.info.origin.position.y])
            d[key + '_frame'] = np.array(g.header.frame_id)
        except rospy.ROSException:
            pass
    try:
        c = rospy.wait_for_message('/camera/points_nav', PointCloud2, timeout=3.0)
        d['cloud'] = np.frombuffer(c.data, np.float32).reshape(-1, 3).copy()
        d['cloud_frame'] = np.array(c.header.frame_id)
    except rospy.ROSException:
        pass
    try:
        d['pose_map'] = np.array(robot_pose(buf, frame))
    except (tf2_ros.LookupException, tf2_ros.ExtrapolationException, tf2_ros.ConnectivityException):
        pass
    np.savez_compressed(path, **d)
    return path


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('waypoints')
    ap.add_argument('--tolerance', type=float, default=0.25)
    ap.add_argument('--timeout', type=float, default=120.0, help='seconds per waypoint before giving up')
    ap.add_argument('--pause', action='store_true', help='wait for Enter before each waypoint')
    ap.add_argument('--dry-run', action='store_true')
    ap.add_argument('--retries', type=int, default=1, help='retries per waypoint after clearing the costmaps')
    ap.add_argument('--out', default='')
    ap.add_argument('--events', default='', help='append one JSON line per step to this file (live progress)')
    ap.add_argument('--control', default='', help='file holding "pause" or "run"; checked while driving')
    args = ap.parse_args(rospy.myargv()[1:])
    spec = yaml.safe_load(open(args.waypoints))
    frame = spec.get('frame', 'map')
    wps = spec['waypoints']

    rospy.init_node('nav_waypoints')
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf)
    rospy.sleep(1.0)
    try:
        x, y, yaw = robot_pose(buf, frame)
    except (tf2_ros.LookupException, tf2_ros.ExtrapolationException, tf2_ros.ConnectivityException) as e:
        raise SystemExit('no %s -> base_link transform: is the localiser running and localised? (%s)' % (frame, e))
    print('robot now at x=%.2f y=%.2f yaw=%.0f deg in %s' % (x, y, math.degrees(yaw), frame))
    if args.dry_run:
        for w in wps:
            print('  %-6s goal x=%.2f y=%.2f  distance from here %.2f m' % (w['name'], w['x'], w['y'], math.hypot(w['x'] - x, w['y'] - y)))
        return

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    if not client.wait_for_server(rospy.Duration(30.0)):
        raise SystemExit('move_base is not running')
    out = args.out or os.path.join(orio_paths.DATASETS_DIR, 'nav_%s.csv' % time.strftime('%Y%m%d_%H%M%S'))
    rows = []
    # driven path at 5 Hz -> <out>_path.csv (~/orio/scripts/nav_plot.py draws it)
    path_file = open(out.replace('.csv', '_path.csv'), 'w')
    path_file.write('stamp,x,y,yaw\n')
    stop_log = threading.Event()

    def log_path():
        rate = rospy.Rate(5.0)
        while not stop_log.is_set() and not rospy.is_shutdown():
            try:
                px, py, pyaw = robot_pose(buf, frame)
                path_file.write('%.3f,%.3f,%.3f,%.3f\n' % (rospy.Time.now().to_sec(), px, py, pyaw))
            except (tf2_ros.LookupException, tf2_ros.ExtrapolationException, tf2_ros.ConnectivityException):
                pass
            rate.sleep()
        path_file.close()
    threading.Thread(target=log_path, daemon=True).start()
    ev_file = open(args.events, 'a', buffering=1) if args.events else None

    def event(kind, **kw):
        if ev_file:
            ev_file.write(json.dumps(dict(t=round(time.time(), 3), kind=kind, **kw)) + '\n')

    def pause_wanted():
        if not args.control:
            return False
        try:
            return open(args.control).read().strip() == 'pause'
        except OSError:
            return False

    def hold(name):
        """block while the control file says pause -> seconds spent paused"""
        if not pause_wanted():
            return 0.0
        t_p = time.time()
        print('   PAUSED before/at waypoint %s (waiting for resume)' % name)
        event('paused', name=str(name))
        while pause_wanted() and not rospy.is_shutdown():
            time.sleep(0.2)
        event('resumed', name=str(name), paused_s=round(time.time() - t_p, 1))
        print('   resumed after %.0f s' % (time.time() - t_p))
        return time.time() - t_p

    def wait_goal():
        """True = move_base finished, False = timed out, None = pause requested (goal cancelled)"""
        t_end = time.time() + args.timeout
        while not rospy.is_shutdown():
            if client.wait_for_result(rospy.Duration(0.2)):
                return True
            if pause_wanted():
                client.cancel_goal()
                rospy.sleep(0.5)
                return None
            if time.time() > t_end:
                return False
        return True

    event('start', n=len(wps), names=[str(w['name']) for w in wps])
    for wi, w in enumerate(wps):
        if args.pause:
            input('\nPress Enter to drive to waypoint %s (x=%.2f y=%.2f) ... ' % (w['name'], w['x'], w['y']))
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = frame
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x, goal.target_pose.pose.position.y = float(w['x']), float(w['y'])
        q = quaternion_from_euler(0, 0, float(w.get('yaw', 0.0)))
        goal.target_pose.pose.orientation.x, goal.target_pose.pose.orientation.y, goal.target_pose.pose.orientation.z, goal.target_pose.pose.orientation.w = q
        t0 = rospy.Time.now()
        print('\n-> waypoint %s: x=%.2f y=%.2f yaw=%.0f deg' % (w['name'], w['x'], w['y'], math.degrees(float(w.get('yaw', 0.0)))))
        tries = 0
        paused_s = 0.0
        while True:
            paused_s += hold(w['name'])
            goal.target_pose.header.stamp = rospy.Time.now()
            client.send_goal(goal)
            event('goal', i=wi + 1, n=len(wps), name=str(w['name']), x=float(w['x']), y=float(w['y']), tries=tries)
            done = wait_goal()
            if done is None:        # paused: hold() blocks at the top of the loop, then the same goal is sent again
                continue
            if not done:
                client.cancel_goal()
                rospy.sleep(1.0)
            status = STATUS.get(client.get_state(), str(client.get_state())) if done else 'TIMEOUT'
            if status in ('SUCCEEDED', 'PREEMPTED') or tries >= args.retries or rospy.is_shutdown():
                break
            tries += 1
            event('retry', i=wi + 1, name=str(w['name']), status=status, tries=tries)
            snap = save_failure(out, w['name'], tries, buf, frame)
            print('   %s - saved the robot\'s view to %s; clearing the costmaps and retrying (%d/%d)' % (status, snap, tries, args.retries))
            try:
                rospy.ServiceProxy('/move_base/clear_costmaps', Empty)()
            except rospy.ServiceException as e:
                print('   could not clear the costmaps: %s' % e)
            rospy.sleep(2.0)       # let the camera repopulate what is really there
        dt = (rospy.Time.now() - t0).to_sec()
        try:
            x, y, yaw = robot_pose(buf, frame)
            err = math.hypot(x - float(w['x']), y - float(w['y']))
            yaw_err = math.degrees(abs(math.atan2(math.sin(yaw - float(w.get('yaw', 0.0))), math.cos(yaw - float(w.get('yaw', 0.0))))))
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException, tf2_ros.ConnectivityException):
            x = y = yaw = err = yaw_err = float('nan')
        ok = status == 'SUCCEEDED' and err <= args.tolerance
        print('   %s after %.0f s, stopped at x=%.2f y=%.2f: %.2f m from the goal, heading off by %.0f deg  -> %s'
              % (status, dt, x, y, err, yaw_err, 'ok' if ok else 'FAIL'))
        rows.append(dict(name=w['name'], gx=w['x'], gy=w['y'], gyaw=w.get('yaw', 0.0), status=status, retries=tries, seconds=round(dt, 1),
                         x=round(x, 3), y=round(y, 3), yaw=round(yaw, 3), error_m=round(err, 3), yaw_error_deg=round(yaw_err, 1), ok=ok,
                         paused_s=round(paused_s, 1)))
        event('result', i=wi + 1, n=len(wps), name=str(w['name']), status=status, ok=bool(ok), retries=tries, seconds=round(dt, 1),
              error_m=None if math.isnan(err) else round(err, 3), paused_s=round(paused_s, 1))
        if rospy.is_shutdown():
            break
    stop_log.set()
    with open(out, 'w') as f:
        wr = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        wr.writeheader()
        wr.writerows(rows)
    n_ok = sum(r['ok'] for r in rows)
    event('done', ok=int(n_ok), n=len(rows))
    print('\n%d of %d waypoints reached within %.2f m (by the localiser\'s own estimate). Log: %s' % (n_ok, len(rows), args.tolerance, out))


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
