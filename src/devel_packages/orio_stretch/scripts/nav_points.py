#!/usr/bin/env python3
"""
nav_points.py - make or extend a waypoint file for nav_waypoints.py.

    scripts/nav_points.py config/waypoints_basement.yaml --from-run basement_map --orbslam ir-rgbd_lap1
        the taped points of a recorded run, where ORB-SLAM3 put them (first visit), in the upright map frame
        of that run - the frame the saved map / 2D map / frame_upright.json of that run share
    scripts/nav_points.py config/waypoints_basement.yaml --add shelf_A 3.1 -1.2 90
        add a point by hand: name x y yaw_deg (map frame)
    scripts/nav_points.py config/waypoints_basement.yaml --capture shelf_B
        add the robot's CURRENT pose (needs a running localiser: TF map -> base_link)
    scripts/nav_points.py config/waypoints_basement.yaml --face-next
        point every waypoint's heading towards the next one (the last keeps its own). Use it when a marked heading
        faces a wall: the camera then sees nothing to track there and the robot hesitates (corridor v2, point 3)
    scripts/nav_points.py config/waypoints_basement.yaml --list
"""
import argparse
import math
import os
import sys

import yaml


def load(path):
    if os.path.isfile(path):
        return yaml.safe_load(open(path)) or {'frame': 'map', 'waypoints': []}
    return {'frame': 'map', 'waypoints': []}


def save(path, spec):
    with open(path, 'w') as f:
        f.write('# waypoints for scripts/nav_waypoints.py (map frame, yaw in radians)\n')
        yaml.safe_dump(spec, f, sort_keys=False)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('file')
    ap.add_argument('--from-run', default='')
    ap.add_argument('--orbslam', default='ir-rgbd_lap1', help='ORB-SLAM3 run (mode[_tag]) whose frame the points are given in')
    ap.add_argument('--add', nargs=4, metavar=('NAME', 'X', 'Y', 'YAW_DEG'))
    ap.add_argument('--capture', metavar='NAME')
    ap.add_argument('--list', action='store_true')
    ap.add_argument('--face-next', action='store_true')
    args = ap.parse_args()
    spec = load(args.file)
    wps = spec['waypoints']

    if args.from_run:
        import rosbag
        sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
        import slam_bag as sb
        import marks_report as mr
        run_dir, bag_path = sb.resolve_run(args.from_run)
        bag = rosbag.Bag(bag_path)
        pose_at, _ = mr.traj_source(mr.orbslam_base_traj(run_dir, bag, args.orbslam))
        v = mr.visits(bag, pose_at)
        for p, vs in sorted(v.items()):
            ok = [one for one in vs if one is not None]
            if ok:
                wps = [w for w in wps if str(w['name']) != str(p)]
                wps.append({'name': str(p), 'x': round(float(ok[0][1]), 3), 'y': round(float(ok[0][2]), 3), 'yaw': round(float(ok[0][4]), 3)})
        spec['source'] = '%s / orbslam %s (frame_upright.json of that run)' % (os.path.basename(run_dir), args.orbslam)
    if args.add:
        name, x, y, yaw = args.add
        wps = [w for w in wps if str(w['name']) != name]
        wps.append({'name': name, 'x': float(x), 'y': float(y), 'yaw': round(math.radians(float(yaw)), 3)})
    if args.capture:
        import rospy
        import tf2_ros
        from tf.transformations import euler_from_quaternion
        rospy.init_node('nav_points_capture', anonymous=True)
        buf = tf2_ros.Buffer()
        tf2_ros.TransformListener(buf)
        tr = buf.lookup_transform(spec.get('frame', 'map'), 'base_link', rospy.Time(0), rospy.Duration(3.0))
        t, q = tr.transform.translation, tr.transform.rotation
        wps = [w for w in wps if str(w['name']) != args.capture]
        wps.append({'name': args.capture, 'x': round(t.x, 3), 'y': round(t.y, 3), 'yaw': round(euler_from_quaternion((q.x, q.y, q.z, q.w))[2], 3)})
    if args.face_next:
        for i, x in enumerate(wps[:-1]):
            n = wps[i + 1]
            if math.hypot(n['x'] - x['x'], n['y'] - x['y']) >= 0.05:
                x['yaw'] = round(math.atan2(n['y'] - x['y'], n['x'] - x['x']), 3)
    spec['waypoints'] = wps
    if args.from_run or args.add or args.capture or args.face_next:
        save(args.file, spec)
    print('%s (%s frame):' % (args.file, spec.get('frame', 'map')))
    for w in wps:
        print('  %-8s x=%7.3f  y=%7.3f  yaw=%5.0f deg' % (w['name'], w['x'], w['y'], math.degrees(w['yaw'])))


if __name__ == '__main__':
    main()
