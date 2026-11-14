#!/usr/bin/env python3
"""
marks_report.py - score a run at the taped reference points you marked while recording
(number keys 1..9 in the slam_record.sh teleop -> /slam_test/mark in the bag).

    scripts/marks_report.py <run>                                    # where does wheel odometry put the points?
    scripts/marks_report.py <run> --truth config/truth_b512.txt      # + compare with tape-measured distances
    scripts/marks_report.py <run> --orbslam ir-rgbd                  # use an ORB-SLAM3 run's poses instead
    scripts/marks_report.py <run> --traj poses.tum                   # or any method's robot poses (TUM file)
    scripts/marks_report.py <run> --reference <mapping_run> [...]    # + compare with the same points in another run

Position source (where the robot "was" at each mark):
    default        wheel odometry, odom -> base_link from the bag
    --orbslam M    the final trajectory of orbslam_run.sh mode M on this run (<run>/orbslam/M/final_cam_traj.txt),
                   moved from the camera to base_link with the robot's TF and rotated upright (one fixed
                   rotation + shift for the whole run, so drift and distances are untouched).
                   Also written as <run>/orbslam/M/base_link.tum for other tools.
    --traj FILE    TUM file "stamp x y z qx qy qz qw" of base_link in ANY fixed frame (a SLAM or
                   localisation method's output). Poses are interpolated to the mark time.

What it prints:
    points         x, y, heading of the robot at every mark (a point marked twice = two visits)
    revisits       distance between two visits of the same point: the drift accumulated in between
                   (for a method with loop closure / a localiser this should be ~0)
    --truth        for every tape-measured pair: distance according to this run vs the tape
                   (distances do not depend on any coordinate frame, so no alignment is involved)
    --reference    per-point position difference to the reference run. Wheel odometry starts at a
                   different origin every run, so by default the two point sets are first fitted onto
                   each other (rotation + shift); with --same-frame they are compared as they are
                   (use that when both come from a localiser in the same saved map: this is T5's error).

Truth file: one tape measurement per line, "point_a point_b distance_m", '#' starts a comment, e.g.
    1 2 3.42
    2 3 2.10
    1 3 4.05
Measure between the SAME spot of the robot's footprint at each point (the floor point under the centre
of the wheel axle = base_link), i.e. tape cross to tape cross, and park the robot the same way each time.
"""
import argparse
import bisect
import itertools
import json
import math
import sys

import numpy as np
import rosbag
import rospy

import slam_bag as sb

MARK_TOPIC = '/slam_test/mark'


def yaw_of(q):
    return math.atan2(2.0 * (q[3] * q[2] + q[0] * q[1]), 1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2]))


def read_marks(bag):
    """[(point, stamp)] in time order"""
    out = []
    for _, msg, t in bag.read_messages(topics=[MARK_TOPIC]):
        try:
            d = json.loads(msg.data)
            out.append((int(d['point']), float(d.get('stamp', t.to_sec()))))
        except (ValueError, KeyError):
            continue
    return out


def odom_source(bag):
    """pose_at(t) -> (x, y, yaw) of base_link in the odom frame, from TF (or the /odom topic)"""
    buf = sb.tf_buffer(bag)
    if 'odom' in buf.all_frames_as_string():
        def pose_at(t):
            try:
                tr = buf.lookup_transform_core('odom', 'base_link', rospy.Time.from_sec(t))
            except Exception:      # tf2 lookup / extrapolation errors
                return None
            p, q = tr.transform.translation, tr.transform.rotation
            return p.x, p.y, p.z, yaw_of((q.x, q.y, q.z, q.w))
        return pose_at, 'wheel odometry (TF odom -> base_link)'
    rows = []
    for _, msg, _ in bag.read_messages(topics=['/odom']):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        rows.append([msg.header.stamp.to_sec(), p.x, p.y, p.z, q.x, q.y, q.z, q.w])
    return traj_source(np.array(rows))[0], 'wheel odometry (/odom topic)'


def traj_source(traj, max_gap=0.5):
    stamps = list(traj[:, 0]) if len(traj) else []

    def pose_at(t):
        i = bisect.bisect_left(stamps, t)
        if i < len(stamps) and abs(stamps[i] - t) < 1e-4:
            a = b = traj[i]
            w = 0.0
        elif 0 < i < len(stamps) and stamps[i] - stamps[i - 1] <= max_gap:
            a, b = traj[i - 1], traj[i]
            w = (t - a[0]) / (b[0] - a[0])
        else:
            return None            # outside the trajectory or inside a gap (method was lost)
        ya, yb = yaw_of(a[4:8]), yaw_of(b[4:8])
        yaw = ya + w * math.atan2(math.sin(yb - ya), math.cos(yb - ya))
        return (1 - w) * a[1] + w * b[1], (1 - w) * a[2] + w * b[2], (1 - w) * a[3] + w * b[3], yaw
    return pose_at, 'trajectory file'


def visits(bag, pose_at):
    """{point: [(t_since_start, x, y, z, yaw) or None per visit]}"""
    t0 = bag.get_start_time()
    out = {}
    for point, stamp in read_marks(bag):
        p = pose_at(stamp)
        out.setdefault(point, []).append(None if p is None else (stamp - t0,) + tuple(p))
    return out


def first_xyz(v):
    """{point: (x, y, z)} using the FIRST valid visit of each point"""
    out = {}
    for point, vs in v.items():
        for one in vs:
            if one is not None:
                out[point] = np.array(one[1:4])
                break
    return out


def fit_2d(src, dst):
    """rotation + shift (no scale) that best maps src points onto dst points; returns mapped src"""
    mu_s, mu_d = src.mean(0), dst.mean(0)
    a, b = src - mu_s, dst - mu_d
    th = math.atan2((a[:, 0] * b[:, 1] - a[:, 1] * b[:, 0]).sum(), (a * b).sum())
    c, s = math.cos(th), math.sin(th)
    return a.dot(np.array([[c, s], [-s, c]])) + mu_d


def rot_to_quat(m):
    w = math.sqrt(max(0.0, 1 + m[0, 0] + m[1, 1] + m[2, 2])) / 2
    x = math.sqrt(max(0.0, 1 + m[0, 0] - m[1, 1] - m[2, 2])) / 2
    y = math.sqrt(max(0.0, 1 - m[0, 0] + m[1, 1] - m[2, 2])) / 2
    z = math.sqrt(max(0.0, 1 - m[0, 0] - m[1, 1] + m[2, 2])) / 2
    x = math.copysign(x, m[2, 1] - m[1, 2])
    y = math.copysign(y, m[0, 2] - m[2, 0])
    z = math.copysign(z, m[1, 0] - m[0, 1])
    return x, y, z, w


def orbslam_base_traj(run_dir, bag, mode, raw=False):
    """ORB-SLAM3's camera trajectory as base_link poses (TUM rows), using the bag's TF for camera -> base_link"""
    import os
    from orbslam_eval import read_traj
    from orbslam_map import quat_to_rot
    out_dir = os.path.join(run_dir, 'orbslam', mode)
    traj = read_traj(os.path.join(out_dir, 'final_cam_traj.txt'))
    if not len(traj):
        raise SystemExit('no final trajectory in %s' % out_dir)
    frame = None
    for line in open(os.path.join(out_dir, 'settings.yaml')):
        if line.startswith('# camera frame '):
            frame = line.split()[3].rstrip(',')
    if 'inertial' in mode:
        frame = sb.first_msg(bag, sb.IMU).header.frame_id
    from orbslam_map import rigid_from_poses
    buf = sb.tf_buffer(bag)
    # ORB-SLAM3's world frame is its first camera image (z = optical axis, y = down). Rotate the whole
    # trajectory upright (z up, x/y = floor) with the same alignment orbslam_map.py uses, so that the
    # x/y positions this report works with are floor positions.
    est_p, est_r, ref_p, ref_r = [], [], [], []
    for r in traj:
        try:
            tr = buf.lookup_transform_core('odom', frame, rospy.Time.from_sec(r[0]))
        except Exception:
            continue
        q, pp = tr.transform.rotation, tr.transform.translation
        est_p.append(r[1:4]); est_r.append(quat_to_rot(r[4:8]))
        ref_p.append([pp.x, pp.y, pp.z]); ref_r.append(quat_to_rot((q.x, q.y, q.z, q.w)))
    if len(est_p) < 10:
        raise SystemExit('no wheel-odometry TF for the camera frame %s: cannot put the ORB-SLAM3 trajectory upright' % frame)
    R_up, t_up = rigid_from_poses(np.array(est_p), est_r, np.array(ref_p), ref_r)
    if raw:
        # two ORB-SLAM3 runs sharing one map frame are compared as they are: an upright rotation fitted
        # per run would differ slightly between them and show up as a fake error
        R_up, t_up = np.eye(3), np.zeros(3)
    rows = []
    for r in traj:
        try:
            tr = buf.lookup_transform_core(frame, 'base_link', rospy.Time.from_sec(r[0]))     # base_link as seen from the camera
        except Exception:
            continue
        q, pp = tr.transform.rotation, tr.transform.translation
        R_wc, t_wc = R_up.dot(quat_to_rot(r[4:8])), R_up.dot(r[1:4]) + t_up
        R_cb, t_cb = quat_to_rot((q.x, q.y, q.z, q.w)), np.array([pp.x, pp.y, pp.z])
        R_wb, t_wb = R_wc.dot(R_cb), R_wc.dot(t_cb) + t_wc
        rows.append([r[0]] + list(t_wb) + list(rot_to_quat(R_wb)))
    rows = np.array(rows)
    np.savetxt(os.path.join(out_dir, 'base_link.tum'), rows, fmt='%.9f %.6f %.6f %.6f %.7f %.7f %.7f %.7f')
    if not raw:
        # the fixed rotation + shift that turns ORB-SLAM3's world frame into the upright map frame
        # (used live by orbslam_tf_bridge.py so the robot's map frame is the one the 2D map was drawn in)
        json.dump({'R_map_orb': R_up.tolist(), 't_map_orb': t_up.tolist(), 'camera_frame': frame, 'mode': mode,
                   'run': os.path.basename(run_dir)}, open(os.path.join(out_dir, 'frame_upright.json'), 'w'), indent=1)
    return rows


def load_source(run, traj_file, orbslam_mode='', raw=False):
    run_dir, bag_path = sb.resolve_run(run)
    bag = rosbag.Bag(bag_path)
    if orbslam_mode:
        pose_at, _ = traj_source(orbslam_base_traj(run_dir, bag, orbslam_mode, raw))
        return bag, pose_at, 'ORB-SLAM3 %s (camera trajectory moved to base_link%s)' % (
            orbslam_mode, ", ORB-SLAM3's own map frame" if raw else ', rotated upright to match the odometry frame')
    if traj_file:
        rows = [[float(x) for x in line.split()] for line in open(traj_file) if len(line.split()) == 8 and not line.startswith('#')]
        pose_at, name = traj_source(np.array(rows))
        name = '%s %s' % (name, traj_file)
    else:
        pose_at, name = odom_source(bag)
    return bag, pose_at, name


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('run')
    ap.add_argument('--traj', default='', help='TUM trajectory of base_link from the method under test (default: wheel odometry)')
    ap.add_argument('--orbslam', default='', metavar='MODE', help='use the ORB-SLAM3 run <run>/orbslam/MODE instead of wheel odometry')
    ap.add_argument('--truth', default='', help='tape measurements: lines "point_a point_b distance_m"')
    ap.add_argument('--reference', default='', help='another run with the same points marked (e.g. the mapping run)')
    ap.add_argument('--reference-traj', default='', help='TUM trajectory for the reference run (default: its wheel odometry)')
    ap.add_argument('--reference-orbslam', default='', metavar='MODE', help='ORB-SLAM3 mode to use for the reference run')
    ap.add_argument('--same-frame', action='store_true', help='with --reference: both are in the same map frame, do not fit them onto each other '
                    '(e.g. two ORB-SLAM3 runs where the second loaded the first one\'s map)')
    ap.add_argument('--tolerance', type=float, default=0.25, help='pass/fail threshold in metres (T5: 0.25)')
    args = ap.parse_args()

    raw = bool(args.same_frame and args.orbslam and args.reference_orbslam)
    bag, pose_at, name = load_source(args.run, args.traj, args.orbslam, raw)
    v = visits(bag, pose_at)
    if not v:
        raise SystemExit('no reference points were marked in this run (%s is empty). Press 1..9 in the slam_record.sh teleop '
                         'while standing on a taped point.' % MARK_TOPIC)
    print('run:     %s' % args.run)
    print('source:  %s' % name)
    print('\npoint  visit   time [s]      x [m]      y [m]   heading [deg]')
    for point in sorted(v):
        for k, one in enumerate(v[point]):
            if one is None:
                print('%5d  %5d   (no pose at this moment: the method was lost or had not started)' % (point, k + 1))
            else:
                print('%5d  %5d   %8.1f   %8.3f   %8.3f   %8.1f' % (point, k + 1, one[0], one[1], one[2], math.degrees(one[4])))

    re = [(p, k, j, float(np.linalg.norm(np.array(vs[j][1:4]) - np.array(vs[k][1:4]))))
          for p, vs in sorted(v.items()) for k, j in itertools.combinations(range(len(vs)), 2)
          if vs[k] is not None and vs[j] is not None]
    if re:
        print('\nrevisits of the same point (0 = no drift in between):')
        for p, k, j, d in re:
            print('  point %d  visit %d -> %d:  %.3f m%s' % (p, k + 1, j + 1, d, '' if d <= args.tolerance else '   > %.2f m' % args.tolerance))

    xy = first_xyz(v)
    if args.truth:
        print('\ndistances against the tape (%s):' % args.truth)
        print('  pair      tape [m]   this run [m]   error [m]')
        errs = []
        for line in open(args.truth):
            f = line.split('#')[0].split()
            if len(f) != 3:
                continue
            a, b, d = int(f[0]), int(f[1]), float(f[2])
            if a not in xy or b not in xy:
                print('  %d - %d   %8.3f   (point not marked in this run)' % (a, b, d))
                continue
            est = float(np.linalg.norm(xy[a] - xy[b]))
            errs.append(est - d)
            print('  %d - %d   %8.3f   %10.3f   %+9.3f' % (a, b, d, est, est - d))
        if errs:
            print('  mean |error| %.3f m, worst %.3f m over %d pairs' % (np.mean(np.abs(errs)), np.max(np.abs(errs)), len(errs)))

    if args.reference:
        rbag, rpose, rname = load_source(args.reference, args.reference_traj, args.reference_orbslam, raw)
        rxy = first_xyz(visits(rbag, rpose))
        common = sorted(set(xy) & set(rxy))
        print('\nagainst reference run %s (%s):' % (args.reference, rname))
        if len(common) < (1 if args.same_frame else 3):
            print('  only %d common point(s) - need %s' % (len(common), '1' if args.same_frame else '3 to fit the two runs onto each other'))
        else:
            src = np.array([xy[p] for p in common])
            dst = np.array([rxy[p] for p in common])
            if args.same_frame:
                print('  compared in the shared map frame, no fitting')
            else:
                src = fit_2d(src[:, :2], dst[:, :2])
                dst = dst[:, :2]
                print('  fitted onto the reference (rotation + shift): this shows shape agreement, not absolute position')
            d = np.linalg.norm(src - dst, axis=1)
            for p, e in zip(common, d):
                print('  point %d:  %.3f m   %s' % (p, e, 'ok' if e <= args.tolerance else 'FAIL (> %.2f m)' % args.tolerance))
            print('  worst %.3f m -> %s at %.2f m tolerance' % (d.max(), 'PASS' if d.max() <= args.tolerance else 'FAIL', args.tolerance))


if __name__ == '__main__':
    sys.exit(main())
