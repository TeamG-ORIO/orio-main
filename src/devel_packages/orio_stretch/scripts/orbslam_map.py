#!/usr/bin/env python3
"""
orbslam_map.py - pictures of the map ORB-SLAM3 built in one offline run.

    scripts/orbslam_map.py <run> <mode[_tag]>          e.g.  scripts/orbslam_map.py lab_loop_01 rgbd

Needs <run>/orbslam/<mode>/map_points.npy (written by orbslam_run.sh) and final_cam_traj.txt.
The ORB-SLAM3 map lives in the frame of its first camera image; here it is rotated into the robot's
odometry frame (z up) by matching ORB-SLAM3's camera orientations to the robot's (TF), so "top view"
really is seen from above - also on a straight drive, where positions alone leave the roll undefined.

Writes into the same folder:
    map_top.png   top view: map points coloured by height, camera path, wheel-odometry path
    map_3d.png    the same points from two oblique viewpoints
    map.ply       the aligned points (open in MeshLab / CloudCompare to rotate it yourself)

This is ORB-SLAM3's SPARSE feature map (the corners it tracks), not a dense or occupancy map: walls show
up as bands of points, plain floor and blank walls stay empty.
"""
import argparse
import os
import sys

import numpy as np
import rosbag
import rospy

import slam_bag as sb
from orbslam_eval import read_traj


def rigid(est, ref):
    """R, t with ref ~= R est + t (least squares, no scale)"""
    mu_e, mu_r = est.mean(0), ref.mean(0)
    u, _, vt = np.linalg.svd((est - mu_e).T.dot(ref - mu_r))
    d = np.sign(np.linalg.det(vt.T.dot(u.T)))
    r = vt.T.dot(np.diag([1, 1, d])).dot(u.T)
    return r, mu_r - r.dot(mu_e)


def quat_to_rot(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def rigid_from_poses(est_pos, est_rot, ref_pos, ref_rot):
    """R, t with ref ~= R est + t, using the camera ORIENTATIONS for R (average of R_ref R_est^T).

    Positions alone cannot do this on a straight out-and-back drive: every rotation about the line of
    travel fits them equally well, and the map comes out rolled (floor standing up like a wall).
    """
    m = np.zeros((3, 3))
    for re, rr in zip(est_rot, ref_rot):
        m += rr.dot(re.T)
    u, _, vt = np.linalg.svd(m)
    r = u.dot(np.diag([1, 1, np.sign(np.linalg.det(u.dot(vt)))])).dot(vt)
    # The orientations fix "which way is up" (roll, pitch). The heading is better taken from the
    # positions (a straight drive defines it well, and the camera's yaw in TF depends on the head-pan
    # calibration): rotate about the vertical so the two paths fit best in the horizontal plane.
    ref_pos = np.asarray(ref_pos)
    p = np.asarray(est_pos).dot(r.T)
    a, b = p[:, :2] - p[:, :2].mean(0), ref_pos[:, :2] - ref_pos[:, :2].mean(0)
    yaw = np.arctan2((a[:, 0] * b[:, 1] - a[:, 1] * b[:, 0]).sum(), (a * b).sum())
    c, s_ = np.cos(yaw), np.sin(yaw)
    r = np.array([[c, -s_, 0.0], [s_, c, 0.0], [0.0, 0.0, 1.0]]).dot(r)
    t = (ref_pos - np.asarray(est_pos).dot(r.T)).mean(0)
    return r, t


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('run')
    ap.add_argument('mode')
    ap.add_argument('--zmin', type=float, default=-0.3, help='hide points below this height [m] (noise under the floor)')
    ap.add_argument('--zmax', type=float, default=3.5, help='hide points above this height [m]')
    ap.add_argument('--margin', type=float, default=5.0,
                    help='hide points further than this from the area the camera drove through [m]; far points have '
                         'poor depth and only stretch the picture (0 = show everything)')
    args = ap.parse_args()
    run_dir, bag_path = sb.resolve_run(args.run)
    out = os.path.join(run_dir, 'orbslam', args.mode)
    pts_file = os.path.join(out, 'map_points.npy')
    if not os.path.isfile(pts_file):
        raise SystemExit('no %s - (re)run:  scripts/orbslam_run.sh %s %s' % (pts_file, args.run, args.mode.split('_')[0]))
    pts = np.load(pts_file).astype(float)
    traj = read_traj(os.path.join(out, 'final_cam_traj.txt'))
    if len(traj) < 10:
        raise SystemExit('no usable final trajectory in %s' % out)
    frame = None
    for line in open(os.path.join(out, 'settings.yaml')):
        if line.startswith('# camera frame '):
            frame = line.split()[3].rstrip(',')
    inertial = 'inertial' in args.mode

    # wheel-odometry position of the same frame at the trajectory stamps -> rigid alignment
    bag = rosbag.Bag(bag_path)
    buf = sb.tf_buffer(bag)
    if inertial:
        frame = sb.first_msg(bag, sb.IMU).header.frame_id
    est, ref, est_rot, ref_rot = [], [], [], []
    for row in traj:
        try:
            tr = buf.lookup_transform_core('odom', frame, rospy.Time.from_sec(row[0]))
        except Exception:      # tf2 lookup / extrapolation errors
            continue
        q = tr.transform.rotation
        est.append(row[1:4])
        est_rot.append(quat_to_rot(row[4:8]))
        ref.append([tr.transform.translation.x, tr.transform.translation.y, tr.transform.translation.z])
        ref_rot.append(quat_to_rot((q.x, q.y, q.z, q.w)))
    if len(est) < 10:
        raise SystemExit('no wheel-odometry TF for this trajectory: cannot put the map upright')
    est, ref = np.array(est), np.array(ref)
    r, t = rigid_from_poses(est, est_rot, ref, ref_rot)
    path = est.dot(r.T) + t
    # The saved trajectory is relative to ORB-SLAM3's first keyframe, the map points are in its world
    # frame. Without an IMU the two are the same frame; with an IMU the world is gravity-aligned instead.
    if inertial:
        sys.stderr.write('NOTE: inertial mode - map points are drawn in ORB-SLAM3\'s own gravity-aligned frame, '
                         'not aligned to the odometry path\n')
        m = pts
        path = est
        ref = None
    else:
        m = pts.dot(r.T) + t
    keep = (m[:, 2] > args.zmin) & (m[:, 2] < args.zmax)
    if args.margin > 0:
        lo, hi = path[:, :2].min(0) - args.margin, path[:, :2].max(0) + args.margin
        keep &= np.all((m[:, :2] > lo) & (m[:, :2] < hi), axis=1)
    hidden = int((~keep).sum())
    m = m[keep]

    with open(os.path.join(out, 'map.ply'), 'w') as f:
        f.write('ply\nformat ascii 1.0\nelement vertex %d\nproperty float x\nproperty float y\nproperty float z\nend_header\n' % len(m))
        np.savetxt(f, m, fmt='%.4f')

    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    title = '%s  -  %s   (%d map points)' % (os.path.basename(run_dir), args.mode, len(m))

    fig, ax = plt.subplots(figsize=(10, 10), dpi=120)
    sc = ax.scatter(m[:, 0], m[:, 1], c=m[:, 2], s=1.5, cmap='viridis', vmin=0.0, vmax=2.5, linewidths=0)
    if ref is not None:
        ax.plot(ref[:, 0], ref[:, 1], color='#888888', lw=2.5, label='wheel odometry')
    ax.plot(path[:, 0], path[:, 1], color='#d62728', lw=1.2, label='ORB-SLAM3 camera path')
    ax.plot(path[0, 0], path[0, 1], 'o', color='#2ca02c', ms=9, label='start')
    ax.plot(path[-1, 0], path[-1, 1], 's', color='k', ms=7, label='end')
    ax.set_aspect('equal', adjustable='datalim')
    ax.grid(True, alpha=0.3)
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.set_title(title + '\ntop view, points coloured by height', fontsize=11)
    ax.legend(loc='best', fontsize=9)
    fig.colorbar(sc, ax=ax, shrink=0.7, label='height above the floor [m]')
    fig.tight_layout()
    fig.savefig(os.path.join(out, 'map_top.png'))
    plt.close(fig)

    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401  (registers the 3d projection)
    fig = plt.figure(figsize=(16, 8), dpi=110)
    span = np.ptp(m, axis=0)
    for i, (elev, azim) in enumerate(((35, -60), (20, 30))):
        ax = fig.add_subplot(1, 2, i + 1, projection='3d')
        ax.scatter(m[:, 0], m[:, 1], m[:, 2], c=m[:, 2], s=1.0, cmap='viridis', vmin=0.0, vmax=2.5, linewidths=0, depthshade=False)
        ax.plot(path[:, 0], path[:, 1], path[:, 2], color='#d62728', lw=1.5)
        ax.view_init(elev=elev, azim=azim)
        try:
            ax.set_box_aspect((span[0], span[1], max(span[2], 0.5)))
        except AttributeError:      # matplotlib < 3.3
            pass
        ax.set_xlabel('x [m]')
        ax.set_ylabel('y [m]')
        ax.set_zlabel('z [m]')
    fig.suptitle(title + '\nsparse feature map, camera path in red', fontsize=11)
    fig.tight_layout()
    fig.savefig(os.path.join(out, 'map_3d.png'))
    plt.close(fig)

    print('%d map points shown, %d hidden (outside %.1f..%.1f m height, or more than %.0f m from the driven area)'
          % (len(m), hidden, args.zmin, args.zmax, args.margin))
    for n in ('map_top.png', 'map_3d.png', 'map.ply'):
        print(os.path.join(out, n))


if __name__ == '__main__':
    main()
