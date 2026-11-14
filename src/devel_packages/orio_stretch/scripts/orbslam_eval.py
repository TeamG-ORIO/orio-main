#!/usr/bin/env python3
"""
orbslam_eval.py - turn one ORB-SLAM3 offline run into numbers (written to <out_dir>/summary.txt and printed).

    orbslam_eval.py <out_dir> --bag run.bag --mode ir-stereo --frame camera_infra1_optical_frame

Inputs in <out_dir> (written by orbslam_run.sh): frames.csv (orbslam_log.py), final_cam_traj.txt
(ORB-SLAM3's own optimised trajectory of its BIGGEST map, saved at the end).

Tracking numbers (from frames.csv, i.e. what happened live, frame by frame):
    tracked %       frames with tracking state OK / frames offered
    dropped         frames the node never processed (CPU too slow at this playback rate -> lower --rate)
    lost episodes   stretches without OK tracking after the first OK; the longest one in seconds
    new maps        times ORB-SLAM3 gave up and started a fresh map (it could not relocalise)

Accuracy numbers (from final_cam_traj.txt; drawn top-down in trajectory.png): the trajectory is rigidly aligned (rotation + translation,
no scale) to the WHEEL ODOMETRY pose of the same frame from the bag's TF, then
    ATE rmse        root-mean-square position difference after alignment
    path            distance travelled according to ORB-SLAM3 and according to the wheels
Wheel odometry is NOT ground truth (it drifts, mostly in heading); over a short loop it is a fair
sanity reference, nothing more. final.tum (TUM format) is written for `evo` if you install it.
"""
import argparse
import csv
import os
import sys

import numpy as np
import rosbag
import rospy

import slam_bag as sb

OK_STATES = (2, 5)


def read_frames(path):
    rows = []
    with open(path) as f:
        for r in csv.DictReader(f):
            rows.append((float(r['stamp']), int(r['state']) if r['state'] else None,
                         int(r['n_points']) if r['n_points'] else None))
    return rows


def tracking_stats(rows):
    s = {}
    n = len(rows)
    s['offered'] = n
    if not n:
        return s
    t = np.array([r[0] for r in rows])
    st = [r[1] for r in rows]
    ok = np.array([x in OK_STATES for x in st])
    s['duration'] = t[-1] - t[0]
    s['processed'] = sum(x is not None for x in st)
    s['ok'] = int(ok.sum())
    pts = [r[2] for r in rows if r[1] in OK_STATES and r[2] is not None]
    s['median_points'] = float(np.median(pts)) if pts else float('nan')
    if not ok.any():
        return s
    first = int(np.argmax(ok))
    s['init_delay'] = t[first] - t[0]
    # lost episodes after the first OK frame; a "new map" = the episode passed through
    # LOST (4) or NOT_INITIALIZED (1) before tracking came back
    episodes, new_maps, i = [], 0, first
    while i < n:
        if ok[i]:
            i += 1
            continue
        j = i
        while j < n and not ok[j]:
            j += 1
        processed = [x for x in st[i:j] if x is not None]
        if processed:      # frames that were merely dropped are not a tracking loss
            episodes.append((t[min(j, n - 1)] - t[i], j < n))
            if j < n and any(x in (1, 4) for x in processed):
                new_maps += 1
        i = j
    s['episodes'] = len(episodes)
    s['longest_gap'] = max([e[0] for e in episodes]) if episodes else 0.0
    s['lost_at_end'] = bool(episodes) and not episodes[-1][1]
    s['new_maps'] = new_maps
    return s


def read_traj(path):
    """ORB-SLAM3 SaveTrajectoryEuRoC: stamp[ns] x y z qx qy qz qw"""
    out = []
    with open(path) as f:
        for line in f:
            v = line.split()
            if len(v) == 8:
                out.append([float(v[0]) * 1e-9] + [float(x) for x in v[1:]])
    return np.array(out) if out else np.zeros((0, 8))


def align(est, ref):
    """rigid (no scale) least-squares alignment est -> ref (Horn/Umeyama); returns aligned est"""
    mu_e, mu_r = est.mean(0), ref.mean(0)
    h = (est - mu_e).T.dot(ref - mu_r)
    u, _, vt = np.linalg.svd(h)
    d = np.sign(np.linalg.det(vt.T.dot(u.T)))
    r = vt.T.dot(np.diag([1, 1, d])).dot(u.T)
    return (r.dot((est - mu_e).T)).T + mu_r


def path_len(p):
    return float(np.linalg.norm(np.diff(p, axis=0), axis=1).sum()) if len(p) > 1 else 0.0


def accuracy(traj, bag_path, frame):
    a = {'traj_poses': len(traj)}
    if len(traj) < 10:
        a['note'] = 'final trajectory has fewer than 10 poses'
        return a
    bag = rosbag.Bag(bag_path)
    buf = sb.tf_buffer(bag)
    if 'odom' not in buf.all_frames_as_string():
        # the driver was not broadcasting the odom TF (broadcast_odom_tf off): rebuild it from the /odom topic
        from geometry_msgs.msg import TransformStamped
        for _, msg, _ in bag.read_messages(topics=['/odom']):
            tr = TransformStamped()
            tr.header.stamp, tr.header.frame_id, tr.child_frame_id = msg.header.stamp, 'odom', 'base_link'
            p = msg.pose.pose.position
            tr.transform.translation.x, tr.transform.translation.y, tr.transform.translation.z = p.x, p.y, p.z
            tr.transform.rotation = msg.pose.pose.orientation
            buf.set_transform(tr, 'bag')
    est, ref = [], []
    for row in traj:
        try:
            tr = buf.lookup_transform_core('odom', frame, rospy.Time.from_sec(row[0]))
        except Exception:      # tf2 extrapolation / lookup errors: no odometry for this stamp
            continue
        est.append(row[1:4])
        ref.append([tr.transform.translation.x, tr.transform.translation.y, tr.transform.translation.z])
    if len(est) < 10:
        a['note'] = 'no wheel-odometry TF (odom -> %s) in the bag for these stamps' % frame
        return a
    est, ref = np.array(est), np.array(ref)
    aligned = align(est, ref)
    err = np.linalg.norm(aligned - ref, axis=1)
    a['plot'] = (aligned, ref, err)
    a.update(matched=len(est), span=traj[-1, 0] - traj[0, 0],
             ate_rmse=float(np.sqrt((err ** 2).mean())), ate_max=float(err.max()),
             path_orb=path_len(est), path_odom=path_len(ref),
             end_orb=float(np.linalg.norm(est[-1] - est[0])), end_odom=float(np.linalg.norm(ref[-1] - ref[0])))
    return a


def plot(path, aligned, ref, err, title):
    """top-down view (odom frame, x forward / y left at the start of the run) of both paths"""
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        return False
    fig, ax = plt.subplots(figsize=(7, 7), dpi=110)
    ax.plot(ref[:, 0], ref[:, 1], color='#888888', lw=2.5, label='wheel odometry')
    ax.plot(aligned[:, 0], aligned[:, 1], color='#1f77b4', lw=1.5, label='ORB-SLAM3 (aligned)')
    ax.plot(ref[0, 0], ref[0, 1], 'o', color='#2ca02c', ms=8, label='start')
    ax.plot(ref[-1, 0], ref[-1, 1], 's', color='#d62728', ms=7, label='end')
    ax.set_aspect('equal', adjustable='datalim')
    ax.grid(True, alpha=0.3)
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.set_title('%s\nposition difference: rmse %.3f m, max %.3f m' % (title, np.sqrt((err ** 2).mean()), err.max()), fontsize=10)
    ax.legend(loc='best', fontsize=9)
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)
    return True


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('out_dir')
    ap.add_argument('--bag', required=True)
    ap.add_argument('--mode', default='?')
    ap.add_argument('--frame', required=True, help='TF frame the trajectory describes (camera optical frame; IMU frame for inertial modes)')
    args = ap.parse_args()

    L = []
    rows = read_frames(os.path.join(args.out_dir, 'frames.csv'))
    s = tracking_stats(rows)
    L.append('mode:            %s' % args.mode)
    L.append('bag:             %s' % args.bag)
    if not s.get('offered'):
        L.append('NO FRAMES were offered to ORB-SLAM3 - check feed.log / play.log in %s' % args.out_dir)
    else:
        n = s['offered']
        L.append('frames offered:  %d over %.1f s (%.1f Hz)' % (n, s['duration'], (n - 1) / max(s['duration'], 1e-6)))
        dropped = n - s['processed']
        L.append('frames dropped:  %d (%.1f %%)%s' % (dropped, 100.0 * dropped / n,
                 '   <-- the node could not keep up: rerun with a lower --rate' if dropped > 0.05 * n else ''))
        L.append('tracked OK:      %d = %.1f %% of offered' % (s['ok'], 100.0 * s['ok'] / n))
        if s['ok']:
            L.append('first OK after:  %.1f s' % s['init_delay'])
            L.append('lost episodes:   %d, longest %.1f s%s' % (s['episodes'], s['longest_gap'],
                     ' (still lost at the end)' if s['lost_at_end'] else ''))
            L.append('new maps:        %d (lost for good and started over)' % s['new_maps'])
            L.append('map points/frame: %.0f (median while OK)' % s['median_points'])
        else:
            L.append('ORB-SLAM3 NEVER tracked a frame (never initialised) - see orbslam.log')

    traj_file = os.path.join(args.out_dir, 'final_cam_traj.txt')
    a = {}
    if os.path.isfile(traj_file):
        traj = read_traj(traj_file)
        if len(traj):
            np.savetxt(os.path.join(args.out_dir, 'final.tum'), traj, fmt='%.9f %.6f %.6f %.6f %.7f %.7f %.7f %.7f')
        a = accuracy(traj, args.bag, args.frame)
        L.append('final trajectory: %d poses (biggest map only)' % a['traj_poses'])
        if 'ate_rmse' in a:
            L.append('  covers:        %.1f s' % a['span'])
            L.append('  ATE vs wheels: rmse %.3f m, max %.3f m  (wheel odometry is not ground truth)' % (a['ate_rmse'], a['ate_max']))
            L.append('  path length:   ORB-SLAM3 %.2f m, wheels %.2f m' % (a['path_orb'], a['path_odom']))
            L.append('  start->end:    ORB-SLAM3 %.3f m, wheels %.3f m  (both ~0 if you drove back onto the start mark)' % (a['end_orb'], a['end_odom']))
            png = os.path.join(args.out_dir, 'trajectory.png')
            if plot(png, *a['plot'], title='%s  -  %s' % (os.path.basename(os.path.dirname(os.path.dirname(os.path.abspath(args.out_dir)))), args.mode)):
                L.append('  top-down plot: %s' % png)
        elif 'note' in a:
            L.append('  no accuracy numbers: %s' % a['note'])
    else:
        L.append('final trajectory: not saved (see orbslam.log)')

    pct = 100.0 * s.get('ok', 0) / s['offered'] if s.get('offered') else 0.0
    L.append('RESULT %-19s tracked %5.1f %%  lost episodes %2s  longest gap %5s s  new maps %2s  ATE %s' % (
        args.mode, pct, s.get('episodes', '-'), ('%.1f' % s['longest_gap']) if 'longest_gap' in s else '-',
        s.get('new_maps', '-'), ('%.3f m' % a['ate_rmse']) if 'ate_rmse' in a else 'n/a'))
    text = '\n'.join(L) + '\n'
    with open(os.path.join(args.out_dir, 'summary.txt'), 'w') as f:
        f.write(text)
    sys.stdout.write(text)


if __name__ == '__main__':
    main()
