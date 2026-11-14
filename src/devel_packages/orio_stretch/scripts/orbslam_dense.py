#!/usr/bin/env python3
"""
orbslam_dense.py - dense 3D map + 2D map from the DEPTH IMAGES of a run, placed at the camera poses
ORB-SLAM3 estimated (the same idea as the RTAB-Map pipeline: SLAM gives the poses, depth gives the map).
The same map is also built from WHEEL-ODOMETRY poses, so the two can be compared with nothing else changed.
Pose errors show up directly as smeared or doubled walls.

    scripts/orbslam_dense.py <run> <mode[_tag]> [--every N] [--voxel 0.05] [--range 3.5]

Needs <run>/orbslam/<mode>/final_cam_traj.txt (any mode; the depth always comes from the colour-aligned,
projector-on depth frames, whatever images ORB-SLAM3 itself used).

Writes into <run>/orbslam/<mode>/:
    dense_orbslam.ply, dense_odom.ply    voxel-filtered clouds in the odometry frame (z up) - MeshLab / CloudCompare
    dense_2d.png                         the two 2D maps side by side (black = obstacle, white = floor seen, grey = unknown)
    dense_3d.png                         the ORB-SLAM3 cloud from two oblique viewpoints
    dense_2d_orbslam.pgm + .yaml         the ORB-SLAM3 2D map in map_server format

2D map rule (same spirit as orio's depth map): a cell is an obstacle if it holds points between --zobs-min
and --zobs-max above the floor; it is free if it only holds floor points. Heights are measured from a floor plane
fitted in EVERY depth frame (--no-floor-fit to turn off). Obstacles need >= --min-hits frames, and obstacle points
that OTHER frames saw through (a person who walked past, the operator next to the robot) are dropped (--no-carve): the head camera's TF is ~2.5 deg off in pitch, so without
it the floor appears to rise ~4-5 cm per metre and a 10 cm threshold would mark distant floor as obstacle.
"""
import argparse
import bisect
import os
import sys

import numpy as np
import rosbag
import rospy

import slam_bag as sb
from orbslam_eval import read_traj
from orbslam_map import quat_to_rot, rigid_from_poses


class Voxels(object):
    """keeps one point per voxel, with the colour it was first seen in and the number of depth frames that saw it"""
    def __init__(self, size):
        self.size = size
        self.cells = {}          # (i, j, k) -> (r, g, b)
        self.hits = {}           # (i, j, k) -> frames that put a point in this voxel

    def add(self, pts, rgb=None):
        q = np.floor(pts / self.size).astype(np.int32)
        q, idx = np.unique(q, axis=0, return_index=True)
        if rgb is None:
            rgb = np.full((len(q), 3), 160, np.uint8)
        else:
            rgb = rgb[idx]
        cells, hits = self.cells, self.hits
        for key, c in zip(map(tuple, q), rgb):      # q is unique per call = per frame
            if key not in cells:
                cells[key] = c
            hits[key] = hits.get(key, 0) + 1

    def hit_counts(self):
        """frames per voxel, in the same order as points()"""
        return np.array([self.hits[k] for k in sorted(self.cells)], np.int32)

    def points(self):
        """(N x 3 float positions, N x 3 uint8 colours)"""
        if not self.cells:
            return np.zeros((0, 3)), np.zeros((0, 3), np.uint8)
        keys = sorted(self.cells)
        return (np.array(keys, dtype=float) + 0.5) * self.size, np.array([self.cells[k] for k in keys], np.uint8)


def write_ply(path, pts, rgb):
    with open(path, 'w') as f:
        f.write('ply\nformat ascii 1.0\nelement vertex %d\nproperty float x\nproperty float y\nproperty float z\n'
                'property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n' % len(pts))
        np.savetxt(f, np.column_stack([pts, rgb]), fmt='%.3f %.3f %.3f %d %d %d')


def floor_corrected(pw, ground_z=0.0):
    """heights above a floor plane z = a x + b y + c fitted to this frame's low points (map frame, z up).
    Returns the points with z replaced by height above that plane, or unchanged if no credible floor is seen."""
    low = pw[np.abs(pw[:, 2] - ground_z) < 0.25]
    if len(low) < 200:
        return pw
    A = np.column_stack([low[:, 0], low[:, 1], np.ones(len(low))])
    sel = np.ones(len(low), bool)
    coef = None
    for _ in range(3):
        if sel.sum() < 100:
            return pw
        coef = np.linalg.lstsq(A[sel], low[sel, 2], rcond=None)[0]
        resid = low[:, 2] - A.dot(coef)
        sel = np.abs(resid) < 0.03
    if coef is None or np.hypot(coef[0], coef[1]) > 0.15 or sel.mean() < 0.3:      # not a floor (steeper than ~8.5 deg)
        return pw
    out = pw.copy()
    out[:, 2] = pw[:, 2] - (coef[0] * pw[:, 0] + coef[1] * pw[:, 1] + coef[2])
    return out


def grid_2d(pts, res, bounds, zfloor, zobs_min, zobs_max, hits=None, min_hits=3):
    """-> uint8 image (0 obstacle, 254 free, 205 unknown), row 0 = max y (map_server convention).
    A cell is an obstacle only if the depth frames that put points in its obstacle band add up to >= min_hits
    (a real object is seen in many frames; a one-off flying pixel or reflection is not - corridor v2, 2026-11-07:
    10 stray points at 1.12 m blocked the passage between points 3 and 4). One-cell unknown gaps between known
    cells (stripes from sampling every 4th depth pixel) are filled from their neighbours."""
    x0, y0, x1, y1 = bounds
    w, h = int(np.ceil((x1 - x0) / res)), int(np.ceil((y1 - y0) / res))
    img = np.full((h, w), 205, np.uint8)
    ix = ((pts[:, 0] - x0) / res).astype(int)
    iy = ((pts[:, 1] - y0) / res).astype(int)
    ok = (ix >= 0) & (ix < w) & (iy >= 0) & (iy < h)
    ix, iy, z = ix[ok], iy[ok], pts[ok, 2]
    floor = np.abs(z) < zfloor
    img[h - 1 - iy[floor], ix[floor]] = 254
    obs = (z > zobs_min) & (z < zobs_max)
    if hits is not None and min_hits > 1:
        hc = np.zeros((h, w), np.int32)
        np.add.at(hc, (h - 1 - iy[obs], ix[obs]), hits[ok][obs])
        img[hc >= min_hits] = 0
    else:
        img[h - 1 - iy[obs], ix[obs]] = 0
    # fill one-cell unknown stripes: horizontal pass, then vertical
    for axis in (1, 0):
        a = img
        prev = np.roll(a, 1, axis=axis); nxt = np.roll(a, -1, axis=axis)
        gap = (a == 205) & (prev != 205) & (nxt != 205)
        if axis == 1:
            gap[:, 0] = gap[:, -1] = False
        else:
            gap[0, :] = gap[-1, :] = False
        img = np.where(gap & ((prev == 0) | (nxt == 0)), 0, np.where(gap, 254, a)).astype(np.uint8)
    return img


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('run')
    ap.add_argument('mode')
    ap.add_argument('--every', type=int, default=3, help='use every Nth depth frame (default 3 = 5 Hz from a 15 Hz stream)')
    ap.add_argument('--voxel', type=float, default=0.05, help='voxel size of the 3D cloud and cell size of the 2D map [m]')
    ap.add_argument('--range', type=float, default=3.5, help='ignore depth beyond this [m] (D435i depth noise grows fast with range)')
    ap.add_argument('--stride', type=int, default=4, help='use every Nth pixel in both directions')
    ap.add_argument('--zobs-min', type=float, default=0.10, help='obstacle band bottom above the floor [m] (user, 2026-11-06: 10 cm)')
    ap.add_argument('--zobs-max', type=float, default=1.143, help='obstacle band top [m] (user, 2026-11-06: 45 in)')
    ap.add_argument('--zfloor', type=float, default=0.06, help='points within +-this of the floor count as floor [m]')
    ap.add_argument('--no-floor-fit', action='store_true', help='measure heights from z = 0 instead of a per-frame floor plane')
    ap.add_argument('--min-hits', type=int, default=3, help='depth frames that must see something in a cell before it counts as an obstacle')
    ap.add_argument('--no-carve', action='store_true', help='keep obstacle points that other frames saw THROUGH (people who walked past)')
    args = ap.parse_args()
    run_dir, bag_path = sb.resolve_run(args.run)
    out = os.path.join(run_dir, 'orbslam', args.mode)
    traj = read_traj(os.path.join(out, 'final_cam_traj.txt'))
    if len(traj) < 10:
        raise SystemExit('no usable final trajectory in %s' % out)
    orb_frame = None
    for line in open(os.path.join(out, 'settings.yaml')):
        if line.startswith('# camera frame '):
            orb_frame = line.split()[3].rstrip(',')

    bag = rosbag.Bag(bag_path)
    counts = sb.topic_counts(bag)
    if not counts.get(sb.ALIGNED_DEPTH):
        raise SystemExit('no aligned depth images in this bag')
    if 'inertial' in args.mode:
        orb_frame = sb.first_msg(bag, sb.IMU).header.frame_id      # inertial modes save the IMU (body) trajectory
    sys.stderr.write('reading TF ...\n')
    buf = sb.tf_buffer(bag)
    info = sb.first_msg(bag, '/camera/aligned_depth_to_color/camera_info') or sb.first_msg(bag, sb.COLOR_INFO)
    fx, fy, cx, cy = info.K[0], info.K[4], info.K[2], info.K[5]
    depth_frame = info.header.frame_id            # colour optical frame

    def odom_T(frame, t):
        """4x4 pose of `frame` in the odom frame at time t, or None"""
        try:
            tr = buf.lookup_transform_core('odom', frame, rospy.Time.from_sec(t))
        except Exception:      # tf2 lookup / extrapolation errors
            return None
        T = np.eye(4)
        q, p = tr.transform.rotation, tr.transform.translation
        T[:3, :3] = quat_to_rot((q.x, q.y, q.z, q.w))
        T[:3, 3] = (p.x, p.y, p.z)
        return T

    # ---- ORB-SLAM3 trajectory -> odom frame: ONE fixed rotation + shift for the whole run, from the camera
    # orientations (positions alone leave the roll undefined on a straight drive) ----
    est, ref, est_rot, ref_rot, keep = [], [], [], [], []
    for i, row in enumerate(traj):
        T = odom_T(orb_frame, row[0])
        if T is not None:
            est.append(row[1:4]); ref.append(T[:3, 3]); keep.append(i)
            est_rot.append(quat_to_rot(row[4:8])); ref_rot.append(T[:3, :3])
    if len(est) < 10:
        raise SystemExit('no wheel-odometry TF for this trajectory')
    R, t = rigid_from_poses(np.array(est), est_rot, np.array(ref), ref_rot)
    traj = traj[keep]
    stamps = list(traj[:, 0])
    # the depth image is in the colour optical frame; an IR-mode trajectory is in the infra1 optical frame
    T_orbcam_depth = np.eye(4)
    if orb_frame != depth_frame:
        try:
            tr = buf.lookup_transform_core(orb_frame, depth_frame, rospy.Time(0))
            q, p = tr.transform.rotation, tr.transform.translation
            T_orbcam_depth[:3, :3] = quat_to_rot((q.x, q.y, q.z, q.w))
            T_orbcam_depth[:3, 3] = (p.x, p.y, p.z)
        except Exception as e:
            raise SystemExit('no TF %s <- %s: %s' % (orb_frame, depth_frame, e))

    def orb_T(tstamp):
        """4x4 pose of the DEPTH camera in the odom frame according to ORB-SLAM3, or None.

        Interpolated between the two neighbouring trajectory poses: the IR modes track the projector-OFF
        frames, the depth comes from the projector-ON frames in between (33 ms away at 30 fps).
        """
        i = bisect.bisect_left(stamps, tstamp)
        if i < len(stamps) and abs(stamps[i] - tstamp) < 1e-4:
            pos, quat = traj[i, 1:4], traj[i, 4:8]
        elif 0 < i < len(stamps) and stamps[i] - stamps[i - 1] < 0.25:
            a, b = traj[i - 1], traj[i]
            w = (tstamp - a[0]) / (b[0] - a[0])
            pos = (1 - w) * a[1:4] + w * b[1:4]
            qb = b[4:8] if np.dot(a[4:8], b[4:8]) >= 0 else -b[4:8]
            quat = (1 - w) * a[4:8] + w * qb          # poses are 1-2 frames apart: normalised lerp is enough
            quat = quat / np.linalg.norm(quat)
        else:
            return None                               # before the start, after the end, or inside a tracking gap
        T = np.eye(4)
        T[:3, :3] = R.dot(quat_to_rot(quat))
        T[:3, 3] = R.dot(pos) + t
        return T.dot(T_orbcam_depth)

    # ---- projector state per depth frame ----
    meta_t, meta_mode = [], []
    for _, msg, _ in bag.read_messages(topics=['/camera/depth/metadata']):
        m = sb.laser_mode_of(msg)
        if m is not None:
            meta_t.append(msg.header.stamp.to_sec()); meta_mode.append(m)
    alternate = len(set(meta_mode)) > 1

    def laser_on(tstamp):
        if not alternate:
            return True
        i = bisect.bisect_left(meta_t, tstamp)
        best = min((j for j in (i - 1, i) if 0 <= j < len(meta_t)), key=lambda j: abs(meta_t[j] - tstamp), default=None)
        return best is not None and abs(meta_t[best] - tstamp) < 0.002 and meta_mode[best] == 1

    # ---- pixel rays (subsampled) ----
    s = args.stride
    v, u = np.mgrid[0:info.height:s, 0:info.width:s]
    rx, ry = (u - cx) / fx, (v - cy) / fy

    vox = {'orbslam': Voxels(args.voxel), 'odom': Voxels(args.voxel)}
    used = k = 0
    sys.stderr.write('projecting depth frames ...\n')
    colour = {}              # stamp [ns] -> colour image (the driver stamps synchronised colour + depth alike)
    pending = []             # depth messages waiting for the colour frame with the same stamp (bag order is arbitrary)

    band_raw = {'orbslam': {}, 'odom': {}}      # voxel key -> [sum of raw map positions, count], obstacle band only
    frames = []                                 # (T_orb, T_odo, depth at half resolution) for the free-space pass

    def project(msg):
        nonlocal used
        ts = msg.header.stamp.to_sec()
        T_orb, T_odo = orb_T(ts), odom_T(depth_frame, ts)
        if T_orb is None or T_odo is None:
            return
        z = np.frombuffer(msg.data, np.uint16).reshape(msg.height, msg.width)[::s, ::s].astype(np.float32) * 0.001
        ok = (z > 0.3) & (z < args.range)
        if not ok.any():
            return
        pc = np.stack([rx[ok] * z[ok], ry[ok] * z[ok], z[ok]], axis=1)
        rgb = None
        cm = colour.get(msg.header.stamp.to_nsec())
        if cm is not None and cm.encoding in ('rgb8', 'bgr8') and (cm.height, cm.width) == (msg.height, msg.width):
            img = np.frombuffer(cm.data, np.uint8).reshape(cm.height, cm.width, 3)[::s, ::s]
            rgb = img[ok] if cm.encoding == 'rgb8' else img[ok][:, ::-1]
        p_orb, p_odo = pc.dot(T_orb[:3, :3].T) + T_orb[:3, 3], pc.dot(T_odo[:3, :3].T) + T_odo[:3, 3]
        raw = {'orbslam': p_orb, 'odom': p_odo}
        if not args.no_floor_fit:
            p_orb, p_odo = floor_corrected(p_orb), floor_corrected(p_odo)
        vox['orbslam'].add(p_orb, rgb)
        vox['odom'].add(p_odo, rgb)
        if not args.no_carve:
            for n, pcorr in (('orbslam', p_orb), ('odom', p_odo)):
                band = (pcorr[:, 2] > args.zobs_min) & (pcorr[:, 2] < args.zobs_max)
                if band.any():
                    keys = map(tuple, np.floor(pcorr[band] / args.voxel).astype(np.int32))
                    acc = band_raw[n]
                    for k, rp in zip(keys, raw[n][band]):
                        a = acc.get(k)
                        if a is None:
                            acc[k] = [rp.astype(np.float64), 1]
                        else:
                            a[0] += rp; a[1] += 1
            full = np.frombuffer(msg.data, np.uint16).reshape(msg.height, msg.width)[::2, ::2].copy()
            frames.append((T_orb, T_odo, full))
        used += 1
        if used % 200 == 0:
            sys.stderr.write('  %d frames\n' % used)

    for topic, msg, _ in bag.read_messages(topics=[sb.ALIGNED_DEPTH, sb.COLOR], start_time=rospy.Time.from_sec(stamps[0] - 0.1),
                                           end_time=rospy.Time.from_sec(stamps[-1] + 0.1)):
        ns = msg.header.stamp.to_nsec()
        if topic == sb.COLOR:
            colour[ns] = msg
            for old in [c for c in colour if c < ns - int(0.5e9)]:
                del colour[old]
        else:
            if not laser_on(msg.header.stamp.to_sec()):
                continue
            k += 1
            if k % args.every:
                continue
            pending.append(msg)
        # a depth frame is projected once its colour frame is in, or once colour has moved 50 ms past it
        still = []
        for d in pending:
            dns = d.header.stamp.to_nsec()
            if dns in colour or (colour and max(colour) > dns + int(0.05e9)):
                project(d)
            else:
                still.append(d)
        pending = still
    for d in pending:
        project(d)
    if not used:
        raise SystemExit('no depth frame matched the trajectory')

    transient = {'orbslam': set(), 'odom': set()}
    if not args.no_carve and frames:
        sys.stderr.write('free-space check over %d frames ...\n' % len(frames))
        for n, ti in (('orbslam', 0), ('odom', 1)):
            keys = list(band_raw[n].keys())
            if not keys:
                continue
            P = np.array([band_raw[n][k][0] / band_raw[n][k][1] for k in keys])
            through = np.zeros(len(P), np.int32); occ = np.zeros(len(P), np.int32)
            for fr in frames:
                T, D = fr[ti], fr[2]
                Pc = (P - T[:3, 3]).dot(T[:3, :3])               # map -> depth camera optical frame
                z = Pc[:, 2]
                m = (z > 0.3) & (z < args.range)
                if not m.any():
                    continue
                idx = np.nonzero(m)[0]
                u = fx * Pc[idx, 0] / z[idx] + cx; v = fy * Pc[idx, 1] / z[idx] + cy
                inside = (u >= 0) & (u < info.width - 1) & (v >= 0) & (v < info.height - 1)
                idx, u, v = idx[inside], u[inside], v[inside]
                d = D[(v / 2).astype(int), (u / 2).astype(int)].astype(np.float32) * 0.001
                good = d > 0.3
                idx, d = idx[good], d[good]
                through[idx[d > z[idx] + 0.15]] += 1
                occ[idx[np.abs(d - z[idx]) < 0.10]] += 1
            bad = (through >= 3) & (through > occ)
            transient[n] = {keys[i] for i in np.nonzero(bad)[0]}
            sys.stderr.write('  %s: %d of %d obstacle-band voxels were seen through by other frames (something that moved) - dropped\n' % (n, len(transient[n]), len(keys)))
        for n, vx in vox.items():
            for k in transient[n]:
                vx.cells.pop(k, None); vx.hits.pop(k, None)
    frames = None

    colours = {}
    clouds = {}
    for n, vx in vox.items():
        clouds[n], colours[n] = vx.points()
        write_ply(os.path.join(out, 'dense_%s.ply' % n), clouds[n], colours[n])
    # compact binary for the browser viewer (~/orio/scripts/orbslam_viewer.py): float32 xyz + uint8 rgb per point, plus the path
    np.save(os.path.join(out, 'dense_orbslam_xyz.npy'), clouds['orbslam'].astype(np.float32))
    np.save(os.path.join(out, 'dense_orbslam_rgb.npy'), colours['orbslam'])
    np.save(os.path.join(out, 'path_orbslam.npy'), (np.array(est).dot(R.T) + t).astype(np.float32))
    allp = np.vstack(list(clouds.values()))
    bounds = (allp[:, 0].min() - 0.3, allp[:, 1].min() - 0.3, allp[:, 0].max() + 0.3, allp[:, 1].max() + 0.3)
    hitc = {n: vx.hit_counts() for n, vx in vox.items()}
    grids = {n: grid_2d(pts, args.voxel, bounds, args.zfloor, args.zobs_min, args.zobs_max, hitc[n], args.min_hits) for n, pts in clouds.items()}

    import cv2
    cv2.imwrite(os.path.join(out, 'dense_2d_orbslam.pgm'), grids['orbslam'])
    with open(os.path.join(out, 'dense_2d_orbslam.yaml'), 'w') as f:
        f.write('image: dense_2d_orbslam.pgm\nresolution: %.3f\norigin: [%.3f, %.3f, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n'
                % (args.voxel, bounds[0], bounds[1]))

    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    path_orb = np.array(est).dot(R.T) + t
    path_odo = np.array(ref)
    ext = (bounds[0], bounds[0] + grids['odom'].shape[1] * args.voxel, bounds[1], bounds[1] + grids['odom'].shape[0] * args.voxel)
    fig, axes = plt.subplots(1, 2, figsize=(18, 9), dpi=120)
    for ax, n, label, path in ((axes[0], 'orbslam', 'ORB-SLAM3 poses (%s)' % args.mode, path_orb),
                               (axes[1], 'odom', 'wheel-odometry poses', path_odo)):
        ax.imshow(grids[n], cmap='gray', vmin=0, vmax=255, extent=ext, interpolation='nearest')
        ax.plot(path[:, 0], path[:, 1], color='#d62728', lw=1.0)
        ax.plot(path[0, 0], path[0, 1], 'o', color='#2ca02c', ms=7)
        ax.set_title('depth images placed at %s' % label, fontsize=11)
        ax.set_xlabel('x [m]'); ax.set_ylabel('y [m]')
    fig.suptitle('%s: 2D map from %d depth frames, %.0f cm cells (black = obstacle %.2f-%.2f m high, white = floor seen, grey = unknown; red = camera path)'
                 % (os.path.basename(run_dir), used, args.voxel * 100, args.zobs_min, args.zobs_max), fontsize=11)
    fig.tight_layout()
    fig.savefig(os.path.join(out, 'dense_2d.png'))
    plt.close(fig)

    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
    m, mc = clouds['orbslam'], colours['orbslam'] / 255.0
    keep = (m[:, 2] > -0.3) & (m[:, 2] < 3.0)
    m, mc = m[keep], mc[keep]
    if len(m) > 400000:
        pick = np.random.RandomState(0).choice(len(m), 400000, replace=False)
        m, mc = m[pick], mc[pick]
    span = np.ptp(m, axis=0)
    fig = plt.figure(figsize=(18, 9), dpi=110)
    for i, (elev, azim) in enumerate(((40, -60), (25, 30))):
        ax = fig.add_subplot(1, 2, i + 1, projection='3d')
        ax.scatter(m[:, 0], m[:, 1], m[:, 2], c=mc, s=0.3, linewidths=0, depthshade=False)
        ax.plot(path_orb[:, 0], path_orb[:, 1], path_orb[:, 2], color='#d62728', lw=1.5)
        ax.view_init(elev=elev, azim=azim)
        try:
            ax.set_box_aspect((span[0], span[1], max(span[2], 0.5)))
        except AttributeError:
            pass
        ax.set_xlabel('x [m]'); ax.set_ylabel('y [m]'); ax.set_zlabel('z [m]')
    fig.suptitle('%s: dense cloud from depth at ORB-SLAM3 poses (%s), camera colours, camera path in red'
                 % (os.path.basename(run_dir), args.mode), fontsize=11)
    fig.tight_layout()
    fig.savefig(os.path.join(out, 'dense_3d.png'))
    plt.close(fig)

    print('%d depth frames; voxels: ORB-SLAM3 %d, odometry %d  (fewer voxels for the same data = a tighter, less smeared map)'
          % (used, len(clouds['orbslam']), len(clouds['odom'])))
    for n in ('dense_2d.png', 'dense_3d.png', 'dense_orbslam.ply', 'dense_odom.ply', 'dense_2d_orbslam.pgm'):
        print(os.path.join(out, n))


if __name__ == '__main__':
    main()
