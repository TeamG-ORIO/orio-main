#!/usr/bin/env python3
"""
orbslam_settings.py - write the ORB-SLAM3 settings file for one mode FROM THE BAG ITSELF
(intrinsics from camera_info, IR baseline from the infra2 projection matrix, camera<->IMU
extrinsics from /tf_static, frame rate from the message counts), and tell orbslam_run.sh how
to feed the bag to the node.

    orbslam_settings.py <run|run_dir|bag> <mode> --out settings.yaml [--set Key=Value ...]
                        [--save-atlas NAME] [--load-atlas NAME]

modes:  rgbd                colour + depth aligned to colour            (rolling-shutter colour sensor)
        rgbd-inertial       the same + /camera/imu
        ir-stereo           left + right IR                             (global shutter, no depth image used)
        ir-stereo-inertial  the same + /camera/imu
        ir-rgbd             left IR + native depth (same pixel grid)    (global shutter + depth)

stdout: shell assignments (NODE=..., IMAGE=..., LASER=..., START=..., ...) for `eval`.
"""
import argparse
import sys

import numpy as np
import rosbag
import rospy

import slam_bag as sb

MODES = {
    #                     node                    primary image  second topic       role of second   info         laser
    'rgbd':               ('ros_rgbd',            sb.COLOR,      sb.ALIGNED_DEPTH,  'depth',         sb.COLOR_INFO, 'on'),
    'rgbd-inertial':      ('ros_rgbd_inertial',   sb.COLOR,      sb.ALIGNED_DEPTH,  'depth',         sb.COLOR_INFO, 'on'),
    'ir-stereo':          ('ros_stereo',          sb.IR1,        sb.IR2,            'right',         sb.IR1_INFO,   'off'),
    'ir-stereo-inertial': ('ros_stereo_inertial', sb.IR1,        sb.IR2,            'right',         sb.IR1_INFO,   'off'),
    'ir-rgbd':            ('ros_rgbd',            sb.IR1,        sb.DEPTH_RAW,      'depth',         sb.IR1_INFO,   'off'),
}

ORB_DEFAULTS = [
    ('ORBextractor.nFeatures', 1250),
    ('ORBextractor.scaleFactor', 1.2),
    ('ORBextractor.nLevels', 8),
    ('ORBextractor.iniThFAST', 20),
    ('ORBextractor.minThFAST', 7),
]
# D435i (BMI055) values used by the ORB-SLAM3 RealSense examples; continuous-time noise densities
IMU_DEFAULTS = [
    ('IMU.NoiseGyro', 1e-3),
    ('IMU.NoiseAcc', 1e-2),
    ('IMU.GyroWalk', 1e-6),
    ('IMU.AccWalk', 1e-4),
    ('IMU.InsertKFsWhenLost', 0),
]
VIEWER = [('Viewer.KeyFrameSize', 0.05), ('Viewer.KeyFrameLineWidth', 1.0), ('Viewer.GraphLineWidth', 0.9),
          ('Viewer.PointSize', 2.0), ('Viewer.CameraSize', 0.08), ('Viewer.CameraLineWidth', 3.0),
          ('Viewer.ViewpointX', 0.0), ('Viewer.ViewpointY', -0.7), ('Viewer.ViewpointZ', -3.5), ('Viewer.ViewpointF', 500.0)]


def fmt(v):
    if isinstance(v, str):
        return '"%s"' % v
    if isinstance(v, float):
        return repr(v) if v != int(v) or abs(v) >= 1e15 else '%.1f' % v
    return str(v)


def tf_matrix(tr):
    q, t = tr.transform.rotation, tr.transform.translation
    x, y, z, w = q.x, q.y, q.z, q.w
    m = np.eye(4)
    m[:3, :3] = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                 [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                 [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
    m[:3, 3] = [t.x, t.y, t.z]
    return m


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('run')
    ap.add_argument('mode', choices=sorted(MODES))
    ap.add_argument('--out', required=True)
    ap.add_argument('--set', action='append', default=[], metavar='Key=Value', help='override any settings key, e.g. ORBextractor.iniThFAST=12')
    ap.add_argument('--save-atlas', default='', help='ORB-SLAM3 saves its map as <NAME>.osa in the node working directory at shutdown')
    ap.add_argument('--load-atlas', default='', help='ORB-SLAM3 loads <NAME>.osa from the node working directory at start')
    args = ap.parse_args()
    node, image, second, role, info_topic, want_laser = MODES[args.mode]
    inertial = args.mode.endswith('inertial')

    _, bag_path = sb.resolve_run(args.run)
    bag = rosbag.Bag(bag_path)
    counts = sb.topic_counts(bag)
    need = [image, second, info_topic] + ([sb.IMU] if inertial else []) + ([sb.IR2_INFO] if image == sb.IR1 else [])
    missing = [t for t in need if not counts.get(t)]
    if missing:
        raise SystemExit('this bag cannot run mode %s - missing topics: %s\n(IR modes need a bag from scripts/slam_record.sh)'
                         % (args.mode, ' '.join(missing)))

    duration = bag.get_end_time() - bag.get_start_time()
    start = sb.settings_time(bag)
    emitter = sb.emitter_mode(bag, after=min(start + 1.0, 0.5 * duration))
    # which frames to feed: only filter when the bag really has both kinds
    laser = want_laser if emitter == 'alternate' else 'any'
    fps = counts[image] / max(duration, 1e-3) * (0.5 if emitter == 'alternate' else 1.0)
    notes = []
    if emitter == 'on' and want_laser == 'off':
        notes.append('the IR projector was ON for the whole run: the IR images are full of projector dots, expect poor tracking')
    if emitter == 'off' and want_laser == 'on':
        notes.append('the IR projector was OFF for the whole run: depth is passive stereo only (holes on blank surfaces)')
    if emitter == 'unknown':
        notes.append('no camera metadata in the bag: cannot tell the projector state, feeding every frame')

    info = sb.first_msg(bag, info_topic)
    fx, fy, cx, cy = info.K[0], info.K[4], info.K[2], info.K[5]
    lines = [('File.version', '1.0')]
    # IR baseline: infra2 camera_info carries P[3] = -fx * baseline (frame = infra1 optical)
    baseline = 0.05
    info2 = sb.first_msg(bag, sb.IR2_INFO) if counts.get(sb.IR2_INFO) else None
    if info2 is not None and info2.P[0] > 0 and info2.P[3] != 0:
        baseline = abs(info2.P[3]) / info2.P[0]
    elif image == sb.IR1:
        notes.append('no usable infra2 projection matrix: assuming the nominal 50 mm IR baseline')

    if role == 'right':
        lines += [('Camera.type', 'Rectified'), ('Camera1.fx', fx), ('Camera1.fy', fy), ('Camera1.cx', cx), ('Camera1.cy', cy),
                  ('Stereo.b', baseline)]
    else:
        d = list(info.D) + [0.0] * 5
        lines += [('Camera.type', 'PinHole'), ('Camera1.fx', fx), ('Camera1.fy', fy), ('Camera1.cx', cx), ('Camera1.cy', cy),
                  ('Camera1.k1', d[0]), ('Camera1.k2', d[1]), ('Camera1.p1', d[2]), ('Camera1.p2', d[3]),
                  ('Stereo.b', baseline), ('RGBD.DepthMapFactor', 1000.0)]     # depth images are uint16 millimetres
    lines += [('Camera.width', int(info.width)), ('Camera.height', int(info.height)),
              ('Camera.fps', int(round(fps))), ('Camera.RGB', 1),
              ('Stereo.ThDepth', 40.0)]       # points closer than 40 x baseline (2 m) count as "close"

    cam_frame = info.header.frame_id
    body_frame = cam_frame
    tbc = None
    if inertial:
        imu = sb.first_msg(bag, sb.IMU)
        body_frame = imu.header.frame_id
        buf = sb.tf_buffer(bag)
        try:
            # T_b_c1: pose of the camera in the IMU (body) frame, i.e. p_body = T_b_c1 * p_cam
            tbc = tf_matrix(buf.lookup_transform_core(body_frame, cam_frame, rospy.Time(0)))
        except Exception as e:     # tf2 raises its own exception types
            raise SystemExit('no TF %s <- %s in the bag (%s): cannot run an inertial mode' % (body_frame, cam_frame, e))
        lines += IMU_DEFAULTS + [('IMU.Frequency', float(round(counts[sb.IMU] / max(duration, 1e-3))))]
    lines += ORB_DEFAULTS
    if args.load_atlas:
        lines.append(('System.LoadAtlasFromFile', args.load_atlas))
    if args.save_atlas:
        lines.append(('System.SaveAtlasToFile', args.save_atlas))
    lines += VIEWER

    keys = [k for k, _ in lines]
    for kv in args.set:
        if '=' not in kv:
            raise SystemExit('--set wants Key=Value, got %r' % kv)
        k, v = kv.split('=', 1)
        try:
            v = int(v)
        except ValueError:
            try:
                v = float(v)
            except ValueError:
                pass
        if k in keys:
            lines[keys.index(k)] = (k, v)
        else:
            lines.append((k, v))
            keys.append(k)

    with open(args.out, 'w') as f:
        f.write('%YAML:1.0\n')
        f.write('# generated by orbslam_settings.py for mode %s from %s\n' % (args.mode, bag_path))
        f.write('# camera frame %s, projector during the run: %s, frames fed: laser %s\n' % (cam_frame, emitter, laser))
        for k, v in lines:
            f.write('%s: %s\n' % (k, fmt(v)))
        if tbc is not None:
            f.write('IMU.T_b_c1: !!opencv-matrix\n   rows: 4\n   cols: 4\n   dt: f\n   data: [%s]\n'
                    % ',\n          '.join(', '.join('%.9g' % x for x in row) for row in tbc))

    for n in notes:
        sys.stderr.write('NOTE: %s\n' % n)
    out = dict(NODE=node, IMAGE=image, SECOND=second, ROLE=role, LASER=laser, EMITTER=emitter, INERTIAL=int(inertial),
               START='%.1f' % start, FPS='%.1f' % fps, CAM_FRAME=cam_frame, BODY_FRAME=body_frame, BAG=bag_path)
    for k, v in out.items():
        print("%s='%s'" % (k, v))


if __name__ == '__main__':
    main()
