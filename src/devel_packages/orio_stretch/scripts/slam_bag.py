"""
slam_bag.py - helpers shared by the offline SLAM scripts (orbslam_settings.py, orbslam_eval.py,
marks_report.py, orbslam_dense.py). Not a node; import it.
"""
import glob
import json
import os

import rosbag
import rospy

import orio_paths

DATASETS = orio_paths.DATASETS_DIR

COLOR = '/camera/color/image_raw'
COLOR_INFO = '/camera/color/camera_info'
ALIGNED_DEPTH = '/camera/aligned_depth_to_color/image_raw'
DEPTH_RAW = '/camera/depth/image_rect_raw'
IR1 = '/camera/infra1/image_rect_raw'
IR1_INFO = '/camera/infra1/camera_info'
IR2 = '/camera/infra2/image_rect_raw'
IR2_INFO = '/camera/infra2/camera_info'
IMU = '/camera/imu'
META = ('/camera/infra1/metadata', '/camera/infra2/metadata', '/camera/depth/metadata')
CONFIG_TOPICS = ('/slam_test/config', '/blur_test/config')


def resolve_run(arg):
    """run name under $ORIO_DATA/datasets, a run folder, or a .bag file -> (run_dir, bag_path)"""
    if os.path.isfile(arg):
        return os.path.dirname(os.path.abspath(arg)), os.path.abspath(arg)
    run_dir = arg if os.path.isdir(arg) else os.path.join(DATASETS, arg)
    if not os.path.isdir(run_dir):
        raise SystemExit('no such run: %s (looked for a folder %s)' % (arg, run_dir))
    bags = sorted(glob.glob(os.path.join(run_dir, '*.bag')))
    if not bags:
        raise SystemExit('no .bag in %s' % run_dir)
    if len(bags) > 1:
        raise SystemExit('%d bags in %s (split recording?) - pass one .bag file explicitly' % (len(bags), run_dir))
    return os.path.abspath(run_dir), bags[0]


def topic_counts(bag):
    return {t: i.message_count for t, i in bag.get_type_and_topic_info().topics.items()}


def laser_mode_of(meta_msg):
    """frame_laser_power_mode of a realsense2_camera/Metadata message: 1 = projector on, 0 = off, None = unknown"""
    try:
        return json.loads(meta_msg.json_data).get('frame_laser_power_mode')
    except ValueError:
        return None


def settings_time(bag):
    """bag time (s since bag start) at which cam_setup.py / blur_run.py had applied the camera settings; 0 if unknown"""
    t_cfg = None
    for _, _, t in bag.read_messages(topics=list(CONFIG_TOPICS)):
        t_cfg = t.to_sec()
    return max(0.0, t_cfg - bag.get_start_time()) if t_cfg else 0.0


def emitter_mode(bag, after=0.0):
    """'none' (no IR streams), 'unknown' (no metadata), 'on', 'off' or 'alternate' - sampled after `after` s"""
    counts = topic_counts(bag)
    if not counts.get(IR1):
        return 'none'
    if not counts.get(META[0]):
        return 'unknown'
    start = rospy.Time.from_sec(bag.get_start_time() + after)
    modes = []
    for _, msg, _ in bag.read_messages(topics=[META[0]], start_time=start):
        modes.append(laser_mode_of(msg))
        if len(modes) >= 60:
            break
    seen = set(modes) - {None}
    if seen == {0, 1}:
        return 'alternate'
    if seen == {1}:
        return 'on'
    if seen == {0}:
        return 'off'
    return 'unknown'


def tf_buffer(bag):
    """all of /tf and /tf_static of the bag in one tf2 BufferCore (lookups at any bag time)"""
    import warnings
    import tf2_ros
    warnings.filterwarnings('ignore', message='.*should be of type.*')     # tf2_py's type nagging about bag messages
    buf = tf2_ros.BufferCore(rospy.Duration(1e6))
    for topic, msg, _ in bag.read_messages(topics=['/tf', '/tf_static']):
        for tr in msg.transforms:
            if topic == '/tf_static':
                buf.set_transform_static(tr, 'bag')
            else:
                buf.set_transform(tr, 'bag')
    return buf


def first_msg(bag, topic):
    for _, msg, _ in bag.read_messages(topics=[topic]):
        return msg
    return None
