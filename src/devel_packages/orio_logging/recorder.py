#!/usr/bin/env python3
"""Sidecar rerun recorder: ROS topics -> <run_dir>/recorder.rrd + run.json.

    python3 recorder.py [--run-dir DIR] [--robots 1 2] [--rate 20] [--live]

Subscribes only; nothing in the pipeline depends on it. Runs in the perception venv.
"""
import argparse
import datetime
import json
import os
import shutil
import signal
import socket
import subprocess
import sys
import threading
import time
from collections import defaultdict
from pathlib import Path

import numpy as np
import yaml

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[2]
ROS_PATHS = ['/opt/ros/noetic/lib/python3/dist-packages', str(REPO / 'devel/lib/python3/dist-packages')]
sys.path[:0] = [p for p in ROS_PATHS if p not in sys.path]
os.environ['ROS_PACKAGE_PATH'] = str(HERE) + os.pathsep + os.environ.get('ROS_PACKAGE_PATH', '')

import rerun as rr  # noqa: E402
import rerun.blueprint as rrb  # noqa: E402
import rospy  # noqa: E402
from rosgraph_msgs.msg import Log  # noqa: E402
from std_msgs.msg import Bool, String  # noqa: E402

URDF = REPO / 'src/devel_packages/orio/panda_arm_hand.urdf'
TARGETS = REPO / 'src/devel_packages/orio/Target_Task_Poses.json'
CONFIGS = [REPO / 'src/devel_packages/orio_perception/config/grasp.yaml',
           REPO / 'src/devel_packages/orio/config/motion.yaml',
           HERE / 'config/cell.yaml']
JOINTS = [f'panda_joint{i}' for i in range(1, 8)]
LOG_LEVELS = {Log.DEBUG: 'DEBUG', Log.INFO: 'INFO', Log.WARN: 'WARN', Log.ERROR: 'ERROR', Log.FATAL: 'CRITICAL'}
PROCS = {'state_machine': 'state_machine.py', 'perception': 'perception_control_combined',
         'pneumatics': 'pneumatic_control', 'dexnet': 'dexnet_grasp_planner.py',
         'cameras': 'cameras.launch'}


def git_info():
    def run(*a):
        return subprocess.run(['git', *a], cwd=REPO, capture_output=True, text=True, timeout=10).stdout.strip()
    try:
        return {'sha': run('rev-parse', 'HEAD'), 'branch': run('rev-parse', '--abbrev-ref', 'HEAD'),
                'dirty': bool(run('status', '--porcelain'))}
    except Exception as exc:  # git missing or timeout
        return {'error': str(exc)}


def default_run_dir():
    if os.environ.get('ORIO_RUN_DIR'):
        return Path(os.environ['ORIO_RUN_DIR'])
    run_id = os.environ.get('ORIO_RUN_ID') or datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    return REPO / 'logging/rerun' / run_id


def check_disk(runs_root, min_free_gb):
    runs_root.mkdir(parents=True, exist_ok=True)
    free_gb = shutil.disk_usage(runs_root).free / 1e9
    if free_gb >= min_free_gb:
        return free_gb
    print(f'[recorder] only {free_gb:.1f} GB free (< {min_free_gb} GB); refusing to start.', file=sys.stderr)
    print('[recorder] existing runs, oldest first:', file=sys.stderr)
    for d in sorted(p for p in runs_root.iterdir() if p.is_dir()):
        size = sum(f.stat().st_size for f in d.rglob('*') if f.is_file()) / 1e6
        print(f'    {d.name}  {size:8.1f} MB', file=sys.stderr)
    sys.exit(3)


def pose_from_colmajor(m):
    m = np.asarray(m, dtype=np.float64).reshape(4, 4).T  # column-major -> row-major
    return m[:3, 3], m[:3, :3]


def blueprint():
    return rrb.Blueprint(
        rrb.Horizontal(
            rrb.Vertical(
                rrb.Spatial3DView(name='Cell', origin='/'),
                rrb.TextLogView(name='Log', origin='/log'),
                row_shares=[3, 1]),
            rrb.Vertical(
                rrb.TimeSeriesView(name='Arm1 ext force', origin='/arm1/f_ext'),
                rrb.TimeSeriesView(name='Arm2 ext force', origin='/arm2/f_ext'),
                rrb.TimeSeriesView(name='Vacuum', origin='/vacuum'),
                rrb.Spatial2DView(name='Perception', origin='/perception'),
                rrb.TextLogView(name='Events', origin='/events')),
            column_shares=[3, 2]),
        collapse_panels=False)


class Recorder:
    def __init__(self, args):
        self.args = args
        self.run_dir = Path(args.run_dir) if args.run_dir else default_run_dir()
        self.run_id = self.run_dir.name
        self.period = 1.0 / args.rate
        self.lock = threading.Lock()
        self.buf = {n: [] for n in args.robots}
        self.last_kept = {n: 0.0 for n in args.robots}
        self.last_mode = {n: None for n in args.robots}
        self.last_errors = {n: None for n in args.robots}
        self.trees = {}
        self.sm_active = {}
        self.sm_since = {}
        self.state_durations = defaultdict(list)
        self.service_dt = defaultdict(list)
        self.cmd_dt = defaultdict(list)
        self.pick_n = 0
        self.counts = defaultdict(int)
        self.manifest = {}
        self.stopped = False

    # ── lifecycle ─────────────────────────────────────────────────────────
    def start(self):
        free_gb = check_disk(self.run_dir.parent, self.args.min_free_gb)
        self.run_dir.mkdir(parents=True, exist_ok=True)
        (self.run_dir / 'configs').mkdir(exist_ok=True)
        for c in CONFIGS:
            if c.exists():
                shutil.copy(c, self.run_dir / 'configs' / c.name)
        cell = yaml.safe_load((HERE / 'config/cell.yaml').read_text())['arm2_in_arm1']
        self.manifest = {
            'run_id': self.run_id, 'started_at': datetime.datetime.now().isoformat(timespec='seconds'),
            'host': socket.gethostname(), 'git': git_info(), 'argv': sys.argv[1:],
            'env': {k: os.environ.get(k) for k in ('ORIO_NO_VACUUM', 'ORIO_DEXNET', 'ORIO_LOGGING', 'ORIO_RERUN_LIVE')},
            'rerun_version': rr.__version__, 'robots': self.args.robots, 'rate_hz': self.args.rate,
            'cell_placeholder': bool(cell.get('placeholder', False)), 'free_gb_at_start': round(free_gb, 1),
        }
        self.write_manifest()

        rr.init('orio', recording_id=self.run_id)
        sinks = [rr.FileSink(str(self.run_dir / 'recorder.rrd'))]
        if self.args.live:
            sinks.append(rr.GrpcSink())
        rr.set_sinks(*sinks, default_blueprint=blueprint())
        rospy.init_node('orio_recorder', anonymous=True, disable_signals=False)
        signal.signal(signal.SIGHUP, lambda *_: rospy.signal_shutdown('SIGHUP'))
        rospy.on_shutdown(self.stop)

        self.log_static(cell)
        self.subscribe()
        rospy.Timer(rospy.Duration(0.5), lambda _: self.flush())
        rospy.Timer(rospy.Duration(1.0), lambda _: self.sample_system())
        rospy.loginfo('[recorder] run %s -> %s', self.run_id, self.run_dir)

    def stop(self):
        if self.stopped:
            return
        self.stopped = True
        self.flush()
        self.manifest.update({
            'ended_at': datetime.datetime.now().isoformat(timespec='seconds'),
            'counts': dict(self.counts),
            'summary': self.summary(),
        })
        rr.disconnect()
        self.manifest['files'] = {p.name: p.stat().st_size for p in self.run_dir.glob('*.rrd')}
        self.write_manifest()

    def write_manifest(self):
        (self.run_dir / 'run.json').write_text(json.dumps(self.manifest, indent=2))

    def summary(self):
        def stats(d):
            return {k: {'n': len(v), 'median_s': float(np.median(v)), 'max_s': float(np.max(v))}
                    for k, v in d.items() if v}
        return {'picks': self.pick_n, 'state_durations': stats(self.state_durations),
                'service_latency': stats(self.service_dt), 'goto_joints': stats(self.cmd_dt)}

    # ── static scene ──────────────────────────────────────────────────────
    def log_static(self, cell):
        rr.log('world', rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
        for n in self.args.robots:
            tree = rr.urdf.UrdfTree.from_file_path(str(URDF), entity_path_prefix=f'arm{n}', frame_prefix=f'arm{n}/')
            tree.log_urdf_to_recording()
            self.trees[n] = {j: tree.get_joint_by_name(j) for j in JOINTS}
            rr.log(f'arm{n}/ee', rr.TransformAxes3D(axis_length=0.1), static=True)
        if 2 in self.args.robots and 1 in self.args.robots:
            yaw = np.radians(float(cell['yaw_deg']))
            rr.log('world/arm2_base', rr.Transform3D(
                translation=[float(v) for v in cell['xyz']],
                rotation=rr.RotationAxisAngle(axis=[0, 0, 1], radians=yaw),
                parent_frame='arm1/panda_link0', child_frame='arm2/panda_link0'), static=True)
        if TARGETS.exists():
            targets = {k: v['translation'] for k, v in json.loads(TARGETS.read_text()).items() if 'translation' in v}
            rr.log('world/targets', rr.Points3D(list(targets.values()), labels=list(targets), radii=0.01),
                   rr.CoordinateFrame(frame='arm1/panda_link0'), static=True)

    # ── subscriptions ─────────────────────────────────────────────────────
    def subscribe(self):
        rospy.Subscriber('/rosout_agg', Log, self.on_rosout, queue_size=200)
        rospy.Subscriber('/orio/events', String, self.on_event, queue_size=200)
        for cup in ('pnp', 'lbl'):
            rospy.Subscriber(f'/orio/vacuum/{cup}_has_item', Bool, self.on_vacuum, callback_args=cup, queue_size=10)
        try:
            from smach_msgs.msg import SmachContainerStatus
            rospy.Subscriber('/orio_visualiser/smach/container_status', SmachContainerStatus, self.on_smach, queue_size=20)
        except ImportError:
            rospy.logwarn('[recorder] smach_msgs missing: no state transitions')
        try:
            from franka_interface_msgs.msg import RobotState
            for n in self.args.robots:
                rospy.Subscriber(f'/robot_state_publisher_node_{n}/robot_state', RobotState, self.on_robot_state,
                                 callback_args=n, queue_size=1, buff_size=2 ** 20)
        except ImportError:
            rospy.logwarn('[recorder] franka_interface_msgs missing: no arm state')

    def set_time(self, t):
        rr.set_time('ros_time', timestamp=t)
        rr.set_time('pick', sequence=self.pick_n)

    def on_rosout(self, msg):
        self.counts['rosout'] += 1
        self.set_time(msg.header.stamp.to_sec() or time.time())
        rr.log(f'log/{msg.name.strip("/")}', rr.TextLog(msg.msg, level=LOG_LEVELS.get(msg.level, 'INFO')))

    def on_vacuum(self, msg, cup):
        self.set_time(time.time())
        rr.log(f'vacuum/{cup}/has_item', rr.Scalars(1.0 if msg.data else 0.0))

    def on_smach(self, msg):
        active = tuple(msg.active_states)
        if not active or self.sm_active.get(msg.path) == active:
            return
        t = msg.header.stamp.to_sec() or time.time()
        prev = self.sm_active.get(msg.path)
        if prev is not None:
            for s in prev:
                self.state_durations[s].append(t - self.sm_since[msg.path])
        self.sm_active[msg.path], self.sm_since[msg.path] = active, t
        self.set_time(t)
        rr.log(f'sm/{msg.path.strip("/")}', rr.TextLog(f'{"+".join(prev or ["-"])} -> {"+".join(active)}'))

    def on_event(self, msg):
        try:
            ev = json.loads(msg.data)
        except ValueError:
            self.counts['bad_events'] += 1
            return
        self.counts['events'] += 1
        kind, name, data = ev.get('kind', '?'), ev.get('name', ''), ev.get('data') or {}
        if kind == 'pick':
            self.pick_n = int(data.get('n', self.pick_n + 1))
        self.set_time(float(ev.get('t') or time.time()))
        ok = data.get('ok', True)
        level = 'INFO' if ok else 'WARN'
        if kind == 'error':
            level = 'ERROR'
        rr.log(f'events/{kind}', rr.TextLog(f'{name} {json.dumps(data, separators=(",", ":"))}', level=level))
        if kind == 'service' and 'dt_s' in data:
            self.service_dt[name].append(float(data['dt_s']))
        if kind == 'cmd':
            if 'dt_s' in data:
                self.cmd_dt[name].append(float(data['dt_s']))
            if 'joints' in data:
                rr.log(f'{name}/cmd/target_joints', rr.Scalars(data['joints']))
        if kind == 'ik':
            for k in ('pre', 'final'):
                if k in data:
                    rr.log(f'{name}/ik/{k}', rr.Scalars(data[k]))
            if 'target' in data:
                rr.log(f'{name}/ik/target', rr.Points3D([data['target'][:3]], radii=0.01, labels=['ik target']),
                       rr.CoordinateFrame(frame=f'{name}/panda_link0'))
        for k, v in data.items():
            if isinstance(v, (int, float)) and not isinstance(v, bool):
                rr.log(f'events/{kind}/{name.strip("/") or "all"}/{k}', rr.Scalars(float(v)))

    def on_robot_state(self, msg, n):
        t = time.time()
        if t - self.last_kept[n] < self.period:
            return
        self.last_kept[n] = t
        errors = msg.current_errors
        sample = (t, msg.header.stamp.to_sec(), np.asarray(msg.q), np.asarray(msg.dq), np.asarray(msg.tau_J),
                  np.asarray(msg.tau_ext_hat_filtered), np.asarray(msg.O_F_ext_hat_K), np.asarray(msg.O_T_EE),
                  msg.robot_mode, float(msg.control_command_success_rate),
                  tuple(k for k in errors.__slots__ if getattr(errors, k) is True))
        with self.lock:
            self.buf[n].append(sample)

    # ── periodic ──────────────────────────────────────────────────────────
    def flush(self):
        with self.lock:
            batches = {n: b for n, b in self.buf.items() if b}
            for n in batches:
                self.buf[n] = []
        for n, b in batches.items():
            self.counts[f'arm{n}_samples'] += len(b)
            ts = np.array([s[0] for s in b])
            sensor_ts = np.array([s[1] for s in b])
            idx = [rr.TimeColumn('ros_time', timestamp=ts), rr.TimeColumn('sensor_time', timestamp=sensor_ts),
                   rr.TimeColumn('pick', sequence=[self.pick_n] * len(b))]
            for key, col in (('q', 2), ('dq', 3), ('tau_J', 4), ('tau_ext', 5), ('f_ext', 6)):
                rr.send_columns(f'arm{n}/{key}', indexes=idx, columns=rr.Scalars.columns(scalars=np.stack([s[col] for s in b])))
            rr.send_columns(f'arm{n}/success_rate', indexes=idx, columns=rr.Scalars.columns(scalars=[s[9] for s in b]))
            q = np.stack([s[2] for s in b])
            for i, jn in enumerate(JOINTS):
                rr.send_columns(f'arm{n}/joints/{jn}', indexes=idx,
                                columns=self.trees[n][jn].compute_transform_columns(q[:, i].tolist(), clamp=False))
            poses = [pose_from_colmajor(s[7]) for s in b]
            rr.send_columns(f'arm{n}/ee', indexes=idx, columns=rr.Transform3D.columns(
                translation=np.stack([p[0] for p in poses]), mat3x3=np.stack([p[1].reshape(9) for p in poses]),
                parent_frame=[f'arm{n}/panda_link0'] * len(b), child_frame=[f'arm{n}/ee'] * len(b)))
            for s in b:
                if s[8] != self.last_mode[n] or s[10] != self.last_errors[n]:
                    self.last_mode[n], self.last_errors[n] = s[8], s[10]
                    self.set_time(s[0])
                    rr.log(f'arm{n}/mode', rr.TextLog(f'mode={s[8]} errors={list(s[10])}',
                                                      level='WARN' if s[10] else 'INFO'))

    def sample_system(self):
        try:
            import psutil
        except ImportError:
            return
        t = time.time()
        if not hasattr(self, '_procs'):
            self._procs, self._self = {}, psutil.Process()
            self._self.cpu_percent(None)
        if int(t) % 10 == 0 or not self._procs:
            for p in psutil.process_iter(['pid', 'cmdline']):
                cmd = ' '.join(p.info['cmdline'] or [])
                for name, needle in PROCS.items():
                    if needle in cmd and name not in self._procs:
                        self._procs[name] = p
                        p.cpu_percent(None)
        self.set_time(t)
        rr.log('proc/recorder/cpu', rr.Scalars(self._self.cpu_percent(None)))
        rr.log('proc/recorder/rss_mb', rr.Scalars(self._self.memory_info().rss / 1e6))
        rr.log('proc/system/cpu', rr.Scalars(psutil.cpu_percent(None)))
        for name, p in list(self._procs.items()):
            try:
                rr.log(f'proc/{name}/cpu', rr.Scalars(p.cpu_percent(None)))
                rr.log(f'proc/{name}/rss_mb', rr.Scalars(p.memory_info().rss / 1e6))
            except psutil.Error:
                del self._procs[name]


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--run-dir', help='default: $ORIO_RUN_DIR or <repo>/logging/rerun/<timestamp>')
    ap.add_argument('--robots', nargs='*', type=int, default=[1, 2])
    ap.add_argument('--rate', type=float, default=20.0, help='arm state Hz (source is 100)')
    ap.add_argument('--live', action='store_true', default=os.environ.get('ORIO_RERUN_LIVE') == '1',
                    help='also stream to a running `rerun` viewer')
    ap.add_argument('--min-free-gb', type=float, default=5.0)
    args = ap.parse_args(rospy.myargv(sys.argv)[1:])
    if os.environ.get('ORIO_LOGGING', '1') == '0':
        print('[recorder] ORIO_LOGGING=0: not recording.')
        return
    rec = Recorder(args)
    rec.start()
    rospy.spin()


if __name__ == '__main__':
    main()
