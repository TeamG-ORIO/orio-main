#!/usr/bin/env python3
"""
demo_agent.py - the robot's end of the demo GUI: takes commands over HTTP, runs the existing scripts, reports state.

    python3 scripts/demo_agent.py [--port 8123] [--pc http://<pc>:8000] [--token T] [--engine pc|robot|off] [--method F4]

The GUI (orio_rfid/demo/scan_dashboard.py, on the PC) polls GET /status and sends POST /cmd. Nothing here drives the
robot by itself: every command starts one of the scripts in this folder, exactly as a person would in a terminal.

    command        what runs
    preflight      scripts/rfid_preflight.sh --auto-run          (the checks that matter for the autonomous run)
    home           scripts/home_payload.py                       (the lift goes to the top of the mast: keep clear)
    start          scripts/rfid_nav_run.sh <map> --run-dir .. --waypoints .. --start-at N --laps K --label L
                   then, next to it: RFID/rfid_pose_live.py (posed reads while it runs) and, with --pc,
                   sync/robot_sync.py --follow (presence, and the reads themselves when --engine pc)
    start + replay_from   no robot: an old run's reads are played back at real time (rehearsal of the GUI and the PC side)
    pause/resume   writes pause / run into the run's nav_control file (nav_waypoints.py holds the base; lift and reader go on)
    stop           SIGINT to rfid_nav_run.sh: it stops everything and still poses the reads
    estop          /runstop of stretch_driver (all motion stops and stays stopped), then stop. Release: the button on
                   the robot's head (hold 2 s) or the GUI's RELEASE, which needs the driver running
    manual on/off  on: scripts/drive.sh --hold --no-pose (driver only); as soon as the driver is up the robot goes to the
                   run pose by itself (see pose) and scripts/cam_stream.py starts: the head camera's picture, served at
                   GET /camera.jpg. off: the head goes back to the saved pan / tilt FIRST (scripts/set_head_pose.py), then
                   the driver stops, so the camera looks the same way at the start of every run
    head           manual mode: scripts/set_head_pose.py back to the saved pan / tilt
    drive          base velocity through scripts/ros_bridge.py (manual mode, or a paused run)
    pose           manual mode: scripts/prepare_pose.py to the saved data-collection pose, lift up FIRST (with the payload
                   effort), then arm in, then the head. The wrist is NOT stowed all the way as a run does: it stops
                   MANUAL_WRIST_OUT_DEG (30 degrees) short of the saved yaw, clockwise seen from above (user, 2026-11-14).
                   A run still stows it fully at its start (nav_orbslam_start.sh), which is the antenna pose the item
                   localiser was trained with.
    jog            manual mode, hold-to-move like drive: the lift and the head camera move while the command keeps coming
                   (scripts/ros_bridge.py; they stop 0.5 s after the last one). Lift: anywhere in its homed range
                   (stretch_body range_m, 0 .. 1.099 m on this robot; user, 2026-11-14. Runs keep 0.42 .. 0.90 m), 0.10 m/s
                   at full speed, not while the arm is out more than 5 cm (lift_motion.py's rule; homing leaves the arm
                   10 cm out: use pose first). Camera: pan -1.6 .. 1.6, tilt -1.0 .. 0.3 rad, 0.5 rad/s at full speed.
    clean          scripts/ros_clean.sh: leftovers of a crashed run

preflight, home, start and manual first stop ~/teleop_server's processes (robot_server.py, camera_server.py and
their start script): they are started at boot and hold the motors and the camera. Nothing in ~/teleop_server is
changed; they are back after the next reboot. (Team decision 2026-11-13: stop them without asking.)

State comes from the files a run writes (nav_events.jsonl, reads_posed_live.csv, the run log) and, while a ROS
master is up, from scripts/ros_bridge.py. Standard library + PyYAML; no ROS in this process.
"""
import argparse
import collections
import csv
import hmac
import io
import json
import os
import signal
import socket
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import math

import yaml

import orio_paths

S = os.path.dirname(os.path.realpath(__file__))
RFID = os.path.realpath(os.path.join(orio_paths.PKG_DIR, '..', 'orio_rfid', 'RFID'))
SYNC = os.path.realpath(os.path.join(orio_paths.PKG_DIR, '..', 'orio_rfid', 'sync'))
ENGINE = os.path.realpath(os.path.join(orio_paths.PKG_DIR, '..', 'orio_rfid', 'loc', 'live', 'engine.py'))
LOG_DIR = os.path.join(orio_paths.MAPS_DIR, 'logs')
TELEOP = ('bash start_teleop.sh', 'robot_server.py', 'camera_server.py')
CAM_FILE = '/dev/shm/orio_cam.jpg'       # newest head-camera frame, written by cam_stream.py while manual mode is on
MANUAL_WRIST_OUT_DEG = 30     # the manual pose leaves the wrist this far short of the saved (stowed) yaw, clockwise seen from above
LIFT_RANGE = None             # (lo, hi) m of the manual lift buttons, read once from stretch_body (Agent.lift_range)
JOG_LIFT_MPS, JOG_HEAD_RPS = 0.10, 0.5      # lift / head-camera speed with a button held and the speed slider at the top
sys.path.insert(0, RFID)
from rfid_pose_live import Tail      # noqa: E402


def tail_text(path, n=30, nbytes=16384):
    try:
        with open(path, 'rb') as f:
            f.seek(0, os.SEEK_END); size = f.tell(); f.seek(max(0, size - nbytes))
            return f.read().decode('utf-8', 'replace').splitlines()[-n:]
    except OSError:
        return []


def default_sigint():
    """for a child: SIGINT back to its default. An agent started in the background (nohup ... &, as run_scan_demo.sh
    does) has SIGINT ignored, its children would inherit that, and a script cannot trap a signal it was born ignoring:
    STOP (SIGINT to rfid_nav_run.sh) would do nothing."""
    signal.signal(signal.SIGINT, signal.SIG_DFL)


def spawn(cmd, log, **kw):
    """start a command in its own session, output appended to a log file"""
    os.makedirs(os.path.dirname(log), exist_ok=True)
    f = open(log, 'ab')
    return subprocess.Popen(cmd, stdout=f, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL, start_new_session=True,
                            preexec_fn=default_sigint, **kw)


def end(proc, sig=signal.SIGINT, wait=15.0):
    """signal a process started by spawn() and wait; kill its whole session if it does not go"""
    if proc is None or proc.poll() is not None:
        return
    try:
        os.kill(proc.pid, sig)
        proc.wait(timeout=wait)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(proc.pid, signal.SIGKILL)
        except OSError:
            pass
    except OSError:
        pass


def master_up():
    try:
        socket.create_connection(('127.0.0.1', 11311), timeout=0.2).close()
        return True
    except OSError:
        return False


def teleop_pids():
    """the teleop server's processes: the command line names one of TELEOP AND the process runs in a teleop_server
    folder. (The folder test keeps an editor or a shell that merely mentions robot_server.py out of it.)"""
    out = []
    for pat in TELEOP:
        r = subprocess.run(['pgrep', '-f', pat], capture_output=True, text=True)
        for p in r.stdout.split():
            try:
                if os.path.basename(os.readlink('/proc/%s/cwd' % p)) == 'teleop_server' and int(p) not in out:
                    out.append(int(p))
            except OSError:
                pass
    return out


class Bridge:
    """scripts/ros_bridge.py, kept running while a ROS master is up"""

    def __init__(self):
        self.proc, self.state, self.replies = None, {}, collections.deque(maxlen=5)

    def tick(self):
        up = master_up()
        if self.proc is not None and (self.proc.poll() is not None or not up):
            end(self.proc, signal.SIGTERM, 3.0); self.proc, self.state = None, {}
        if self.proc is None and up:
            self.proc = subprocess.Popen(['bash', '-c', 'source "%s/orio_env.sh" >/dev/null 2>&1; exec python3 "%s/ros_bridge.py"' % (S, S)],
                                         stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True, start_new_session=True,
                                         preexec_fn=default_sigint)
            threading.Thread(target=self._read, args=(self.proc,), daemon=True).start()

    def _read(self, proc):
        for line in proc.stdout:
            try:
                d = json.loads(line)
            except ValueError:
                continue
            if 'reply' in d:
                self.replies.append(d)
            else:
                self.state = d

    def send(self, obj):
        if self.proc is None or self.proc.poll() is not None:
            return False
        try:
            self.proc.stdin.write(json.dumps(obj) + '\n'); self.proc.stdin.flush()
            return True
        except (BrokenPipeError, OSError):
            return False

    def fresh(self):
        return dict(self.state, age_s=round(time.time() - self.state['t'], 1)) if self.state.get('t') else None


class Run:
    """one autonomous run (or the replay of an old one) and the helpers next to it"""
    MARKS = (('navigation ready', 'reader starting'), ('RFID reads coming in', 'driving'), ('run finished', 'finishing'),
             ('posing the reads', 'posing reads'))

    def __init__(self, agent, p):
        self.agent, self.p = agent, p
        self.label = p['label']; self.out = os.path.join(orio_paths.DATASETS_DIR, self.label)
        self.log = os.path.join(LOG_DIR, 'demo_%s.log' % self.label)
        self.replay = p.get('replay_from')
        self.started, self.ended, self.stop_asked, self.estop = time.time(), None, False, False
        self.helpers, self.helpers_up = {}, False
        self.reads = Tail(os.path.join(self.out, 'reads_posed_live.csv')); self.head = None
        self.n_reads, self.epcs, self.names, self.recent, self.last = 0, set(), set(), collections.deque(), None
        self.events = Tail(os.path.join(self.out, 'nav_events.jsonl'))
        self.nav = {'i': 0, 'n': 0, 'name': None, 'ok': 0, 'fail': 0, 'retries': 0, 'paused': False, 'done': False, 'results': []}
        self.path = collections.deque(maxlen=4000)
        self.waypoints, self.wp_run = [], False
        if self.replay:
            self.proc = None
            os.makedirs(self.out, exist_ok=True)
            src = os.path.join(orio_paths.DATASETS_DIR, self.replay)
            self.helpers['pose_live'] = spawn([sys.executable, os.path.join(RFID, 'rfid_pose_live.py'), self.out, '--replay-from', src,
                                               '--speed', str(p.get('speed', 1))], self.log)
            threading.Thread(target=self._replay_nav, args=(src, float(p.get('speed', 1))), daemon=True).start()
            self._load_waypoints(os.path.join(src, 'waypoints_run.yaml'))
            self._start_uplink()
        else:
            cmd = [os.path.join(S, 'rfid_nav_run.sh'), p['map'], '--run-dir', p['run_dir'], '--waypoints', p['waypoints'],
                   '--start-at', str(p.get('start_at', 1)), '--laps', str(p.get('laps', 1)), '--label', self.label]
            for k, opt in (('lift_lo', '--lift-lo'), ('lift_hi', '--lift-hi'), ('period', '--period'), ('lift_fixed', '--lift-fixed')):
                if p.get(k) not in (None, ''):
                    cmd += [opt, str(p[k])]
            self.proc = spawn(cmd, self.log, cwd=orio_paths.PKG_DIR)
            self._load_waypoints(p['waypoints'], int(p.get('laps', 1)))

    def _load_waypoints(self, path, laps=1):
        try:
            w = yaml.safe_load(open(path))['waypoints']
            self.waypoints = [dict(name=str(x['name']), x=float(x['x']), y=float(x['y'])) for x in w]
            self.nav['n'] = len(w) * laps
        except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError):
            pass

    def _start_uplink(self):
        a = self.agent.args
        live = os.path.join(self.out, 'reads_posed_live.csv')
        if not self.replay:
            self.helpers['pose_live'] = spawn([sys.executable, os.path.join(RFID, 'rfid_pose_live.py'), self.out], self.log)
        if a.pc:
            cmd = [sys.executable, '-u', os.path.join(SYNC, 'robot_sync.py'), '--pc', a.pc, '--db', os.path.join(self.out, 'robot_items.db'),
                   '--follow', live, '--run', self.label, '--send-every', '0.5']
            stream = ['--stream-reads', '--method', self.p.get('method') or a.method] if a.engine == 'pc' else []
            self.helpers['sync'] = spawn(cmd + stream, os.path.join(self.out, 'robot_sync.out'))
        if a.engine == 'robot' and os.path.exists(ENGINE):
            cmd = [sys.executable, '-u', ENGINE, '--follow', live, '--run', self.label, '--method', self.p.get('method') or a.method]
            self.helpers['engine'] = spawn(cmd + (['--post', a.pc] if a.pc else []), os.path.join(self.out, 'engine.out'))
        self.helpers_up = True

    def _replay_nav(self, src, speed):
        """rehearsal: turn the old run's nav.csv into nav events at the times they happened"""
        try:
            rows = list(csv.DictReader(open(os.path.join(src, 'nav.csv'))))
        except OSError:
            return
        ev = open(os.path.join(self.out, 'nav_events.jsonl'), 'a', buffering=1)

        def emit(**kw):
            ev.write(json.dumps(dict(t=round(time.time(), 3), **kw)) + '\n')
        emit(kind='start', n=len(rows), names=[r['name'] for r in rows])
        for i, r in enumerate(rows):
            if self.ended:
                return
            emit(kind='goal', i=i + 1, n=len(rows), name=r['name'], x=float(r['gx']), y=float(r['gy']), tries=0)
            t_end = time.time() + float(r['seconds']) / speed
            while time.time() < t_end and not self.ended:
                time.sleep(0.2)
            emit(kind='result', i=i + 1, n=len(rows), name=r['name'], status=r['status'], ok=r['ok'] == 'True', retries=int(r['retries']),
                 seconds=float(r['seconds']), error_m=float(r['error_m']), paused_s=0.0)
        emit(kind='done', ok=sum(r['ok'] == 'True' for r in rows), n=len(rows))

    def alive(self):
        return (self.helpers.get('pose_live') if self.replay else self.proc).poll() is None

    def control(self, word):
        if self.replay:
            return False
        try:
            open(os.path.join(self.out, 'nav_control'), 'w').write(word + '\n')
            return True
        except OSError:
            return False

    def stop(self):
        self.stop_asked = True
        if self.replay:
            end(self.helpers.get('pose_live'), signal.SIGTERM, 5.0)
        elif self.proc.poll() is None:
            os.kill(self.proc.pid, signal.SIGINT)        # the script's own trap stops everything and poses the reads

    def tick(self):
        if not self.helpers_up and os.path.isdir(self.out):
            self._start_uplink()
        wp = os.path.join(self.out, 'waypoints_run.yaml')      # the list this run really drives: one entry per waypoint per lap
        if not self.wp_run and not self.replay and os.path.exists(wp):
            self.wp_run = True
            self._load_waypoints(wp)
        now = time.time()
        for line in self.reads.lines():
            row = next(csv.reader(io.StringIO(line)))
            if self.head is None:
                self.head = {c: i for i, c in enumerate(row)}
                continue
            self.n_reads += 1; self.last = row; self.recent.append(now)
            self.epcs.add(row[self.head['epc']])
            if row[self.head['name']]:
                self.names.add(row[self.head['name']])
            if self.n_reads % 20 == 0:                   # driven path for the GUI's map: a point every 5 cm
                try:
                    x, y = float(row[self.head['base_x']]), float(row[self.head['base_y']])
                    if not self.path or abs(x - self.path[-1][0]) + abs(y - self.path[-1][1]) > 0.05:
                        self.path.append((round(x, 3), round(y, 3)))
                except (ValueError, KeyError):
                    pass
        while self.recent and now - self.recent[0] > 5.0:
            self.recent.popleft()
        for line in self.events.lines():
            try:
                e = json.loads(line)
            except ValueError:
                continue
            k, nav = e.get('kind'), self.nav
            if k == 'start':
                nav['n'] = e['n']
            elif k == 'goal':
                nav.update(i=e['i'], name=e['name'], n=e['n'])
            elif k == 'retry':
                nav['retries'] += 1
            elif k == 'result':
                nav['ok'] += bool(e['ok']); nav['fail'] += not e['ok']
                nav['results'].append({x: e.get(x) for x in ('i', 'name', 'status', 'ok', 'seconds', 'error_m', 'retries')})
            elif k in ('paused', 'resumed'):
                nav['paused'] = k == 'paused'
            elif k == 'done':
                nav['done'] = True
        if self.ended is None and not self.alive():
            self.ended = now
            threading.Thread(target=self._finish, daemon=True).start()

    def close(self):
        """the agent is exiting: stop the helpers now (the run script, if any, was told to stop and cleans up by itself)"""
        for k, sig in (('pose_live', signal.SIGTERM), ('sync', signal.SIGINT), ('engine', signal.SIGINT)):
            end(self.helpers.get(k), sig, 5.0)

    def _finish(self):
        """the run is over: let the helpers deliver what is left, then stop them"""
        time.sleep(3.0)
        end(self.helpers.get('pose_live'), signal.SIGTERM, 10.0)
        time.sleep(4.0)
        end(self.helpers.get('sync'), signal.SIGINT, 10.0)
        time.sleep(20.0)                                  # the localiser gets a last update in
        end(self.helpers.get('engine'), signal.SIGINT, 20.0)

    def phase(self):
        log = '\n'.join(tail_text(self.log, 400, 65536))
        if self.replay:
            if self.ended is None:
                return 'replaying'
            if self.stop_asked:
                return 'stopped'
            return 'replay finished' if self.helpers['pose_live'].poll() == 0 else 'failed'
        if self.ended:
            if self.estop:
                return 'E-STOPPED'
            if 'done:' in log and not self.stop_asked:
                return 'done'
            return 'stopped' if self.stop_asked else 'failed'
        ph = 'starting navigation'
        for mark, name in self.MARKS:
            if mark in log:
                ph = name
        if self.stop_asked:
            return 'stopping'
        if ph == 'driving' and self.nav['paused']:
            return 'paused'
        return ph

    def error(self):
        for line in reversed(tail_text(self.log, 60)):
            if 'ERROR:' in line:
                return line.split('ERROR:', 1)[1].strip()
        return None

    def status(self):
        h, pose = self.head, None
        if self.last and h:
            try:
                pose = {k: float(self.last[h[k]]) for k in ('base_x', 'base_y', 'base_yaw', 'lift_m', 'ant_z')}
                pose['age_s'] = round(time.time() - float(self.last[h['host_time_s']]), 1)
            except (ValueError, KeyError):
                pass
        return {'label': self.label, 'phase': self.phase(), 'replay': self.replay, 'alive': self.ended is None,
                'elapsed_s': round((self.ended or time.time()) - self.started),
                'error': (self.error() or (tail_text(self.log, 1) or [None])[0]) if self.ended and self.phase() == 'failed' else None,
                'nav': dict(self.nav, results=self.nav['results'][-40:]),
                'reads': {'rows': self.n_reads, 'tags': len(self.epcs), 'named': len(self.names), 'rate': round(len(self.recent) / 5.0)},
                'pose': pose, 'path': list(self.path)[-1500:], 'waypoints': self.waypoints,
                'helpers': {k: v.poll() is None for k, v in self.helpers.items()},
                'sync': (tail_text(os.path.join(self.out, 'robot_sync.out'), 1) or [None])[0],
                'engine': (tail_text(os.path.join(self.out, 'engine.out'), 1) or [None])[0],
                'log': tail_text(self.log, 12), 'out': self.out}


class Agent:
    def __init__(self, args):
        self.args, self.lock = args, threading.RLock()
        self.run, self.job, self.manual, self.bridge = None, None, None, Bridge()
        self.notes = collections.deque(maxlen=30)
        self.manual_log = os.path.join(LOG_DIR, 'demo_manual.log')
        self.manual_from, self.mover = 0, None          # log size when manual mode started; the one lift / pose move in progress
        self.cam, self.cam_tried = None, 0.0
        self.manual_posed = True                        # False from MANUAL ON until the run pose has been asked for once
        self.code_at = self.code_mtime()                # to tell when the files on disk are newer than this running agent
        self.n_teleop, self.k = 0, 0
        threading.Thread(target=self._loop, daemon=True).start()

    def note(self, text):
        self.notes.append({'t': time.time(), 'text': text}); print(time.strftime('%H:%M:%S'), text, flush=True)

    def _loop(self):
        while True:
            try:
                with self.lock:
                    self.bridge.tick()
                    if self.run:
                        self.run.tick()
                    if self.manual_ready() and not self.manual_posed and not self.moving():
                        self.manual_posed = True                     # once per MANUAL ON: lift up, arm in, wrist, head
                        ok, msg = self.do_pose({}); self.note('manual: %s' % msg)
                    # the camera only after the driver is up: drive.sh refuses to start while anything holds the camera
                    if self.manual_ready() and (self.cam is None or self.cam.poll() is not None) and time.time() - self.cam_tried > 5.0:
                        self.cam_tried = time.time()
                        self.cam = spawn([sys.executable, os.path.join(S, 'cam_stream.py'), '--out', CAM_FILE], self.manual_log)
                self.k += 1
                if self.k % 6 == 1:
                    self.n_teleop = len(teleop_pids())
            except Exception as e:      # the agent must outlive a bad line in a log
                print('agent loop:', repr(e), flush=True)
            time.sleep(0.5)

    # ---------------------------------------------------------------- pieces
    def busy(self):
        if self.run and self.run.ended is None:
            return 'a run is going (%s)' % self.run.label
        if self.job and self.job['proc'].poll() is None:
            return '%s is still running' % self.job['name']
        return None

    def free_hardware(self):
        found = teleop_pids()
        for p in found:
            try:
                os.kill(p, signal.SIGTERM)
            except OSError:
                pass
        if found:
            time.sleep(1.5)
            self.n_teleop = len(teleop_pids())
            self.note('stopped the teleop server (%d processes, %d left); it returns after a reboot' % (len(found), self.n_teleop))

    @staticmethod
    def code_mtime():
        files = [os.path.join(S, f) for f in ('demo_agent.py', 'ros_bridge.py', 'cam_stream.py', 'orio_env.sh')] + \
                [os.path.join(RFID, 'rfid_pose_live.py'), os.path.join(SYNC, 'robot_sync.py')]
        return max((os.path.getmtime(f) for f in files if os.path.exists(f)), default=0.0)

    @staticmethod
    def saved_pose():
        """config/data_pose.env (the data-collection pose) and config/payload.env (today's lift effort) as one dict"""
        env = {}
        for name in ('data_pose.env', 'payload.env'):
            try:
                for line in open(os.path.join(orio_paths.SITE_DIR, name)):
                    k, eq, v = line.strip().partition('=')
                    if eq and not k.startswith('#'):
                        env[k] = v
            except OSError:
                pass
        return env

    def stop_manual(self):
        end(self.mover, signal.SIGTERM, 5.0); self.mover = None
        if self.manual_ready():          # the head back to its usual pan / tilt while the driver is still there, so that the
            env = self.saved_pose()      # camera looks the same way at the start of every run
            try:
                subprocess.run([sys.executable, os.path.join(S, 'set_head_pose.py'), '--pan', env.get('POSE_HEAD_PAN', '0'), '--tilt', env.get('POSE_HEAD_TILT', '-0.35'),
                                '--timeout', '8'], stdout=open(self.manual_log, 'ab'), stderr=subprocess.STDOUT, timeout=14)
                self.note('head back to its usual pose (pan %s, tilt %s)' % (env.get('POSE_HEAD_PAN', '0'), env.get('POSE_HEAD_TILT', '-0.35')))
            except (subprocess.TimeoutExpired, OSError) as e:
                self.note('the head did NOT go back to its usual pose (%s); a run puts it there at its start anyway' % e)
        end(self.cam, signal.SIGTERM, 5.0); self.cam = None        # frees the camera before anything else needs it
        if self.manual and self.manual.poll() is None:
            end(self.manual, signal.SIGINT, 20.0); self.note('manual driving off')
        self.manual = None

    def start_job(self, name, cmd):
        log = os.path.join(LOG_DIR, 'demo_%s_%s.log' % (name, time.strftime('%Y%m%d_%H%M%S')))
        self.job = {'name': name, 'proc': spawn(cmd, log, cwd=orio_paths.PKG_DIR), 'log': log, 'started': time.time()}
        self.note('%s started' % name)

    # ---------------------------------------------------------------- commands
    def cmd(self, c):
        name = c.get('cmd')
        with self.lock:
            f = getattr(self, 'do_' + str(name), None)
            if f is None:
                return False, 'unknown command %r' % name
            return f(c)

    def do_preflight(self, c):
        b = self.busy()
        if b:
            return False, b
        self.stop_manual(); self.free_hardware()
        self.start_job('preflight', [os.path.join(S, 'rfid_preflight.sh'), '--auto-run'] + (['--no-reader'] if c.get('no_reader') else []))
        return True, 'preflight running (about 1 min)'

    def do_home(self, c):
        b = self.busy()
        if b:
            return False, b
        self.stop_manual(); self.free_hardware()
        self.start_job('home', [sys.executable, os.path.join(S, 'home_payload.py')])
        return True, 'homing with the payload: the lift goes to the top, keep clear'

    def do_clean(self, c):
        b = self.busy()
        if b:
            return False, b
        self.stop_manual()
        self.start_job('clean', [os.path.join(S, 'ros_clean.sh')])
        return True, 'cleaning up leftover ROS processes'

    def do_start(self, c):
        b = self.busy()
        if b:
            return False, b
        p = dict(c); p['label'] = c.get('label') or '%s_%s' % ('replay' if c.get('replay_from') else 'rfid_auto', time.strftime('%Y%m%d_%H%M%S'))
        if os.path.exists(os.path.join(orio_paths.DATASETS_DIR, p['label'])):
            return False, 'datasets/%s exists: pick another label' % p['label']
        if not c.get('replay_from'):
            for k in ('map', 'run_dir', 'waypoints'):
                if not p.get(k):
                    return False, 'start needs %s' % k
                if k != 'map':
                    p[k] = os.path.expanduser(p[k])
                    if not os.path.exists(p[k]):
                        return False, '%s not found on the robot: %s' % (k, p[k])
            self.stop_manual(); self.free_hardware()
        elif not os.path.exists(os.path.join(orio_paths.DATASETS_DIR, c['replay_from'], 'reads_posed.csv')):
            return False, 'no reads_posed.csv in datasets/%s' % c['replay_from']
        self.run = Run(self, p)
        self.note(('replaying %s as %s' % (c['replay_from'], p['label'])) if c.get('replay_from') else 'run %s started' % p['label'])
        return True, 'started %s' % p['label']

    def do_stop(self, c):
        if not self.run or self.run.ended is not None:
            return False, 'no run is going'
        self.run.stop(); self.note('stop requested')
        return True, 'stopping: the reads recorded so far are kept and posed'

    def do_pause(self, c):
        if not self.run or self.run.ended is not None:
            return False, 'no run is going'
        ok = self.run.control('pause')
        return ok, 'pausing the base (lift and reader keep going)' if ok else 'this run cannot be paused'

    def do_resume(self, c):
        if not self.run or self.run.ended is not None:
            return False, 'no run is going'
        ok = self.run.control('run')
        return ok, 'resuming' if ok else 'this run cannot be resumed'

    def do_estop(self, c):
        sent = self.bridge.send({'runstop': True})
        if self.run and self.run.ended is None:
            self.run.estop = True; self.run.control('pause'); self.run.stop()
        self.note('E-STOP: runstop %s' % ('sent to the driver' if sent else 'NOT sent (no driver running)'))
        return True, ('E-STOP sent: motion stopped and latched. Release with the head button (2 s) or RELEASE.' if sent
                      else 'no driver is running, so nothing is moving under ROS; the run (if any) is being stopped')

    def do_runstop(self, c):
        ok = self.bridge.send({'runstop': bool(c.get('on'))})
        return ok, 'runstop %s sent' % ('on' if c.get('on') else 'release') if ok else 'no driver running: use the button on the robot\'s head'

    def do_manual(self, c):
        if not c.get('on'):
            self.stop_manual()
            return True, 'manual driving off'
        b = self.busy()
        if b:
            return False, b
        if self.manual and self.manual.poll() is None:
            return True, 'manual driving is already on'
        self.free_hardware()
        self.manual_from = os.path.getsize(self.manual_log) if os.path.exists(self.manual_log) else 0
        self.manual = spawn([os.path.join(S, 'drive.sh'), '--hold', '--no-pose'], self.manual_log, cwd=orio_paths.PKG_DIR)
        self.manual_posed = False
        self.note('manual driving on (driver starting)')
        return True, 'starting the driver (about 20 s); then the lift goes up and the arm retracts by itself'

    def can_drive(self):
        if self.manual and self.manual.poll() is None:
            return True
        return bool(self.run and self.run.ended is None and self.run.nav['paused'])

    def do_drive(self, c):
        if not self.can_drive():
            return False, 'driving needs manual mode, or a paused run'
        ok = self.bridge.send({'drive': [float(c.get('v', 0.0)), float(c.get('w', 0.0))]})
        return ok, 'ok' if ok else 'the driver is not up yet'

    def manual_ready(self):
        if not (self.manual and self.manual.poll() is None):
            return False
        try:
            with open(self.manual_log, 'rb') as f:
                f.seek(self.manual_from)
                return b'MANUAL READY' in f.read()
        except OSError:
            return False

    def moving(self):
        return self.mover is not None and self.mover.poll() is None

    @staticmethod
    def lift_range():
        """(lo, hi) in m for the manual lift buttons: the lift's whole homed range, as stretch_body has it"""
        global LIFT_RANGE
        if LIFT_RANGE is None:
            try:
                import stretch_body.robot_params as rp
                lo, hi = rp.RobotParams.get_params()[1]['lift']['range_m']
                LIFT_RANGE = (round(float(lo), 3), math.floor(float(hi) * 1000) / 1000.0)
            except Exception as e:      # no stretch_body here: the limits every other script uses
                print('lift range: stretch_body parameters not readable (%r), keeping 0.42 .. 1.00 m' % e, flush=True)
                LIFT_RANGE = (0.42, 1.0)
        return LIFT_RANGE

    def do_jog(self, c):
        """hold-to-move for the lift and the head camera: {'lift': -1|0|1, 'pan': .., 'tilt': .., 'speed': 0.2 .. 1}, repeated
        while a button is held (ros_bridge.py stops the joint 0.5 s after the last one)"""
        if not self.manual_ready():
            return False, 'the lift and the camera move from here only in manual mode, once the driver is up'
        if self.moving():
            return False, 'the last move is still going'
        st, speed = self.bridge.state or {}, min(max(float(c.get('speed', 0.6)), 0.1), 1.0)
        d = {j: max(-1, min(1, int(c.get(j, 0)))) for j in ('lift', 'pan', 'tilt')}
        if d['lift'] and st.get('arm_m') is None:
            return False, 'the arm position is not known yet: wait a second'
        if d['lift'] and st['arm_m'] > 0.05:           # the lift_motion.py rule: the lift does not move with the arm out
            return False, 'the arm is out %.2f m: the lift does not move with the arm out. Press POSE AGAIN first' % st['arm_m']
        env = self.saved_pose()
        ok = self.bridge.send({'jog': {'lift': d['lift'] * JOG_LIFT_MPS * speed, 'pan': d['pan'] * JOG_HEAD_RPS * speed, 'tilt': d['tilt'] * JOG_HEAD_RPS * speed},
                               'lift_lim': self.lift_range(), 'effort': float(env.get('LIFT_EFFORT') or env.get('POSE_LIFT_EFFORT') or 0)})
        if not ok:
            return False, 'the driver is not up yet'
        blocked = {j: w for j, w in (st.get('jog_blocked') or {}).items() if d.get(j)}
        if blocked:
            return False, '; '.join('%s: %s' % ({'lift': 'lift', 'pan': 'camera', 'tilt': 'camera'}[j], w) for j, w in blocked.items())
        return True, 'ok'

    def do_pose(self, c):
        if not self.manual_ready():
            return False, 'RUN POSE needs manual mode, once the driver is up'
        if self.moving():
            return False, 'the last move is still going'
        env = self.saved_pose()
        if 'POSE_LIFT' not in env:
            return False, 'no saved pose (config/data_pose.env): drive.sh saves one when its teleop is closed'
        effort = env.get('LIFT_EFFORT') or env.get('POSE_LIFT_EFFORT') or '0'      # today's homing measurement first
        # positive wrist yaw is counter-clockwise seen from above (stretch.urdf: 0 = straight out, 3.4 = folded in)
        wrist = float(env.get('POSE_WRIST_YAW', '3.4')) - math.radians(MANUAL_WRIST_OUT_DEG)
        self.mover = spawn(['timeout', '90', sys.executable, os.path.join(S, 'prepare_pose.py'), '--split-lift', '--no-gripper', '--lift-effort', effort,
                            '--lift', env['POSE_LIFT'], '--arm', env.get('POSE_ARM', '0'), '--wrist-yaw', '%.3f' % wrist,
                            '--pan', env.get('POSE_HEAD_PAN', '0'), '--tilt', env.get('POSE_HEAD_TILT', '-0.35')], self.manual_log)
        return True, 'going to the manual pose: lift up to %s m first (effort %s %%), then arm in and wrist to %.2f rad (%d deg short of stowed)' % (
            env['POSE_LIFT'], effort, wrist, MANUAL_WRIST_OUT_DEG)

    def do_head(self, c):
        """the head camera back to its usual pan / tilt (turning it is do_jog)"""
        if not self.manual_ready():
            return False, 'the camera moves from here only in manual mode, once the driver is up'
        if self.moving():
            return False, 'the last move is still going'
        env = self.saved_pose(); pan, tilt = float(env.get('POSE_HEAD_PAN', 0)), float(env.get('POSE_HEAD_TILT', -0.35))
        self.mover = spawn(['timeout', '15', sys.executable, os.path.join(S, 'set_head_pose.py'), '--pan', '%.3f' % pan, '--tilt', '%.3f' % tilt, '--timeout', '8'],
                           self.manual_log)
        return True, 'camera back to its usual pose (pan %.2f, tilt %.2f rad)' % (pan, tilt)

    def camera(self):
        """the newest head-camera frame (JPEG bytes) if it is less than 2 s old"""
        try:
            if time.time() - os.path.getmtime(CAM_FILE) < 2.0:
                return open(CAM_FILE, 'rb').read()
        except OSError:
            pass
        return None

    # ---------------------------------------------------------------- state
    def status(self):
        with self.lock:
            job = None
            if self.job:
                rc = self.job['proc'].poll()
                job = {'name': self.job['name'], 'running': rc is None, 'rc': rc, 'elapsed_s': round(time.time() - self.job['started']),
                       'lines': tail_text(self.job['log'], 60)}
            manual = None
            if self.manual:
                up = self.manual.poll() is None
                manual = {'on': up, 'ready': self.manual_ready(), 'moving': self.moving(), 'camera': self.camera() is not None,
                          'lines': tail_text(self.manual_log, 6)}
            return {'ok': True, 'host': socket.gethostname(), 'now': time.time(), 'pc': self.args.pc, 'engine': self.args.engine,
                    'method': self.args.method, 'ros_master': self.bridge.proc is not None, 'robot': self.bridge.fresh(),
                    'teleop_server': self.n_teleop, 'can_drive': self.can_drive(), 'stale': self.code_mtime() > self.code_at,
                    'job': job, 'manual': manual, 'run': self.run.status() if self.run else None, 'notes': list(self.notes)[-12:],
                    'replayable': sorted(d for d in os.listdir(orio_paths.DATASETS_DIR)
                                         if os.path.exists(os.path.join(orio_paths.DATASETS_DIR, d, 'reads_posed.csv')))}


class Handler(BaseHTTPRequestHandler):
    agent, token = None, ''

    def reply(self, code, obj):
        body = json.dumps(obj).encode()
        self.send_response(code); self.send_header('Content-Type', 'application/json'); self.send_header('Content-Length', str(len(body)))
        self.end_headers(); self.wfile.write(body)

    def allowed(self):
        if not self.token or hmac.compare_digest(self.headers.get('X-Orio-Token', ''), self.token):
            return True
        self.reply(403, {'ok': False, 'error': 'wrong or missing token'})
        return False

    def do_GET(self):
        if not self.allowed():
            return
        if self.path == '/status':
            return self.reply(200, self.agent.status())
        if self.path.startswith('/camera.jpg'):
            jpg = self.agent.camera()
            if jpg is None:
                return self.reply(404, {'ok': False, 'error': 'no camera picture (it runs only in manual mode)'})
            self.send_response(200); self.send_header('Content-Type', 'image/jpeg'); self.send_header('Content-Length', str(len(jpg)))
            self.end_headers(); self.wfile.write(jpg)
            return
        self.reply(404, {'ok': False, 'error': 'unknown path'})

    def do_POST(self):
        if not self.allowed():
            return
        if self.path != '/cmd':
            return self.reply(404, {'ok': False, 'error': 'unknown path'})
        try:
            c = json.loads(self.rfile.read(int(self.headers.get('Content-Length', 0))))
            ok, msg = self.agent.cmd(c)
        except Exception as e:
            return self.reply(400, {'ok': False, 'message': repr(e)})
        self.reply(200, {'ok': bool(ok), 'message': msg})

    def log_message(self, *a):
        pass


def adopt_env():
    """take the environment of scripts/orio_env.sh (ROS, HELLO_FLEET_*, ORIO_DATA) for this process and its children.
    Started over ssh the agent has none of it: ~/.bashrc sets it only in interactive shells, and home_payload.py
    then died with KeyError 'HELLO_FLEET_PATH' (2026-11-13)."""
    r = subprocess.run(['bash', '-c', 'source "%s/orio_env.sh" >/dev/null 2>&1; env -0' % S], capture_output=True)
    for kv in r.stdout.split(b'\0'):
        k, _, v = kv.partition(b'=')
        if k and k not in (b'_', b'SHLVL', b'PWD', b'OLDPWD'):
            os.environ[k.decode()] = v.decode('utf-8', 'replace')
    return [k for k in ('ROS_DISTRO', 'HELLO_FLEET_PATH', 'HELLO_FLEET_ID') if not os.environ.get(k)]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--port', type=int, default=8123)
    ap.add_argument('--pc', default='', help='address of sync/pc_receiver.py on the PC, e.g. http://192.168.50.20:8000 (empty: nothing is sent)')
    ap.add_argument('--token', default=os.environ.get('ORIO_DEMO_TOKEN', ''), help='shared secret the GUI must send (default: $ORIO_DEMO_TOKEN; empty = none)')
    ap.add_argument('--engine', choices=['pc', 'robot', 'off'], default='pc', help='where the item localiser runs: pc = send it the reads')
    ap.add_argument('--method', default='F4', choices=['F1', 'F2', 'F3', 'F4'])
    args = ap.parse_args()
    args.pc = args.pc.rstrip('/')
    missing = adopt_env()
    if missing:
        print('WARNING: not set even after scripts/orio_env.sh: %s (the robot scripts will fail)' % ', '.join(missing), flush=True)
    Handler.agent, Handler.token = Agent(args), args.token
    os.makedirs(LOG_DIR, exist_ok=True)
    pid_file = os.path.join(LOG_DIR, 'demo_agent.pid')       # run_scan_demo.sh stops the agent by this pid
    open(pid_file, 'w').write('%d\n' % os.getpid())
    print('demo_agent on port %d, PC %s, item localiser: %s%s' % (args.port, args.pc or '(none)', args.engine, '' if args.token else ', NO TOKEN'), flush=True)
    srv = ThreadingHTTPServer(('0.0.0.0', args.port), Handler)
    signal.signal(signal.SIGINT, signal.default_int_handler)      # also when started with SIGINT ignored
    signal.signal(signal.SIGTERM, lambda *a: (_ for _ in ()).throw(KeyboardInterrupt()))
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    ag = Handler.agent
    with ag.lock:                       # leave nothing of ours behind; a run in progress is stopped the clean way
        if ag.run and ag.run.ended is None:
            ag.run.stop()
        if ag.run:
            ag.run.close()
        ag.stop_manual()
        end(ag.bridge.proc, signal.SIGTERM, 3.0)
    try:
        os.remove(pid_file)
    except OSError:
        pass


if __name__ == '__main__':
    main()
