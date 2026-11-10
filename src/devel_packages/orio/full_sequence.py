#!/usr/bin/env python3
"""Dual-arm pick → zone → label → output sequence, driven by the new grasp planners.

The successor to state_machine.py for the single-terminal launcher
(run_full_demo.sh). Same structure — two SMACH machines running concurrently,
zones A/B managed between them, Tkinter recovery GUIs per arm, item metadata GUI
at start — with the pick path upgraded to everything dexnet_pnp.py learned:

    grasp source : /compute_grasps (Trigger) → first pose on /grasp_poses
                   (the perception node's suction/dexnet backend publishes it)
    orientation  : tilted approach along the grasp normal, refused above
                   --max-tilt-deg; --straight-down forces vertical
    correction   : --offset-x/--offset-y world-frame metres, applied here only
    declines     : --max-declines planner rejections in a row = empty bin →
                   keep emptying the zones; INTERVENE → FETCH_INPUT picks again
    confirmation : --confirm = per-pick Enter / 'r' regenerate / 'q' abort
    motions      : hard-timeout _move() wrapper — a dead control PC raises a
                   clear error instead of hanging frankapy forever

Arm 2 (labelling) is unchanged from state_machine.py: label from the dispenser,
pose from /compute_grasps_labelling via the ZED, collision-checked IK, RFID
registration through /inventory/add_item (degrades to a warning if absent).

Runs INSIDE the main docker container (needs its ROS + frankapy + smach + X11
for the GUIs), from this directory:
    docker exec -it <container> bash -c \
      'source /home/ros_ws/devel/setup.bash \
       && cd /home/ros_ws/src/devel_packages/orio && python3 full_sequence.py'

Type 'q' + Enter at any idle moment to stop cleanly.
"""

import argparse
import datetime
import gc
import json
import logging
import os
import pickle
import queue
import random
import signal
import subprocess
import sys
import threading
import time
import zlib

import ikpy.chain
import numpy as np
import rospy
import smach
import smach_ros
import tkinter as tk
from threading import Lock
from tkinter import ttk, messagebox

from frankapy import FrankaArm
from autolab_core import RigidTransform
from geometry_msgs.msg import PoseArray
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import Bool, Int32, String
from std_srvs.srv import Trigger, TriggerResponse
from custom_msgs.srv import AddLabeledItem
import CollisionChecker
from item_setup_gui import show_item_setup_gui
import RobotUtil as rt

try:
    from orio_core.events import TOPIC as EVENTS_TOPIC, EventSink
except ImportError:  # orio_core not on path: run events disabled
    EVENTS_TOPIC, EventSink = None, None


# ── Constants ──────────────────────────────────────────────────────────────────
APPROACH_DISTANCE = 0.1
PICK_PUSH_DISTANCE = 0.01     # m the bin pick goes past the planner's grasp point, along the cup axis (a firmer seal)
INPUT_RETRACT_DISTANCE = 0.15  # m arm 1 backs out of the input bin after a pick (along the cup axis)
PICK_UP_ZONE_GROUND_Z = 0.09
LABEL_ZONE_GROUND_Z = 0.055
LABEL_PLACE_CORRECTION_Z = 0.00  # To prevent excessive interference with the object while placing the label

POSES_FILE        = "joint_angles.json"
TARGET_POSES_FILE = "Target_Task_Poses.json"
URDF_FILE         = "panda_arm_hand.urdf"

# The ClearCore's has-item thresholds on the raw vacuum reading (0-4095). Only quoted in
# failure messages; the firmware decides. Keep in step with pneumatic_control.ino.
VAC_THRESHOLD_HIGH = 1000
VAC_THRESHOLD_LOW = 800
# The pick cup's check that it holds an item also passes on a raw reading above this
# (--pnp-vacuum-min), without the firmware's has-item. Raw readings on past runs: about 430
# with the cup off, 585-615 sucking open air (a real miss), 640-880 an item held by an
# imperfect seal, 1000-1500 a good seal.
PNP_PICK_RAW_MIN = 800

# ── MOTION SPEED ─────────────────────────────────────────────────────────────────
# Every move's duration in the code is its 1x time; _move divides it by --speed. A move is
# never made shorter than its largest joint travel needs at MAX_PEAK_JOINT_SPEED (frankapy's
# goto_joints is a min-jerk move, whose peak speed is 1.875x its average), nor shorter than
# MIN_MOVE_S. Franka's own joint speed limits are 2.2-2.6 rad/s.
DEFAULT_SPEED = 1.0
MAX_PEAK_JOINT_SPEED = 1.2   # rad/s
MIN_MOVE_S = 1.0
MIN_JERK_PEAK = 1.875
HOME_JOINTS = [0.0, -np.pi / 4, 0.0, -3 * np.pi / 4, 0.0, np.pi / 2, np.pi / 4]  # reset_joints
# Wait after each move reports done before the next command (--settle-s). frankapy reports a
# skill done while the arm can still be settling; a new move started then trips
# "Motion finished commanded, but the robot is still moving!" (velocity / acceleration
# discontinuity), which got much more frequent with shorter moves (--speed).
DEFAULT_SETTLE_S = 0.0

# Same safe workspace box FetchInput has always enforced on the incoming grasp pose (world).
PICK_BOUNDS = {'x_min': -0.2, 'x_max': 0.55, 'y_min': -0.8, 'y_max': -0.1,
               'z_min': 0.0, 'z_max': 0.5}

# Straight-down tool orientation, the IK default.
VERTICAL_ORI = np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]])
IK_LIMIT_MARGIN = 0.05  # rad an IK solution keeps from every joint limit (pose_to_joints)
IK_RESEEDS = 24         # extra IK seeds tried when the first solution is unusable
# Start of the flat-arm IK (pose_to_joints flat=True): joints 3 and 5 at 0, the arm
# reaching forward with the cup down (close to the drop solutions at OUTPUT2).
FLAT_ARM_SEED = [1.2, 0.9, 0.0, -0.6, 0.0, 1.5, 0.8]
# How far short of its target a flat-arm solution may land. OUTPUT2 (and OUTPUT1 for tall
# items) is past the arm's reach: every arm shape falls 1.5-6 cm short, the flat one within
# 5 mm of the best, landing a little nearer the robot and lower.
FLAT_ARM_MAX_SHORT = 0.08
# Input pick IK (compute_flat_pick_joints): the arm shape with joints 3 and 5 nearest 0
# among solutions that keep this far (rad) from every joint limit at pick and pre-pick.
# ikpy's own solutions twist J3/J5 past 1 rad on about a third of logged picks, and a
# folded, twisted arm near the base set off Franka's self-collision reflex.
FLAT_PICK_MIN_LIMIT_GAP = 0.15
# 180 deg about the cup axis: the two cups swap places, the same grasp.
CUP_FLIP = np.diag([-1.0, -1.0, 1.0])
# Of the two cup orders, the pick IK prefers the one with joint 7 nearer its home value
# (frankapy's reset pose), so the wrist does not spin up to ~180 deg more than needed.
PICK_J7_HOME = np.pi / 4
PICK_J7_WEIGHT = 0.1   # cost per rad^2 of J7 away from PICK_J7_HOME, against J3^2 + J5^2

# ── LABEL PLACEMENT ──────────────────────────────────────────────────────────────
# compute_label_joints asks perception for the label pose this many times, this far apart,
# when it gets none (service failure, "ZoneN=FAIL", or no pose within 5 s) before arm 2's
# recovery window takes over.
LABEL_POSE_ATTEMPTS = 3
LABEL_POSE_RETRY_S = 1.0
# The label pose arrives from perception in panda_link0 with its z measured
# (Fast-FoundationStereo on the ZED), so nothing is carried over from arm 1's pick.

# The XY box ApplyLabel has always accepted a label pose in, plus a z band: z is the
# item's top, and items lie on the table (which the camera sees at z ~ 0.004), so a flat
# pouch comes out near 0 and a box at its height. Below the table is a depth error, and
# nothing we label is taller than ~0.35 m. Outside the box the pose is refused unmoved.
LABEL_POSE_BOUNDS = {'x_min': 0.2, 'x_max': 0.7, 'y_min': -0.6, 'y_max': 0.6,
                     'z_min': -0.01, 'z_max': 0.40}
# Label IK retries re-seed with this much random change (rad) per joint around the nominal
# seed. compute_pick_joints re-seeds over the whole joint range, which can converge on a
# configuration that reaches the zone from behind; on arm 2 that swings out over the
# dispenser side of the cell.
LABEL_SEED_JITTER = 0.6
# A label solution with the base (joint 1) or upper arm (joint 3) turned further than this
# (rad) is that from-behind configuration, whatever seed produced it: refused.
LABEL_MAX_BASE_TURN = 2.0
# A label is the same rectangle turned by 180 deg about the cup, so the published pose and
# the pose turned by this about the tool axis place it identically. Both are solved and the
# one with the smaller joint motion wins, instead of turning the wrist 120 deg one way when
# 30 deg the other way would do.
LABEL_TOOL_FLIP = np.diag([-1.0, -1.0, 1.0])

# ──────────────────────────────────────────────────────────────────────────────────

# Module-level logger; records go to rospy's rosout logger (terminal + /rosout, recorded per run).
log = logging.getLogger("orio_fsm")


class _ToRosout(logging.Handler):
    def emit(self, record):
        logging.getLogger('rosout').handle(record)


def _setup_logger():
    log.setLevel(logging.DEBUG)
    log.handlers.clear()
    log.addHandler(_ToRosout())
    log.propagate = False


# Run events on /orio/events (docs/LOGGING.md). Null until _init_events() after rospy.init_node.
class _NullTimed:
    def __enter__(self):
        self.data = {}
        return self

    def __exit__(self, *exc):
        return False


class _NullSink:
    enabled = False

    def emit(self, *a, **kw):
        return False

    def timed(self, *a, **kw):
        return _NullTimed()


events = _NullSink()


def _init_events():
    global events
    if EventSink is None or os.environ.get('ORIO_LOGGING', '1') == '0':
        return
    pub = rospy.Publisher(EVENTS_TOPIC, String, queue_size=50)
    events = EventSink(lambda s: pub.publish(String(data=s)), src='full_sequence')


# ==============================================================================
# GLOBAL EVENTS & SHARED STATE
# ==============================================================================

# Set to trigger a clean shutdown from any thread
shutdown_event = threading.Event()

# Set by Ctrl-C (_install_sigint_handler). Unlike a 'q' shutdown, which lets the
# in-flight motion finish, this stops both arms mid-motion.
interrupted = threading.Event()


def _install_sigint_handler():
    """Make Ctrl-C stop BOTH arms. Call after the FrankaArm objects exist.

    Each FrankaArm() installs its own SIGINT handler, and the last one wins: it
    cancels only that arm's motion and calls sys.exit(0). That neither shuts ROS
    down nor stops the smach threads, so the other arm carries on (and switches
    its cup back on). This handler only sets flags: the motion loop of each arm
    sees them, cancels its own skill and unwinds the state. A second Ctrl-C exits
    at once, for when something is stuck.
    """
    signal.signal(signal.SIGINT, lambda sig, frame: request_estop("Ctrl-C"))


def request_estop(source):
    """Stop both arms mid-motion (Ctrl-C, or the dashboard's E-STOP). A second
    request exits at once, skipping the clean-up."""
    if interrupted.is_set():
        print("\n[ORIO] Second %s: exiting now." % source, flush=True)
        os._exit(130)
    print("\n[ORIO] %s: stopping both arms (%s again to force exit)." % (source, source),
          flush=True)
    events.emit('operator', 'estop', source=source)
    interrupted.set()
    shutdown_event.set()


# Set by the dashboard's SHUTDOWN: each arm finishes its move and ends its task at the
# next pause point (never in contact with an item), then main() sends both arms home,
# turns the cups off and exits.
stop_home = threading.Event()


def request_home_shutdown():
    if stop_home.is_set() or interrupted.is_set():
        return False
    log.warning("[Stop] Shutdown requested: each arm finishes its move, then both go home")
    events.emit('operator', 'shutdown_home')
    stop_home.set()
    shutdown_event.set()
    return True


def _stopping():
    """True once an E-stop/Ctrl-C or a SHUTDOWN is under way."""
    return interrupted.is_set() or stop_home.is_set()


def _raise_if_interrupted():
    if interrupted.is_set():
        raise KeyboardInterrupt

# Per-arm recovery events — set while that arm is in the recovery GUI.
# The other arm only waits on the opposing event when it needs zone access.
recovery_event_arm1 = threading.Event()
recovery_event_arm2 = threading.Event()

# Ensures only one Tkinter window exists at a time — Tkinter is not thread-safe
_gui_lock = Lock()


# ==============================================================================
# PAUSE (dashboard) & LIVE STATUS
# ==============================================================================

class InterventionRequested(Exception):
    """Raised at an arm's next pause point after the dashboard's INTERVENE: the state
    ends there and the arm's recovery window opens, as after an error."""


# Set by /orio/intervene/<arm> (the dashboard's INTERVENE), cleared when that arm acts on it
intervene = {'arm1': threading.Event(), 'arm2': threading.Event()}


# Set while an INTERVENE paused both arms; the recovery window's answer resumes them
_intervention_pause = threading.Event()


def _request_intervention(arm):
    """INTERVENE on `arm`: pause BOTH arms at their next pause point, and have `arm`
    leave its pause wait for its recovery window. A run that was already paused stays
    paused afterwards; otherwise _end_intervention_pause resumes it."""
    if pause_ctl.pause():
        _intervention_pause.set()
    intervene[arm].set()


def _end_intervention_pause():
    """After a recovery window: undo the pause an INTERVENE made, if any."""
    if _intervention_pause.is_set():
        _intervention_pause.clear()
        pause_ctl.resume()


def _take_intervention(arm):
    """True (once) if the operator asked to intervene on `arm`."""
    if intervene[arm].is_set():
        intervene[arm].clear()
        return True
    return False


class PauseControl:
    """Operator pause, from the dashboard (/orio/pause, /orio/resume).

    A paused arm finishes the motion it is in and waits before its next one (gate()).
    The cups are left as they are, so a held item or label stays held. The move that
    follows a cup action at contact (retract after a pick, move away after a release)
    is never gated, so an arm does not stop touching an item.

    `epoch` counts pauses: FetchInput compares it to tell whether the input bin may have
    been changed after it planned its grasp.
    """

    def __init__(self):
        self._running = threading.Event()
        self._running.set()
        self._lock = Lock()
        self.epoch = 0
        self.held = {'arm1': False, 'arm2': False}   # True while that arm waits in gate()

    @property
    def paused(self):
        return not self._running.is_set()

    def pause(self):
        with self._lock:
            if self.paused:
                return False
            self.epoch += 1
            self._running.clear()
        log.warning("[Pause] Pause requested: each arm stops after its current motion")
        events.emit('operator', 'pause', epoch=self.epoch)
        return True

    def resume(self):
        with self._lock:
            if not self.paused:
                return False
            self._running.set()
        log.warning("[Pause] Resumed")
        events.emit('operator', 'resume', epoch=self.epoch)
        return True

    def gate(self, arm, raise_on_stop=True):
        """Wait here while paused. Returns True if it waited.

        This is also where a SHUTDOWN (stop_home) ends the arm's task, paused or not.

        A 'q' or Ctrl-C while waiting stops both arms where they are rather than
        letting them finish their task, since the operator may have hands in the
        cell: it sets `interrupted`.

        Either way it then raises KeyboardInterrupt, or returns for callers outside
        _safe_execute that check shutdown_event themselves (raise_on_stop=False).
        """
        if stop_home.is_set():
            if raise_on_stop:
                raise KeyboardInterrupt
            return False
        if raise_on_stop and _take_intervention(arm):
            raise InterventionRequested("Operator asked to intervene (dashboard)")
        if self._running.is_set():
            return False
        self.held[arm] = True
        log.info("[Pause] %s paused (last: %s)", arm.upper(), arm_status.get(arm, 'activity'))
        try:
            while not self._running.wait(0.2):
                if stop_home.is_set():
                    if raise_on_stop:
                        raise KeyboardInterrupt
                    return True
                if shutdown_event.is_set() or interrupted.is_set() or rospy.is_shutdown():
                    interrupted.set()
                    if raise_on_stop:
                        raise KeyboardInterrupt
                    return True
                if intervene[arm].is_set():   # INTERVENE while paused: go to it now
                    if raise_on_stop and _take_intervention(arm):
                        raise InterventionRequested("Operator asked to intervene (dashboard)")
                    return True
        finally:
            self.held[arm] = False
        log.info("[Pause] %s resuming", arm.upper())
        return True


class ArmStatus:
    """What each arm is doing, for the dashboard. Written by the states, read by the
    status publisher. `target` is where the arm is working: bin, A, B, output,
    dispenser, or home/safe (none)."""

    def __init__(self):
        self._lock = Lock()
        self._arms = {a: {'activity': 'Starting up', 'moving': False, 'cup': False,
                          'target': None} for a in ('arm1', 'arm2')}

    def set(self, arm, **fields):
        with self._lock:
            self._arms[arm].update(fields)

    def get(self, arm, key):
        with self._lock:
            return self._arms[arm][key]

    def snapshot(self):
        with self._lock:
            return {a: dict(v) for a, v in self._arms.items()}


pause_ctl = PauseControl()
arm_status = ArmStatus()

no_label_vacuum_check = False  # Set from --no-label-vacuum-check: skip the label cup's sensor checks
no_pnp_vacuum_test = False     # Set from --no-pnp-vacuum-test: skip the pick cup's sensor checks
pnp_pick_raw_min = PNP_PICK_RAW_MIN   # Set from --pnp-vacuum-min (see PNP_PICK_RAW_MIN)
motion_speed = DEFAULT_SPEED          # Set from --speed (see DEFAULT_SPEED)
settle_s = DEFAULT_SETTLE_S           # Set from --settle-s (see DEFAULT_SETTLE_S)

# When True, vacuum service calls and sensor asserts are skipped (dry-run
# without pneumatics). Set in main() from --no-vacuum / ORIO_DISABLE_PNEUMATICS.
disable_pneumatics = False

# Valid re-entry states for each arm (used by recovery GUI radio buttons)
ARM1_STATES = ['DECIDE_HUB', 'FETCH_INPUT', 'DROP_TO_ZONE', 'RETRIEVE', 'PLACE_OUTPUT']
ARM2_STATES = ['GET_LABEL', 'APPLY']
# Resuming ARM1 into one of these leads straight to a bin scan, so it goes home first
RESCAN_STATES = ('FETCH_INPUT', 'DECIDE_HUB')


# ==============================================================================
# TERMINAL INPUT — one owner of stdin
# ==============================================================================

class Console:
    """Single owner of stdin.

    state_machine.py had one listener thread ('q' to quit); dexnet_pnp.py read
    stdin directly for its confirm/refill prompts. Here both exist, so ONE
    background thread reads lines forever and routes them: to an active prompt
    (ask()) when there is one, otherwise 'q' requests shutdown.
    """

    def __init__(self):
        self._q = queue.Queue()
        self._waiting = threading.Event()
        self._lock = Lock()   # one prompt at a time
        self.prompt = None    # the open prompt, shown on the dashboard
        threading.Thread(target=self._reader, daemon=True).start()

    def _reader(self):
        while not shutdown_event.is_set() and not rospy.is_shutdown():
            try:
                line = input()
            except EOFError:
                break
            if self._waiting.is_set():
                self._q.put(line)
            elif line.strip().lower() == 'q':
                print("[ORIO] Shutdown requested.")
                shutdown_event.set()
                break

    def ask(self, prompt):
        """Print `prompt` as its own line and return the operator's answer
        (stripped, lowercased), or None on shutdown.

        The flush ordering matters under the launcher's pipeline: rospy logs go
        to stderr and the prompt to stdout, and with neither being a tty both
        are block-buffered — flush stderr first so the lines the operator is
        answering about are visible, then the prompt as its own full line (the
        launcher's awk classifier reads line-by-line).
        """
        with self._lock:
            sys.stderr.flush()
            print(prompt)
            sys.stdout.flush()
            self.prompt = prompt
            self._waiting.set()
            try:
                while not shutdown_event.is_set() and not rospy.is_shutdown():
                    try:
                        return self._q.get(timeout=0.2).strip().lower()
                    except queue.Empty:
                        continue
                return None
            finally:
                self._waiting.clear()
                self.prompt = None


console = None  # constructed in main()


# ==============================================================================
# UTILITIES
# ==============================================================================

def call_add_item_service(item_name, expiry):
    """Call the RFID registration service to associate a freshly-applied label with the item."""
    service_name = '/inventory/add_item'
    try:
        rospy.wait_for_service(service_name, timeout=5.0)
        proxy = rospy.ServiceProxy(service_name, AddLabeledItem)
        response = proxy(item_name=item_name, expiration_date=expiry)
        events.emit('rfid', item_name, expiry=expiry, ok=bool(response.success), msg=response.message)
        if response.success:
            log.info("[RFID] Registered label — item='%s' expiry='%s' msg='%s'",
                     item_name, expiry, response.message)
        else:
            log.warning("[RFID] Registration failed — item='%s' expiry='%s' msg='%s'",
                        item_name, expiry, response.message)
        return response.success
    except Exception as e:
        log.error("[RFID] Service call failed for %s: %s", service_name, e)
    return False


def _is_vacuum_service(service_name):
    """True for the pneumatic cup services (/orio/pnp_cup/*, /orio/lbl_cup/*)."""
    return '_cup/' in service_name


_service_messages = {}   # service name -> the message of its last response (call_trigger_service)


def call_trigger_service(service_name):
    """Call a ROS Trigger service. With disable_pneumatics, vacuum cup services
    are skipped (no-op, returns True); vision services are unaffected. The response's
    message is kept in _service_messages[service_name]."""
    _service_messages.pop(service_name, None)
    for prefix, arm in (('/orio/pnp_cup/', 'arm1'), ('/orio/lbl_cup/', 'arm2')):
        if service_name.startswith(prefix):   # commanded cup state, for the dashboard
            arm_status.set(arm, cup=service_name.endswith('/on'))
    if disable_pneumatics and _is_vacuum_service(service_name):
        log.info("[Vacuum] disabled — skipping %s", service_name)
        return True
    try:
        rospy.wait_for_service(service_name, timeout=5.0)
        proxy = rospy.ServiceProxy(service_name, Trigger)
        with events.timed('service', service_name) as ev:
            response = proxy()
            ev.data['ok'] = bool(response.success)
        log.info("[Service] %s → success=%s msg='%s'",
                 service_name, response.success,
                 getattr(response, 'message', ''))
        _service_messages[service_name] = str(getattr(response, 'message', ''))
        if service_name == '/compute_grasps':
            _grasp_verdict(response.success, getattr(response, 'message', ''))
        return response.success
    except Exception as e:
        log.error("[Service] Call failed for %s: %s", service_name, e)
        if service_name == '/compute_grasps':
            _grasp_verdict(False, "grasp service call failed: %s" % e)
    return False


def _grasp_verdict(ok, msg):
    """The latest bin grasp's outcome, for the dashboard's grasp picture."""
    arm_status.set('arm1', grasp={'t': time.time(), 'ok': bool(ok), 'msg': str(msg)})


# ==============================================================================
# SECTION 1: SYSTEM SETUP & DATA MANAGEMENT
# ==============================================================================

def _rot_err(R, Rd):
    """Small rotation (axis * angle, world frame) taking R to Rd."""
    E = Rd @ R.T
    return 0.5 * np.array([E[2, 1] - E[1, 2], E[0, 2] - E[2, 0], E[1, 0] - E[0, 1]])


class RobotHardware:
    """Wrapper holding Franka interfaces, IK, Vision, and poses."""

    def __init__(self, use_grasp_orientation=True, max_tilt_deg=45.0,
                 grasp_offset=(0.0, 0.0), zero_item_depth=False,
                 motion_timeout_margin=8.0):
        log.info("[Hardware] Initializing Robot Hardware...")
        # Pick-behaviour settings (ported from dexnet_pnp.py)
        self.use_grasp_orientation = use_grasp_orientation
        self.max_tilt_deg = float(max_tilt_deg)
        # World-frame XY correction added to every grasp position received from
        # perception, before the bounds check and IK. The planner and the
        # published pose stay raw; only what the arm is sent to moves.
        self.grasp_offset = np.array([float(grasp_offset[0]), float(grasp_offset[1]), 0.0])
        if np.any(self.grasp_offset != 0.0):
            log.info("[Grasp] world XY offset applied after the planner: (%+.3f, %+.3f) m",
                     self.grasp_offset[0], self.grasp_offset[1])
        # When set, PlaceOutput reaches into the output box by the fixed margin
        # alone, so every item is released at the same height.
        self.zero_item_depth = bool(zero_item_depth)
        # Extra seconds beyond a motion's `duration` before _move() gives up.
        self.motion_timeout_margin = float(motion_timeout_margin)

        self.pick_and_place_arm = FrankaArm(with_gripper=False, old_gripper=False, robot_num=1, init_node=False)
        self.label_arm = FrankaArm(with_gripper=False, old_gripper=False, robot_num=2, init_node=False)
        log.info("[Hardware] Resetting both arms to home joints")
        self._move('arm1', self.pick_and_place_arm.reset_joints, desc="initial reset arm1")
        self._move('arm2', self.label_arm.reset_joints, desc="initial reset arm2")

        # IK setup
        self.ik_chain = ikpy.chain.Chain.from_urdf_file(URDF_FILE, base_elements=["panda_link0"])
        self.chain_length = len(self.ik_chain.links)
        self.ik_chain.active_links_mask = [False] + [True]*7 + [False]*(self.chain_length-8)

        # Base transforms
        self.retract = RigidTransform(translation=[0.0, 0.0, APPROACH_DISTANCE + 0.1], from_frame='world', to_frame='world')

        # Load Poses
        with open(POSES_FILE) as f:
            raw = json.load(f)
        self.fixed_poses = {
            'ppa_DROP_ZONE':      raw['pick_place_arm']['DROP_ZONE'],
            'ppa_A':              raw['pick_place_arm']['LABEL_ZONE1'],
            'ppa_B':              raw['pick_place_arm']['LABEL_ZONE2'],
            'la_LABEL_DISPENSER': raw['label_arm']['LABEL_DISPENSER'],
            'la_A':               raw['label_arm']['LABEL_ZONE1'],
            'la_B':               raw['label_arm']['LABEL_ZONE2'],
        }
        self.safe_pos = raw['SAFE_POS']
        self.label_arm_dispenser_contact = raw['label_arm']['LABEL_DISPENSER']
        self.label_arm_dispenser_pre = self._contact_to_pre_joints(self.label_arm_dispenser_contact)
        log.info("[Hardware] Poses loaded from %s", POSES_FILE)

        # Vision State
        self.latest_pose_msg = None
        self.latest_label_pose_z1 = None
        self.latest_label_pose_z2 = None
        rospy.Subscriber("/grasp_poses",              PoseArray, self._pose_callback,          queue_size=1)
        # World-frame label poses with measured z (perception's /compute_grasps_labelling)
        rospy.Subscriber("/label_pose_z1", PoseArray, self._label_pose_callback_z1, queue_size=1)
        rospy.Subscriber("/label_pose_z2", PoseArray, self._label_pose_callback_z2, queue_size=1)

        # Stored dynamically per cycle
        self.current_fetch_depth = 0.0
        self.label_measured_z = {}   # zone number -> item top z the label arm last measured
        self.pick_up_pre_joints = None
        self.pick_up_final_joints = None

        # Collision Checker
        self.la_collision_checker = CollisionChecker.CollisionChecker(arm_number=2)

        # Vacuum suction flags, updated by the pneumatic_control node
        self.pnp_has_item = False
        self.lbl_has_item = False
        rospy.Subscriber("orio/vacuum/pnp_has_item", Bool,
                         lambda msg: setattr(self, 'pnp_has_item', msg.data), queue_size=1)
        rospy.Subscriber("orio/vacuum/lbl_has_item", Bool,
                         lambda msg: setattr(self, 'lbl_has_item', msg.data), queue_size=1)
        # Raw sensor readings, only to explain a failed vacuum check (None = none received)
        self.pnp_raw = None
        self.lbl_raw = None
        rospy.Subscriber("orio/vacuum/pnp_raw", Int32,
                         lambda msg: setattr(self, 'pnp_raw', msg.data), queue_size=1)
        rospy.Subscriber("orio/vacuum/lbl_raw", Int32,
                         lambda msg: setattr(self, 'lbl_raw', msg.data), queue_size=1)

        log.info("[Hardware] Initialization complete")

    def _pose_callback(self, msg):
        self.latest_pose_msg = msg

    def _label_pose_callback_z1(self, msg):
        self.latest_label_pose_z1 = msg

    def _label_pose_callback_z2(self, msg):
        self.latest_label_pose_z2 = msg

    def _arm(self, name):
        return self.pick_and_place_arm if name == 'arm1' else self.label_arm

    # ── Motion with a timeout (ported from dexnet_pnp.py) ─────────────────────
    def _move(self, arm_name, motion_fn, *args, timeout=None, desc="motion",
              pausable=True, **kwargs):
        """Run a blocking FrankaArm motion (reset_joints/goto_joints) with a HARD
        timeout, so a dead control PC can't hang us forever.

        frankapy's wait_for_skill() busy-loops with no timeout: if franka-interface
        stops executing (e.g. control PC crash), a blocking motion never returns.
        We instead issue it non-blocking and poll is_skill_done() against a
        deadline, raising a clear error on expiry — which the per-arm recovery
        GUI then catches like any other state failure.

        While paused (dashboard), waits before issuing the motion. pausable=False
        for the move right after a cup action at contact (see PauseControl).
        """
        if pausable:
            pause_ctl.gate(arm_name)
        if "duration" in kwargs:
            kwargs["duration"] = self._scaled_duration(arm_name, motion_fn, args,
                                                       kwargs["duration"])
        arm_status.set(arm_name, activity=desc, moving=True)
        try:
            self._move_timed(arm_name, motion_fn, args, kwargs, timeout, desc)
        finally:
            arm_status.set(arm_name, moving=False)

    def _scaled_duration(self, arm_name, motion_fn, args, duration):
        """duration / --speed, but no shorter than the joint travel allows (see
        MAX_PEAK_JOINT_SPEED) or MIN_MOVE_S. Unscaled if the arm's joints can't be read."""
        if motion_speed <= 1.0:
            return duration / motion_speed   # slower than base: no floor needed
        arm = self._arm(arm_name)
        if getattr(motion_fn, '__name__', '') == 'reset_joints':
            target = HOME_JOINTS
        elif args and np.ndim(args[0]) == 1 and len(args[0]) == 7:
            target = args[0]
        else:
            return duration
        try:
            travel = float(np.max(np.abs(np.asarray(target, dtype=float)
                                         - np.asarray(arm.get_joints(), dtype=float))))
        except Exception as exc:
            log.warning("[Move] %s: could not read joints (%s); not speeding this move up",
                        arm_name, exc)
            return duration
        floor = max(MIN_MOVE_S, MIN_JERK_PEAK * travel / MAX_PEAK_JOINT_SPEED)
        return float(min(duration, max(duration / motion_speed, floor)))

    def _move_timed(self, arm_name, motion_fn, args, kwargs, timeout, desc):
        duration = kwargs.get("duration", 5)
        if timeout is None:
            timeout = float(duration) + self.motion_timeout_margin
        kwargs["block"] = False
        # One timed 'cmd' event per motion (docs/LOGGING.md). name=arm1/arm2: the
        # recorder uses a cmd event's NAME as an entity prefix, so it must be the
        # arm, with the motion described in the data.
        with events.timed('cmd', arm_name) as ev:
            ev.data.update(desc=desc, duration=float(duration), timeout=float(timeout))
            if args and np.ndim(args[0]) == 1:
                ev.data['joints'] = [float(j) for j in args[0]]
            faults = self._run_motion(arm_name, motion_fn, args, kwargs, timeout, desc)
            ev.data['faults'] = faults

    def _run_motion(self, arm_name, motion_fn, args, kwargs, timeout, desc):
        """Issue the motion and poll it to completion; returns the recovered fault count.

        is_skill_done(ignore_errors=False) RAISES on a controller fault rather
        than silently swallowing it. Most of what it raises is RECOVERABLE
        (acceleration-discontinuity reflexes that franka-interface self-heals),
        so faults are logged and polling continues; only a fault the controller
        never recovers from hits the deadline and aborts the state.
        """
        arm = self._arm(arm_name)
        _raise_if_interrupted()      # Ctrl-C: issue no new motion
        motion_fn(*args, **kwargs)   # issue asynchronously
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        rate = rospy.Rate(20)
        faults = 0
        while True:
            try:
                if arm.is_skill_done(ignore_errors=False):
                    _raise_if_interrupted()   # done because Ctrl-C cancelled it
                    if settle_s > 0:
                        rospy.sleep(settle_s)   # let the arm settle (see DEFAULT_SETTLE_S)
                    return faults
            except Exception as exc:   # FrankaArmException / ...NotReadyException
                faults += 1
                if faults == 1:        # once per motion; the fault repeats while recovering
                    log.warning("[Move] %s (%s): controller fault (recovering): %s",
                                desc, arm_name, str(exc).split('\n')[0])
                try:
                    arm.wait_for_franka_interface()
                except Exception:
                    pass
            if rospy.Time.now() > deadline:
                # Cancel the stuck skill before raising, or frankapy keeps it
                # marked active and the recovery GUI's Reset Joints then dies
                # with "Cannot send another command when the previous skill is
                # active!".
                self._abort_skill(arm_name)
                raise RuntimeError(
                    "%s (%s) did not complete in %.1fs%s — the robot is not executing "
                    "(control PC crashed / FCI not active / e-stop, or it never recovered "
                    "from a fault)."
                    % (desc, arm_name, timeout,
                       "" if not faults else " after %d controller fault(s)" % faults))
            # On a 'q' shutdown the in-flight motion is allowed to FINISH (as in
            # state_machine.py); the states exit at their loop points. Ctrl-C (or a
            # ROS shutdown) aborts mid-motion.
            if interrupted.is_set() or rospy.is_shutdown():
                self._abort_skill(arm_name)
                raise KeyboardInterrupt
            rate.sleep()

    def _abort_skill(self, arm_name):
        """Best-effort cancel of an in-flight skill so the next command is accepted.

        Deliberately NOT frankapy's stop_skill(): it ends with an unbounded
        wait_for_skill() busy-loop, which hangs forever on exactly the
        dead-control-PC case this abort exists for.
        """
        arm = self._arm(arm_name)
        try:
            if getattr(arm, '_connected', False) and getattr(arm, '_in_skill', False):
                arm._client.cancel_goal()
        except Exception as exc:  # noqa: BLE001 — never mask the original failure
            log.warning("[Move] cancel_goal during abort failed (%s): %s", arm_name, exc)
        try:
            arm._in_skill = False
        except Exception:
            pass
        rospy.sleep(0.5)          # let franka-interface settle before the next command

    # ── Vacuum verification ──────────────────────────────────────────────────

    def assert_vacuum(self, cup, expected_state, timeout=1.0):
        """Poll the vacuum sensor for `cup` ('pnp'|'lbl') until it matches
        `expected_state`; raise RuntimeError if it doesn't within `timeout` s."""
        global disable_pneumatics, no_label_vacuum_check, no_pnp_vacuum_test, pnp_pick_raw_min
        if disable_pneumatics:
            return
        if cup == 'lbl' and no_label_vacuum_check:
            return
        if cup == 'pnp' and no_pnp_vacuum_test:
            return
        attr = 'pnp_has_item' if cup == 'pnp' else 'lbl_has_item'
        raw_attr = 'pnp_raw' if cup == 'pnp' else 'lbl_raw'
        readings = []
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        # The pick cup counts as holding an item on a weaker seal than the firmware's
        # has-item asks for (see PNP_PICK_RAW_MIN).
        weak_ok = cup == 'pnp' and expected_state
        while rospy.Time.now() < deadline:
            if getattr(self, attr) == expected_state:
                return
            raw_now = getattr(self, raw_attr)
            if raw_now is not None:
                readings.append(raw_now)
                if weak_ok and raw_now > pnp_pick_raw_min:
                    log.info("[Vacuum] PNP holds the item on a weak seal (raw %d; firmware "
                             "has-item needs > %d)", raw_now, VAC_THRESHOLD_HIGH)
                    return
            rospy.sleep(0.05)
        actual = getattr(self, attr)
        action = "pick up" if expected_state else "release"
        if not readings:
            raw = "no raw readings received"
        elif expected_state:
            need = pnp_pick_raw_min if cup == 'pnp' else VAC_THRESHOLD_HIGH
            raw = f"peak raw {max(readings)}, needs > {need}"
        else:
            raw = f"lowest raw {min(readings)}, needs < {VAC_THRESHOLD_LOW}"
        raise RuntimeError(
            f"{cup.upper()} vacuum failed to {action} item "
            f"(expected={expected_state}, actual={actual}; {raw})"
        )

    def _get_obs_file(self, arm_number, label_zone):
        """Return the obstacle file path for the given arm and label zone.

        label_zone=1|2   →  obstacles_files/obs_arm{n}_zone{k}.p
        other            →  obstacles_files/obs_arm{n}_free.p
        """
        script_dir = os.path.dirname(os.path.abspath(__file__))
        if label_zone in (1, 2):
            fname = f"obs_arm{arm_number}_zone{label_zone}.p"
        else:
            fname = f"obs_arm{arm_number}_free.p"
        return os.path.join(script_dir, "obstacles_files", fname)

    def _obstacles(self, arm_number, label_zone=None):
        """(obs_points, obs_axes) from the obstacle file for the given arm / label_zone,
        cached after the first call so subsequent checks are fast."""
        cache_key = (arm_number, label_zone)
        if not hasattr(self, '_obs_cache'):
            self._obs_cache = {}
        if cache_key not in self._obs_cache:
            obs_file = self._get_obs_file(arm_number, label_zone)
            with open(obs_file, 'rb') as f:
                obs_points = pickle.load(f)
                obs_axes = pickle.load(f)
            self._obs_cache[cache_key] = (obs_points, obs_axes)
            log.info("Arm%d: cached %d obstacles from '%s'",
                     arm_number, len(obs_points), obs_file)
        return self._obs_cache[cache_key]

    def check_collision(self, q7, arm_number, label_zone=None):
        """Return True if the 7-DOF config q7 is in collision with scene obstacles."""
        obs_points, obs_axes = self._obstacles(arm_number, label_zone)
        return self.la_collision_checker.DetectCollision(q7, obs_points, obs_axes)

    def _label_collisions(self, q7, allow_tool_on_table=False):
        """(block, obstacle) pairs of the arm-2 configuration q7 that collide.

        With allow_tool_on_table the cup's block may touch a table-level obstacle: the
        table is modelled with its top at z = 0 and the block reaches ~1 cm past the cup,
        so a label on a flat pouch (z ~ 0.005) would otherwise always count as a table
        collision, though touching there is the point of the placement."""
        obs_points, obs_axes = self._obstacles(2)
        cc = self.la_collision_checker
        cc.CompCollisionBlockPoints(q7)
        tool_block = len(cc.Cidx) - 1
        hits = []
        for i in range(len(cc.Cpoints)):
            for j in range(len(obs_points)):
                if (allow_tool_on_table and i == tool_block
                        and np.asarray(obs_points[j])[:, 2].max() <= 0.001):
                    continue
                if rt.CheckBoxBoxCollision(cc.Cpoints[i], cc.Caxes[i], obs_points[j], obs_axes[j]):
                    hits.append((i, j))
        return hits

    def compute_pick_joints(self, task_pos, target_ori=None,
                            label_zone=None, max_attempts=10,
                            check_collisions=False, arm_number=None, obstacle_zone_check=False):
        """(pre_joints, final_joints) for a pick at task_pos with orientation
        target_ori (defaults straight down).

        The pre waypoint is offset along the TOOL APPROACH AXIS (ported from
        dexnet_pnp.py), so a tilted cup travels down its own axis instead of
        diagonally; for the vertical default this is the world +Z offset the
        old code used. Collision checks and random IK re-seeding are kept from
        state_machine.py (the label arm needs them)."""
        target_ori = VERTICAL_ORI if target_ori is None else np.asarray(target_ori)
        log.info("[IK] compute_pick_joints called — task_pos=%s label_zone=%s max_attempts=%d check_collisions=%s arm_number=%s",
                 np.round(task_pos, 4), label_zone, max_attempts, check_collisions, arm_number)
        # Tool Z is the approach direction; retreat backs off along its negation.
        retreat = -target_ori[:, 2]
        task_pos = np.asarray(task_pos, dtype=float)
        pre_pos = list(task_pos + retreat * APPROACH_DISTANCE)
        final_pos = list(task_pos)

        for attempt in range(max_attempts):
            initial_guess = [0.0] * self.chain_length
            # joint[4] always set to -1.5 (gripper pointing down); all others randomized on retries
            initial_guess[4] = -1.5
            if attempt > 0:
                for i in range(self.chain_length):
                    if i == 4:
                        continue
                    lo, hi = self.ik_chain.links[i].bounds
                    initial_guess[i] = random.uniform(lo, hi)

            pre_angles = self.ik_chain.inverse_kinematics(
                target_position=pre_pos, target_orientation=target_ori,
                orientation_mode="all", initial_position=initial_guess)
            final_angles = self.ik_chain.inverse_kinematics(
                target_position=final_pos, target_orientation=target_ori,
                orientation_mode="all", initial_position=pre_angles)

            if check_collisions:
                if not obstacle_zone_check:
                    pre_collision   = self.check_collision(pre_angles[1:8],   arm_number, label_zone=None)
                    final_collision = self.check_collision(final_angles[1:8], arm_number, label_zone=None)
                else:
                    pre_collision   = self.check_collision(pre_angles[1:8],   arm_number, label_zone=label_zone)
                    final_collision = self.check_collision(final_angles[1:8], arm_number, label_zone=label_zone)
                if pre_collision or final_collision:
                    log.warning("[IK] attempt %d/%d in collision (pre=%s final=%s), retrying with new seed",
                                attempt + 1, max_attempts, pre_collision, final_collision)
                    continue

            # FK residual validation — reject solutions where IK converged poorly
            IK_RESIDUAL_THRESHOLD = 0.02  # metres
            fk_pre   = self.ik_chain.forward_kinematics(list(pre_angles))
            fk_final = self.ik_chain.forward_kinematics(list(final_angles))
            pre_err   = np.linalg.norm(fk_pre[:3, 3]   - np.array(pre_pos))
            final_err = np.linalg.norm(fk_final[:3, 3] - np.array(final_pos))
            if pre_err > IK_RESIDUAL_THRESHOLD or final_err > IK_RESIDUAL_THRESHOLD:
                log.warning("[IK] attempt %d/%d large FK residual (pre=%.4f m, final=%.4f m > %.3f m threshold) — retrying",
                            attempt + 1, max_attempts, pre_err, final_err, IK_RESIDUAL_THRESHOLD)
                continue

            log.info("[IK] Arm%s compute_pick_joints attempt %d/%d OK — target=(%s) pre=%s final=%s (FK err pre=%.4f m final=%.4f m)",
                     arm_number if arm_number is not None else "?", attempt + 1, max_attempts,
                     np.round(task_pos, 4),
                     np.round(pre_angles[1:8], 4),
                     np.round(final_angles[1:8], 4),
                     pre_err, final_err)
            events.emit('ik', f'arm{arm_number or 1}', target=list(task_pos), pre=pre_angles[1:8],
                        final=final_angles[1:8], attempt=attempt + 1, err_pre=pre_err, err_final=final_err,
                        label_zone=label_zone)

            return pre_angles[1:8], final_angles[1:8]

        raise RuntimeError(
            f"[IK] Failed to find a collision-free configuration for target={np.round(task_pos, 4)} "
            f"after {max_attempts} attempts"
        )

    # ── Grasp acquisition (ported from dexnet_pnp.py) ─────────────────────────
    def request_grasp(self, wait_s=5.0):
        """Trigger vision and return (task_pos, target_ori) for the best grasp,
        or None if the bin is empty / no valid grasp (a 'decline'). target_ori
        is None when approaching straight down."""
        self.latest_pose_msg = None
        if not call_trigger_service('/compute_grasps'):
            log.warning("[Grasp] /compute_grasps failed")
            events.emit('grasp', 'declined', reason='compute_grasps_failed', ok=False)
            return None

        deadline = rospy.Time.now() + rospy.Duration(wait_s)
        while self.latest_pose_msg is None and rospy.Time.now() < deadline:
            rospy.sleep(0.1)

        if self.latest_pose_msg is None or not self.latest_pose_msg.poses:
            log.info("[Grasp] No grasp pose returned (bin empty?)")
            _grasp_verdict(False, "no grasp pose returned")
            events.emit('grasp', 'declined',
                        reason='timeout' if self.latest_pose_msg is None else 'no_poses',
                        wait_s=float(wait_s), ok=False)
            return None

        p = self.latest_pose_msg.poses[0]
        raw_pos = [p.position.x, p.position.y, p.position.z]
        if any(np.isnan(v) for v in raw_pos):
            log.warning("[Grasp] NaN in grasp pose %s", raw_pos)
            _grasp_verdict(False, "NaN in the grasp pose")
            events.emit('grasp', 'declined', reason='nan_pose', ok=False)
            return None
        # Calibration residual, applied AFTER the planner: the published pose
        # stays raw; only what the arm is sent to moves.
        task_pos = [float(v) for v in np.asarray(raw_pos) + self.grasp_offset]
        if np.any(self.grasp_offset != 0.0):
            log.info("[Grasp] planner pose (%.3f, %.3f, %.3f) + offset (%+.3f, %+.3f) "
                     "-> (%.3f, %.3f, %.3f)", *raw_pos, self.grasp_offset[0],
                     self.grasp_offset[1], *task_pos)

        x, y, z = task_pos
        if not (PICK_BOUNDS['x_min'] <= x <= PICK_BOUNDS['x_max'] and
                PICK_BOUNDS['y_min'] <= y <= PICK_BOUNDS['y_max'] and
                PICK_BOUNDS['z_min'] <= z <= PICK_BOUNDS['z_max']):
            log.warning("[Grasp] pose (%.3f, %.3f, %.3f) outside safe bounds %s",
                        x, y, z, PICK_BOUNDS)
            _grasp_verdict(False, "outside the safe pick area")
            events.emit('grasp', 'declined', reason='out_of_bounds',
                        pos=[x, y, z], ok=False)
            return None

        target_ori = None
        tilt_deg = None          # stays None when approaching straight down
        if self.use_grasp_orientation:
            q = [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]
            if abs(np.linalg.norm(q) - 1.0) > 1e-3:
                log.warning("[Grasp] quaternion not unit (%.4f); approaching "
                            "vertically instead", np.linalg.norm(q))
            else:
                ori = R.from_quat(q).as_matrix()
                tilt = np.degrees(np.arccos(np.clip(-ori[2, 2], -1.0, 1.0)))
                if tilt > self.max_tilt_deg:
                    log.warning("[Grasp] tilt %.1f deg > max_tilt_deg %.1f; "
                                "refusing pick", tilt, self.max_tilt_deg)
                    _grasp_verdict(False, "tilt %.1f deg > max %.1f" % (tilt, self.max_tilt_deg))
                    events.emit('grasp', 'declined', reason='tilt_exceeded',
                                pos=[x, y, z], tilt_deg=float(tilt),
                                max_tilt_deg=self.max_tilt_deg, ok=False)
                    return None
                log.info("[Grasp] using grasp orientation, tilt %.1f deg", tilt)
                target_ori = ori
                tilt_deg = float(tilt)
        log.info("[Grasp] pick at (%.3f, %.3f, %.3f)", x, y, z)
        events.emit('grasp', 'accepted', pos=[x, y, z], raw_pos=list(raw_pos),
                    offset=self.grasp_offset[:2].tolist(), tilt_deg=tilt_deg,
                    straight_down=target_ori is None,
                    n_poses=len(self.latest_pose_msg.poses), ok=True)
        return task_pos, target_ori

    def compute_label_joints(self, zone_number, arm1_depth=None, max_attempts=10):
        """Call /compute_grasps_labelling, wait for the world-frame label pose on the
        matching topic, and return (pre_joints, final_joints) via the label IK.

        The pose's z is the item's top measured by Fast-FoundationStereo. arm1_depth, the
        pick height arm 1 carried over (what this used to place at), is only logged next
        to it for comparison."""
        log.info("[compute_label_joints] Requesting label pose for zone %d", zone_number)

        misses = 0   # service failures / no pose for this zone, up to LABEL_POSE_ATTEMPTS
        for attempt in range(1, max_attempts + 1):
            if zone_number == 1:
                self.latest_label_pose_z1 = None
            else:
                self.latest_label_pose_z2 = None

            miss = None
            if not call_trigger_service('/compute_grasps_labelling'):
                miss = "service /compute_grasps_labelling failed"
            elif "Zone%d=FAIL" % zone_number in _service_messages.get('/compute_grasps_labelling', ''):
                # perception found no placement in this zone: no pose is coming, don't wait for it
                miss = "perception found no label placement in zone %d" % zone_number
            else:
                deadline = rospy.Time.now() + rospy.Duration(5.0)
                while rospy.Time.now() < deadline:
                    msg = self.latest_label_pose_z1 if zone_number == 1 else self.latest_label_pose_z2
                    if msg is not None and msg.poses:
                        break
                    rospy.sleep(0.1)
                else:
                    miss = "no labelling pose received for zone %d within 5 s" % zone_number
            if miss:
                # Often the zone was looked at too soon (arm 1 still above it right after the
                # drop, or the item settling): look again a moment later.
                misses += 1
                if misses >= LABEL_POSE_ATTEMPTS:
                    raise RuntimeError("%s (%d attempts, %.0f s apart)"
                                       % (miss[0].upper() + miss[1:], misses, LABEL_POSE_RETRY_S))
                log.warning("[compute_label_joints] %s (attempt %d/%d); looking again in %.0f s",
                            miss, misses, LABEL_POSE_ATTEMPTS, LABEL_POSE_RETRY_S)
                rospy.sleep(LABEL_POSE_RETRY_S)
                continue

            p = msg.poses[0]
            task_pos = [p.position.x, p.position.y, p.position.z + LABEL_PLACE_CORRECTION_Z]
            task_ori = [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]

            if any(np.isnan(v) for v in task_pos + task_ori):
                log.warning("[compute_label_joints] Attempt %d/%d: NaN values in pose for zone %d "
                            "(pos=%s ori=%s), retrying...",
                            attempt, max_attempts, zone_number, task_pos, task_ori)
                continue

            if arm1_depth is not None:
                log.info("[compute_label_joints] Zone %d measured z=%.4f m (arm 1 carried over "
                         "%.4f m, difference %+.4f m)", zone_number, task_pos[2], arm1_depth,
                         task_pos[2] - arm1_depth)

            x, y, z = task_pos
            b = LABEL_POSE_BOUNDS
            if not (b['x_min'] <= x <= b['x_max'] and b['y_min'] <= y <= b['y_max']):
                err_msg = (f"[compute_label_joints] Zone {zone_number} pose (x={x:.4f}, y={y:.4f}) "
                           f"is outside bounds {b}")
                log.error(err_msg)
                raise RuntimeError(err_msg)
            if not (b['z_min'] <= z <= b['z_max']):
                err_msg = (f"[compute_label_joints] Zone {zone_number} z={z:.4f} m is outside "
                           f"[{b['z_min']}, {b['z_max']}]: the measured depth looks wrong "
                           f"(the table is at z ~ 0)")
                log.error(err_msg)
                raise RuntimeError(err_msg)

            self.label_measured_z[zone_number] = float(p.position.z)   # item top, for arm 1's retrieve
            task_ori_Mat = R.from_quat(task_ori).as_matrix()
            rpy = R.from_quat(task_ori).as_euler('xyz', degrees=True)
            log.info("[compute_label_joints] Zone %d received pose: x=%.4f y=%.4f z=%.4f  RPY r=%.2f p=%.2f y=%.2f deg",
                     zone_number,
                     task_pos[0], task_pos[1], task_pos[2],
                     rpy[0], rpy[1], rpy[2])

            return self._label_ik(task_pos, task_ori_Mat, zone_number)

        raise RuntimeError(f"NaN values in labelling pose for zone {zone_number} after {max_attempts} attempts")

    def _label_ik(self, task_pos, target_ori, zone_number, max_attempts=10):
        """(pre_joints, final_joints) for the label pose, from whichever of the two
        equivalent tool orientations (LABEL_TOOL_FLIP) needs the smaller joint motion from
        where arm 2 is now."""
        try:
            q_now = np.asarray(self.label_arm.get_joints(), dtype=float)
        except Exception as exc:  # noqa: BLE001 — LABELLING_SAFE is where APPLY starts from
            log.warning("[IK] arm2 get_joints failed (%s); using LABELLING_SAFE", exc)
            q_now = np.asarray(self.pose_to_joints("LABELLING_SAFE"), dtype=float)
        best = None
        for flipped in (False, True):
            ori = target_ori @ LABEL_TOOL_FLIP if flipped else target_ori
            name = "flipped" if flipped else "published"
            try:
                pre, final = self._solve_label_joints(task_pos, ori, zone_number, max_attempts)
            except RuntimeError as exc:
                log.warning("[IK] %s orientation: %s", name, exc)
                continue
            travel = float(np.abs(np.asarray(pre) - q_now).sum())
            log.info("[IK] %s orientation: joint 7 %.0f deg (%+.0f deg from now), "
                     "total joint travel to pre %.2f rad",
                     name, np.degrees(pre[6]), np.degrees(pre[6] - q_now[6]), travel)
            if best is None or travel < best[0]:
                best = (travel, pre, final, name)
        if best is None:
            raise RuntimeError(f"[IK] no collision-free label solution for {np.round(task_pos, 4)} "
                               f"in either tool orientation")
        travel, pre, final, name = best
        log.info("[IK] using the %s orientation (travel %.2f rad)", name, travel)
        return pre, final

    def _solve_label_joints(self, task_pos, target_ori, zone_number, max_attempts):
        """(pre_joints, final_joints) for one label orientation: collision-checked against
        the arm-2 obstacles and FK-validated as compute_pick_joints does, with the cup
        allowed to meet the table at the final pose (see _label_collisions), retries
        re-seeded near the nominal seed and from-behind configurations refused (see
        LABEL_SEED_JITTER / LABEL_MAX_BASE_TURN)."""
        task_pos = np.asarray(task_pos, dtype=float)
        pre_pos = task_pos + np.array([0.0, 0.0, APPROACH_DISTANCE])
        for attempt in range(max_attempts):
            seed = [0.0] * self.chain_length
            seed[4] = -1.5   # tool pointing down
            if attempt > 0:
                for i in range(1, 8):
                    lo, hi = self.ik_chain.links[i].bounds
                    seed[i] = float(np.clip(seed[i] + random.uniform(-LABEL_SEED_JITTER, LABEL_SEED_JITTER), lo, hi))
            pre = self.ik_chain.inverse_kinematics(
                target_position=list(pre_pos), target_orientation=target_ori,
                orientation_mode="all", initial_position=seed)
            final = self.ik_chain.inverse_kinematics(
                target_position=list(task_pos), target_orientation=target_ori,
                orientation_mode="all", initial_position=pre)
            if max(abs(pre[1]), abs(pre[3]), abs(final[1]), abs(final[3])) > LABEL_MAX_BASE_TURN:
                log.warning("[IK] attempt %d/%d reaches the zone from behind (joint 1 %.2f, "
                            "joint 3 %.2f rad), retrying with a new seed",
                            attempt + 1, max_attempts, final[1], final[3])
                continue
            hits_pre = self._label_collisions(pre[1:8])
            hits_final = self._label_collisions(final[1:8], allow_tool_on_table=True)
            if hits_pre or hits_final:
                log.warning("[IK] attempt %d/%d in collision (pre %s, final %s as "
                            "(block, obstacle)), retrying with a new seed",
                            attempt + 1, max_attempts, hits_pre, hits_final)
                continue
            IK_RESIDUAL_THRESHOLD = 0.02  # metres
            pre_err = np.linalg.norm(self.ik_chain.forward_kinematics(list(pre))[:3, 3] - pre_pos)
            final_err = np.linalg.norm(self.ik_chain.forward_kinematics(list(final))[:3, 3] - task_pos)
            if pre_err > IK_RESIDUAL_THRESHOLD or final_err > IK_RESIDUAL_THRESHOLD:
                log.warning("[IK] attempt %d/%d large FK residual (pre=%.4f m, final=%.4f m > %.3f m threshold) — retrying",
                            attempt + 1, max_attempts, pre_err, final_err, IK_RESIDUAL_THRESHOLD)
                continue
            log.info("[IK] Arm2 label IK attempt %d/%d OK — target=(%s) pre=%s final=%s (FK err pre=%.4f m final=%.4f m)",
                     attempt + 1, max_attempts, np.round(task_pos, 4),
                     np.round(pre[1:8], 4), np.round(final[1:8], 4), pre_err, final_err)
            events.emit('ik', 'arm2', target=list(task_pos), pre=pre[1:8], final=final[1:8],
                        attempt=attempt + 1, err_pre=pre_err, err_final=final_err,
                        label_zone=zone_number)
            return pre[1:8], final[1:8]
        raise RuntimeError(f"[IK] no collision-free configuration for target={np.round(task_pos, 4)} "
                           f"after {max_attempts} attempts")

    def pose_to_joints(self, location_tag, depth_offset=0.0, flat=False):
        """Resolve a location tag to joint angles via IK.

        Args:
            location_tag (str): Key in Target_Task_Poses.json, e.g. 'LABEL_ZONE_1'.
            depth_offset (float): Offset added to the z-translation before IK (default 0.0).
            flat (bool): prefer the solution with joints 3 and 5 at 0 (see _flat_ik), so
                the same pose always gets the same untwisted arm; falls back to ikpy.

        Returns:
            np.ndarray: 7-element joint angle array compatible with goto_joints().
        """
        with open(TARGET_POSES_FILE) as f:
            task_poses = json.load(f)

        if location_tag not in task_poses:
            raise KeyError(f"Location tag '{location_tag}' not found in {TARGET_POSES_FILE}. "
                           f"Available: {list(task_poses.keys())}")

        entry = task_poses[location_tag]
        tx, ty, tz = entry["translation"]
        tz += depth_offset
        target_pos = [tx, ty, tz]
        target_rot = np.array(entry["rotation"])

        if flat:
            q, err = self._flat_ik(target_pos, target_rot, max_err=FLAT_ARM_MAX_SHORT)
            if q is not None:
                log.info("[IK] pose_to_joints '%s' depth_offset=%.4f (flat arm%s) → joints=%s",
                         location_tag, depth_offset,
                         "" if err < 0.002 else ", %.0f mm short: out of reach" % (1000 * err),
                         np.round(q, 4))
                return q
            log.warning("[IK] pose_to_joints '%s': no flat-arm solution, using ikpy's", location_tag)

        initial_guess = [0.0] * self.chain_length
        if 4 < self.chain_length:
            initial_guess[4] = -1.5

        angles = self.ik_chain.inverse_kinematics(
            target_position=target_pos,
            target_orientation=target_rot,
            orientation_mode="all",
            initial_position=initial_guess,
        )
        problem, first_err = self._ik_check(angles, target_pos, target_rot)
        if problem:
            # e.g. OUTPUT1 from a tall item: this seed gives a twisted arm with joint 3
            # past franka-interface's limit, which then refuses every motion. Other seeds,
            # keeping the valid solution nearest to this one (similar arm shape) that is
            # no less accurate.
            log.warning("[IK] pose_to_joints '%s': first solution rejected (%s); trying "
                        "other seeds", location_tag, problem)
            rng = np.random.default_rng(zlib.crc32(location_tag.encode()))   # repeatable
            best = None
            for _ in range(IK_RESEEDS):
                guess = [0.0] * self.chain_length
                for i in range(1, 8):
                    lo, hi = self.ik_chain.links[i].bounds
                    guess[i] = rng.uniform(lo + IK_LIMIT_MARGIN, hi - IK_LIMIT_MARGIN)
                cand = self.ik_chain.inverse_kinematics(
                    target_position=target_pos, target_orientation=target_rot,
                    orientation_mode="all", initial_position=guess)
                bad, err = self._ik_check(cand, target_pos, target_rot)
                if bad or err > max(first_err, 0.005) + 0.002:
                    continue
                dist = np.abs(np.asarray(cand[1:8]) - np.asarray(angles[1:8])).sum()
                if best is None or dist < best[0]:
                    best = (dist, cand)
            if best is None:
                raise RuntimeError(
                    "[IK] no joint solution for '%s' (depth offset %.3f m) at least %.2f rad "
                    "inside the joint limits: %s" % (location_tag, depth_offset,
                                                     IK_LIMIT_MARGIN, problem))
            angles = best[1]
        log.info("[IK] pose_to_joints '%s' depth_offset=%.4f → joints=%s",
                 location_tag, depth_offset, np.round(angles[1:8], 4))
        return angles[1:8]

    def _ik_check(self, angles, target_pos, target_rot):
        """(problem, position error in m) for an IK result. problem is None, or why it is
        unusable: a joint within IK_LIMIT_MARGIN of its limit (franka-interface stops
        ~0.017 rad short of the hardware limit and then refuses every motion)."""
        fk = self.ik_chain.forward_kinematics(list(angles))
        pos_err = float(np.linalg.norm(fk[:3, 3] - np.asarray(target_pos)))
        for i in range(1, 8):
            lo, hi = self.ik_chain.links[i].bounds
            if not (lo + IK_LIMIT_MARGIN <= angles[i] <= hi - IK_LIMIT_MARGIN):
                return "joint %d at %.3f rad, limits %.3f..%.3f" % (i, angles[i], lo, hi), pos_err
        return None, pos_err

    def compute_retract_joints(self, start_joints, distance, step=0.005, quiet=False):
        """Joints `distance` m further back along the tool axis from the pose at
        start_joints, keeping the orientation; None if there is no usable solution.

        Solved by small steps along that line with the Jacobian pseudo-inverse (the
        least joint motion each step) rather than ikpy, whose solution can change the
        arm's shape: one goto_joints move then curves well off the line (up to 70 mm on
        logged picks for 20 cm, vs 39 mm max / 5 mm median this way)."""
        fk, rot_err = self._fk7, _rot_err
        q = np.array(start_joints, dtype=float)
        T0 = fk(q)
        ori, start = T0[:3, :3], T0[:3, 3]
        n = max(1, int(np.ceil(distance / step)))
        for k in range(1, n + 1):
            target = start - ori[:, 2] * distance * k / n
            for _ in range(5):
                T, J = self._fk_jacobian7(q)
                e = np.r_[target - T[:3, 3], rot_err(T[:3, :3], ori)]
                if np.linalg.norm(e) < 1e-6:
                    break
                q = q + np.linalg.pinv(J) @ e
        full = [0.0] * self.chain_length
        full[1:8] = q
        problem, err = self._ik_check(full, target, ori)
        if problem or err > 0.005:
            if not quiet:
                log.warning("[IK] no %.2f m retract solution (%s, FK err %.4f m)", distance,
                            problem or "inaccurate", err)
            return None
        return q

    def _fk7(self, q):
        """Flange pose (4x4) for the 7 arm joints q."""
        full = [0.0] * self.chain_length
        full[1:8] = q
        return self.ik_chain.forward_kinematics(full)

    def _fk_jacobian7(self, q, h=1e-6):
        """(pose, 6x7 Jacobian: position rows, then rotation rows) at q, numerically."""
        T = self._fk7(q)
        J = np.zeros((6, 7))
        for j in range(7):
            dq = np.zeros(7)
            dq[j] = h
            T2 = self._fk7(q + dq)
            J[:3, j] = (T2[:3, 3] - T[:3, 3]) / h
            J[3:, j] = _rot_err(T[:3, :3], T2[:3, :3]) / h
        return T, J

    def _flat_ik(self, target_pos, target_rot, max_err=0.002, iters=300, seed=None,
                 quiet=False):
        """Joints reaching the pose with the arm as flat as it allows: joints 3 and 5
        pulled to 0 (the arm in one vertical plane, upper arm not twisted), which for a
        cup pointing down is an exact solution wherever it can reach. Gauss-Newton on the
        pose, with that pull in the Jacobian's null space so it never costs accuracy, kept
        IK_LIMIT_MARGIN inside the joint limits. Where the pose is out of reach it ends
        at the nearest it can get. Returns (joints, position error in m), or (None, err)
        if that error is over max_err or a joint ends near its limit. seed: the start
        (default FLAT_ARM_SEED turned to face the target); quiet: no warning on failure."""
        target_pos = np.asarray(target_pos, dtype=float)
        lims = np.array([self.ik_chain.links[i].bounds for i in range(1, 8)], dtype=float)
        lo, hi = lims[:, 0] + IK_LIMIT_MARGIN, lims[:, 1] - IK_LIMIT_MARGIN
        if seed is None:
            q = np.array(FLAT_ARM_SEED, dtype=float)
            q[0] = np.arctan2(target_pos[1], target_pos[0])   # face the target
        else:
            q = np.clip(np.array(seed, dtype=float), lo, hi)
        pull = np.diag([0, 0, 1, 0, 1, 0, 0]).astype(float)
        for _ in range(iters):
            T, J = self._fk_jacobian7(q)
            e = np.r_[target_pos - T[:3, 3], _rot_err(T[:3, :3], target_rot)]
            Jp = np.linalg.pinv(J)
            dq = Jp @ e + (np.eye(7) - Jp @ J) @ (pull @ -q)
            n = np.linalg.norm(dq)
            if n < 1e-9:
                break
            q = np.clip(q + (dq * (0.3 / n) if n > 0.3 else dq), lo, hi)
        full = [0.0] * self.chain_length
        full[1:8] = q
        problem, err = self._ik_check(full, target_pos, target_rot)
        if problem or err > max_err:
            if not quiet:
                log.warning("[IK] no flat-arm solution (%s, FK err %.4f m)",
                            problem or "inaccurate", err)
            return None, err
        return q, err

    def compute_flat_pick_joints(self, task_pos, target_ori=None):
        """(pre_joints, final_joints) for the input pick with joints 3 and 5 as near 0 as
        the grasp allows, or None if no such solution keeps FLAT_PICK_MIN_LIMIT_GAP from
        every joint limit (the caller then uses compute_pick_joints).

        The grasp is kept exactly: position, cup axis and cup line, or the cup line turned
        180 deg (CUP_FLIP), which only swaps the two identical cups. The final joints come
        from _flat_ik from a few starts; the pre joints from backing straight up the cup
        axis by APPROACH_DISTANCE (compute_retract_joints), so pre-pick and pick have the
        same arm shape and the descent is a straight line. Of the valid solutions the one
        with the smallest J3^2 + J5^2 (+ PICK_J7_WEIGHT x J7's distance from PICK_J7_HOME
        squared, which picks the cup order with less wrist turn) wins, preferring one that
        also allows the INPUT_RETRACT_DISTANCE retract."""
        target_ori = VERTICAL_ORI if target_ori is None else np.asarray(target_ori)
        task_pos = np.asarray(task_pos, dtype=float)
        lims = np.array([self.ik_chain.links[i].bounds for i in range(1, 8)], dtype=float)

        def limit_gap(q):
            return float(np.min(np.minimum(q - lims[:, 0], lims[:, 1] - q)))

        yaw = np.arctan2(task_pos[1], task_pos[0])
        starts = [[yaw, 0.3, 0.0, -2.2, 0.0, 2.5, j7] for j7 in (-2.0, -0.8, 0.4, 1.6, 2.6)]
        starts += [[yaw, -0.3, 0.0, -2.6, 0.0, 2.3, j7] for j7 in (-0.8, 0.4, 1.6)]
        best = None
        tried = 0
        for ori in (target_ori, target_ori @ CUP_FLIP):
            for seed in starts:
                tried += 1
                final, _ = self._flat_ik(task_pos, ori, seed=seed, quiet=True)
                if final is None:
                    continue
                pre = self.compute_retract_joints(final, APPROACH_DISTANCE, quiet=True)
                if pre is None:
                    continue
                gap = min(limit_gap(pre), limit_gap(final))
                if gap < FLAT_PICK_MIN_LIMIT_GAP:
                    continue
                retract_ok = self.compute_retract_joints(
                    final, INPUT_RETRACT_DISTANCE, quiet=True) is not None
                flat_cost = final[2] ** 2 + final[4] ** 2
                cost = (flat_cost + PICK_J7_WEIGHT * (final[6] - PICK_J7_HOME) ** 2
                        + (0.0 if retract_ok else 10.0))
                if best is None or cost < best[0]:
                    best = (cost, pre, final, ori is not target_ori, gap)
                if flat_cost < 0.05 and retract_ok:
                    break   # this cup order: J3/J5 within ~0.2 rad, more starts add nothing
        if best is None:
            log.warning("[IK] no flat-arm pick solution %.2f rad inside the joint limits "
                        "(%d starts) for target %s", FLAT_PICK_MIN_LIMIT_GAP, tried,
                        np.round(task_pos, 4))
            return None
        _, pre, final, flipped, gap = best
        log.info("[IK] flat-arm pick: J3/J5 %+.2f/%+.2f, %.2f rad from the nearest joint "
                 "limit%s — pre=%s final=%s", final[2], final[4], gap,
                 ", cups swapped (180 deg)" if flipped else "",
                 np.round(pre, 4), np.round(final, 4))
        events.emit('ik', 'arm1', target=list(task_pos), pre=pre, final=final,
                    flat=True, cups_swapped=bool(flipped), limit_gap=gap)
        return pre, final

    def _contact_to_pre_joints(self, contact_joints):
        full = [0.0] * self.chain_length
        full[1:8] = contact_joints
        tf = self.ik_chain.forward_kinematics(full)
        pre_pos = tf[:3, 3] + np.array([0.0, 0.0, APPROACH_DISTANCE])
        initial_guess = [0.0] * self.chain_length
        if 4 < self.chain_length:
            initial_guess[4] = -1.5
        pre_angles = self.ik_chain.inverse_kinematics(
            target_position=pre_pos, target_orientation=tf[:3, :3], orientation_mode="all", initial_position=initial_guess)
        return pre_angles[1:8]


class ZoneManager:
    def __init__(self):
        self.lock = Lock()
        self.states = {'A': 'EMPTY', 'B': 'EMPTY'}  # Initial states: EMPTY, NEEDS_LABEL, READY_FOR_PICKUP
        self.busy = {'A': False, 'B': False}        # False, or the arm using it ('arm1'/'arm2')
        self.depths = {'A': 0.0, 'B': 0.0}          # Tracks object depth per zone
        self.measured_z = {'A': None, 'B': None}    # item top z measured by the ZED before labelling

    def set_state(self, zone, new_state):
        """Set zone state and log the transition."""
        old_state = self.states[zone]
        self.states[zone] = new_state
        log.info("[ZoneManager] Zone %s: %s → %s", zone, old_state, new_state)
        events.emit('zone', zone, old=old_state, new=new_state)


# Globals
manager = ZoneManager()
hardware = None  # Will be initialized in main()

# Operator settings from argparse (set in main())
opts = None

# Item metadata provided by the operator at startup
_item_name = ""
_item_expiry = ""

# Used instead of the setup GUI with --skip-item-setup
DEFAULT_ITEM_NAME = "test_item"
DEFAULT_EXPIRY_DAYS = 7

# Cycle counter for output tracking
_items_placed_output = 0
# Pick counter: the 'pick' timeline in the run recording
_pick_n = 0
# Consecutive planner declines (module level: FetchInput is re-entered per attempt)
_declines = 0
# Set after --max-declines empty scans: arm 1 loads nothing until the operator resumes it
# into FETCH_INPUT or DECIDE_HUB from its recovery window (INTERVENE on the dashboard)
_bin_empty = threading.Event()
BIN_EMPTY_ACTIVITY = "Input bin empty: refill it, then INTERVENE"
# Labels applied (dashboard)
_labels_applied = 0
# The two arm state machines and the run's start time (dashboard), set in main()
_machines = {}
_run_started = None

# ==============================================================================
# SECTION 2: HUMAN RECOVERY GUI & HELPERS
# ==============================================================================

def _show_recovery_gui(arm_name, failed_state, error_msg, valid_states,
                       fa, vacuum_on_service, vacuum_off_service):
    """Show a blocking Tkinter recovery dialog for the failed arm only.

    Returns:
        The chosen state name (str), or 'shutdown'.
    """
    result = {'choice': 'shutdown'}

    def _run():
        root = tk.Tk()
        root.title(f"[ORIO] Human Recovery — {arm_name}")
        root.configure(bg='#1e1e2e')
        root.resizable(False, False)

        # ── Header ──────────────────────────────────────────────────────────
        header = tk.Frame(root, bg='#e55', pady=8)
        header.pack(fill='x')
        tk.Label(header, text="  HUMAN INTERVENTION REQUIRED",
                 font=('Helvetica', 14, 'bold'), fg='white', bg='#e55').pack()

        # ── Failure info ─────────────────────────────────────────────────────
        info = tk.Frame(root, bg='#1e1e2e', padx=16, pady=10)
        info.pack(fill='x')
        tk.Label(info, text=f"Arm:          {arm_name}",
                 font=('Courier', 11), fg='#cdd6f4', bg='#1e1e2e', anchor='w').pack(fill='x')
        tk.Label(info, text=f"Failed state: {failed_state}",
                 font=('Courier', 11), fg='#f38ba8', bg='#1e1e2e', anchor='w').pack(fill='x')
        tk.Label(info, text=f"Error:        {str(error_msg)[:120]}",
                 font=('Courier', 10), fg='#fab387', bg='#1e1e2e', anchor='w',
                 wraplength=480, justify='left').pack(fill='x', pady=(0, 6))

        # ── Arm controls ─────────────────────────────────────────────────────
        ctrl_frame = tk.LabelFrame(root, text=f" Arm Controls — {arm_name} ",
                                   bg='#1e1e2e', fg='#89b4fa',
                                   font=('Helvetica', 10, 'bold'), padx=12, pady=8)
        ctrl_frame.pack(fill='x', padx=16, pady=(0, 6))

        status_var = tk.StringVar(value="")
        status_lbl = tk.Label(ctrl_frame, textvariable=status_var, font=('Courier', 10),
                              fg='#a6e3a1', bg='#1e1e2e', anchor='w')
        status_lbl.pack(fill='x', pady=(0, 6))

        def _call_vacuum(service_name):
            """Call a vacuum service, with a single reconnect attempt on failure."""
            if disable_pneumatics:
                log.info("[Recovery GUI] vacuum disabled — skipping %s", service_name)
                return
            try:
                rospy.wait_for_service(service_name, timeout=5.0)
                proxy = rospy.ServiceProxy(service_name, Trigger)
                proxy()
            except Exception as exc:
                raise RuntimeError(f"Vacuum service {service_name} failed: {exc}")

        btn_row1 = tk.Frame(ctrl_frame, bg='#1e1e2e')
        btn_row1.pack(fill='x', pady=(0, 4))
        btn_row2 = tk.Frame(ctrl_frame, bg='#1e1e2e')
        btn_row2.pack(fill='x')

        _BTN = dict(relief='flat', padx=10, pady=4, font=('Courier', 10))

        def _run_in_thread_guarded(fn, label):
            """Run fn() in a background thread; buttons stay enabled so multiple
            actions can be queued while a previous one is executing."""
            status_var.set(f"Running: {label}…")

            def _task():
                try:
                    fn()
                    root.after(0, lambda: status_var.set(f"Done: {label}"))
                except Exception as exc:
                    root.after(0, lambda: status_var.set(f"Error: {label} — {exc}"))
                    log.error("[Recovery GUI] %s failed: %s", label, exc)
            threading.Thread(target=_task, daemon=True).start()

        b_stop = tk.Button(btn_row1, text="Stop Skill", bg='#f38ba8', fg='#1e1e2e',
                           command=lambda: _run_in_thread_guarded(fa.stop_skill, "Stop Skill"),
                           **_BTN)
        b_stop.pack(side='left', padx=(0, 6))

        b_reset = tk.Button(btn_row1, text="Reset Joints", bg='#89b4fa', fg='#1e1e2e',
                            command=lambda: _run_in_thread_guarded(fa.reset_joints, "Reset Joints"),
                            **_BTN)
        b_reset.pack(side='left', padx=(0, 6))

        b_von = tk.Button(btn_row2, text="Vacuum ON", bg='#a6e3a1', fg='#1e1e2e',
                          command=lambda: _run_in_thread_guarded(
                              lambda: _call_vacuum(vacuum_on_service), "Vacuum ON"),
                          **_BTN)
        b_von.pack(side='left', padx=(0, 6))

        b_voff = tk.Button(btn_row2, text="Vacuum OFF", bg='#fab387', fg='#1e1e2e',
                           command=lambda: _run_in_thread_guarded(
                               lambda: _call_vacuum(vacuum_off_service), "Vacuum OFF"),
                           **_BTN)
        b_voff.pack(side='left')

        # ── Zone state editor ────────────────────────────────────────────────
        zone_frame = tk.LabelFrame(root, text=" Zone Manager State ",
                                   bg='#1e1e2e', fg='#89b4fa',
                                   font=('Helvetica', 10, 'bold'), padx=12, pady=8)
        zone_frame.pack(fill='x', padx=16, pady=(0, 6))

        zone_options = ['EMPTY', 'NEEDS_LABEL', 'READY_FOR_PICKUP']
        zone_vars = {}
        for zone in ['A', 'B']:
            row = tk.Frame(zone_frame, bg='#1e1e2e')
            row.pack(fill='x', pady=2)
            tk.Label(row, text=f"Zone {zone}:", width=8,
                     font=('Courier', 11), fg='#cdd6f4', bg='#1e1e2e').pack(side='left')
            var = tk.StringVar(value=manager.states[zone])
            zone_vars[zone] = var
            ttk.Combobox(row, textvariable=var, values=zone_options,
                         state='readonly', width=22).pack(side='left', padx=6)

        def apply_zone_changes():
            with manager.lock:
                for zone in ['A', 'B']:
                    manager.states[zone] = zone_vars[zone].get()
                    manager.busy[zone] = False
            log.info("[Recovery] Zones updated: %s  busy: %s", manager.states, manager.busy)

        tk.Button(zone_frame, text="Apply Zone Changes", command=apply_zone_changes,
                  bg='#313244', fg='#cdd6f4', relief='flat', padx=8).pack(pady=(6, 0))

        # ── State selector ───────────────────────────────────────────────────
        sel_frame = tk.LabelFrame(root, text=f" Jump to State — {arm_name} ",
                                  bg='#1e1e2e', fg='#89b4fa',
                                  font=('Helvetica', 10, 'bold'), padx=12, pady=8)
        sel_frame.pack(fill='x', padx=16, pady=(0, 8))
        tk.Label(sel_frame, text="Choose the state to run next:",
                 font=('Courier', 11), fg='#cdd6f4', bg='#1e1e2e').pack(anchor='w')
        choice_var = tk.StringVar(value=valid_states[0])
        for s in valid_states:
            tk.Radiobutton(sel_frame, text=s, variable=choice_var, value=s,
                           font=('Courier', 11), fg='#a6e3a1', bg='#1e1e2e',
                           selectcolor='#313244', activebackground='#1e1e2e').pack(anchor='w')

        # ── Action buttons ───────────────────────────────────────────────────
        btn_frame = tk.Frame(root, bg='#1e1e2e', pady=10)
        btn_frame.pack(fill='x', padx=16)

        def on_resume():
            apply_zone_changes()
            result['choice'] = choice_var.get()
            root.destroy()

        def on_estop():
            if messagebox.askyesno("E-Stop", "Emergency stop — shut down everything?", parent=root):
                result['choice'] = 'shutdown'
                root.destroy()

        tk.Button(btn_frame, text="Resume →", command=on_resume,
                  bg='#a6e3a1', fg='#1e1e2e', font=('Helvetica', 12, 'bold'),
                  relief='flat', padx=16, pady=6).pack(side='left', padx=(0, 12))
        tk.Button(btn_frame, text="E-Stop / Shutdown", command=on_estop,
                  bg='#f38ba8', fg='#1e1e2e', font=('Helvetica', 12, 'bold'),
                  relief='flat', padx=16, pady=6).pack(side='left')

        root.protocol("WM_DELETE_WINDOW", on_estop)

        def poll_stop():   # E-STOP/Ctrl-C or SHUTDOWN from elsewhere: close and end this arm
            if _stopping():
                result['choice'] = 'shutdown'
                root.destroy()
                return
            root.after(300, poll_stop)
        root.after(300, poll_stop)
        root.lift()
        root.focus_force()
        root.mainloop()

    def _run_and_free():
        _run()
        # The window's Tk objects sit in reference cycles (poll_stop reschedules itself),
        # so they are not freed when _run returns. Free them HERE, in the thread that made
        # them: left to the garbage collector, they are freed later from whichever thread
        # it runs in, and Tcl aborts the whole process ("Tcl_AsyncDelete: async handler
        # deleted by the wrong thread").
        gc.collect()

    with _gui_lock:  # only one Tkinter window at a time — Tk is not thread-safe
        gui_thread = threading.Thread(target=_run_and_free, daemon=True)
        gui_thread.start()
        gui_thread.join()  # block the smach thread until the operator responds
    return result['choice']


def _safe_execute(state_name, fn, userdata):
    """Run fn(userdata); catch any exception, store diagnostics, and return 'failed'.

    After an E-stop/Ctrl-C or SHUTDOWN the state was stopped on purpose: return
    'failed' without the error report, and the recovery state ends this arm instead
    of opening its GUI.
    """
    try:
        return fn(userdata)
    except (Exception, KeyboardInterrupt) as exc:
        if _stopping():
            log.warning("[%s] stopped (%s)", state_name,
                        "E-stop" if interrupted.is_set() else "shutdown")
            return 'failed'
        if isinstance(exc, KeyboardInterrupt):
            raise
        if isinstance(exc, InterventionRequested):
            log.warning("[%s] operator intervention: opening the recovery window", state_name)
            events.emit('operator', 'intervene', state=state_name)
        else:
            log.error("[%s] Unhandled exception: %s", state_name, exc)
            events.emit('error', state_name, msg=str(exc), ok=False)
        userdata.recovery_failed_state = state_name
        userdata.recovery_error_msg = str(exc)
        return 'failed'


class HumanRecoveryArm1(smach.State):
    """Entered whenever an ARM1 state raises an exception.
    Only ARM1 is paused; ARM2 keeps running independently."""

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=ARM1_STATES + ['shutdown'],
                             input_keys=['recovery_failed_state', 'recovery_error_msg'])

    def execute(self, userdata):
        global _declines
        if _stopping():
            return 'shutdown'
        log.warning("[ARM1 Recovery] Pausing ARM1 — showing GUI (ARM2 continues)")
        recovery_event_arm1.set()
        with manager.lock:  # release any zone locks held by the failed state
            for z in ['A', 'B']:
                manager.busy[z] = False
        failed = getattr(userdata, 'recovery_failed_state', 'UNKNOWN')
        err_msg = getattr(userdata, 'recovery_error_msg', 'No details available.')
        while True:
            choice = _show_recovery_gui(
                "ARM1 (Pick-and-Place)", failed, err_msg, ARM1_STATES,
                fa=hardware.pick_and_place_arm,
                vacuum_on_service='/orio/pnp_cup/on',
                vacuum_off_service='/orio/pnp_cup/off')
            log.info("[ARM1 Recovery] choice=%s", choice)
            events.emit('recovery', 'arm1', failed_state=failed, error=err_msg, choice=choice)
            recovery_event_arm1.clear()
            _end_intervention_pause()
            if choice == 'shutdown':
                shutdown_event.set()
                return choice
            if choice not in RESCAN_STATES:
                break
            if _bin_empty.is_set():   # the operator says the bin has items again
                log.info("[ARM1 Recovery] resuming picking: the input bin no longer counts "
                         "as empty")
                events.emit('operator', 'refilled', pick=_pick_n)
                _bin_empty.clear()
                _declines = 0
            # The next state photographs the bin: go home first, so the arm is not in
            # the picture the grasp is planned from.
            try:
                log.info("[ARM1 Recovery] going home before re-scanning the bin")
                hardware._move('arm1', hardware.pick_and_place_arm.reset_joints, duration=5,
                               desc="go home before re-scanning the bin")
                arm_status.set('arm1', target=None)
                break
            except Exception as exc:  # noqa: BLE001 — back to the operator, not a crash
                if _stopping():
                    return 'shutdown'
                log.error("[ARM1 Recovery] could not go home: %s", exc)
                recovery_event_arm1.set()
                failed, err_msg = 'GO HOME', "Could not go home before re-scanning: %s" % exc
        log.info("[ARM1 Recovery] resuming")
        return choice


class HumanRecoveryArm2(smach.State):
    """Entered whenever an ARM2 state raises an exception.
    Only ARM2 is paused; ARM1 keeps running independently."""

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=ARM2_STATES + ['shutdown'],
                             input_keys=['recovery_failed_state', 'recovery_error_msg'])

    def execute(self, userdata):
        if _stopping():
            return 'shutdown'
        log.warning("[ARM2 Recovery] Pausing ARM2 — showing GUI (ARM1 continues)")
        recovery_event_arm2.set()
        with manager.lock:  # release any zone locks held by the failed state
            for z in ['A', 'B']:
                manager.busy[z] = False
        failed = getattr(userdata, 'recovery_failed_state', 'UNKNOWN')
        err_msg = getattr(userdata, 'recovery_error_msg', 'No details available.')
        choice = _show_recovery_gui(
            "ARM2 (Labeling)", failed, err_msg, ARM2_STATES,
            fa=hardware.label_arm,
            vacuum_on_service='/orio/lbl_cup/on',
            vacuum_off_service='/orio/lbl_cup/off')
        log.info("[ARM2 Recovery] choice=%s", choice)
        _end_intervention_pause()
        events.emit('recovery', 'arm2', failed_state=failed, error=err_msg, choice=choice)
        recovery_event_arm2.clear()
        log.info("[ARM2 Recovery] resuming")

        if choice == 'shutdown':
            shutdown_event.set()
        return choice


# ==============================================================================
# SECTION 3: ARM 1 (LOGISTICS SM) STATES
# ==============================================================================

class TaskSelector(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['do_load', 'do_unload', 'wait', 'shutdown',
                                             'intervene'],
                             output_keys=['recovery_failed_state', 'recovery_error_msg'])

    def execute(self, userdata):
        if shutdown_event.is_set():
            return 'shutdown'
        # Spin-wait while ARM2 is in human recovery (it may be accessing zones)
        while recovery_event_arm2.is_set():
            if shutdown_event.is_set():
                return 'shutdown'
            rospy.sleep(0.2)
        bin_empty = _bin_empty.is_set()
        arm_status.set('arm1', activity=BIN_EMPTY_ACTIVITY if bin_empty
                       else "Choosing the next task")
        pause_ctl.gate('arm1', raise_on_stop=False)
        if shutdown_event.is_set():
            return 'shutdown'
        if _take_intervention('arm1'):
            log.warning("[TaskSelector] operator intervention: opening the recovery window")
            events.emit('operator', 'intervene', state='DECIDE_HUB')
            userdata.recovery_failed_state = 'DECIDE_HUB'
            userdata.recovery_error_msg = "Operator asked to intervene (dashboard)"
            return 'intervene'

        log.debug("[TaskSelector] Evaluating zone states: %s  busy: %s",
                  manager.states, manager.busy)
        with manager.lock:
            has_empty = any(manager.states[z] == 'EMPTY' and not manager.busy[z]
                            for z in ['A', 'B'])
            has_ready = any(manager.states[z] == 'READY_FOR_PICKUP' and not manager.busy[z]
                            for z in ['A', 'B'])

            # Prioritise loading: fetch and drop into any empty zone unless both zones
            # are occupied (no EMPTY slot available), in which case unload a ready zone.
            # With the bin empty there is nothing to load: keep emptying the zones until
            # the operator resumes picking (INTERVENE → FETCH_INPUT).
            if has_empty and not bin_empty:
                log.info("[TaskSelector] → do_load (empty zone available)")
                return 'do_load'
            if has_ready:
                log.info("[TaskSelector] → do_unload (%s, ready zone exists)",
                         "bin empty" if bin_empty else "both zones occupied")
                return 'do_unload'
        log.debug("[TaskSelector] → wait (no actionable zones)")
        rospy.sleep(0.5)
        return 'wait'


class FetchInput(smach.State):
    """Scan the bin, plan a grasp with the new planner, confirm (optionally),
    pick. Declines (low score / no pose / out of bounds / too tilted) loop back
    through DECIDE_HUB; after --max-declines in a row the bin counts as empty
    (_bin_empty): arm 1 keeps emptying the zones, and picks again once the operator
    resumes it into FETCH_INPUT or DECIDE_HUB (dashboard INTERVENE)."""

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'no_grasp', 'failed'],
                             output_keys=['item_depth',
                                          'recovery_failed_state', 'recovery_error_msg'])

    def _decline(self, what):
        """A scan that gave no pickable grasp: count it, and after --max-declines in a
        row treat the bin as empty (see the class docstring). Returns 'no_grasp'."""
        global _declines
        _declines += 1
        arm_status.set('arm1', activity="%s (%d in a row), scanning again" % (what, _declines),
                       target=None)
        if _declines >= opts.max_declines:
            log.info("[FetchInput] %d declined grasps in a row (last: %s): input bin counts "
                     "as empty. Arm 1 keeps emptying the zones; refill the bin (or move the "
                     "item), then INTERVENE on arm 1 and resume into FETCH_INPUT.",
                     _declines, what)
            events.emit('operator', 'bin_empty', declines=_declines,
                        picks=_pick_n, placed=_items_placed_output)
            _bin_empty.set()
            arm_status.set('arm1', activity=BIN_EMPTY_ACTIVITY)
            return 'no_grasp'
        log.warning("[FetchInput] Grasp declined: %s (%d/%d) — re-scanning…",
                    what, _declines, opts.max_declines)
        rospy.sleep(0.5)
        return 'no_grasp'

    def _confirm_pick(self, task_pos):
        """Ask the operator what to do with the planned grasp.
        Returns 'execute' (Enter), 'regenerate' (r), or 'abort' ('q' / shutdown)."""
        while True:
            answer = console.ask(
                "[Loop] Press Enter to execute this pick, "
                "'r' to regenerate the grasp ('q' to abort)...")
            if answer is None or answer in ('q', 'quit', 'abort'):
                return 'abort'
            if answer == '':
                return 'execute'
            if answer in ('r', 'regen', 'regenerate'):
                return 'regenerate'
            print("[Loop] Unrecognised input %r — Enter to execute, 'r' to regenerate, "
                  "'q' to abort." % answer)

    def _bin_changed(self, epoch, above_bin):
        """Pause point before the cup reaches the bin. True if there was a pause after
        the grasp was planned (the operator may have swapped the bin's items): the grasp
        is dropped and planned again. An arm above the bin goes home first, out of the
        camera's view."""
        pause_ctl.gate('arm1')
        if pause_ctl.epoch == epoch:
            return False
        log.info("[FetchInput] Paused after the grasp was planned: the bin may have "
                 "changed, so planning a new grasp")
        events.emit('operator', 'regrasp_after_pause', pick=_pick_n, above_bin=above_bin)
        if above_bin:
            hardware._move('arm1', hardware.pick_and_place_arm.reset_joints, duration=3,
                           desc="clear the bin to re-scan", pausable=False)
        arm_status.set('arm1', target=None)
        return True

    def _run(self, userdata):
        global _pick_n, _declines
        if shutdown_event.is_set():
            return 'no_grasp'   # DECIDE_HUB turns this into a clean shutdown
        pause_ctl.gate('arm1')
        _pick_n += 1
        events.emit('pick', n=_pick_n)
        log.info("[FetchInput] Entering state")
        arm_status.set('arm1', activity="Planning a grasp", target='bin')

        epoch = pause_ctl.epoch   # a pause after this point invalidates the grasp
        grasp = hardware.request_grasp()
        if grasp is None:
            return self._decline("No grasp found")
        task_pos, target_ori = grasp

        if opts.confirm:
            choice = self._confirm_pick(task_pos)
            events.emit('operator', choice, pick=_pick_n, pos=list(task_pos))
            if choice == 'abort':
                log.info("[FetchInput] Aborted by operator")
                events.emit('shutdown', 'operator_abort', picks=_pick_n,
                            placed=_items_placed_output)
                shutdown_event.set()
                return 'no_grasp'
            if choice == 'regenerate':
                log.info("[FetchInput] Regenerating grasp — re-scanning…")
                return 'no_grasp'

        hardware.current_fetch_depth = float(task_pos[2])   # the item's surface, not the push
        userdata.item_depth = hardware.current_fetch_depth
        # Go PICK_PUSH_DISTANCE past the grasp point, along the cup axis (tool Z, into the item).
        approach = (VERTICAL_ORI if target_ori is None else np.asarray(target_ori))[:, 2]
        task_pos = list(np.asarray(task_pos, dtype=float) + PICK_PUSH_DISTANCE * approach)

        if opts.flat_pick:
            joints = hardware.compute_flat_pick_joints(task_pos, target_ori=target_ori)
            if joints is None:
                # No arm shape reaches this grasp safely (e.g. a steep tilt close to the
                # base folds the arm onto a joint limit): reject it, as ikpy's twisted
                # fallback set off the self-collision reflex.
                _grasp_verdict(False, "no safe arm pose for this grasp")
                events.emit('grasp', 'declined', reason='no_safe_ik', pos=list(task_pos),
                            ok=False)
                return self._decline("No safe arm pose for the grasp")
        else:
            joints = hardware.compute_pick_joints(task_pos, target_ori=target_ori)
        _declines = 0  # a grasp the arm can reach; reset the streak
        hardware.pick_up_pre_joints, hardware.pick_up_final_joints = joints
        log.info("[FetchInput] IK pre_joints:   %s", np.round(hardware.pick_up_pre_joints, 4))
        log.info("[FetchInput] IK final_joints: %s", np.round(hardware.pick_up_final_joints, 4))

        if self._bin_changed(epoch, above_bin=False):
            return 'no_grasp'
        log.info("[FetchInput] Moving to pre-pick")
        hardware._move('arm1', hardware.pick_and_place_arm.goto_joints,
                       hardware.pick_up_pre_joints, duration=3, desc="goto pre-pick",
                       pausable=False)

        if self._bin_changed(epoch, above_bin=True):
            return 'no_grasp'
        log.info("[FetchInput] Descending to pick")
        hardware._move('arm1', hardware.pick_and_place_arm.goto_joints,
                       hardware.pick_up_final_joints, duration=2, desc="descend to contact",
                       pausable=False)

        call_trigger_service('/orio/pnp_cup/on')
        rospy.sleep(1.0)

        # One move straight back out of the bin, INPUT_RETRACT_DISTANCE along the cup axis.
        retract_joints = hardware.compute_retract_joints(hardware.pick_up_final_joints,
                                                         INPUT_RETRACT_DISTANCE)
        if retract_joints is None:
            log.warning("[FetchInput] No IK for the %.2f m retract; retracting to pre-pick "
                        "height (%.2f m) instead", INPUT_RETRACT_DISTANCE, APPROACH_DISTANCE)
            retract_joints = hardware.pick_up_pre_joints
        else:
            log.info("[FetchInput] Retracting %.2f m out of the bin", INPUT_RETRACT_DISTANCE)
        hardware._move('arm1', hardware.pick_and_place_arm.goto_joints,
                       retract_joints, duration=2, desc="retract",
                       pausable=False)

        hardware.assert_vacuum('pnp', True)
        log.info("[FetchInput] Exiting: succeeded (item_depth=%.4f)", hardware.current_fetch_depth)
        return 'succeeded'

    def execute(self, userdata):
        return _safe_execute('FETCH_INPUT', self._run, userdata)


class DropToZone(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['dropped', 'wait', 'failed'],
                             input_keys=['item_depth'],
                             output_keys=['recovery_failed_state', 'recovery_error_msg'])

    def _run(self, userdata):
        log.info("[DropToZone] Entering state (item_depth=%.4f)", userdata.item_depth)
        pause_ctl.gate('arm1')
        target_zone = None
        with manager.lock:
            for z in ['A', 'B']:
                if manager.states[z] == 'EMPTY' and not manager.busy[z]:
                    target_zone = z
                    manager.busy[z] = 'arm1'
                    break

        if not target_zone:
            _raise_if_interrupted()   # this loop has no other way out on Ctrl-C
            log.debug("[DropToZone] No empty zone available → wait")
            arm_status.set('arm1', activity="Holding the item, waiting for a free zone")
            rospy.sleep(0.5)
            return 'wait'

        log.info("[DropToZone] Targeting zone %s", target_zone)
        arm_status.set('arm1', target=target_zone)
        manager.depths[target_zone] = userdata.item_depth
        manager.measured_z[target_zone] = None

        try:
            arm1 = hardware.pick_and_place_arm
            if target_zone == 'A':
                int_joints = hardware.pose_to_joints("L1_INTER", depth_offset=manager.depths[target_zone])
                place_joints = hardware.pose_to_joints("LABEL_ZONE_1", depth_offset=manager.depths[target_zone] + 0.01)
                log.info("[DropToZone] place_joints for LABEL_ZONE_1: %s", np.round(place_joints, 4))
                hardware._move('arm1', arm1.goto_joints, int_joints, duration=3, desc="transit L1_INTER")
                hardware._move('arm1', arm1.goto_joints, place_joints, duration=3, desc="place in zone A")
                hardware.assert_vacuum('pnp', True)
                call_trigger_service('/orio/pnp_cup/off')
                rospy.sleep(1.0)

                hardware._move('arm1', arm1.reset_joints, duration=2, desc="reset after drop A",
                               pausable=False)
            else:
                int1_joints = hardware.pose_to_joints("L1_INTER", depth_offset=manager.depths[target_zone])
                int2_joints = hardware.pose_to_joints("L2_INTER", depth_offset=manager.depths[target_zone])
                place_joints = hardware.pose_to_joints("LABEL_ZONE_2", depth_offset=manager.depths[target_zone] + 0.01)
                log.info("[DropToZone] place_joints for LABEL_ZONE_2: %s", np.round(place_joints, 4))
                hardware._move('arm1', arm1.goto_joints, int1_joints, duration=2, desc="transit L1_INTER")
                hardware._move('arm1', arm1.goto_joints, int2_joints, duration=2, desc="transit L2_INTER")
                hardware._move('arm1', arm1.goto_joints, place_joints, duration=3, desc="place in zone B")
                hardware.assert_vacuum('pnp', True)
                call_trigger_service('/orio/pnp_cup/off')
                rospy.sleep(1.0)
                hardware._move('arm1', arm1.reset_joints, duration=2, desc="reset after drop B",
                               pausable=False)
            arm_status.set('arm1', target=None)


            hardware.assert_vacuum('pnp', False)

            with manager.lock:
                manager.set_state(target_zone, 'NEEDS_LABEL')
                manager.busy[target_zone] = False
            log.info("[DropToZone] Exiting: dropped (zone %s)", target_zone)
            return 'dropped'
        except Exception:
            with manager.lock:
                manager.busy[target_zone] = False
            raise

    def execute(self, userdata):
        return _safe_execute('DROP_TO_ZONE', self._run, userdata)


class RetrieveFromZone(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['picked', 'no_items', 'failed'],
                             output_keys=['retrieved_depth', 'retrieved_zone',
                                          'recovery_failed_state', 'recovery_error_msg'])

    def _run(self, userdata):
        log.info("[RetrieveFromZone] Entering state")
        target_zone = None
        with manager.lock:
            for z in ['A', 'B']:
                if manager.states[z] == 'READY_FOR_PICKUP' and not manager.busy[z]:
                    target_zone = z
                    manager.busy[z] = 'arm1'
                    break

        if not target_zone:
            log.debug("[RetrieveFromZone] No ready zones → no_items")
            return 'no_items'
        arm_status.set('arm1', target=target_zone)

        # The pick height is the item's top as the label arm's ZED measured it in the zone;
        # the height arm 1 grasped it at in the bin is often wrong there (the item settles
        # lower, or was resting on other items). Only without a measurement (e.g. the zone
        # was set ready by hand) does it fall back to that.
        carried = manager.depths[target_zone]
        measured = manager.measured_z[target_zone]
        if measured is None:
            depth = carried
            log.warning("[RetrieveFromZone] Zone %s: no ZED height measured; using arm 1's "
                        "carried-over depth %.4f m", target_zone, carried)
        else:
            depth = measured
        userdata.retrieved_depth = carried   # the output drop height stays as before
        userdata.retrieved_zone = target_zone
        log.info("[RetrieveFromZone] Picking from zone %s (depth=%.4f, %s; carried over %.4f)",
                 target_zone, depth, 'ZED' if measured is not None else 'carried over', carried)

        try:
            arm1 = hardware.pick_and_place_arm
            if target_zone == 'A':
                int_joints = hardware.pose_to_joints("L1_INTER", depth_offset=depth)
                pre_pick_joints = hardware.pose_to_joints("LABEL_ZONE_1", depth_offset=depth + APPROACH_DISTANCE)
                pick_joints = hardware.pose_to_joints("LABEL_ZONE_1", depth_offset=depth - 0.01)
            else:
                int_joints = hardware.pose_to_joints("L2_INTER", depth_offset=depth)
                pre_pick_joints = hardware.pose_to_joints("LABEL_ZONE_2", depth_offset=depth + APPROACH_DISTANCE)
                pick_joints = hardware.pose_to_joints("LABEL_ZONE_2", depth_offset=depth - 0.01)
            log.info("[RetrieveFromZone] %s pre_pick=%s pick=%s", target_zone,
                     np.round(pre_pick_joints, 4), np.round(pick_joints, 4))
            hardware._move('arm1', arm1.goto_joints, int_joints, duration=3, desc="transit to zone")
            hardware._move('arm1', arm1.goto_joints, pre_pick_joints, duration=3, desc="pre-pick at zone")
            hardware._move('arm1', arm1.goto_joints, pick_joints, duration=2, desc="descend at zone")

            call_trigger_service('/orio/pnp_cup/on')
            rospy.sleep(1.0)

            log.info("[RetrieveFromZone] Retracting to pre-pick height")
            hardware._move('arm1', arm1.goto_joints, pre_pick_joints, duration=2, desc="retract at zone",
                           pausable=False)
            hardware._move('arm1', arm1.goto_joints, int_joints, duration=2, desc="transit from zone")

            hardware.assert_vacuum('pnp', True)

            with manager.lock:
                manager.set_state(target_zone, 'EMPTY')
                manager.busy[target_zone] = False
            log.info("[RetrieveFromZone] Exiting: picked (zone %s)", target_zone)
            return 'picked'
        except Exception:
            with manager.lock:
                manager.busy[target_zone] = False
            raise

    def execute(self, userdata):
        return _safe_execute('RETRIEVE', self._run, userdata)


class PlaceOutput(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'],
                             input_keys=['retrieved_depth', 'retrieved_zone'],
                             output_keys=['recovery_failed_state', 'recovery_error_msg'])

    def _run(self, userdata):
        global _items_placed_output
        log.info("[PlaceOutput] Entering state (retrieved_depth=%.4f)", userdata.retrieved_depth)
        # --zero-item-depth: drop every item at the same height, reaching into
        # the output box by the fixed margin alone (ported from dexnet_pnp.py).
        item_depth = 0.0 if hardware.zero_item_depth else userdata.retrieved_depth
        if (_items_placed_output % 2) == 0:
            drop_joints = hardware.pose_to_joints("OUTPUT1", depth_offset=item_depth + 0.15, flat=True)
        else:
            drop_joints = hardware.pose_to_joints("OUTPUT2", depth_offset=item_depth + 0.15, flat=True)
        int_joints = hardware.pose_to_joints("L2_INTER", depth_offset=item_depth)
        log.info("[PlaceOutput] drop_joints: %s", np.round(drop_joints, 4))

        arm1 = hardware.pick_and_place_arm
        arm_status.set('arm1', target='output')
        if userdata.retrieved_zone == 'A':
            hardware._move('arm1', arm1.goto_joints, int_joints, duration=3, desc="transit L2_INTER")
        hardware._move('arm1', arm1.goto_joints, drop_joints, duration=3, desc="goto output")
        hardware.assert_vacuum('pnp', True)
        call_trigger_service('/orio/pnp_cup/off')
        rospy.sleep(1.0)

        hardware._move('arm1', arm1.reset_joints, duration=3, desc="reset after output",
                       pausable=False)
        arm_status.set('arm1', target=None)

        hardware.assert_vacuum('pnp', False)

        _items_placed_output += 1
        log.info("[PlaceOutput] Exiting: succeeded (total items placed: %d)", _items_placed_output)
        return 'succeeded'

    def execute(self, userdata):
        return _safe_execute('PLACE_OUTPUT', self._run, userdata)


# ==============================================================================
# SECTION 4: ARM 2 (LABELING SM) STATES
# ==============================================================================

class GetLabel(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'],
                             output_keys=['recovery_failed_state', 'recovery_error_msg'])

    def _run(self, userdata):
        log.info("[GetLabel] Entering state")
        pause_ctl.gate('arm2')
        arm_status.set('arm2', target='dispenser')
        arm2 = hardware.label_arm
        hardware._move('arm2', arm2.goto_joints, hardware.label_arm_dispenser_pre,
                       duration=5, desc="goto dispenser pre")
        hardware._move('arm2', arm2.goto_joints, hardware.label_arm_dispenser_contact,
                       duration=5, desc="goto dispenser contact")
        call_trigger_service('/orio/lbl_cup/on')
        rospy.sleep(1.0)

        hardware._move('arm2', arm2.goto_joints, hardware.label_arm_dispenser_pre,
                       duration=2, desc="retract from dispenser", pausable=False)
        safe_joints = hardware.pose_to_joints("LABELLING_SAFE")
        hardware._move('arm2', arm2.goto_joints, safe_joints, duration=3, desc="goto labelling safe")
        arm_status.set('arm2', target=None)

        hardware.assert_vacuum('lbl', True)
        log.info("[GetLabel] Exiting: succeeded")
        return 'succeeded'

    def execute(self, userdata):
        return _safe_execute('GET_LABEL', self._run, userdata)


class ApplyLabel(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['labeled', 'wait', 'failed', 'shutdown'],
                             output_keys=['recovery_failed_state', 'recovery_error_msg'])

    def _run(self, userdata):
        global _labels_applied
        if shutdown_event.is_set():
            return 'shutdown'
        # Spin-wait while ARM1 is in human recovery (it may be accessing zones)
        while recovery_event_arm1.is_set():
            if shutdown_event.is_set():
                return 'shutdown'
            rospy.sleep(0.2)
        pause_ctl.gate('arm2')

        log.info("[ApplyLabel] Entering state (zones: %s  busy: %s)",
                 manager.states, manager.busy)
        target_zone = None
        with manager.lock:
            for z in ['A', 'B']:
                if manager.states[z] == 'NEEDS_LABEL' and not manager.busy[z]:
                    target_zone = z
                    manager.busy[z] = 'arm2'
                    break

        if not target_zone:
            log.debug("[ApplyLabel] No zone needs label → wait")
            arm_status.set('arm2', activity="Holding a label, waiting for an item")
            rospy.sleep(0.5)
            return 'wait'

        arm_status.set('arm2', target=target_zone, activity="Finding where the label goes (ZED)")
        try:
            zone_number = 1 if target_zone == 'A' else 2
            log.info("[ApplyLabel] Applying label to zone %s (zone_number=%d)",
                     target_zone, zone_number)
            hardware.label_measured_z.pop(zone_number, None)
            pre_joints, final_joints = hardware.compute_label_joints(
                zone_number, arm1_depth=manager.depths[target_zone])
            manager.measured_z[target_zone] = hardware.label_measured_z.get(zone_number)
            log.info("[ApplyLabel] IK pre_joints=%s  final_joints=%s",
                     np.round(pre_joints, 4), np.round(final_joints, 4))

            if opts.no_rfid:
                log.info("[ApplyLabel] RFID off (--no-rfid): label not registered")
            else:
                log.info("[ApplyLabel] Registering label — item='%s' expiry='%s'",
                         _item_name, _item_expiry)
                call_add_item_service(_item_name, _item_expiry)

            arm2 = hardware.label_arm
            hardware._move('arm2', arm2.goto_joints, pre_joints, duration=5, desc="goto label pre")
            hardware._move('arm2', arm2.goto_joints, final_joints, duration=5, desc="press label")
            hardware.assert_vacuum('lbl', True)
            call_trigger_service('/orio/lbl_cup/off')
            rospy.sleep(1.0)

            hardware._move('arm2', arm2.goto_joints, pre_joints, duration=2, desc="retract from label",
                           pausable=False)

            safe_joints = hardware.pose_to_joints("LABELLING_SAFE")
            hardware._move('arm2', arm2.goto_joints, safe_joints, duration=3, desc="goto labelling safe")
            arm_status.set('arm2', target=None)

            hardware.assert_vacuum('lbl', False)

            _labels_applied += 1
            with manager.lock:
                manager.set_state(target_zone, 'READY_FOR_PICKUP')
                manager.busy[target_zone] = False
            log.info("[ApplyLabel] Exiting: labeled (zone %s)", target_zone)
            return 'labeled'
        except Exception:
            with manager.lock:
                manager.busy[target_zone] = False
            raise

    def execute(self, userdata):
        return _safe_execute('APPLY_LABEL', self._run, userdata)


# ==============================================================================
# SECTION 5: MAIN EXECUTION FUNCTION
# ==============================================================================

def _parse_args():
    parser = argparse.ArgumentParser(
        description="ORIO dual-arm pick/place/label sequence (new grasp planners).")
    parser.add_argument('--no-vacuum', '--disable-pneumatics', dest='no_vacuum',
                        action='store_true',
                        help="Dry-run without pneumatics (skip vacuum commands + sensor checks).")
    parser.add_argument('--straight-down', action='store_true',
                        help="Ignore the planner's grasp orientation and always "
                             "approach vertically.")
    parser.add_argument('--max-tilt-deg', type=float, default=45.0,
                        help="Refuse grasps whose approach tilt exceeds this "
                             "(only when honouring grasp orientation).")
    parser.add_argument('--confirm', action='store_true',
                        help="Before each bin pick, wait for Enter (execute), 'r' "
                             "(regenerate the grasp), or 'q' (abort). Safe for bring-up.")
    parser.add_argument('--zero-item-depth', action='store_true',
                        help="Ignore the grasp z when computing the output drop height: "
                             "reach into the output box by the fixed margin alone.")
    parser.add_argument('--max-declines', type=int, default=5,
                        help="Consecutive planner declines (low q / no grasp) before the "
                             "input bin counts as empty (picking resumes from the dashboard: "
                             "INTERVENE on arm 1 → FETCH_INPUT).")
    parser.add_argument('--offset-x', type=float, default=0.0, metavar='M',
                        help="World-frame X correction (metres) added to every grasp "
                             "position after the planner returns it, before the arm moves.")
    parser.add_argument('--offset-y', type=float, default=0.0, metavar='M',
                        help="World-frame Y correction (metres), as --offset-x.")
    parser.add_argument('--no-label-vacuum-check', action='store_true',
                        help="Testing without labels: skip the label cup's vacuum checks, "
                             "so the run carries on as if a label had been picked up.")
    parser.add_argument('--no-pnp-vacuum-test', action='store_true',
                        help="Skip the pick cup's vacuum checks (e.g. while its sensor is "
                             "broken), so the run carries on as if the item had been picked up.")
    parser.add_argument('--pnp-vacuum-min', type=int, default=PNP_PICK_RAW_MIN,
                        help="Raw vacuum reading above which the pick cup counts as holding "
                             "an item, even on a seal too weak for the firmware's has-item "
                             "(> %d). Open air reads about 600. Default %d." % (VAC_THRESHOLD_HIGH, PNP_PICK_RAW_MIN))
    parser.add_argument('--speed', type=float, default=DEFAULT_SPEED, metavar='X',
                        help="Run every arm move X times faster than its base time (default "
                             "%.1f; 1 = the old speeds). Long moves stay slow enough that no "
                             "joint peaks above %.1f rad/s." % (DEFAULT_SPEED, MAX_PEAK_JOINT_SPEED))
    parser.add_argument('--settle-s', type=float, default=DEFAULT_SETTLE_S, metavar='S',
                        help="Seconds to wait after each arm move finishes before the next "
                             "command, so the arm has stopped (default %.1f; 0 = no wait)."
                             % DEFAULT_SETTLE_S)
    parser.add_argument('--no-flat-pick', dest='flat_pick', action='store_false',
                        help="Input pick: use ikpy's solution as before, not the arm shape "
                             "with joints 3 and 5 nearest 0 and the least wrist turn "
                             "(compute_flat_pick_joints), which skips grasps it cannot "
                             "reach safely.")
    parser.add_argument('--skip-item-setup', action='store_true',
                        help="Skip the item setup GUI and use a default item name and "
                             "expiry (%s, today + %d days)." % (DEFAULT_ITEM_NAME, DEFAULT_EXPIRY_DAYS))
    parser.add_argument('--no-rfid', action='store_true',
                        help="Do not register labels with the RFID inventory node "
                             "(skips the 5 s wait for /inventory/add_item on each label).")
    parser.add_argument('--no-dashboard', action='store_true',
                        help="Do not open the live dashboard window (status + Pause). "
                             "The status topic and pause services stay up.")
    parser.add_argument('--check-only', action='store_true',
                        help="Only check that BOTH robots (franka-interface 1 and 2) are "
                             "ready, then exit 0 if ready / non-zero if not. No motion. "
                             "Used as the launcher's pre-flight.")
    args, _ = parser.parse_known_args(rospy.myargv(argv=sys.argv)[1:])
    return args


def _resolve_disable_pneumatics(args):
    """True if any of: --no-vacuum flag, ORIO_DISABLE_PNEUMATICS env,
    ~disable_pneumatics rosparam."""
    env = os.environ.get('ORIO_DISABLE_PNEUMATICS', '').strip().lower() in ('1', 'true', 'yes', 'on')
    param = bool(rospy.get_param('~disable_pneumatics', False))
    return args.no_vacuum or env or param


STATUS_TOPIC = '/orio/dashboard/status'
STATUS_HZ = 5
DASHBOARD_SCRIPT = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'orio_dashboard.py')


def _active_state(sm):
    try:
        label = sm.get_active_states()[0] if sm is not None and sm.is_running() else None
    except Exception:  # noqa: BLE001 — a status read must never break the run
        label = None
    return None if label in (None, 'None') else label


def _status_snapshot():
    """Everything the dashboard shows, as one JSON-able dict."""
    with manager.lock:
        zones = {z: {'state': manager.states[z], 'busy': manager.busy[z] or None,
                     'depth': float(manager.depths[z])} for z in ('A', 'B')}
    arms = arm_status.snapshot()
    for arm, rec in (('arm1', recovery_event_arm1), ('arm2', recovery_event_arm2)):
        arms[arm].update(state=_active_state(_machines.get(arm)),
                         held=pause_ctl.held[arm], recovery=rec.is_set(),
                         intervene=intervene[arm].is_set())
    if hardware is not None:
        arms['arm1'].update(has_item=bool(hardware.pnp_has_item), raw=hardware.pnp_raw)
        arms['arm2'].update(has_item=bool(hardware.lbl_has_item), raw=hardware.lbl_raw)
    return {
        't': time.time(), 'started': _run_started,
        'paused': pause_ctl.paused, 'shutdown': shutdown_event.is_set(),
        'stopping': 'estop' if interrupted.is_set() else ('home' if stop_home.is_set() else None),
        'prompt': console.prompt if console is not None else None,
        'arms': arms, 'zones': zones,
        'counts': {'scans': _pick_n, 'placed': _items_placed_output,
                   'labels': _labels_applied, 'declines': _declines},
        'item': {'name': _item_name, 'expiry': _item_expiry},
        'vacuum': not disable_pneumatics,
    }


def _start_status_server():
    """Publish _status_snapshot() on STATUS_TOPIC and serve /orio/pause, /orio/resume,
    /orio/estop (as Ctrl-C) and /orio/shutdown (arms home, then exit)."""
    pub = rospy.Publisher(STATUS_TOPIC, String, queue_size=1, latch=True)

    def on_pause(_req):
        changed = pause_ctl.pause()
        return TriggerResponse(success=True, message="paused" if changed else "already paused")

    def on_resume(_req):
        changed = pause_ctl.resume()
        return TriggerResponse(success=True, message="resumed" if changed else "not paused")

    def on_estop(_req):
        request_estop("E-STOP")   # a second call exits the process
        return TriggerResponse(success=True, message="stopping both arms")

    def on_shutdown(_req):
        started = request_home_shutdown()
        return TriggerResponse(success=started,
                               message="arms going home, then exit" if started else "already stopping")

    rospy.Service('/orio/pause', Trigger, on_pause)
    rospy.Service('/orio/resume', Trigger, on_resume)
    rospy.Service('/orio/estop', Trigger, on_estop)
    rospy.Service('/orio/shutdown', Trigger, on_shutdown)

    def on_intervene(arm):
        def handler(_req):
            rec = recovery_event_arm1 if arm == 'arm1' else recovery_event_arm2
            if rec.is_set():
                return TriggerResponse(success=False, message="%s is already in recovery" % arm)
            if _stopping():
                return TriggerResponse(success=False, message="the run is stopping")
            _request_intervention(arm)
            log.warning("[Main] Operator intervention for %s: both arms stop at their next "
                        "pause point, then %s's recovery window opens", arm.upper(), arm.upper())
            return TriggerResponse(success=True, message="both arms stopping; %s recovery next" % arm)
        return handler

    rospy.Service('/orio/intervene/arm1', Trigger, on_intervene('arm1'))
    rospy.Service('/orio/intervene/arm2', Trigger, on_intervene('arm2'))

    def loop():
        rate = rospy.Rate(STATUS_HZ)
        while not rospy.is_shutdown():
            try:
                pub.publish(String(data=json.dumps(_status_snapshot(), default=str)))
            except Exception as exc:  # noqa: BLE001
                log.debug("[Dashboard] status publish failed: %s", exc)
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                return
    threading.Thread(target=loop, daemon=True, name='status').start()


def _start_dashboard():
    """Open the dashboard window as its own process (Tk here is used from several
    threads already). It gets no stdin (the Console owns it), its own session (so the
    terminal's Ctrl-C is ours to handle), and our pid, so it closes if we die."""
    try:
        return subprocess.Popen([sys.executable, '-u', DASHBOARD_SCRIPT,
                                 '--parent-pid', str(os.getpid())],
                                stdin=subprocess.DEVNULL, start_new_session=True)
    except OSError as exc:
        log.warning("[Dashboard] could not start %s: %s", DASHBOARD_SCRIPT, exc)
        return None


def _home_both_arms():
    """SHUTDOWN: both state machines have ended at a pause point (no arm touching an
    item), so send each arm home in turn, as at start-up. A held item stays held until
    main() turns the cups off at home. An arm that cannot get home is logged and left."""
    for arm_name, arm in (('arm1', hardware.pick_and_place_arm), ('arm2', hardware.label_arm)):
        log.warning("[Stop] %s going home", arm_name.upper())
        try:
            hardware._move(arm_name, arm.reset_joints, duration=5,
                           desc="go home (shutdown)", pausable=False)
        except Exception as exc:  # noqa: BLE001 — still try the other arm
            log.error("[Stop] %s could not go home: %s", arm_name.upper(), exc)


def _check_only():
    """Pre-flight: constructing FrankaArm runs wait_for_franka_interface(), which
    raises after ~10s if that robot is not ready. No reset, no motion."""
    ok = True
    for n in (1, 2):
        try:
            FrankaArm(with_gripper=False, old_gripper=False, robot_num=n, init_node=False)
            rospy.loginfo("[Check] Robot %d franka-interface is READY.", n)
        except Exception as exc:  # noqa: BLE001 — report cleanly for the pre-flight
            rospy.logerr("[Check] Robot %d NOT ready: %s", n, exc)
            ok = False
    return 0 if ok else 1


def main():
    global hardware, _item_name, _item_expiry, disable_pneumatics, no_label_vacuum_check, no_pnp_vacuum_test, pnp_pick_raw_min, motion_speed, settle_s, opts, console
    global _run_started
    opts = _parse_args()
    rospy.init_node('full_sequence_fsm')

    if opts.check_only:
        return _check_only()

    _setup_logger()
    _init_events()
    log.info("[Main] Node started (run events %s)", "on" if events.enabled else "off")

    disable_pneumatics = _resolve_disable_pneumatics(opts)
    if disable_pneumatics:
        log.warning("[Main] PNEUMATICS DISABLED — vacuum commands and sensor "
                    "checks are no-ops (dry-run mode). Move the cups by hand.")
    no_label_vacuum_check = opts.no_label_vacuum_check
    if no_label_vacuum_check:
        log.warning("[Main] NO LABEL VACUUM CHECK — the label cup's vacuum checks are skipped.")
    no_pnp_vacuum_test = opts.no_pnp_vacuum_test
    if no_pnp_vacuum_test:
        log.warning("[Main] NO PNP VACUUM TEST — the pick cup's vacuum checks are skipped.")
    pnp_pick_raw_min = opts.pnp_vacuum_min
    motion_speed = max(0.25, min(float(opts.speed), 3.0))
    settle_s = max(0.0, float(opts.settle_s))
    log.info("[Main] Motion speed %.2fx (moves no shorter than %.1f s, joints peak <= %.1f rad/s), "
             "%.2f s settle after each move", motion_speed, MIN_MOVE_S, MAX_PEAK_JOINT_SPEED, settle_s)

    random.seed(1)

    # Collect item metadata via GUI before any motion starts.
    if opts.skip_item_setup:
        _item_name = DEFAULT_ITEM_NAME
        _item_expiry = (datetime.date.today()
                        + datetime.timedelta(days=DEFAULT_EXPIRY_DAYS)).isoformat()
    else:
        _item_name, _item_expiry = show_item_setup_gui()
        gc.collect()   # free its Tk objects in this thread, before the arm threads start
    if _item_name is None:
        log.warning("[Main] Setup cancelled by operator — exiting cleanly.")
        events.emit('shutdown', 'operator_cancel')
        return
    log.info("[Main] Operator input — item='%s'  expiry='%s'", _item_name, _item_expiry)
    events.emit('operator', 'setup', item=_item_name, expiry=_item_expiry,
                no_vacuum=disable_pneumatics, straight_down=opts.straight_down,
                confirm=opts.confirm, max_tilt_deg=opts.max_tilt_deg,
                max_declines=opts.max_declines,
                no_label_vacuum_check=no_label_vacuum_check,
                no_pnp_vacuum_test=no_pnp_vacuum_test, skip_item_setup=opts.skip_item_setup,
                zero_item_depth=opts.zero_item_depth,
                offset_x=opts.offset_x, offset_y=opts.offset_y)

    console = Console()
    print("\n[ORIO] Type 'q' + Enter to stop cleanly.")

    _start_status_server()
    dashboard = None if opts.no_dashboard else _start_dashboard()
    if dashboard is not None:
        log.info("[Main] Dashboard opened (Pause/Resume there, or "
                 "rosservice call /orio/pause | /orio/resume)")

    # Initialize hardware once node is running
    hardware = RobotHardware(
        use_grasp_orientation=not opts.straight_down,
        max_tilt_deg=opts.max_tilt_deg,
        grasp_offset=(opts.offset_x, opts.offset_y),
        zero_item_depth=opts.zero_item_depth)
    _install_sigint_handler()   # after the FrankaArms: they install their own

    # Map each ARM1 recovery outcome back to the corresponding state (or finish on shutdown)
    arm1_recovery_transitions = {s: s for s in ARM1_STATES}
    arm1_recovery_transitions['shutdown'] = 'finished'

    sm_logistics = smach.StateMachine(outcomes=['finished'])
    with sm_logistics:
        smach.StateMachine.add('DECIDE_HUB', TaskSelector(),
                               transitions={'do_load': 'FETCH_INPUT', 'do_unload': 'RETRIEVE',
                                            'wait': 'DECIDE_HUB', 'shutdown': 'finished',
                                            'intervene': 'HUMAN_RECOVERY_ARM1'})
        smach.StateMachine.add('FETCH_INPUT', FetchInput(),
                               transitions={'succeeded': 'DROP_TO_ZONE',
                                            'no_grasp':  'DECIDE_HUB',
                                            'failed':    'HUMAN_RECOVERY_ARM1'})
        smach.StateMachine.add('DROP_TO_ZONE', DropToZone(),
                               transitions={'dropped': 'DECIDE_HUB', 'wait': 'DROP_TO_ZONE',
                                            'failed':  'HUMAN_RECOVERY_ARM1'})
        smach.StateMachine.add('RETRIEVE', RetrieveFromZone(),
                               transitions={'picked': 'PLACE_OUTPUT', 'no_items': 'DECIDE_HUB',
                                            'failed': 'HUMAN_RECOVERY_ARM1'})
        smach.StateMachine.add('PLACE_OUTPUT', PlaceOutput(),
                               transitions={'succeeded': 'DECIDE_HUB',
                                            'failed':    'HUMAN_RECOVERY_ARM1'})
        smach.StateMachine.add('HUMAN_RECOVERY_ARM1', HumanRecoveryArm1(),
                               transitions=arm1_recovery_transitions)

    # Map each ARM2 recovery outcome back to the corresponding state (or finish on shutdown)
    arm2_recovery_transitions = {s: s for s in ARM2_STATES}
    arm2_recovery_transitions['shutdown'] = 'finished'

    sm_labeling = smach.StateMachine(outcomes=['finished'])
    with sm_labeling:
        smach.StateMachine.add('GET_LABEL', GetLabel(),
                               transitions={'succeeded': 'APPLY',
                                            'failed':    'HUMAN_RECOVERY_ARM2'})
        smach.StateMachine.add('APPLY', ApplyLabel(),
                               transitions={'labeled': 'GET_LABEL', 'wait': 'APPLY',
                                            'failed':  'HUMAN_RECOVERY_ARM2',
                                            'shutdown': 'finished'})
        smach.StateMachine.add('HUMAN_RECOVERY_ARM2', HumanRecoveryArm2(),
                               transitions=arm2_recovery_transitions)

    top_sm = smach.Concurrence(
        outcomes=['done', 'error'],
        default_outcome='error',
        outcome_map={'done': {'ARM1': 'finished', 'ARM2': 'finished'}}
    )
    with top_sm:
        smach.Concurrence.add('ARM1', sm_logistics)
        smach.Concurrence.add('ARM2', sm_labeling)
    _machines.update(arm1=sm_logistics, arm2=sm_labeling)
    _run_started = time.time()

    try:
        sis = smach_ros.IntrospectionServer('orio_visualiser', top_sm, '/ORIO_ROOT')
        sis.start()
        log.info("[Main] Starting state machine execution")
        outcome = top_sm.execute()
        if stop_home.is_set() and not interrupted.is_set():
            _home_both_arms()
            log.warning("[Main] Shutdown from the dashboard: arms sent home")
            events.emit('shutdown', 'operator_home', outcome=outcome)
        elif interrupted.is_set():
            log.warning("[Main] Stopped by E-STOP, Ctrl-C or 'q' while paused: both arms stopped")
            events.emit('shutdown', 'interrupt')
        else:
            log.info("[Main] State machine finished with outcome: %s", outcome)
            events.emit('shutdown', 'finished', outcome=outcome)
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        log.warning("[Main] Interrupt received — requesting clean shutdown")
        events.emit('shutdown', 'interrupt')
        shutdown_event.set()
    finally:
        log.warning("[Main] Shutting down: turning off both vacuums")
        log.info("[Main] Total items placed at output: %d", _items_placed_output)
        events.emit('summary', 'run', items_placed=_items_placed_output, picks=_pick_n)
        call_trigger_service('/orio/pnp_cup/off')
        call_trigger_service('/orio/lbl_cup/off')
        if 'sis' in locals():
            sis.stop()
        if dashboard is not None and dashboard.poll() is None:
            dashboard.terminate()


if __name__ == '__main__':
    sys.exit(main() or 0)
