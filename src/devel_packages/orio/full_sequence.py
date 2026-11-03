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
                   park at home and wait for a refill (Enter) or 'q'
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
import json
import logging
import os
import pickle
import queue
import random
import sys
import threading

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
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from custom_msgs.srv import AddLabeledItem
import CollisionChecker
import RobotUtil as rt

try:
    from orio_core.events import TOPIC as EVENTS_TOPIC, EventSink
except ImportError:  # orio_core not on path: run events disabled
    EVENTS_TOPIC, EventSink = None, None


# ── Constants ──────────────────────────────────────────────────────────────────
APPROACH_DISTANCE = 0.1
PICK_UP_ZONE_GROUND_Z = 0.09
LABEL_ZONE_GROUND_Z = 0.055
LABEL_PLACE_CORRECTION_Z = 0.00  # To prevent excessive interference with the object while placing the label

POSES_FILE        = "joint_angles.json"
TARGET_POSES_FILE = "Target_Task_Poses.json"
URDF_FILE         = "panda_arm_hand.urdf"

# Same safe workspace box FetchInput has always enforced on the incoming grasp pose (world).
PICK_BOUNDS = {'x_min': -0.2, 'x_max': 0.55, 'y_min': -0.8, 'y_max': -0.1,
               'z_min': 0.0, 'z_max': 0.5}

# Straight-down tool orientation, the IK default.
VERTICAL_ORI = np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]])

# ── LABEL PLACEMENT ──────────────────────────────────────────────────────────────
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

# Per-arm recovery events — set while that arm is in the recovery GUI.
# The other arm only waits on the opposing event when it needs zone access.
recovery_event_arm1 = threading.Event()
recovery_event_arm2 = threading.Event()

# Ensures only one Tkinter window exists at a time — Tkinter is not thread-safe
_gui_lock = Lock()

no_label_check = False  # Set from --no-label-check: skip the label cup's sensor checks

# When True, vacuum service calls and sensor asserts are skipped (dry-run
# without pneumatics). Set in main() from --no-vacuum / ORIO_DISABLE_PNEUMATICS.
disable_pneumatics = False

# Valid re-entry states for each arm (used by recovery GUI radio buttons)
ARM1_STATES = ['DECIDE_HUB', 'FETCH_INPUT', 'DROP_TO_ZONE', 'RETRIEVE', 'PLACE_OUTPUT']
ARM2_STATES = ['GET_LABEL', 'APPLY']


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


def call_trigger_service(service_name):
    """Call a ROS Trigger service. With disable_pneumatics, vacuum cup services
    are skipped (no-op, returns True); vision services are unaffected."""
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
        return response.success
    except Exception as e:
        log.error("[Service] Call failed for %s: %s", service_name, e)
    return False


# ==============================================================================
# SECTION 1: SYSTEM SETUP & DATA MANAGEMENT
# ==============================================================================

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
    def _move(self, arm_name, motion_fn, *args, timeout=None, desc="motion", **kwargs):
        """Run a blocking FrankaArm motion (reset_joints/goto_joints) with a HARD
        timeout, so a dead control PC can't hang us forever.

        frankapy's wait_for_skill() busy-loops with no timeout: if franka-interface
        stops executing (e.g. control PC crash), a blocking motion never returns.
        We instead issue it non-blocking and poll is_skill_done() against a
        deadline, raising a clear error on expiry — which the per-arm recovery
        GUI then catches like any other state failure.
        """
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
        motion_fn(*args, **kwargs)   # issue asynchronously
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        rate = rospy.Rate(20)
        faults = 0
        while True:
            try:
                if arm.is_skill_done(ignore_errors=False):
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
            # state_machine.py); the states exit at their loop points. Only a ROS
            # shutdown (Ctrl-C) aborts mid-motion.
            if rospy.is_shutdown():
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
        global disable_pneumatics, no_label_check
        if disable_pneumatics:
            return
        if cup == 'lbl' and no_label_check:
            return
        attr = 'pnp_has_item' if cup == 'pnp' else 'lbl_has_item'
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        while rospy.Time.now() < deadline:
            if getattr(self, attr) == expected_state:
                return
            rospy.sleep(0.05)
        actual = getattr(self, attr)
        action = "pick up" if expected_state else "release"
        raise RuntimeError(
            f"{cup.upper()} vacuum failed to {action} item "
            f"(expected={expected_state}, actual={actual})"
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
            events.emit('grasp', 'declined',
                        reason='timeout' if self.latest_pose_msg is None else 'no_poses',
                        wait_s=float(wait_s), ok=False)
            return None

        p = self.latest_pose_msg.poses[0]
        raw_pos = [p.position.x, p.position.y, p.position.z]
        if any(np.isnan(v) for v in raw_pos):
            log.warning("[Grasp] NaN in grasp pose %s", raw_pos)
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

        for attempt in range(1, max_attempts + 1):
            if zone_number == 1:
                self.latest_label_pose_z1 = None
            else:
                self.latest_label_pose_z2 = None

            if not call_trigger_service('/compute_grasps_labelling'):
                raise RuntimeError("Service /compute_grasps_labelling failed")

            deadline = rospy.Time.now() + rospy.Duration(5.0)
            while rospy.Time.now() < deadline:
                msg = self.latest_label_pose_z1 if zone_number == 1 else self.latest_label_pose_z2
                if msg is not None and msg.poses:
                    break
                rospy.sleep(0.1)
            else:
                raise RuntimeError(f"No labelling pose received for zone {zone_number} within timeout")

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

    def pose_to_joints(self, location_tag, depth_offset=0.0):
        """Resolve a location tag to joint angles via IK.

        Args:
            location_tag (str): Key in Target_Task_Poses.json, e.g. 'LABEL_ZONE_1'.
            depth_offset (float): Offset added to the z-translation before IK (default 0.0).

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

        initial_guess = [0.0] * self.chain_length
        if 4 < self.chain_length:
            initial_guess[4] = -1.5

        angles = self.ik_chain.inverse_kinematics(
            target_position=target_pos,
            target_orientation=target_rot,
            orientation_mode="all",
            initial_position=initial_guess,
        )
        log.info("[IK] pose_to_joints '%s' depth_offset=%.4f → joints=%s",
                 location_tag, depth_offset, np.round(angles[1:8], 4))
        return angles[1:8]

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
        self.busy = {'A': False, 'B': False}        # Initial busy flags
        self.depths = {'A': 0.0, 'B': 0.0}          # Tracks object depth per zone

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
        root.lift()
        root.focus_force()
        root.mainloop()

    with _gui_lock:  # only one Tkinter window at a time — Tk is not thread-safe
        gui_thread = threading.Thread(target=_run, daemon=True)
        gui_thread.start()
        gui_thread.join()  # block the smach thread until the operator responds
    return result['choice']


def _safe_execute(state_name, fn, userdata):
    """Run fn(userdata); catch any exception, store diagnostics, and return 'failed'."""
    try:
        return fn(userdata)
    except Exception as exc:
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
        log.warning("[ARM1 Recovery] Pausing ARM1 — showing GUI (ARM2 continues)")
        recovery_event_arm1.set()
        with manager.lock:  # release any zone locks held by the failed state
            for z in ['A', 'B']:
                manager.busy[z] = False
        failed = getattr(userdata, 'recovery_failed_state', 'UNKNOWN')
        err_msg = getattr(userdata, 'recovery_error_msg', 'No details available.')
        choice = _show_recovery_gui(
            "ARM1 (Pick-and-Place)", failed, err_msg, ARM1_STATES,
            fa=hardware.pick_and_place_arm,
            vacuum_on_service='/orio/pnp_cup/on',
            vacuum_off_service='/orio/pnp_cup/off')
        log.info("[ARM1 Recovery] choice=%s", choice)
        events.emit('recovery', 'arm1', failed_state=failed, error=err_msg, choice=choice)
        recovery_event_arm1.clear()
        log.info("[ARM1 Recovery] resuming")
        if choice == 'shutdown':
            shutdown_event.set()
        return choice


class HumanRecoveryArm2(smach.State):
    """Entered whenever an ARM2 state raises an exception.
    Only ARM2 is paused; ARM1 keeps running independently."""

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=ARM2_STATES + ['shutdown'],
                             input_keys=['recovery_failed_state', 'recovery_error_msg'])

    def execute(self, userdata):
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
        smach.State.__init__(self, outcomes=['do_load', 'do_unload', 'wait', 'shutdown'])

    def execute(self, userdata):
        if shutdown_event.is_set():
            return 'shutdown'
        # Spin-wait while ARM2 is in human recovery (it may be accessing zones)
        while recovery_event_arm2.is_set():
            if shutdown_event.is_set():
                return 'shutdown'
            rospy.sleep(0.2)

        log.debug("[TaskSelector] Evaluating zone states: %s  busy: %s",
                  manager.states, manager.busy)
        with manager.lock:
            has_empty = any(manager.states[z] == 'EMPTY' and not manager.busy[z]
                            for z in ['A', 'B'])
            has_ready = any(manager.states[z] == 'READY_FOR_PICKUP' and not manager.busy[z]
                            for z in ['A', 'B'])

            # Prioritise loading: fetch and drop into any empty zone unless both zones
            # are occupied (no EMPTY slot available), in which case unload a ready zone.
            if has_empty:
                log.info("[TaskSelector] → do_load (empty zone available)")
                return 'do_load'
            if has_ready:
                log.info("[TaskSelector] → do_unload (both zones occupied, ready zone exists)")
                return 'do_unload'
        log.debug("[TaskSelector] → wait (no actionable zones)")
        rospy.sleep(0.5)
        return 'wait'


class FetchInput(smach.State):
    """Scan the bin, plan a grasp with the new planner, confirm (optionally),
    pick. Declines (low score / no pose / out of bounds / too tilted) loop back
    through DECIDE_HUB; after --max-declines in a row the bin counts as empty
    and the state blocks on a refill prompt (ARM2 keeps running)."""

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'no_grasp', 'failed'],
                             output_keys=['item_depth',
                                          'recovery_failed_state', 'recovery_error_msg'])

    def _await_refill(self):
        """Bin looks empty: block until the operator refills it (Enter) or quits.
        Returns True to rescan, False to end the run."""
        global _declines
        events.emit('operator', 'awaiting_refill', declines=_declines,
                    picks=_pick_n, placed=_items_placed_output)
        while True:
            answer = console.ask(
                "[Loop] Bin appears empty (%d scans with no valid grasp). "
                "Total placed so far: %d.\n"
                "[Loop] Add items to the bin, then press Enter to resume "
                "('q' to finish)..." % (_declines, _items_placed_output))
            if answer is None or answer in ('q', 'quit', 'abort', 'stop'):
                return False
            if answer == '':
                return True
            print("[Loop] Unrecognised input %r — Enter to resume, 'q' to finish." % answer)

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

    def _run(self, userdata):
        global _pick_n, _declines
        if shutdown_event.is_set():
            return 'no_grasp'   # DECIDE_HUB turns this into a clean shutdown
        _pick_n += 1
        events.emit('pick', n=_pick_n)
        log.info("[FetchInput] Entering state")

        grasp = hardware.request_grasp()
        if grasp is None:
            _declines += 1
            if _declines >= opts.max_declines:
                log.info("[FetchInput] Planner declined %d times in a row — bin empty "
                         "or no reliable grasp. Waiting for a refill.", _declines)
                if not self._await_refill():
                    log.info("[FetchInput] Operator ended the run.")
                    events.emit('shutdown', 'bin_empty', declines=_declines,
                                picks=_pick_n, placed=_items_placed_output)
                    shutdown_event.set()
                    return 'no_grasp'
                log.info("[FetchInput] Resuming after refill.")
                events.emit('operator', 'refilled', pick=_pick_n)
                _declines = 0      # fresh bin: start the streak over
                return 'no_grasp'
            log.warning("[FetchInput] Grasp declined (%d/%d) — re-scanning…",
                        _declines, opts.max_declines)
            rospy.sleep(0.5)
            return 'no_grasp'
        _declines = 0  # got a valid grasp; reset the streak
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

        hardware.current_fetch_depth = float(task_pos[2])
        userdata.item_depth = hardware.current_fetch_depth

        hardware.pick_up_pre_joints, hardware.pick_up_final_joints = \
            hardware.compute_pick_joints(task_pos, target_ori=target_ori)
        log.info("[FetchInput] IK pre_joints:   %s", np.round(hardware.pick_up_pre_joints, 4))
        log.info("[FetchInput] IK final_joints: %s", np.round(hardware.pick_up_final_joints, 4))

        log.info("[FetchInput] Moving to pre-pick")
        hardware._move('arm1', hardware.pick_and_place_arm.goto_joints,
                       hardware.pick_up_pre_joints, duration=3, desc="goto pre-pick")

        log.info("[FetchInput] Descending to pick")
        hardware._move('arm1', hardware.pick_and_place_arm.goto_joints,
                       hardware.pick_up_final_joints, duration=2, desc="descend to contact")

        call_trigger_service('/orio/pnp_cup/on')
        rospy.sleep(1.0)

        log.info("[FetchInput] Retracting to pre-pick height")
        hardware._move('arm1', hardware.pick_and_place_arm.goto_joints,
                       hardware.pick_up_pre_joints, duration=2, desc="retract")
        hardware.label_arm.wait_for_skill()

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
        target_zone = None
        with manager.lock:
            for z in ['A', 'B']:
                if manager.states[z] == 'EMPTY' and not manager.busy[z]:
                    target_zone = z
                    manager.busy[z] = True
                    break

        if not target_zone:
            log.debug("[DropToZone] No empty zone available → wait")
            rospy.sleep(0.5)
            return 'wait'

        log.info("[DropToZone] Targeting zone %s", target_zone)
        manager.depths[target_zone] = userdata.item_depth

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

                hardware._move('arm1', arm1.reset_joints, duration=2, desc="reset after drop A")
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
                hardware._move('arm1', arm1.reset_joints, duration=2, desc="reset after drop B")

            hardware.label_arm.wait_for_skill()

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
                    manager.busy[z] = True
                    break

        if not target_zone:
            log.debug("[RetrieveFromZone] No ready zones → no_items")
            return 'no_items'

        depth = manager.depths[target_zone]
        userdata.retrieved_depth = depth
        userdata.retrieved_zone = target_zone
        log.info("[RetrieveFromZone] Picking from zone %s (depth=%.4f)", target_zone, depth)

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
            hardware._move('arm1', arm1.goto_joints, pre_pick_joints, duration=2, desc="retract at zone")
            hardware._move('arm1', arm1.goto_joints, int_joints, duration=2, desc="transit from zone")
            hardware.label_arm.wait_for_skill()

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
            drop_joints = hardware.pose_to_joints("OUTPUT1", depth_offset=item_depth + 0.15)
        else:
            drop_joints = hardware.pose_to_joints("OUTPUT2", depth_offset=item_depth + 0.15)
        int_joints = hardware.pose_to_joints("L2_INTER", depth_offset=item_depth)
        log.info("[PlaceOutput] drop_joints: %s", np.round(drop_joints, 4))

        arm1 = hardware.pick_and_place_arm
        if userdata.retrieved_zone == 'A':
            hardware._move('arm1', arm1.goto_joints, int_joints, duration=3, desc="transit L2_INTER")
        hardware._move('arm1', arm1.goto_joints, drop_joints, duration=3, desc="goto output")
        hardware.assert_vacuum('pnp', True)
        call_trigger_service('/orio/pnp_cup/off')
        rospy.sleep(1.0)

        hardware._move('arm1', arm1.reset_joints, duration=3, desc="reset after output")
        hardware.label_arm.wait_for_skill()

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
        arm2 = hardware.label_arm
        hardware._move('arm2', arm2.goto_joints, hardware.label_arm_dispenser_pre,
                       duration=5, desc="goto dispenser pre")
        hardware._move('arm2', arm2.goto_joints, hardware.label_arm_dispenser_contact,
                       duration=5, desc="goto dispenser contact")
        call_trigger_service('/orio/lbl_cup/on')
        rospy.sleep(1.0)

        hardware._move('arm2', arm2.goto_joints, hardware.label_arm_dispenser_pre,
                       duration=2, desc="retract from dispenser")
        safe_joints = hardware.pose_to_joints("LABELLING_SAFE")
        hardware._move('arm2', arm2.goto_joints, safe_joints, duration=3, desc="goto labelling safe")

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
        if shutdown_event.is_set():
            return 'shutdown'
        # Spin-wait while ARM1 is in human recovery (it may be accessing zones)
        while recovery_event_arm1.is_set():
            if shutdown_event.is_set():
                return 'shutdown'
            rospy.sleep(0.2)

        log.info("[ApplyLabel] Entering state (zones: %s  busy: %s)",
                 manager.states, manager.busy)
        target_zone = None
        with manager.lock:
            for z in ['A', 'B']:
                if manager.states[z] == 'NEEDS_LABEL' and not manager.busy[z]:
                    target_zone = z
                    manager.busy[z] = True
                    break

        if not target_zone:
            log.debug("[ApplyLabel] No zone needs label → wait")
            rospy.sleep(0.5)
            return 'wait'

        try:
            zone_number = 1 if target_zone == 'A' else 2
            log.info("[ApplyLabel] Applying label to zone %s (zone_number=%d)",
                     target_zone, zone_number)
            pre_joints, final_joints = hardware.compute_label_joints(
                zone_number, arm1_depth=manager.depths[target_zone])
            log.info("[ApplyLabel] IK pre_joints=%s  final_joints=%s",
                     np.round(pre_joints, 4), np.round(final_joints, 4))

            log.info("[ApplyLabel] Registering label — item='%s' expiry='%s'", _item_name, _item_expiry)
            call_add_item_service(_item_name, _item_expiry)

            arm2 = hardware.label_arm
            hardware._move('arm2', arm2.goto_joints, pre_joints, duration=5, desc="goto label pre")
            hardware._move('arm2', arm2.goto_joints, final_joints, duration=5, desc="press label")
            hardware.assert_vacuum('lbl', True)
            call_trigger_service('/orio/lbl_cup/off')
            rospy.sleep(1.0)

            hardware._move('arm2', arm2.goto_joints, pre_joints, duration=2, desc="retract from label")

            safe_joints = hardware.pose_to_joints("LABELLING_SAFE")
            hardware._move('arm2', arm2.goto_joints, safe_joints, duration=3, desc="goto labelling safe")

            hardware.assert_vacuum('lbl', False)

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

def _show_item_setup_gui():
    """Blocking Tkinter dialog to collect item name and expiry date before the run starts.

    Returns (item_name: str, expiry: str).  Closing the window without submitting
    returns (None, None) so the operator cannot accidentally start without metadata.
    """
    result = {'name': None, 'expiry': None}

    def _run():
        root = tk.Tk()
        root.title("[ORIO] Label Registration Setup")
        root.configure(bg='#1e1e2e')
        root.resizable(False, False)

        # ── Header ──────────────────────────────────────────────────────────
        header = tk.Frame(root, bg='#89b4fa', pady=8)
        header.pack(fill='x')
        tk.Label(header, text="  ORIO — Label Registration Setup",
                 font=('Helvetica', 14, 'bold'), fg='#1e1e2e', bg='#89b4fa').pack()

        # ── Fields ───────────────────────────────────────────────────────────
        fields = tk.Frame(root, bg='#1e1e2e', padx=20, pady=16)
        fields.pack(fill='x')

        tk.Label(fields, text="Item name:", font=('Courier', 11),
                 fg='#cdd6f4', bg='#1e1e2e', anchor='w').grid(row=0, column=0, sticky='w', pady=(0, 8))
        name_var = tk.StringVar()
        name_entry = tk.Entry(fields, textvariable=name_var, font=('Courier', 12),
                              bg='#313244', fg='#cdd6f4', insertbackground='white',
                              relief='flat', width=30)
        name_entry.grid(row=0, column=1, padx=(12, 0), pady=(0, 8))

        tk.Label(fields, text="Expiry date\n(YYYY-MM-DD):", font=('Courier', 11),
                 fg='#cdd6f4', bg='#1e1e2e', anchor='w').grid(row=1, column=0, sticky='w')
        expiry_var = tk.StringVar()
        expiry_entry = tk.Entry(fields, textvariable=expiry_var, font=('Courier', 12),
                                bg='#313244', fg='#cdd6f4', insertbackground='white',
                                relief='flat', width=30)
        expiry_entry.grid(row=1, column=1, padx=(12, 0))

        # ── Error label (hidden until validation fails) ──────────────────────
        err_var = tk.StringVar()
        err_label = tk.Label(root, textvariable=err_var, font=('Courier', 10),
                             fg='#f38ba8', bg='#1e1e2e')
        err_label.pack(padx=20, anchor='w')

        # ── Buttons ───────────────────────────────────────────────────────────
        btn_frame = tk.Frame(root, bg='#1e1e2e', pady=12)
        btn_frame.pack(fill='x', padx=20)

        def on_submit():
            name = name_var.get().strip()
            expiry = expiry_var.get().strip()
            if not name:
                err_var.set("Item name cannot be empty.")
                return
            if not expiry:
                err_var.set("Expiry date cannot be empty.")
                return
            # Basic format check
            import re
            if not re.match(r'^\d{4}-\d{2}-\d{2}$', expiry):
                err_var.set("Expiry must be YYYY-MM-DD format.")
                return
            result['name'] = name
            result['expiry'] = expiry
            root.destroy()

        def on_cancel():
            if messagebox.askyesno("Cancel", "Abort the run?", parent=root):
                root.destroy()

        tk.Button(btn_frame, text="Start Run →", command=on_submit,
                  bg='#a6e3a1', fg='#1e1e2e', font=('Helvetica', 12, 'bold'),
                  relief='flat', padx=16, pady=6).pack(side='left', padx=(0, 12))
        tk.Button(btn_frame, text="Cancel", command=on_cancel,
                  bg='#f38ba8', fg='#1e1e2e', font=('Helvetica', 12, 'bold'),
                  relief='flat', padx=16, pady=6).pack(side='left')

        root.protocol("WM_DELETE_WINDOW", on_cancel)
        name_entry.focus_set()
        # Allow Enter key to submit
        root.bind('<Return>', lambda _: on_submit())
        root.lift()
        root.mainloop()

    _run()

    return result['name'], result['expiry']  # both None if cancelled


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
                             "sequence concludes the bin is empty and waits for a refill.")
    parser.add_argument('--offset-x', type=float, default=0.0, metavar='M',
                        help="World-frame X correction (metres) added to every grasp "
                             "position after the planner returns it, before the arm moves.")
    parser.add_argument('--offset-y', type=float, default=0.0, metavar='M',
                        help="World-frame Y correction (metres), as --offset-x.")
    parser.add_argument('--no-label-check', action='store_true',
                        help="Testing without labels: skip the label cup's vacuum checks, "
                             "so the run carries on as if a label had been picked up.")
    parser.add_argument('--skip-item-setup', action='store_true',
                        help="Skip the item setup GUI and use a default item name and "
                             "expiry (%s, today + %d days)." % (DEFAULT_ITEM_NAME, DEFAULT_EXPIRY_DAYS))
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
    global hardware, _item_name, _item_expiry, disable_pneumatics, no_label_check, opts, console
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
    no_label_check = opts.no_label_check
    if no_label_check:
        log.warning("[Main] NO LABEL CHECK — the label cup's vacuum checks are skipped.")

    random.seed(1)

    # Collect item metadata via GUI before any motion starts.
    if opts.skip_item_setup:
        _item_name = DEFAULT_ITEM_NAME
        _item_expiry = (datetime.date.today()
                        + datetime.timedelta(days=DEFAULT_EXPIRY_DAYS)).isoformat()
    else:
        _item_name, _item_expiry = _show_item_setup_gui()
    if _item_name is None:
        log.warning("[Main] Setup cancelled by operator — exiting cleanly.")
        events.emit('shutdown', 'operator_cancel')
        return
    log.info("[Main] Operator input — item='%s'  expiry='%s'", _item_name, _item_expiry)
    events.emit('operator', 'setup', item=_item_name, expiry=_item_expiry,
                no_vacuum=disable_pneumatics, straight_down=opts.straight_down,
                confirm=opts.confirm, max_tilt_deg=opts.max_tilt_deg,
                max_declines=opts.max_declines,
                no_label_check=no_label_check, skip_item_setup=opts.skip_item_setup,
                zero_item_depth=opts.zero_item_depth,
                offset_x=opts.offset_x, offset_y=opts.offset_y)

    console = Console()
    print("\n[ORIO] Type 'q' + Enter to stop cleanly.")

    # Initialize hardware once node is running
    hardware = RobotHardware(
        use_grasp_orientation=not opts.straight_down,
        max_tilt_deg=opts.max_tilt_deg,
        grasp_offset=(opts.offset_x, opts.offset_y),
        zero_item_depth=opts.zero_item_depth)

    # Map each ARM1 recovery outcome back to the corresponding state (or finish on shutdown)
    arm1_recovery_transitions = {s: s for s in ARM1_STATES}
    arm1_recovery_transitions['shutdown'] = 'finished'

    sm_logistics = smach.StateMachine(outcomes=['finished'])
    with sm_logistics:
        smach.StateMachine.add('DECIDE_HUB', TaskSelector(),
                               transitions={'do_load': 'FETCH_INPUT', 'do_unload': 'RETRIEVE',
                                            'wait': 'DECIDE_HUB', 'shutdown': 'finished'})
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

    try:
        sis = smach_ros.IntrospectionServer('orio_visualiser', top_sm, '/ORIO_ROOT')
        sis.start()
        log.info("[Main] Starting state machine execution")
        outcome = top_sm.execute()
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


if __name__ == '__main__':
    sys.exit(main() or 0)
