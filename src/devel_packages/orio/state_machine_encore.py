#!/usr/bin/env python3

import rospy
import smach
import smach_ros
import json
import ikpy.chain
import numpy as np
from threading import Lock
from scipy.spatial.transform import Rotation as R
from frankapy import FrankaArm
from autolab_core import RigidTransform
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from custom_msgs.srv import AddLabeledItem
import sys, os
import logging
import datetime
import threading
import tkinter as tk
from tkinter import ttk, messagebox
import random
import pickle
import yaml
import CollisionChecker


# ── Constants ──────────────────────────────────────────────────────────────────
APPROACH_DISTANCE = 0.1
PICK_UP_ZONE_GROUND_Z = 0.09
LABEL_ZONE_GROUND_Z = 0.055
LABEL_PLACE_CORRECTION_Z = 0.00  # To prevent excessive interference with the object while placing the label

POSES_FILE       = "joint_angles.json"
TARGET_POSES_FILE = "Target_Task_Poses.json"
URDF_FILE        = "panda_arm_hand.urdf"

# ── ZED CAMERA PARAMETERS ────────────────────────────────────────────────────────

LBL_WIDTH, LBL_HEIGHT = 1920, 1080
LBL_FX, LBL_FY        = 1509.65, 1509.65
LBL_CX, LBL_CY        = 964.33, 559.28  # Adjusted for cropping
DEPTH_UNIT_LBL         = 1000.0
CAMERA_HEIGHT          = 1.0  # metres above the robot base frame
LBL_TF_YAML     = '../manipulation/config/zed_to_label_tf.yaml'

# ──────────────────────────────────────────────────────────────────────────────────

_LOG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'logs')

# Module-level logger — configured by _setup_file_logger() once the node starts.
log = logging.getLogger("orio_fsm")

def _setup_file_logger():
    """Configure the orio_fsm logger with a timestamped file handler."""
    os.makedirs(_LOG_DIR, exist_ok=True)
    log_path = os.path.join(_LOG_DIR, f"orio_run_{datetime.datetime.now():%Y%m%d_%H%M%S}.log")

    fmt = logging.Formatter('%(asctime)s %(levelname)-8s %(message)s')
    sh = logging.StreamHandler()
    sh.setFormatter(fmt)
    fh = logging.FileHandler(log_path)
    fh.setFormatter(fmt)

    log.setLevel(logging.DEBUG)
    log.handlers.clear()
    log.addHandler(sh)
    log.addHandler(fh)
    log.propagate = False

    log.info(f"[Logger] File log started: {log_path}")
    return log_path

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

test_vacuum = False  # Set True to enable vacuum sensor checks in assert_vacuum()

# Valid re-entry states for each arm (used by recovery GUI radio buttons)
ARM1_STATES = ['DECIDE_HUB', 'FETCH_INPUT', 'DROP_TO_ZONE', 'RETRIEVE', 'PLACE_OUTPUT']
ARM2_STATES = ['GET_LABEL', 'APPLY']

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


def call_trigger_service(service_name):
    """Helper function to call ROS Trigger services (vacuum OR vision)."""
    try:
        rospy.wait_for_service(service_name, timeout=5.0)
        proxy = rospy.ServiceProxy(service_name, Trigger)
        response = proxy()
        log.info("[Service] %s → success=%s msg='%s'",
                      service_name, response.success,
                      getattr(response, 'message', ''))
        return response.success
    except Exception as e:
        log.error("[Service] Call failed for %s: %s", service_name, e)
    return False

def load_tf(yaml_path):
        with open(yaml_path, 'r') as f:
            data = yaml.safe_load(f)
        t = data['pose']['translation']
        r = data['pose']['rotation']
        rot_mat = R.from_quat([r['x'], r['y'], r['z'], r['w']]).as_matrix()
        tf = np.eye(4)
        tf[:3, :3] = rot_mat
        tf[:3, 3]  = [t['x'], t['y'], t['z']]
        return tf

# ==============================================================================
# SECTION 1: SYSTEM SETUP & DATA MANAGEMENT
# ==============================================================================

class RobotHardware:
    """Wrapper holding Franka interface, IK, Vision, and poses."""
    def __init__(self):
        log.info("[Hardware] Initializing Robot Hardware...")
        self.pick_and_place_arm = FrankaArm(with_gripper=False, old_gripper=False, robot_num=1, init_node=False)
        self.label_arm  = FrankaArm(with_gripper=False, old_gripper=False, robot_num=2, init_node=False)
        log.info("[Hardware] Resetting both arms to home joints")
        self.pick_and_place_arm.reset_joints()
        self.label_arm.reset_joints()

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
        rospy.Subscriber("/grasp_poses",             PoseArray, self._pose_callback,          queue_size=1)
        rospy.Subscriber("/grasp_poses_labelling_z1", PoseArray, self._label_pose_callback_z1, queue_size=1)
        rospy.Subscriber("/grasp_poses_labelling_z2", PoseArray, self._label_pose_callback_z2, queue_size=1)

        # Stored dynamically per cycle
        self.current_fetch_depth = 0.0
        self.pick_up_pre_joints = None
        self.pick_up_final_joints = None

        # Zed to label arm Transform
        self.tf_lbl = load_tf(LBL_TF_YAML)

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

    # ── Vacuum verification ──────────────────────────────────────────────────

    def assert_vacuum(self, cup, expected_state, timeout=1.0):
        """Poll the vacuum sensor for `cup` ('pnp'|'lbl') until it matches
        `expected_state`; raise RuntimeError if it doesn't within `timeout` s."""
        global test_vacuum
        if test_vacuum:
            attr     = 'pnp_has_item' if cup == 'pnp' else 'lbl_has_item'
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
        else:
            pass

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

    def check_collision(self, q7, arm_number, label_zone=None):
        """Return True if the 7-DOF config q7 is in collision with scene obstacles.

        Obstacles are loaded from the obstacle file for the given arm / label_zone
        and cached after the first call so subsequent checks are fast.

        Args:
            q7         : 7-element joint angle array.
            arm_number : 1 = pick-and-place arm, 2 = label arm.
            label_zone : None | 1 | 2  — selects the obstacle file for this zone
                         (default None).
        """
        cache_key = (arm_number, label_zone)
        if not hasattr(self, '_obs_cache'):
            self._obs_cache = {}
        if cache_key not in self._obs_cache:
            obs_file = self._get_obs_file(arm_number, label_zone)
            with open(obs_file, 'rb') as f:
                obs_points = pickle.load(f)
                obs_axes   = pickle.load(f)
            self._obs_cache[cache_key] = (obs_points, obs_axes)
            log.info("Arm%d: cached %d obstacles from '%s'",
                     arm_number, len(obs_points), obs_file)
        obs_points, obs_axes = self._obs_cache[cache_key]
        return self.la_collision_checker.DetectCollision(q7, obs_points, obs_axes)

    def compute_pick_joints(self, task_pos,
                            target_ori=np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]]),
                            label_zone=None, max_attempts=10,
                            check_collisions=False, arm_number=None, obstacle_zone_check=False):
        log.info("[IK] compute_pick_joints called — task_pos=%s label_zone=%s max_attempts=%d check_collisions=%s arm_number=%s",
                 np.round(task_pos, 4), label_zone, max_attempts, check_collisions, arm_number)
        pre_pos   = [task_pos[0], task_pos[1], task_pos[2] + APPROACH_DISTANCE]
        final_pos = [task_pos[0], task_pos[1], task_pos[2]]

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

            return pre_angles[1:8], final_angles[1:8]

        raise RuntimeError(
            f"[IK] Failed to find a collision-free configuration for target={np.round(task_pos, 4)} "
            f"after {max_attempts} attempts"
        )

    def compute_label_joints(self, zone_number, item_depth, max_attempts=10,
                             pose_bounds=None):
        """Call /compute_grasps_labelling, wait for the pose on the matching topic,
        and return (pre_joints, final_joints) via IK — mirrors compute_pick_joints().

        Args:
            zone_number (int): 1 or 2, selects /grasp_poses_labelling_z1 or z2.
            item_depth (float): Z position of the item, used as the target z for IK.
            max_attempts (int): Maximum number of retries if NaN values are received (default 10).
            pose_bounds (dict, optional): Axis-aligned bounds for the received XY pose.
                Keys: 'x_min', 'x_max', 'y_min', 'y_max'. Any omitted key is unchecked.
                Defaults to None (no bounds check).
                Example: {'x_min': 0.2, 'x_max': 0.8, 'y_min': -0.5, 'y_max': 0.5}

        Returns:
            tuple: (pre_joints, final_joints) each a 7-element np.ndarray.

        Raises:
            RuntimeError: if the service fails, no pose is received in time, NaNs persist,
                          or the received pose is outside pose_bounds.
        """
        log.info("[compute_label_joints] Requesting label pose for zone %d, item_depth=%.4f",
                      zone_number, item_depth)

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

            Z = CAMERA_HEIGHT - item_depth
            X = (p.position.x - LBL_CX) * Z / LBL_FX
            Y = (p.position.y - LBL_CY) * Z / LBL_FY
            pos = self.tf_lbl @ np.array([X, Y, Z, 1.0])

            task_pos = [pos[0], pos[1], item_depth]
            task_ori = [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]
            log.info("item_depth=%.4f  X=%.4f Y=%.4f Z=%.4f  pos(world)=%s  task_pos=%s  task_ori=%s",
                     item_depth, X, Y, Z, np.round(pos[:3], 4), np.round(task_pos, 4), np.round(task_ori, 4))

            if any(np.isnan(v) for v in task_pos + task_ori):
                log.warning("[compute_label_joints] Attempt %d/%d: NaN values in pose for zone %d "
                            "(pos=%s ori=%s), retrying...",
                            attempt, max_attempts, zone_number, task_pos, task_ori)
                continue

            if pose_bounds is not None:
                x, y = task_pos[0], task_pos[1]
                if (x < pose_bounds.get('x_min', -np.inf) or x > pose_bounds.get('x_max', np.inf) or
                        y < pose_bounds.get('y_min', -np.inf) or y > pose_bounds.get('y_max', np.inf)):
                    err_msg = (f"[compute_label_joints] Zone {zone_number} pose (x={x:.4f}, y={y:.4f}) "
                               f"is outside bounds {pose_bounds}")
                    log.error(err_msg)
                    raise RuntimeError(err_msg)

            task_ori_Mat = R.from_quat(task_ori).as_matrix()
            rpy = R.from_quat(task_ori).as_euler('xyz', degrees=True)
            log.info("[compute_label_joints] Zone %d received pose: x=%.4f y=%.4f z=%.4f  RPY r=%.2f p=%.2f y=%.2f deg",
                          zone_number,
                          task_pos[0], task_pos[1], task_pos[2],
                          rpy[0], rpy[1], rpy[2])

            return self.compute_pick_joints(task_pos, target_ori=task_ori_Mat, label_zone=zone_number,
                                            check_collisions=True, arm_number=2)

        raise RuntimeError(f"NaN values in labelling pose for zone {zone_number} after {max_attempts} attempts")


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
        if 4 < self.chain_length: initial_guess[4] = -1.5
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

# Globals
manager = ZoneManager()
hardware = None  # Will be initialized in main()

# Item metadata provided by the operator at startup
_item_name = ""
_item_expiry = ""

# Cycle counter for output tracking
_items_placed_output = 0

# ==============================================================================
# SECTION 2: HUMAN RECOVERY GUI & HELPERS
# ==============================================================================

def _show_recovery_gui(arm_name, failed_state, error_msg, valid_states,
                       fa, vacuum_on_service, vacuum_off_service):
    """Show a blocking Tkinter recovery dialog for the failed arm only.

    Args:
        arm_name          : Display name shown in the GUI title / header.
        failed_state      : Name of the state that raised the exception.
        error_msg         : Exception message string.
        valid_states      : List of state names the operator can jump to.
        fa                : FrankaArm instance for this arm.
        vacuum_on_service : ROS service name to turn the vacuum ON  (str).
        vacuum_off_service: ROS service name to turn the vacuum OFF (str).

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

        all_ctrl_buttons = []

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

        all_ctrl_buttons.extend([b_stop, b_reset, b_von, b_voff])

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
                    manager.busy[zone]   = False
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
        userdata.recovery_failed_state = state_name
        userdata.recovery_error_msg    = str(exc)
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
        failed  = getattr(userdata, 'recovery_failed_state', 'UNKNOWN')
        err_msg = getattr(userdata, 'recovery_error_msg',    'No details available.')
        choice = _show_recovery_gui(
            "ARM1 (Pick-and-Place)", failed, err_msg, ARM1_STATES,
            fa=hardware.pick_and_place_arm,
            vacuum_on_service='/orio/pnp_cup/on',
            vacuum_off_service='/orio/pnp_cup/off')
        log.info("[ARM1 Recovery] choice=%s", choice)
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
        failed  = getattr(userdata, 'recovery_failed_state', 'UNKNOWN')
        err_msg = getattr(userdata, 'recovery_error_msg',    'No details available.')
        choice = _show_recovery_gui(
            "ARM2 (Labeling)", failed, err_msg, ARM2_STATES,
            fa=hardware.label_arm,
            vacuum_on_service='/orio/lbl_cup/on',
            vacuum_off_service='/orio/lbl_cup/off')
        log.info("[ARM2 Recovery] choice=%s", choice)
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
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'],
                             output_keys=['item_depth',
                                          'recovery_failed_state', 'recovery_error_msg'])

    def _run(self, userdata):
        log.info("[FetchInput] Entering state")
        hardware.latest_pose_msg = None

        if not call_trigger_service('/compute_grasps'):
            raise RuntimeError("/compute_grasps service failed")

        deadline = rospy.Time.now() + rospy.Duration(5.0)
        while hardware.latest_pose_msg is None and rospy.Time.now() < deadline:
            rospy.sleep(0.1)

        if hardware.latest_pose_msg is None or not hardware.latest_pose_msg.poses:
            raise RuntimeError("No grasp pose received within timeout")

        p = hardware.latest_pose_msg.poses[0]
        task_pos = [p.position.x, p.position.y, p.position.z]

        if any(np.isnan(v) for v in task_pos):
            raise RuntimeError(f"NaN in grasp pose (pos={task_pos})")

        PICK_BOUNDS = {'x_min': -0.2, 'x_max': 0.55, 'y_min': -0.8, 'y_max': -0.1, 'z_min': 0.0, 'z_max': 0.5}
        x, y, z = task_pos
        if not (PICK_BOUNDS['x_min'] <= x <= PICK_BOUNDS['x_max'] and
                PICK_BOUNDS['y_min'] <= y <= PICK_BOUNDS['y_max'] and
                PICK_BOUNDS['z_min'] <= z <= PICK_BOUNDS['z_max']):
            raise RuntimeError(f"Grasp pose (x={x:.4f} y={y:.4f} z={z:.4f}) outside safe bounds {PICK_BOUNDS}")

        hardware.current_fetch_depth = task_pos[2]
        userdata.item_depth = hardware.current_fetch_depth
        log.info("[FetchInput] Grasp pose received: x=%.4f y=%.4f z=%.4f",
                      task_pos[0], task_pos[1], task_pos[2])

        hardware.pick_up_pre_joints, hardware.pick_up_final_joints = hardware.compute_pick_joints(task_pos)
        log.info("[FetchInput] IK pre_joints:   %s", np.round(hardware.pick_up_pre_joints, 4))
        log.info("[FetchInput] IK final_joints: %s", np.round(hardware.pick_up_final_joints, 4))

        log.info("[FetchInput] Moving to pre-pick")
        hardware.pick_and_place_arm.goto_joints(hardware.pick_up_pre_joints, duration=3)

        log.info("[FetchInput] Descending to pick")
        hardware.pick_and_place_arm.goto_joints(hardware.pick_up_final_joints, duration=2)

        call_trigger_service('/orio/pnp_cup/on')
        rospy.sleep(1.0)
        

        log.info("[FetchInput] Retracting to pre-pick height")
        
        hardware.pick_and_place_arm.goto_joints(hardware.pick_up_pre_joints, duration=2)
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
            if target_zone == 'A':
                int_joints = hardware.pose_to_joints("L1_INTER", depth_offset=manager.depths[target_zone])
                place_joints = hardware.pose_to_joints("LABEL_ZONE_1", depth_offset=manager.depths[target_zone] + 0.01)
                log.info("[DropToZone] place_joints for LABEL_ZONE_1: %s", np.round(place_joints, 4))
                hardware.pick_and_place_arm.goto_joints(int_joints, duration=3)
                hardware.pick_and_place_arm.goto_joints(place_joints, duration=3)
                hardware.assert_vacuum('pnp', True)
                call_trigger_service('/orio/pnp_cup/off')
                rospy.sleep(1.0)
                
                hardware.pick_and_place_arm.reset_joints(duration=2)
            else:
                int1_joints = hardware.pose_to_joints("L1_INTER", depth_offset=manager.depths[target_zone])
                int2_joints = hardware.pose_to_joints("L2_INTER", depth_offset=manager.depths[target_zone])
                place_joints = hardware.pose_to_joints("LABEL_ZONE_2", depth_offset=manager.depths[target_zone] + 0.01)
                log.info("[DropToZone] place_joints for LABEL_ZONE_2: %s", np.round(place_joints, 4))
                hardware.pick_and_place_arm.goto_joints(int1_joints, duration=2)
                hardware.pick_and_place_arm.goto_joints(int2_joints, duration=2)
                hardware.pick_and_place_arm.goto_joints(place_joints, duration=3)
                hardware.assert_vacuum('pnp', True)
                call_trigger_service('/orio/pnp_cup/off')
                rospy.sleep(1.0)
                hardware.pick_and_place_arm.reset_joints(duration=2)

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
            if target_zone == 'A':
                int_joints = hardware.pose_to_joints("L1_INTER", depth_offset=depth)
                pre_pick_joints = hardware.pose_to_joints("LABEL_ZONE_1", depth_offset=depth + APPROACH_DISTANCE)
                pick_joints = hardware.pose_to_joints("LABEL_ZONE_1", depth_offset=depth - 0.01)
                log.info("[RetrieveFromZone] A pre_pick=%s pick=%s",
                              np.round(pre_pick_joints, 4), np.round(pick_joints, 4))
                hardware.pick_and_place_arm.goto_joints(int_joints, duration=3)
                hardware.pick_and_place_arm.goto_joints(pre_pick_joints, duration=3)
                hardware.pick_and_place_arm.goto_joints(pick_joints, duration=2)
            else:
                int_joints = hardware.pose_to_joints("L2_INTER", depth_offset=depth)
                pre_pick_joints = hardware.pose_to_joints("LABEL_ZONE_2", depth_offset=depth + APPROACH_DISTANCE)
                pick_joints = hardware.pose_to_joints("LABEL_ZONE_2", depth_offset=depth - 0.01)
                log.info("[RetrieveFromZone] B pre_pick=%s pick=%s",
                              np.round(pre_pick_joints, 4), np.round(pick_joints, 4))
                hardware.pick_and_place_arm.goto_joints(int_joints, duration=3)
                hardware.pick_and_place_arm.goto_joints(pre_pick_joints, duration=3)
                hardware.pick_and_place_arm.goto_joints(pick_joints, duration=2)

            call_trigger_service('/orio/pnp_cup/on')
            rospy.sleep(1.0)
            

            log.info("[RetrieveFromZone] Retracting to pre-pick height")
            hardware.pick_and_place_arm.goto_joints(pre_pick_joints, duration=2)
            hardware.pick_and_place_arm.goto_joints(int_joints, duration=2)
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
        item_depth = userdata.retrieved_depth
        if (_items_placed_output % 2) == 0:
            drop_joints = hardware.pose_to_joints("OUTPUT1", depth_offset=item_depth + 0.15)
        else:
            drop_joints = hardware.pose_to_joints("OUTPUT2", depth_offset=item_depth + 0.15)
        int_joints = hardware.pose_to_joints("L2_INTER", depth_offset=item_depth)
        log.info("[PlaceOutput] drop_joints: %s", np.round(drop_joints, 4))

        
        if userdata.retrieved_zone == 'A':
            hardware.pick_and_place_arm.goto_joints(int_joints, duration=3)
        hardware.pick_and_place_arm.goto_joints(drop_joints, duration=3)
        hardware.assert_vacuum('pnp', True)
        call_trigger_service('/orio/pnp_cup/off')
        rospy.sleep(1.0)
        

        hardware.pick_and_place_arm.reset_joints(duration=3)
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
        hardware.label_arm.goto_joints(hardware.label_arm_dispenser_pre, duration=5)
        hardware.label_arm.goto_joints(hardware.label_arm_dispenser_contact, duration=5)
        call_trigger_service('/orio/lbl_cup/on')
        rospy.sleep(1.0)
        
        hardware.label_arm.goto_joints(hardware.label_arm_dispenser_pre, duration=2)
        safe_joints = hardware.pose_to_joints("LABELLING_SAFE")
        hardware.label_arm.wait_for_skill()
        hardware.label_arm.goto_joints(safe_joints, duration=3)
        hardware.label_arm.wait_for_skill()
        
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
            log.info("[ApplyLabel] Applying label to zone %s (zone_number=%d, depth=%.4f)",
                          target_zone, zone_number, manager.depths[target_zone])
            pre_joints, final_joints = hardware.compute_label_joints(
                zone_number,
                manager.depths[target_zone] + LABEL_PLACE_CORRECTION_Z,
                pose_bounds={'x_min': 0.2, 'x_max': 0.7, 'y_min': -0.6, 'y_max': 0.6})
            log.info("[ApplyLabel] IK pre_joints=%s  final_joints=%s",
                          np.round(pre_joints, 4), np.round(final_joints, 4))

            log.info("[ApplyLabel] Registering label — item='%s' expiry='%s'", _item_name, _item_expiry)
            call_add_item_service(_item_name, _item_expiry)

            hardware.label_arm.goto_joints(pre_joints, duration=5)
            hardware.label_arm.goto_joints(final_joints, duration=5)
            hardware.assert_vacuum('lbl', True)
            call_trigger_service('/orio/lbl_cup/off')
            rospy.sleep(1.0)
            

            hardware.label_arm.goto_joints(pre_joints, duration=2)

            safe_joints = hardware.pose_to_joints("LABELLING_SAFE")
            hardware.label_arm.goto_joints(safe_joints, duration=3)
            hardware.label_arm.wait_for_skill()
            
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
    raises SystemExit so the operator cannot accidentally start without metadata.
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
            name   = name_var.get().strip()
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
            result['name']   = name
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


def _shutdown_listener():
    """Background thread: sets shutdown_event when the operator types 'q' + Enter."""
    print("\n[ORIO] Type 'q' + Enter to stop cleanly.")
    while not shutdown_event.is_set() and not rospy.is_shutdown():
        try:
            line = input()
        except EOFError:
            break
        if line.strip().lower() == 'q':
            print("[ORIO] Shutdown requested.")
            shutdown_event.set()
            break


def main():
    global hardware, _item_name, _item_expiry
    rospy.init_node('dual_arm_labeling_fsm')
    log_path = _setup_file_logger()
    log.info("[Main] Node started. Log file: %s", log_path)

    random.seed(1)

    # Collect item metadata via GUI before any motion starts.
    _item_name, _item_expiry = _show_item_setup_gui()
    if _item_name is None:
        log.warning("[Main] Setup cancelled by operator — exiting cleanly.")
        return
    log.info("[Main] Operator input — item='%s'  expiry='%s'", _item_name, _item_expiry)

    threading.Thread(target=_shutdown_listener, daemon=True).start()

    # Initialize hardware once node is running
    hardware = RobotHardware()

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
                                            'failed':    'HUMAN_RECOVERY_ARM1'})
        smach.StateMachine.add('DROP_TO_ZONE', DropToZone(),
                               transitions={'dropped': 'DECIDE_HUB', 'wait': 'DROP_TO_ZONE',
                                            'failed':  'HUMAN_RECOVERY_ARM1'})
        smach.StateMachine.add('RETRIEVE', RetrieveFromZone(),
                               transitions={'picked': 'PLACE_OUTPUT', 'no_items': 'DECIDE_HUB',
                                            'failed': 'HUMAN_RECOVERY_ARM1'})
        smach.StateMachine.add('PLACE_OUTPUT', PlaceOutput(),
                               transitions={'succeeded': 'FETCH_INPUT',
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
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        log.warning("[Main] Interrupt received — requesting clean shutdown")
        shutdown_event.set()
    finally:
        log.warning("[Main] Shutting down: turning off both vacuums")
        log.info("[Main] Total items placed at output: %d", _items_placed_output)
        call_trigger_service('/orio/pnp_cup/off')
        call_trigger_service('/orio/lbl_cup/off')
        if 'sis' in locals():
            sis.stop()

if __name__ == '__main__':
    main()
