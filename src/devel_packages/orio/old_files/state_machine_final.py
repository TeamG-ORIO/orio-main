#!/usr/bin/env python3

import sys
import os
import json
import logging
import datetime
import threading
import tkinter as tk
from tkinter import ttk, messagebox
from threading import Lock

import numpy as np
import rospy
import smach
import smach_ros
import ikpy.chain
from scipy.spatial.transform import Rotation as R
from autolab_core import RigidTransform
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'prm'))
from franka_prm_single_arm import SingleArmExecutor

# ==============================================================================
# CONSTANTS
# ==============================================================================

APPROACH_DISTANCE    = 0.20       # vertical clearance above pick/place targets (m)
PICK_UP_ZONE_GROUND_Z = 0.09
LABEL_ZONE_GROUND_Z  = 0.055

POSES_FILE        = "joint_angles.json"
TARGET_POSES_FILE = "Target_Task_Poses.json"
URDF_FILE         = "panda_arm_hand.urdf"

_LOG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'logs')

# Module-level logger — configured by _setup_file_logger() once the node starts
log = logging.getLogger("orio_fsm")

# ==============================================================================
# GLOBAL EVENTS & SHARED STATE
# ==============================================================================

# Set to trigger a clean shutdown from any thread
shutdown_event = threading.Event()

# Set while one arm is in human recovery; other arm's poll loops spin-wait on this
recovery_event = threading.Event()

# Queued next-state override set by the recovery GUI for the *other* arm
# Keys: 'arm1', 'arm2' — value is a state name string or None
pending_state_override      = {'arm1': None, 'arm2': None}
pending_state_override_lock = Lock()

# Ensures only one Tkinter window exists at a time — Tkinter is not thread-safe
_gui_lock = Lock()

# Valid re-entry states for each arm (used by recovery GUI radio buttons)
ARM1_STATES = ['DECIDE_HUB', 'FETCH_INPUT', 'DROP_TO_ZONE', 'RETRIEVE', 'PLACE_OUTPUT']
ARM2_STATES = ['GET_LABEL', 'APPLY']

# ==============================================================================
# LOGGING SETUP
# ==============================================================================

def _setup_file_logger():
    """Create a timestamped log file and attach it (+ stdout) to the module logger."""
    os.makedirs(_LOG_DIR, exist_ok=True)
    log_path = os.path.join(_LOG_DIR, f"orio_run_{datetime.datetime.now():%Y%m%d_%H%M%S}.log")

    fmt = logging.Formatter('%(asctime)s %(levelname)-8s %(message)s')
    sh  = logging.StreamHandler()
    sh.setFormatter(fmt)
    fh  = logging.FileHandler(log_path)
    fh.setFormatter(fmt)

    log.setLevel(logging.DEBUG)
    log.handlers.clear()
    log.addHandler(sh)
    log.addHandler(fh)
    log.propagate = False

    log.info("[Logger] File log started: %s", log_path)
    return log_path

# ==============================================================================
# UTILITIES
# ==============================================================================

def call_trigger_service(service_name):
    """Call a ROS Trigger service; return True on success, False on failure."""
    try:
        rospy.wait_for_service(service_name, timeout=5.0)
        proxy    = rospy.ServiceProxy(service_name, Trigger)
        response = proxy()
        log.info("[Service] %s → success=%s msg='%s'",
                 service_name, response.success, getattr(response, 'message', ''))
        return response.success
    except Exception as e:
        log.error("[Service] Call failed for %s: %s", service_name, e)
    return False

# ==============================================================================
# SECTION 1: SYSTEM SETUP & DATA MANAGEMENT
# ==============================================================================

class RobotHardware:
    """Holds both arm executors, IK chain, loaded poses, and vision subscribers."""

    def __init__(self):
        log.info("[Hardware] Initializing...")

        # PRM-aware executor wrappers for each arm
        self.ppa_executor = SingleArmExecutor(arm_number=1, init_node=False)
        self.la_executor  = SingleArmExecutor(arm_number=2, init_node=False)

        log.info("[Hardware] Resetting both arms to home joints")
        self.ppa_executor.fa.reset_joints()
        self.la_executor.fa.reset_joints()

        # IK chain built from URDF; joints 1-7 are active
        self.ik_chain = ikpy.chain.Chain.from_urdf_file(URDF_FILE, base_elements=["panda_link0"])
        self.chain_length = len(self.ik_chain.links)
        self.ik_chain.active_links_mask = [False] + [True] * 7 + [False] * (self.chain_length - 8)

        # Load fixed joint-angle poses from JSON
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
        # Pre-compute the approach joint config for the label dispenser
        self.label_arm_dispenser_contact = raw['label_arm']['LABEL_DISPENSER']
        self.label_arm_dispenser_pre     = self._contact_to_pre_joints(self.label_arm_dispenser_contact)
        log.info("[Hardware] Poses loaded from %s", POSES_FILE)

        # Latest grasp poses published by vision nodes (cleared before each request)
        self.latest_pose_msg       = None
        self.latest_label_pose_z1  = None
        self.latest_label_pose_z2  = None
        rospy.Subscriber("/grasp_poses",              PoseArray, self._pose_callback,          queue_size=1)
        rospy.Subscriber("/grasp_poses_labelling_z1", PoseArray, self._label_pose_callback_z1, queue_size=1)
        rospy.Subscriber("/grasp_poses_labelling_z2", PoseArray, self._label_pose_callback_z2, queue_size=1)

        # Per-cycle pick state, updated by FetchInput
        self.current_fetch_depth   = 0.0
        self.pick_up_pre_joints    = None
        self.pick_up_final_joints  = None

        # Vacuum suction flags, updated by the pneumatic_control node
        self.pnp_has_item = False
        self.lbl_has_item = False
        rospy.Subscriber("orio/vacuum/pnp_has_item", Bool,
                         lambda msg: setattr(self, 'pnp_has_item', msg.data), queue_size=1)
        rospy.Subscriber("orio/vacuum/lbl_has_item", Bool,
                         lambda msg: setattr(self, 'lbl_has_item', msg.data), queue_size=1)

        log.info("[Hardware] Initialization complete")

    # ── ROS topic callbacks ──────────────────────────────────────────────────

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

    # ── Kinematics ───────────────────────────────────────────────────────────

    def compute_pick_joints(self, task_pos,
                            target_ori=np.array([[1.0, 0.0, 0.0],
                                                 [0.0, -1.0, 0.0],
                                                 [0.0, 0.0, -1.0]])):
        """Return (pre_joints, final_joints) for approaching and reaching task_pos via IK."""
        initial_guess = [0.0] * self.chain_length
        initial_guess[4] = -1.5  # bias joint 4 toward a straight-down EE posture

        pre_pos   = [task_pos[0], task_pos[1], task_pos[2] + APPROACH_DISTANCE]
        final_pos = [task_pos[0], task_pos[1], task_pos[2]]

        pre_angles   = self.ik_chain.inverse_kinematics(
            target_position=pre_pos, target_orientation=target_ori,
            orientation_mode="all", initial_position=initial_guess)
        final_angles = self.ik_chain.inverse_kinematics(
            target_position=final_pos, target_orientation=target_ori,
            orientation_mode="all", initial_position=pre_angles)

        log.info("[IK] compute_pick_joints target=%s pre=%s final=%s",
                 np.round(task_pos, 4),
                 np.round(pre_angles[1:8], 4),
                 np.round(final_angles[1:8], 4))
        return pre_angles[1:8], final_angles[1:8]

    def compute_label_joints(self, zone_number, item_depth):
        """Trigger the labelling vision service, wait for a pose, and return
        (pre_joints, final_joints) via IK for the given zone and item depth."""
        log.info("[IK] Requesting label pose zone=%d depth=%.4f", zone_number, item_depth)

        # Clear the relevant pose buffer before requesting a fresh one
        if zone_number == 1:
            self.latest_label_pose_z1 = None
        else:
            self.latest_label_pose_z2 = None

        if not call_trigger_service('/compute_grasps_labelling'):
            raise RuntimeError("Service /compute_grasps_labelling failed")

        # Wait up to 5 s for the vision node to publish a pose
        deadline = rospy.Time.now() + rospy.Duration(5.0)
        while rospy.Time.now() < deadline:
            msg = self.latest_label_pose_z1 if zone_number == 1 else self.latest_label_pose_z2
            if msg is not None and msg.poses:
                break
            rospy.sleep(0.1)
        else:
            raise RuntimeError(f"No labelling pose received for zone {zone_number} within timeout")

        p        = msg.poses[0]
        task_pos = [p.position.x, p.position.y, item_depth]
        task_ori = [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]
        ori_mat  = R.from_quat(task_ori).as_matrix()
        rpy      = R.from_quat(task_ori).as_euler('xyz', degrees=True)
        log.info("[IK] Zone %d pose: x=%.4f y=%.4f z=%.4f  RPY=%.2f,%.2f,%.2f deg",
                 zone_number, task_pos[0], task_pos[1], task_pos[2], rpy[0], rpy[1], rpy[2])
        return self.compute_pick_joints(task_pos, ori_mat)

    def pose_to_joints(self, location_tag, depth_offset=0.0):
        """Look up a named pose in Target_Task_Poses.json and return 7-dof joint angles via IK."""
        with open(TARGET_POSES_FILE) as f:
            task_poses = json.load(f)

        if location_tag not in task_poses:
            raise KeyError(f"'{location_tag}' not found in {TARGET_POSES_FILE}. "
                           f"Available: {list(task_poses.keys())}")

        entry      = task_poses[location_tag]
        tx, ty, tz = entry["translation"]
        tz        += depth_offset
        target_rot = np.array(entry["rotation"])

        initial_guess    = [0.0] * self.chain_length
        initial_guess[4] = -1.5  # bias toward straight-down posture

        angles = self.ik_chain.inverse_kinematics(
            target_position=[tx, ty, tz],
            target_orientation=target_rot,
            orientation_mode="all",
            initial_position=initial_guess,
        )
        log.info("[IK] pose_to_joints '%s' depth_offset=%.4f → %s",
                 location_tag, depth_offset, np.round(angles[1:8], 4))
        return angles[1:8]

    def _contact_to_pre_joints(self, contact_joints):
        """Given contact joint angles, return the approach joint angles APPROACH_DISTANCE above."""
        full      = [0.0] * self.chain_length
        full[1:8] = contact_joints
        tf        = self.ik_chain.forward_kinematics(full)
        pre_pos   = tf[:3, 3] + np.array([0.0, 0.0, APPROACH_DISTANCE])

        initial_guess    = [0.0] * self.chain_length
        initial_guess[4] = -1.5
        pre_angles = self.ik_chain.inverse_kinematics(
            target_position=pre_pos, target_orientation=tf[:3, :3],
            orientation_mode="all", initial_position=initial_guess)
        return pre_angles[1:8]

    # ── PRM motion ───────────────────────────────────────────────────────────

    def _prm_move(self, executor, q_goal, label_zone=None,
                  traj_type=None, duration=None, spline_speed=None, totg_scale=None):
        """Plan a PRM path from current joints to q_goal and execute it.

        Keyword args temporarily override executor settings for this move only.
        Raises RuntimeError if PRM planning fails.
        """
        # Temporarily apply any per-move overrides to the executor
        overrides = {k: v for k, v in {
            'traj_type': traj_type, 'duration': duration,
            'spline_speed': spline_speed, 'totg_scale': totg_scale,
        }.items() if v is not None}
        saved = {k: getattr(executor, k) for k in overrides}
        for k, v in overrides.items():
            setattr(executor, k, v)

        try:
            # FK sanity check: warn if goal EE is not pointing straight down
            full_q    = [0.0] * self.chain_length
            full_q[1:8] = list(q_goal)
            fk        = self.ik_chain.forward_kinematics(full_q)
            ee_z_axis = fk[:3, 2]
            tilt_deg  = np.degrees(np.arccos(np.clip(-ee_z_axis[2], -1.0, 1.0)))
            if tilt_deg > 5.0:
                log.warning("_prm_move: EE tilt=%.1f deg (expected <5) ee_z=[%.3f,%.3f,%.3f]",
                            tilt_deg, *ee_z_axis)
            else:
                log.info("_prm_move: EE orientation OK (tilt=%.1f deg)", tilt_deg)

            log.info("[_prm_move] Arm%d zone=%s traj=%s speed=%s q_goal=%s",
                     executor.arm_number, label_zone, traj_type, spline_speed,
                     np.round(q_goal, 4))

            q_init = executor.fa.get_joints()
            log.info("[_prm_move] Arm%d q_init=%s", executor.arm_number, np.round(q_init, 4))

            plan = executor._plan_from_joints(q_init, q_goal, label_zone)
            if plan is None:
                rpy_goal = R.from_matrix(fk[:3, :3]).as_euler('xyz', degrees=True)
                log.error(
                    "[_prm_move] Arm%d PRM failed zone=%s q_goal=%s FK xyz=(%.4f,%.4f,%.4f) rpy=(%.2f,%.2f,%.2f)",
                    executor.arm_number, label_zone, np.round(q_goal, 4),
                    fk[0, 3], fk[1, 3], fk[2, 3], rpy_goal[0], rpy_goal[1], rpy_goal[2])
                raise RuntimeError(f"Arm{executor.arm_number}: PRM planning failed (zone={label_zone})")
            log.info("[_prm_move] Arm%d plan=%d waypoints", executor.arm_number, len(plan))

            executor._execute_plan_waypoints(np.array(plan))
            executor.fa.wait_for_skill()

            # Log post-move tracking error and EE tilt for diagnostics
            q_actual  = executor.fa.get_joints()
            q_err     = q_actual - q_goal
            full_q[1:8] = list(q_actual)
            fk        = self.ik_chain.forward_kinematics(full_q)
            tilt_deg  = np.degrees(np.arccos(np.clip(-fk[:3, 2][2], -1.0, 1.0)))
            log.info("[_prm_move] Arm%d done q_err_max=%.4f rad EE tilt=%.1f deg",
                     executor.arm_number, float(np.max(np.abs(q_err))), tilt_deg)
        finally:
            # Always restore the executor's original settings
            for k, v in saved.items():
                setattr(executor, k, v)


class ZoneManager:
    """Thread-safe state tracker for the two label zones (A and B)."""

    def __init__(self):
        self.lock   = Lock()
        self.states = {'A': 'EMPTY', 'B': 'EMPTY'}  # EMPTY | NEEDS_LABEL | READY_FOR_PICKUP
        self.busy   = {'A': False,   'B': False}     # True while an arm is actively using the zone
        self.depths = {'A': 0.0,     'B': 0.0}       # z-depth of the item currently in each zone

    def set_state(self, zone, new_state):
        """Update zone state and log the transition."""
        old = self.states[zone]
        self.states[zone] = new_state
        log.info("[ZoneManager] Zone %s: %s → %s", zone, old, new_state)


# Module-level singletons — hardware is initialised in main() after rospy.init_node
manager  = ZoneManager()
hardware = None

# Robot 1 position/context used by _robot1_prm_file() to select the correct PRM graph
robot1_position   = 'NONE'          # 'LABEL_ZONE_1' | 'LABEL_ZONE_2' | 'NONE'
robot1_prev_state = 'PLACE_OUTPUT'  # 'DROP_TO_ZONE' | 'PLACE_OUTPUT'

_items_placed_output = 0  # cycle counter for throughput logging
_ZONE_TO_INT = {'LABEL_ZONE_1': 1, 'LABEL_ZONE_2': 2}


def _set_robot1_state(new_position, new_prev_state):
    """Update the robot1 position/context globals and log the transition."""
    global robot1_position, robot1_prev_state
    log.info("[Robot1State] pos: %s→%s  prev: %s→%s",
             robot1_position, new_position, robot1_prev_state, new_prev_state)
    robot1_position   = new_position
    robot1_prev_state = new_prev_state


def _robot1_prm_file(current_state, target_zone=None):
    """Return the PRM label_zone int for the next _prm_move call.

    The PRM graphs are indexed by which label zone(s) the arm must avoid:
      FETCH_INPUT      from DROP_TO_ZONE → source zone int; from PLACE_OUTPUT → None
      DROP_TO_ZONE     always            → target_zone int
      RETRIEVE_FROM_ZONE from DROP_TO_ZONE → 3 (free path); from PLACE_OUTPUT → target int
      PLACE_OUTPUT     always            → source zone int
    """
    if current_state == 'FETCH_INPUT':
        return _ZONE_TO_INT.get(robot1_position) if robot1_prev_state == 'DROP_TO_ZONE' else None

    if current_state == 'DROP_TO_ZONE':
        return _ZONE_TO_INT.get(target_zone)

    if current_state == 'RETRIEVE_FROM_ZONE':
        return 3 if robot1_prev_state == 'DROP_TO_ZONE' else _ZONE_TO_INT.get(target_zone)

    if current_state == 'PLACE_OUTPUT':
        return _ZONE_TO_INT.get(robot1_position)

    return None

# ==============================================================================
# SECTION 2: HUMAN RECOVERY GUI & HELPERS
# ==============================================================================

def _show_recovery_gui(arm_name, failed_state, error_msg, valid_states,
                       other_arm_name, other_arm_states):
    """Show a blocking Tkinter recovery dialog on a daemon thread.

    The operator can correct zone manager state and choose the next state for
    either arm.  Returns (failed_arm_choice, other_arm_choice_or_None).
    """
    result = {'choice': 'shutdown', 'other_choice': None}

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
            busy_var = tk.BooleanVar(value=manager.busy[zone])
            tk.Checkbutton(row, text="busy", variable=busy_var,
                           bg='#1e1e2e', fg='#cdd6f4', selectcolor='#313244',
                           activebackground='#1e1e2e').pack(side='left')
            zone_vars[f'{zone}_busy'] = busy_var

        def apply_zone_changes():
            with manager.lock:
                for zone in ['A', 'B']:
                    manager.states[zone] = zone_vars[zone].get()
                    manager.busy[zone]   = zone_vars[f'{zone}_busy'].get()
            log.info("[Recovery] Zones updated: %s  busy: %s", manager.states, manager.busy)

        tk.Button(zone_frame, text="Apply Zone Changes", command=apply_zone_changes,
                  bg='#313244', fg='#cdd6f4', relief='flat', padx=8).pack(pady=(6, 0))

        # ── State selector: failed arm ───────────────────────────────────────
        sel_frame = tk.LabelFrame(root, text=f" Jump to State — {arm_name} ",
                                  bg='#1e1e2e', fg='#89b4fa',
                                  font=('Helvetica', 10, 'bold'), padx=12, pady=8)
        sel_frame.pack(fill='x', padx=16, pady=(0, 6))
        tk.Label(sel_frame, text="Choose the state to run next:",
                 font=('Courier', 11), fg='#cdd6f4', bg='#1e1e2e').pack(anchor='w')
        choice_var = tk.StringVar(value=valid_states[0])
        for s in valid_states:
            tk.Radiobutton(sel_frame, text=s, variable=choice_var, value=s,
                           font=('Courier', 11), fg='#a6e3a1', bg='#1e1e2e',
                           selectcolor='#313244', activebackground='#1e1e2e').pack(anchor='w')

        # ── State selector: other arm (opt-in) ───────────────────────────────
        other_frame = tk.LabelFrame(root, text=f" Override Next State — {other_arm_name} ",
                                    bg='#1e1e2e', fg='#89b4fa',
                                    font=('Helvetica', 10, 'bold'), padx=12, pady=8)
        other_frame.pack(fill='x', padx=16, pady=(0, 8))

        other_enable_var = tk.BooleanVar(value=False)
        other_choice_var = tk.StringVar(value=other_arm_states[0])

        def _toggle_other_arm():
            # Enable/disable the other-arm radio buttons based on the checkbox
            s = 'normal' if other_enable_var.get() else 'disabled'
            for rb in other_radiobuttons:
                rb.configure(state=s)

        tk.Checkbutton(other_frame, text="Override this arm's next state",
                       variable=other_enable_var, command=_toggle_other_arm,
                       bg='#1e1e2e', fg='#cdd6f4', selectcolor='#313244',
                       activebackground='#1e1e2e', font=('Courier', 11)).pack(anchor='w')

        other_radiobuttons = []
        for s in other_arm_states:
            rb = tk.Radiobutton(other_frame, text=s, variable=other_choice_var, value=s,
                                font=('Courier', 11), fg='#89dceb', bg='#1e1e2e',
                                selectcolor='#313244', activebackground='#1e1e2e',
                                state='disabled')
            rb.pack(anchor='w')
            other_radiobuttons.append(rb)

        # ── Action buttons ───────────────────────────────────────────────────
        btn_frame = tk.Frame(root, bg='#1e1e2e', pady=10)
        btn_frame.pack(fill='x', padx=16)

        def on_resume():
            apply_zone_changes()
            result['choice']       = choice_var.get()
            result['other_choice'] = other_choice_var.get() if other_enable_var.get() else None
            root.destroy()

        def on_estop():
            if messagebox.askyesno("E-Stop", "Emergency stop — shut down everything?", parent=root):
                result['choice']       = 'shutdown'
                result['other_choice'] = None
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
    return result['choice'], result['other_choice']


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
    Pauses both arms via recovery_event while the operator resolves the fault."""

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=ARM1_STATES + ['shutdown'],
                             input_keys=['recovery_failed_state', 'recovery_error_msg'])

    def execute(self, userdata):
        # If a prior ARM2 recovery already queued an override for us, use it immediately
        with pending_state_override_lock:
            override = pending_state_override['arm1']
            if override is not None:
                pending_state_override['arm1'] = None
                log.info("[ARM1 Recovery] Using pending override: %s", override)
                return override

        log.warning("[ARM1 Recovery] Showing GUI — both arms paused")
        recovery_event.set()
        failed  = getattr(userdata, 'recovery_failed_state', 'UNKNOWN')
        err_msg = getattr(userdata, 'recovery_error_msg',    'No details available.')
        choice, other_choice = _show_recovery_gui(
            "ARM1 (Pick-and-Place)", failed, err_msg, ARM1_STATES,
            "ARM2 (Labeling)",       ARM2_STATES)
        log.info("[ARM1 Recovery] choice=%s  ARM2 override=%s", choice, other_choice)
        if other_choice is not None:
            with pending_state_override_lock:
                pending_state_override['arm2'] = other_choice
        recovery_event.clear()
        if choice == 'shutdown':
            shutdown_event.set()
        return choice


class HumanRecoveryArm2(smach.State):
    """Entered whenever an ARM2 state raises an exception.
    Pauses both arms via recovery_event while the operator resolves the fault."""

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=ARM2_STATES + ['shutdown'],
                             input_keys=['recovery_failed_state', 'recovery_error_msg'])

    def execute(self, userdata):
        # If a prior ARM1 recovery already queued an override for us, use it immediately
        with pending_state_override_lock:
            override = pending_state_override['arm2']
            if override is not None:
                pending_state_override['arm2'] = None
                log.info("[ARM2 Recovery] Using pending override: %s", override)
                return override

        log.warning("[ARM2 Recovery] Showing GUI — both arms paused")
        recovery_event.set()
        failed  = getattr(userdata, 'recovery_failed_state', 'UNKNOWN')
        err_msg = getattr(userdata, 'recovery_error_msg',    'No details available.')
        choice, other_choice = _show_recovery_gui(
            "ARM2 (Labeling)",       failed, err_msg, ARM2_STATES,
            "ARM1 (Pick-and-Place)", ARM1_STATES)
        log.info("[ARM2 Recovery] choice=%s  ARM1 override=%s", choice, other_choice)
        if other_choice is not None:
            with pending_state_override_lock:
                pending_state_override['arm1'] = other_choice
        recovery_event.clear()
        if choice == 'shutdown':
            shutdown_event.set()
        return choice

# ==============================================================================
# SECTION 3: ARM 1 (PICK-AND-PLACE) STATES
# ==============================================================================

class TaskSelector(smach.State):
    """Decide what ARM1 should do next based on current zone states."""

    def __init__(self):
        smach.State.__init__(self, outcomes=['do_load', 'do_unload', 'wait', 'shutdown'])

    def execute(self, userdata):
        if shutdown_event.is_set():
            return 'shutdown'
        # Spin-wait while ARM2 is in human recovery
        while recovery_event.is_set():
            if shutdown_event.is_set():
                return 'shutdown'
            rospy.sleep(0.2)

        log.debug("[TaskSelector] zones=%s busy=%s", manager.states, manager.busy)
        with manager.lock:
            # Unload takes priority over loading
            for z in ['A', 'B']:
                if manager.states[z] == 'READY_FOR_PICKUP' and not manager.busy[z]:
                    log.info("[TaskSelector] → do_unload (zone %s)", z)
                    return 'do_unload'
            for z in ['A', 'B']:
                if manager.states[z] == 'EMPTY' and not manager.busy[z]:
                    log.info("[TaskSelector] → do_load (zone %s)", z)
                    return 'do_load'
        rospy.sleep(0.5)
        return 'wait'


class FetchInput(smach.State):
    """Pick an item from the input area using vision-guided grasping."""

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'],
                             output_keys=['item_depth',
                                          'recovery_failed_state', 'recovery_error_msg'])

    def _run(self, userdata):
        hardware.latest_pose_msg = None  # clear stale vision data
        if not call_trigger_service('/compute_grasps'):
            raise RuntimeError("/compute_grasps service failed")

        # Wait up to 5 s for the vision node to publish a grasp pose
        deadline = rospy.Time.now() + rospy.Duration(5.0)
        while hardware.latest_pose_msg is None and rospy.Time.now() < deadline:
            rospy.sleep(0.1)
        if hardware.latest_pose_msg is None or not hardware.latest_pose_msg.poses:
            raise RuntimeError("No grasp pose received within timeout")

        p        = hardware.latest_pose_msg.poses[0]
        task_pos = [p.position.x, p.position.y, p.position.z]
        hardware.current_fetch_depth  = task_pos[2]
        userdata.item_depth           = task_pos[2]
        log.info("[FetchInput] Grasp pose: x=%.4f y=%.4f z=%.4f", *task_pos)

        hardware.pick_up_pre_joints, hardware.pick_up_final_joints = \
            hardware.compute_pick_joints(task_pos)

        # Move to approach height via PRM, then descend straight for the pick
        prm_zone = _robot1_prm_file('FETCH_INPUT')
        hardware._prm_move(hardware.ppa_executor, hardware.pick_up_pre_joints,
                           prm_zone, traj_type='spline', spline_speed=0.3)
        _set_robot1_state('NONE', robot1_prev_state)
        hardware.ppa_executor.fa.goto_joints(hardware.pick_up_final_joints, duration=5)

        call_trigger_service('/orio/pnp_cup/on')
        rospy.sleep(1.0)
        hardware.assert_vacuum('pnp', True)  # confirm item was picked up

        hardware.ppa_executor.fa.goto_joints(hardware.pick_up_pre_joints, duration=2)
        log.info("[FetchInput] Succeeded (item_depth=%.4f)", hardware.current_fetch_depth)
        return 'succeeded'

    def execute(self, userdata):
        return _safe_execute('FETCH_INPUT', self._run, userdata)


class DropToZone(smach.State):
    """Place the held item into an empty label zone."""

    def __init__(self):
        smach.State.__init__(self, outcomes=['dropped', 'wait', 'failed'],
                             input_keys=['item_depth'],
                             output_keys=['recovery_failed_state', 'recovery_error_msg'])

    def _run(self, userdata):
        # Claim the first available empty zone
        target_zone = None
        with manager.lock:
            for z in ['A', 'B']:
                if manager.states[z] == 'EMPTY' and not manager.busy[z]:
                    target_zone    = z
                    manager.busy[z] = True
                    break

        if not target_zone:
            rospy.sleep(0.5)
            return 'wait'

        log.info("[DropToZone] Targeting zone %s (item_depth=%.4f)",
                 target_zone, userdata.item_depth)
        manager.depths[target_zone] = userdata.item_depth  # share depth with ARM2

        try:
            zone_tag   = 'LABEL_ZONE_1' if target_zone == 'A' else 'LABEL_ZONE_2'
            depth      = manager.depths[target_zone]
            pre_joints = hardware.pose_to_joints(zone_tag, depth_offset=depth + 0.01 + 0.1787)
            pl_joints  = hardware.pose_to_joints(zone_tag, depth_offset=depth + 0.01)

            _set_robot1_state(zone_tag, robot1_prev_state)
            hardware._prm_move(hardware.ppa_executor, pre_joints,
                               _robot1_prm_file('DROP_TO_ZONE', zone_tag),
                               traj_type='spline', spline_speed=0.2)
            hardware.ppa_executor.fa.goto_joints(pl_joints, duration=2)
            hardware.assert_vacuum('pnp', True)   # confirm item still held before release
            call_trigger_service('/orio/pnp_cup/off')
            rospy.sleep(1.0)
            hardware.assert_vacuum('pnp', False)  # confirm item was released
            hardware.ppa_executor.fa.goto_joints(pre_joints, duration=2)

            _set_robot1_state(robot1_position, 'DROP_TO_ZONE')
            with manager.lock:
                manager.set_state(target_zone, 'NEEDS_LABEL')
                manager.busy[target_zone] = False
            log.info("[DropToZone] Dropped into zone %s", target_zone)
            return 'dropped'
        except Exception:
            # Always release the zone lock so neither arm deadlocks on this zone
            with manager.lock:
                manager.busy[target_zone] = False
            raise

    def execute(self, userdata):
        return _safe_execute('DROP_TO_ZONE', self._run, userdata)


class RetrieveFromZone(smach.State):
    """Pick a labeled item back out of a label zone."""

    def __init__(self):
        smach.State.__init__(self, outcomes=['picked', 'no_items', 'failed'],
                             output_keys=['retrieved_depth',
                                          'recovery_failed_state', 'recovery_error_msg'])

    def _run(self, userdata):
        # Claim the first zone that has a labeled item ready
        target_zone = None
        with manager.lock:
            for z in ['A', 'B']:
                if manager.states[z] == 'READY_FOR_PICKUP' and not manager.busy[z]:
                    target_zone    = z
                    manager.busy[z] = True
                    break

        if not target_zone:
            return 'no_items'

        depth                    = manager.depths[target_zone]
        userdata.retrieved_depth = depth
        zone_tag                 = 'LABEL_ZONE_1' if target_zone == 'A' else 'LABEL_ZONE_2'
        pre_joints               = hardware.pose_to_joints(zone_tag, depth_offset=depth + APPROACH_DISTANCE)
        pick_joints              = hardware.pose_to_joints(zone_tag, depth_offset=depth - 0.01)  # small z offset to ensure contact
        log.info("[RetrieveFromZone] Picking zone %s (depth=%.4f)", target_zone, depth)

        try:
            _set_robot1_state(zone_tag, robot1_prev_state)
            hardware._prm_move(hardware.ppa_executor, pre_joints,
                               _robot1_prm_file('RETRIEVE_FROM_ZONE', zone_tag),
                               traj_type='spline', spline_speed=0.2)
            hardware._prm_move(hardware.ppa_executor, pick_joints,
                               _robot1_prm_file('RETRIEVE_FROM_ZONE', zone_tag),
                               traj_type='spline', spline_speed=0.2)

            call_trigger_service('/orio/pnp_cup/on')
            rospy.sleep(1.0)
            hardware.assert_vacuum('pnp', True)  # confirm item was picked up

            hardware.ppa_executor.fa.goto_joints(pre_joints, duration=2)
            with manager.lock:
                manager.set_state(target_zone, 'EMPTY')
                manager.busy[target_zone] = False
            log.info("[RetrieveFromZone] Picked from zone %s", target_zone)
            return 'picked'
        except Exception:
            # Always release the zone lock so neither arm deadlocks on this zone
            with manager.lock:
                manager.busy[target_zone] = False
            raise

    def execute(self, userdata):
        return _safe_execute('RETRIEVE', self._run, userdata)


class PlaceOutput(smach.State):
    """Deposit the held item at the output location."""

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'],
                             input_keys=['retrieved_depth'],
                             output_keys=['recovery_failed_state', 'recovery_error_msg'])

    def _run(self, userdata):
        global _items_placed_output
        depth      = userdata.retrieved_depth
        drop_joints = hardware.pose_to_joints("OUTPUT", depth_offset=depth + 0.1)
        log.info("[PlaceOutput] Moving to output (depth=%.4f)", depth)

        hardware._prm_move(hardware.ppa_executor, drop_joints,
                           _robot1_prm_file('PLACE_OUTPUT'),
                           traj_type='spline', spline_speed=0.2)
        _set_robot1_state('NONE', 'PLACE_OUTPUT')
        hardware.assert_vacuum('pnp', True)   # confirm item still held before release
        call_trigger_service('/orio/pnp_cup/off')
        rospy.sleep(1.0)
        hardware.assert_vacuum('pnp', False)  # confirm item was released

        _items_placed_output += 1
        log.info("[PlaceOutput] Succeeded (total=%d)", _items_placed_output)
        return 'succeeded'

    def execute(self, userdata):
        return _safe_execute('PLACE_OUTPUT', self._run, userdata)

# ==============================================================================
# SECTION 4: ARM 2 (LABELING) STATES
# ==============================================================================

class GetLabel(smach.State):
    """Pick a label from the dispenser and move to the labeling safe position."""

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'],
                             output_keys=['recovery_failed_state', 'recovery_error_msg'])

    def _run(self, userdata):
        # Approach and contact the dispenser, then pick up the label
        hardware.la_executor.fa.goto_joints(hardware.label_arm_dispenser_pre,     duration=5)
        hardware.la_executor.fa.goto_joints(hardware.label_arm_dispenser_contact, duration=5)
        call_trigger_service('/orio/lbl_cup/on')
        rospy.sleep(1.0)
        hardware.assert_vacuum('lbl', True)  # confirm label was picked up

        # Retreat and move to a clear position before approaching the label zone
        hardware.la_executor.fa.goto_joints(hardware.label_arm_dispenser_pre, duration=2)
        safe_joints = hardware.pose_to_joints("LABELLING_SAFE")
        hardware.la_executor.fa.goto_joints(safe_joints, duration=3)
        log.info("[GetLabel] Succeeded")
        return 'succeeded'

    def execute(self, userdata):
        return _safe_execute('GET_LABEL', self._run, userdata)


class ApplyLabel(smach.State):
    """Apply the held label to the next item waiting in a label zone."""

    def __init__(self):
        smach.State.__init__(self, outcomes=['labeled', 'wait', 'failed', 'shutdown'],
                             output_keys=['recovery_failed_state', 'recovery_error_msg'])

    def _run(self, userdata):
        if shutdown_event.is_set():
            return 'shutdown'
        # Spin-wait while ARM1 is in human recovery
        while recovery_event.is_set():
            if shutdown_event.is_set():
                return 'shutdown'
            rospy.sleep(0.2)

        # Claim the first zone that needs a label
        target_zone = None
        with manager.lock:
            for z in ['A', 'B']:
                if manager.states[z] == 'NEEDS_LABEL' and not manager.busy[z]:
                    target_zone    = z
                    manager.busy[z] = True
                    break

        if not target_zone:
            rospy.sleep(0.5)
            return 'wait'

        zone_number = 1 if target_zone == 'A' else 2
        log.info("[ApplyLabel] Labeling zone %s (zone_number=%d, depth=%.4f)",
                 target_zone, zone_number, manager.depths[target_zone])

        try:
            pre_joints, final_joints = hardware.compute_label_joints(
                zone_number, manager.depths[target_zone])

            # Approach, apply, and retreat
            hardware._prm_move(hardware.la_executor, pre_joints,   zone_number, traj_type='spline', spline_speed=0.2)
            hardware._prm_move(hardware.la_executor, final_joints, zone_number, traj_type='spline', spline_speed=0.2)
            hardware.assert_vacuum('lbl', True)   # confirm label still held before application
            call_trigger_service('/orio/lbl_cup/off')
            rospy.sleep(1.0)
            hardware.assert_vacuum('lbl', False)  # confirm label was applied/released
            hardware._prm_move(hardware.la_executor, pre_joints, zone_number, traj_type='spline', spline_speed=0.2)

            # Return to safe position to clear the workspace for ARM1
            safe_joints = hardware.pose_to_joints("LABELLING_SAFE")
            hardware._prm_move(hardware.la_executor, safe_joints, zone_number, traj_type='spline', spline_speed=0.2)

            with manager.lock:
                manager.set_state(target_zone, 'READY_FOR_PICKUP')
                manager.busy[target_zone] = False
            log.info("[ApplyLabel] Labeled zone %s", target_zone)
            return 'labeled'
        except Exception:
            # Always release the zone lock so neither arm deadlocks on this zone
            with manager.lock:
                manager.busy[target_zone] = False
            raise

    def execute(self, userdata):
        return _safe_execute('APPLY_LABEL', self._run, userdata)

# ==============================================================================
# SECTION 5: ENTRY POINT
# ==============================================================================

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
    global hardware
    rospy.init_node('dual_arm_labeling_fsm')
    log_path = _setup_file_logger()
    log.info("[Main] Node started. Log: %s", log_path)

    threading.Thread(target=_shutdown_listener, daemon=True).start()
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

    # Run both arms concurrently; the system finishes only when both reach 'finished'
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
        log.info("[Main] Executing state machine")
        outcome = top_sm.execute()
        log.info("[Main] Finished with outcome: %s", outcome)
    except rospy.ROSInterruptException:
        log.warning("[Main] ROSInterruptException caught")
    finally:
        log.warning("[Main] Shutting down — turning off vacuums")
        log.info("[Main] Total items placed at output: %d", _items_placed_output)
        call_trigger_service('/orio/pnp_cup/off')
        call_trigger_service('/orio/lbl_cup/off')
        if 'sis' in locals():
            sis.stop()
        rospy.signal_shutdown('Process terminated.')


if __name__ == '__main__':
    main()
