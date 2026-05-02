#!/usr/bin/env python3

import rospy
import smach
import smach_ros
import json
import ikpy.chain
import numpy as np
import threading
import tkinter as tk
from tkinter import ttk, messagebox
from threading import Lock
from scipy.spatial.transform import Rotation as R
from frankapy import FrankaArm
from autolab_core import RigidTransform
from geometry_msgs.msg import PoseArray
from std_srvs.srv import Trigger
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'prm'))
from franka_prm_single_arm import SingleArmExecutor

# Global shutdown event — set this to trigger a clean exit
shutdown_event = threading.Event()

# When either arm needs human recovery, this event is set.
# The other arm's states check it and spin-wait before executing.
recovery_event = threading.Event()

# ── Constants ──────────────────────────────────────────────────────────────────
APPROACH_DISTANCE = 0.15
PICK_UP_ZONE_GROUND_Z = 0.09
LABEL_ZONE_GROUND_Z = 0.055

POSES_FILE       = "joint_angles.json"
TARGET_POSES_FILE = "Target_Task_Poses.json"
URDF_FILE        = "panda_arm_hand.urdf"
PRM_SPLINE_SPEED = 0.3

# All states reachable from the recovery GUI, per sub-state-machine.
ARM1_STATES = ['DECIDE_HUB', 'FETCH_INPUT', 'DROP_TO_ZONE', 'RETRIEVE', 'PLACE_OUTPUT']
ARM2_STATES = ['GET_LABEL', 'APPLY']

def call_trigger_service(service_name):
    """Helper function to call ROS Trigger services (vacuum OR vision)."""
    try:
        rospy.wait_for_service(service_name, timeout=5.0)
        proxy = rospy.ServiceProxy(service_name, Trigger)
        response = proxy()
        return response.success
    except Exception as e:
        rospy.logerr(f"Service call failed: {e}")
    return False

# ==============================================================================
# SECTION 1: SYSTEM SETUP & DATA MANAGEMENT
# ==============================================================================

class RobotHardware:
    """Wrapper holding Franka interface, IK, Vision, and poses."""
    def __init__(self):
        rospy.loginfo("Initializing Robot Hardware...")
        self.pick_and_place_arm = FrankaArm(with_gripper=False, old_gripper=False, robot_num=1, init_node=False)
        self.label_arm  = FrankaArm(with_gripper=False, old_gripper=False, robot_num=2, init_node=False)
        self.pick_and_place_arm.reset_joints()
        self.label_arm.reset_joints()

        # PRM executors (reuse existing FrankaArm connections)
        self.ppa_executor = SingleArmExecutor(arm_number=1, init_node=False)
        self.la_executor  = SingleArmExecutor(arm_number=2, init_node=False)
        # Redirect executors to share the already-connected FrankaArm handles
        self.ppa_executor.fa = self.pick_and_place_arm
        self.la_executor.fa  = self.label_arm

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

    def _pose_callback(self, msg):
        self.latest_pose_msg = msg

    def _label_pose_callback_z1(self, msg):
        self.latest_label_pose_z1 = msg

    def _label_pose_callback_z2(self, msg):
        self.latest_label_pose_z2 = msg

    def compute_pick_joints(self, task_pos, target_ori = np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]])):
        initial_guess = [0.0] * self.chain_length
        # 180° rotation around X, meaning the gripper points straight down
        if 4 < self.chain_length: initial_guess[4] = -1.5

        pre_pos   = [task_pos[0], task_pos[1], task_pos[2] + APPROACH_DISTANCE]
        final_pos = [task_pos[0], task_pos[1], task_pos[2]]

        pre_angles = self.ik_chain.inverse_kinematics(
            target_position=pre_pos, target_orientation=target_ori, orientation_mode="all", initial_position=initial_guess)
        final_angles = self.ik_chain.inverse_kinematics(
            target_position=final_pos, target_orientation=target_ori, orientation_mode="all", initial_position=pre_angles)

        return pre_angles[1:8], final_angles[1:8]

    def compute_label_joints(self, zone_number, item_depth):
        """Call /compute_grasps_labelling, wait for the pose on the matching topic,
        and return (pre_joints, final_joints) via IK — mirrors compute_pick_joints().

        Args:
            zone_number (int): 1 or 2, selects /grasp_poses_labelling_z1 or z2.
            item_depth (float): Z position of the item, used as the target z for IK.

        Returns:
            tuple: (pre_joints, final_joints) each a 7-element np.ndarray.

        Raises:
            RuntimeError: if the service fails or no pose is received in time.
        """
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
        print(f"Received labelling pose for zone {zone_number}: ({p.position.x}, {p.position.y}, {p.position.z})")
        task_pos = [p.position.x, p.position.y, item_depth]
        task_ori = [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]
        task_ori_Mat = R.from_quat(task_ori).as_matrix()
        rpy = R.from_quat(task_ori).as_euler('xyz', degrees=True)
        print(f"  Translation : x={task_pos[0]:.4f}  y={task_pos[1]:.4f}  z={task_pos[2]:.4f}")
        print(f"  Rotation RPY: r={rpy[0]:.2f}°  p={rpy[1]:.2f}°  y={rpy[2]:.2f}°")
        return self.compute_pick_joints(task_pos, task_ori_Mat)

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

    def _prm_move(self, executor, q_goal, label_zone=None,
                  traj_type=None, duration=None, spline_speed=None, totg_scale=None):
        """Plan with PRM from current joints to q_goal and stream trajectory.

        Optional overrides (restored after the move):
          traj_type    : 'ramp' | 'minjerk' | 'spline' | 'totg'
          duration     : motion time in seconds (minjerk / spline without spline_speed)
          spline_speed : path speed in rad/s (spline mode only)
          totg_scale   : speed/accel fraction 0..1 (totg mode only)
        """
        overrides = {k: v for k, v in {
            'traj_type': traj_type, 'duration': duration,
            'spline_speed': spline_speed, 'totg_scale': totg_scale,
        }.items() if v is not None}
        saved = {k: getattr(executor, k) for k in overrides}
        for k, v in overrides.items():
            setattr(executor, k, v)
        try:
            # FK sanity check: verify q_goal results in a straight-down EE orientation.
            # The EE Z-axis (approach direction) is the 3rd column of the rotation matrix;
            # for straight-down it should be [0, 0, -1] in the base frame.
            full_q = [0.0] * self.chain_length
            full_q[1:8] = list(q_goal)
            fk = self.ik_chain.forward_kinematics(full_q)
            ee_z_axis = fk[:3, 2]  # 3rd column = EE Z (approach direction)
            expected = np.array([0.0, 0.0, -1.0])
            tilt_deg = np.degrees(np.arccos(np.clip(-ee_z_axis[2], -1.0, 1.0)))
            if tilt_deg > 5.0:
                rospy.logwarn(
                    "_prm_move: q_goal EE is NOT pointing straight down "
                    "(tilt=%.1f deg, ee_z=[%.3f, %.3f, %.3f])",
                    tilt_deg, *ee_z_axis
                )
            else:
                rospy.loginfo(
                    "_prm_move: q_goal EE orientation OK (tilt=%.1f deg from straight-down)",
                    tilt_deg
                )

            q_init = executor.fa.get_joints()
            plan = executor._plan_from_joints(q_init, q_goal, label_zone)
            if plan is None:
                raise RuntimeError(f"Arm{executor.arm_number}: PRM planning failed")
            executor._execute_plan(np.array(plan))
            executor.fa.wait_for_skill()
            q_actual = executor.fa.get_joints()
            q_err = q_actual - q_goal
            full_q = [0.0] * self.chain_length
            full_q[1:8] = list(q_actual)
            fk = self.ik_chain.forward_kinematics(full_q)
            ee_z = fk[:3, 2]
            tilt_deg = np.degrees(np.arccos(np.clip(-ee_z[2], -1.0, 1.0)))
            rospy.loginfo("_prm_move done  q_err=%s (max=%.4f rad)  EE tilt=%.1f deg",
                          np.round(q_err, 4), float(np.max(np.abs(q_err))), tilt_deg)
        finally:
            for k, v in saved.items():
                setattr(executor, k, v)

    def get_zone_approach_retract(self, pre_joints, depth):    # TODO: REMOVE EVENTUALLY
        full = [0.0] * self.chain_length
        full[1:8] = list(pre_joints)
        tf = self.ik_chain.forward_kinematics(full)
        pre_z = tf[2, 3]
        dist = pre_z - depth - LABEL_ZONE_GROUND_Z - 0.1
        dist = depth + 0.005
        approach = RigidTransform(translation=[0.0, 0.0, dist], from_frame='world', to_frame='world')
        retract  = RigidTransform(translation=[0.0, 0.0, dist], from_frame='world', to_frame='world')
        return approach, retract

class ZoneManager:
    def __init__(self):
        self.lock = Lock()
        self.states = {'A': 'EMPTY', 'B': 'EMPTY'}  # Initial states: EMPTY, NEEDS_LABEL, READY_FOR_PICKUP
        self.busy = {'A': False, 'B': False}        # Initial busy flags
        self.depths = {'A': 0.0, 'B': 0.0}          # Tracks object depth per zone

# Globals
manager = ZoneManager()
hardware = None  # Will be initialized in main()

# Tracks where Robot 1 (pick-and-place arm) currently is.
# Values: 'LABEL_ZONE_1', 'LABEL_ZONE_2', 'NONE'
robot1_position = 'NONE'
# Tracks which state Robot 1 came from, to disambiguate PRM file selection.
# Values: 'DROP_TO_ZONE', 'PLACE_OUTPUT', None
robot1_prev_state = 'PLACE_OUTPUT'

_ZONE_TO_INT = {'LABEL_ZONE_1': 1, 'LABEL_ZONE_2': 2}

def _robot1_prm_file(current_state, target_zone=None):
    """Return the PRM label_zone argument for Robot 1's next _prm_move.

    FetchInput:       from DROP_TO_ZONE  → source zone int (robot1_position)
                      from PLACE_OUTPUT  → None
    DropToZone:       always             → target_zone int
    RetrieveFromZone: from DROP_TO_ZONE  → 3
                      from PLACE_OUTPUT  → target_zone int
    PlaceOutput:      always             → source zone int (robot1_position)
    """
    if current_state == 'FETCH_INPUT':
        if robot1_prev_state == 'DROP_TO_ZONE':
            return _ZONE_TO_INT.get(robot1_position)
        return None

    if current_state == 'DROP_TO_ZONE':
        return _ZONE_TO_INT.get(target_zone)

    if current_state == 'RETRIEVE_FROM_ZONE':
        if robot1_prev_state == 'DROP_TO_ZONE':
            return 3  # 3 is free
        return _ZONE_TO_INT.get(target_zone)

    if current_state == 'PLACE_OUTPUT':
        return _ZONE_TO_INT.get(robot1_position)

    return None

# ==============================================================================
# SECTION 2: HUMAN RECOVERY GUI
# ==============================================================================

def _show_recovery_gui(arm_name, failed_state, error_msg, valid_states):
    """Block the calling thread and show a Tk dialog.

    Returns the state name chosen by the operator, or 'shutdown' if they
    pressed E-Stop / closed the window.
    """
    result = {'choice': 'shutdown'}

    def _run():
        root = tk.Tk()
        root.title(f"[ORIO] Human Recovery — {arm_name}")
        root.configure(bg='#1e1e2e')
        root.resizable(False, False)

        # ── Header ────────────────────────────────────────────────────────────
        header = tk.Frame(root, bg='#e55', pady=8)
        header.pack(fill='x')
        tk.Label(header, text="  HUMAN INTERVENTION REQUIRED",
                 font=('Helvetica', 14, 'bold'), fg='white', bg='#e55').pack()

        # ── Info ──────────────────────────────────────────────────────────────
        info = tk.Frame(root, bg='#1e1e2e', padx=16, pady=10)
        info.pack(fill='x')
        tk.Label(info, text=f"Arm:          {arm_name}",
                 font=('Courier', 11), fg='#cdd6f4', bg='#1e1e2e', anchor='w').pack(fill='x')
        tk.Label(info, text=f"Failed state: {failed_state}",
                 font=('Courier', 11), fg='#f38ba8', bg='#1e1e2e', anchor='w').pack(fill='x')
        tk.Label(info, text=f"Error:        {str(error_msg)[:120]}",
                 font=('Courier', 10), fg='#fab387', bg='#1e1e2e', anchor='w',
                 wraplength=480, justify='left').pack(fill='x', pady=(0, 6))

        # ── Zone state editor ─────────────────────────────────────────────────
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
                    manager.busy[zone] = zone_vars[f'{zone}_busy'].get()
            rospy.loginfo("[Recovery] Zone states updated: %s | busy: %s",
                          manager.states, manager.busy)

        tk.Button(zone_frame, text="Apply Zone Changes", command=apply_zone_changes,
                  bg='#313244', fg='#cdd6f4', relief='flat', padx=8).pack(pady=(6, 0))

        # ── State selector ────────────────────────────────────────────────────
        sel_frame = tk.LabelFrame(root, text=" Jump to State ",
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

        # ── Buttons ───────────────────────────────────────────────────────────
        btn_frame = tk.Frame(root, bg='#1e1e2e', pady=10)
        btn_frame.pack(fill='x', padx=16)

        def on_resume():
            apply_zone_changes()
            result['choice'] = choice_var.get()
            root.destroy()

        def on_estop():
            if messagebox.askyesno("E-Stop", "Emergency stop — shut down everything?",
                                   parent=root):
                result['choice'] = 'shutdown'
                root.destroy()

        tk.Button(btn_frame, text="Resume ->", command=on_resume,
                  bg='#a6e3a1', fg='#1e1e2e', font=('Helvetica', 12, 'bold'),
                  relief='flat', padx=16, pady=6).pack(side='left', padx=(0, 12))

        tk.Button(btn_frame, text="E-Stop / Shutdown", command=on_estop,
                  bg='#f38ba8', fg='#1e1e2e', font=('Helvetica', 12, 'bold'),
                  relief='flat', padx=16, pady=6).pack(side='left')

        root.protocol("WM_DELETE_WINDOW", on_estop)
        root.lift()
        root.focus_force()
        root.mainloop()

    gui_thread = threading.Thread(target=_run, daemon=True)
    gui_thread.start()
    gui_thread.join()   # block the smach thread until the operator responds
    return result['choice']


def _safe_execute(state_name, fn, userdata):
    """Run fn(userdata); on any unhandled exception store diagnostics and return 'failed'."""
    try:
        return fn(userdata)
    except Exception as exc:
        rospy.logerr("[%s] Unhandled exception: %s", state_name, exc)
        userdata.recovery_failed_state = state_name
        userdata.recovery_error_msg    = str(exc)
        return 'failed'


class HumanRecoveryArm1(smach.State):
    """Shown whenever an ARM1 state fails or raises an exception."""
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=ARM1_STATES + ['shutdown'],
                             input_keys=['recovery_failed_state', 'recovery_error_msg'])

    def execute(self, userdata):
        rospy.logwarn("[ARM1 Recovery] Entering human recovery GUI...")
        failed  = getattr(userdata, 'recovery_failed_state', 'UNKNOWN')
        err_msg = getattr(userdata, 'recovery_error_msg',   'No details available.')
        choice  = _show_recovery_gui("ARM1 (Pick-and-Place)", failed, err_msg, ARM1_STATES)
        rospy.loginfo("[ARM1 Recovery] Operator chose: %s", choice)
        if choice == 'shutdown':
            shutdown_event.set()
        return choice


class HumanRecoveryArm2(smach.State):
    """Shown whenever an ARM2 state fails or raises an exception."""
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=ARM2_STATES + ['shutdown'],
                             input_keys=['recovery_failed_state', 'recovery_error_msg'])

    def execute(self, userdata):
        rospy.logwarn("[ARM2 Recovery] Entering human recovery GUI...")
        failed  = getattr(userdata, 'recovery_failed_state', 'UNKNOWN')
        err_msg = getattr(userdata, 'recovery_error_msg',   'No details available.')
        choice  = _show_recovery_gui("ARM2 (Labeling)", failed, err_msg, ARM2_STATES)
        rospy.loginfo("[ARM2 Recovery] Operator chose: %s", choice)
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
        with manager.lock:
            for z in ['A', 'B']:
                if manager.states[z] == 'READY_FOR_PICKUP' and not manager.busy[z]:
                    return 'do_unload'
            for z in ['A', 'B']:
                if manager.states[z] == 'EMPTY' and not manager.busy[z]:
                    return 'do_load'
        rospy.sleep(0.5)
        return 'wait'

class FetchInput(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'],
                             output_keys=['item_depth', 'recovery_failed_state', 'recovery_error_msg'])

    def _run(self, userdata):
        hardware.latest_pose_msg = None

        if not call_trigger_service('/compute_grasps'):
            raise RuntimeError("Service /compute_grasps returned failure")

        deadline = rospy.Time.now() + rospy.Duration(5.0)
        while hardware.latest_pose_msg is None and rospy.Time.now() < deadline:
            rospy.sleep(0.1)

        if hardware.latest_pose_msg is None or not hardware.latest_pose_msg.poses:
            raise RuntimeError("No grasp pose received within timeout")

        p = hardware.latest_pose_msg.poses[0]
        task_pos = [p.position.x, p.position.y, p.position.z]
        hardware.current_fetch_depth = task_pos[2]
        userdata.item_depth = hardware.current_fetch_depth

        hardware.pick_up_pre_joints, hardware.pick_up_final_joints = hardware.compute_pick_joints(task_pos)

        global robot1_position, robot1_prev_state
        hardware._prm_move(hardware.ppa_executor, hardware.pick_up_pre_joints, _robot1_prm_file('FETCH_INPUT'), traj_type='spline', spline_speed=0.7)
        hardware.pick_and_place_arm.goto_joints(hardware.pick_up_pre_joints, duration=1)
        robot1_position = 'NONE'
        hardware.pick_and_place_arm.goto_joints(hardware.pick_up_final_joints, duration=5)

        call_trigger_service('/orio/pnp_cup/on')
        rospy.sleep(1.0)
        hardware.pick_and_place_arm.goto_joints(hardware.pick_up_pre_joints, duration=2)

        return 'succeeded'

    def execute(self, userdata):
        return _safe_execute('FETCH_INPUT', self._run, userdata)

class DropToZone(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['dropped', 'wait', 'failed'],
                             input_keys=['item_depth'],
                             output_keys=['recovery_failed_state', 'recovery_error_msg'])

    def _run(self, userdata):
        target_zone = None
        with manager.lock:
            for z in ['A', 'B']:
                if manager.states[z] == 'EMPTY' and not manager.busy[z]:
                    target_zone = z
                    manager.busy[z] = True
                    break

        if not target_zone:
            rospy.sleep(0.5)
            return 'wait'

        # Transfer depth to manager so Arm 2 knows it
        manager.depths[target_zone] = userdata.item_depth

        global robot1_position, robot1_prev_state
        if target_zone == 'A':
            place_joints = hardware.pose_to_joints("LABEL_ZONE_1", depth_offset=manager.depths[target_zone] + 0.01)
            robot1_position = 'LABEL_ZONE_1'
            hardware._prm_move(hardware.ppa_executor, place_joints, _robot1_prm_file('DROP_TO_ZONE', 'LABEL_ZONE_1'), traj_type='spline', spline_speed=0.2)
            call_trigger_service('/orio/pnp_cup/off')
        else:
            place_joints = hardware.pose_to_joints("LABEL_ZONE_2", depth_offset=manager.depths[target_zone] + 0.01)
            robot1_position = 'LABEL_ZONE_2'
            hardware._prm_move(hardware.ppa_executor, place_joints, _robot1_prm_file('DROP_TO_ZONE', 'LABEL_ZONE_2'), traj_type='spline', spline_speed=0.2)
            call_trigger_service('/orio/pnp_cup/off')
        robot1_prev_state = 'DROP_TO_ZONE'

        with manager.lock:
            manager.states[target_zone] = 'NEEDS_LABEL'
            manager.busy[target_zone] = False
        return 'dropped'

    def execute(self, userdata):
        return _safe_execute('DROP_TO_ZONE', self._run, userdata)

class RetrieveFromZone(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['picked', 'no_items', 'failed'],
                             output_keys=['retrieved_depth', 'recovery_failed_state', 'recovery_error_msg'])

    def _run(self, userdata):
        target_zone = None
        with manager.lock:
            for z in ['A', 'B']:
                if manager.states[z] == 'READY_FOR_PICKUP' and not manager.busy[z]:
                    target_zone = z
                    manager.busy[z] = True
                    break

        if not target_zone:
            return 'no_items'

        depth = manager.depths[target_zone]
        userdata.retrieved_depth = depth

        global robot1_position, robot1_prev_state
        if target_zone == 'A':
            pre_pick_joints = hardware.pose_to_joints("LABEL_ZONE_1", depth_offset=depth + APPROACH_DISTANCE)
            pick_joints = hardware.pose_to_joints("LABEL_ZONE_1", depth_offset=depth - 0.01) # Added small offset (0.01) for interference
            robot1_position = 'LABEL_ZONE_1'
            hardware._prm_move(hardware.ppa_executor, pre_pick_joints, _robot1_prm_file('RETRIEVE_FROM_ZONE', 'LABEL_ZONE_1'), traj_type='spline', spline_speed=0.2)
            hardware._prm_move(hardware.ppa_executor, pick_joints, _robot1_prm_file('RETRIEVE_FROM_ZONE', 'LABEL_ZONE_1'), traj_type='spline', spline_speed=0.2)
        else:
            pre_pick_joints = hardware.pose_to_joints("LABEL_ZONE_2", depth_offset=depth + APPROACH_DISTANCE)
            pick_joints = hardware.pose_to_joints("LABEL_ZONE_2", depth_offset=depth - 0.01) # Added small offset (0.01) for interference
            robot1_position = 'LABEL_ZONE_2'
            hardware._prm_move(hardware.ppa_executor, pre_pick_joints, _robot1_prm_file('RETRIEVE_FROM_ZONE', 'LABEL_ZONE_2'), traj_type='spline', spline_speed=0.2)
            hardware._prm_move(hardware.ppa_executor, pre_pick_joints, _robot1_prm_file('RETRIEVE_FROM_ZONE', 'LABEL_ZONE_2'), traj_type='spline', spline_speed=0.2)

        call_trigger_service('/orio/pnp_cup/on')
        rospy.sleep(1.0)

        hardware.pick_and_place_arm.goto_joints(pre_pick_joints, duration=2)

        with manager.lock:
            manager.states[target_zone] = 'EMPTY'
            manager.busy[target_zone] = False
        return 'picked'

    def execute(self, userdata):
        return _safe_execute('RETRIEVE_FROM_ZONE', self._run, userdata)

class PlaceOutput(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'],
                             input_keys=['retrieved_depth'],
                             output_keys=['recovery_failed_state', 'recovery_error_msg'])

    def _run(self, userdata):
        item_depth = userdata.retrieved_depth
        drop_joints = hardware.pose_to_joints("OUTPUT", depth_offset=item_depth + 0.1)

        global robot1_position, robot1_prev_state
        hardware._prm_move(hardware.ppa_executor, drop_joints, _robot1_prm_file('PLACE_OUTPUT'), traj_type='spline', spline_speed=0.2)
        robot1_position = 'NONE'
        robot1_prev_state = 'PLACE_OUTPUT'
        call_trigger_service('/orio/pnp_cup/off')

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
        hardware.label_arm.goto_joints(hardware.label_arm_dispenser_pre, duration=5)
        hardware.label_arm.goto_joints(hardware.label_arm_dispenser_contact, duration=5)
        call_trigger_service('/orio/lbl_cup/on')
        hardware.label_arm.goto_joints(hardware.label_arm_dispenser_pre, duration=2)
        safe_joints = hardware.pose_to_joints("LABELLING_SAFE")
        hardware.label_arm.goto_joints(safe_joints, duration=3)
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

        target_zone = None
        with manager.lock:
            for z in ['A', 'B']:
                if manager.states[z] == 'NEEDS_LABEL' and not manager.busy[z]:
                    target_zone = z
                    manager.busy[z] = True
                    break

        if not target_zone:
            rospy.sleep(0.5)
            return 'wait'

        zone_number = 1 if target_zone == 'A' else 2
        pre_joints, final_joints = hardware.compute_label_joints(zone_number, manager.depths[target_zone])

        hardware._prm_move(hardware.la_executor, pre_joints, zone_number, traj_type='spline', spline_speed=0.2)
        hardware._prm_move(hardware.la_executor, final_joints, zone_number, traj_type='spline', spline_speed=0.2)
        call_trigger_service('/orio/lbl_cup/off')
        rospy.sleep(1.0)
        hardware._prm_move(hardware.la_executor, pre_joints, zone_number, traj_type='spline', spline_speed=0.2)

        safe_joints = hardware.pose_to_joints("LABELLING_SAFE")
        hardware._prm_move(hardware.la_executor, safe_joints, zone_number, traj_type='spline', spline_speed=0.2)

        with manager.lock:
            manager.states[target_zone] = 'READY_FOR_PICKUP'
            manager.busy[target_zone] = False
        return 'labeled'

    def execute(self, userdata):
        return _safe_execute('APPLY_LABEL', self._run, userdata)

# ==============================================================================
# SECTION 5: MAIN EXECUTION FUNCTION
# ==============================================================================

def main():
    global hardware
    rospy.init_node('dual_arm_labeling_fsm')

    # Initialize hardware once node is running
    hardware = RobotHardware()

    # ── ARM 1 state machine ───────────────────────────────────────────────────
    sm_logistics = smach.StateMachine(outcomes=['finished', 'shutdown'])
    sm_logistics.userdata.recovery_failed_state = ''
    sm_logistics.userdata.recovery_error_msg    = ''
    sm_logistics.userdata.item_depth            = 0.0
    sm_logistics.userdata.retrieved_depth       = 0.0

    with sm_logistics:
        smach.StateMachine.add('DECIDE_HUB', TaskSelector(),
                               transitions={'do_load':   'FETCH_INPUT',
                                            'do_unload': 'RETRIEVE',
                                            'wait':      'DECIDE_HUB',
                                            'shutdown':  'shutdown'})

        smach.StateMachine.add('FETCH_INPUT', FetchInput(),
                               transitions={'succeeded': 'DROP_TO_ZONE',
                                            'failed':    'HUMAN_RECOVERY_ARM1'})

        smach.StateMachine.add('DROP_TO_ZONE', DropToZone(),
                               transitions={'dropped': 'DECIDE_HUB',
                                            'wait':    'DROP_TO_ZONE',
                                            'failed':  'HUMAN_RECOVERY_ARM1'})

        smach.StateMachine.add('RETRIEVE', RetrieveFromZone(),
                               transitions={'picked':   'PLACE_OUTPUT',
                                            'no_items': 'DECIDE_HUB',
                                            'failed':   'HUMAN_RECOVERY_ARM1'})

        smach.StateMachine.add('PLACE_OUTPUT', PlaceOutput(),
                               transitions={'succeeded': 'DECIDE_HUB',
                                            'failed':    'HUMAN_RECOVERY_ARM1'})

        # Recovery state — operator picks the next state via GUI
        smach.StateMachine.add('HUMAN_RECOVERY_ARM1', HumanRecoveryArm1(),
                               transitions={
                                   'DECIDE_HUB':   'DECIDE_HUB',
                                   'FETCH_INPUT':  'FETCH_INPUT',
                                   'DROP_TO_ZONE': 'DROP_TO_ZONE',
                                   'RETRIEVE':     'RETRIEVE',
                                   'PLACE_OUTPUT': 'PLACE_OUTPUT',
                                   'shutdown':     'shutdown',
                               })

    # ── ARM 2 state machine ───────────────────────────────────────────────────
    sm_labeling = smach.StateMachine(outcomes=['finished', 'shutdown'])
    sm_labeling.userdata.recovery_failed_state = ''
    sm_labeling.userdata.recovery_error_msg    = ''

    with sm_labeling:
        smach.StateMachine.add('GET_LABEL', GetLabel(),
                               transitions={'succeeded': 'APPLY',
                                            'failed':    'HUMAN_RECOVERY_ARM2'})

        smach.StateMachine.add('APPLY', ApplyLabel(),
                               transitions={'labeled':  'GET_LABEL',
                                            'wait':     'APPLY',
                                            'failed':   'HUMAN_RECOVERY_ARM2',
                                            'shutdown': 'shutdown'})

        smach.StateMachine.add('HUMAN_RECOVERY_ARM2', HumanRecoveryArm2(),
                               transitions={
                                   'GET_LABEL': 'GET_LABEL',
                                   'APPLY':     'APPLY',
                                   'shutdown':  'shutdown',
                               })

    # ── Top-level concurrent machine ──────────────────────────────────────────
    top_sm = smach.Concurrence(
        outcomes=['done', 'shutdown', 'error'],
        default_outcome='error',
        outcome_map={
            'done':     {'ARM1': 'finished', 'ARM2': 'finished'},
            'shutdown': {'ARM1': 'shutdown', 'ARM2': 'shutdown'},
        }
    )

    with top_sm:
        smach.Concurrence.add('ARM1', sm_logistics)
        smach.Concurrence.add('ARM2', sm_labeling)

    try:
        # Introspection server for rqt_smach visualization
        sis = smach_ros.IntrospectionServer('orio_visualiser', top_sm, '/ORIO_ROOT')
        sis.start()
        outcome = top_sm.execute()
    except rospy.ROSInterruptException:
        pass
    finally:
        rospy.logwarn("Shutting down: turning off both vacuums")
        call_trigger_service('/orio/pnp_cup/off')
        call_trigger_service('/orio/lbl_cup/off')
        if 'sis' in locals():
            sis.stop()
        rospy.signal_shutdown('Process terminated.')

if __name__ == '__main__':
    main()
