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
from std_srvs.srv import Trigger
import sys, os
import logging
import datetime
import random
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'prm'))
from franka_prm_single_arm import SingleArmExecutor


# ── Constants ──────────────────────────────────────────────────────────────────
APPROACH_DISTANCE = 0.20
PICK_UP_ZONE_GROUND_Z = 0.09
LABEL_ZONE_GROUND_Z = 0.055

POSES_FILE       = "joint_angles.json"
TARGET_POSES_FILE = "Target_Task_Poses.json"
URDF_FILE        = "panda_arm_hand.urdf"
PRM_SPLINE_SPEED = 0.3

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

# ==============================================================================
# SECTION 1: SYSTEM SETUP & DATA MANAGEMENT
# ==============================================================================

class RobotHardware:
    """Wrapper holding Franka interface, IK, Vision, and poses."""
    def __init__(self):
        log.info("[Hardware] Initializing Robot Hardware...")
        # self.pick_and_place_arm = FrankaArm(with_gripper=False, old_gripper=False, robot_num=1, init_node=False)
        # self.label_arm  = FrankaArm(with_gripper=False, old_gripper=False, robot_num=2, init_node=False)
        # log.info("[Hardware] Resetting both arms to home joints")
        # self.pick_and_place_arm.reset_joints()
        # self.label_arm.reset_joints()

        # PRM executors (reuse existing FrankaArm connections)
        self.ppa_executor = SingleArmExecutor(arm_number=1, init_node=False)
        self.la_executor  = SingleArmExecutor(arm_number=2, init_node=False)

        log.info("[Hardware] Resetting both arms to home joints")
        self.ppa_executor.fa.reset_joints()
        self.la_executor.fa.reset_joints()

        # # Redirect executors to share the already-connected FrankaArm handles
        # self.ppa_executor.fa = self.pick_and_place_arm
        # self.la_executor.fa  = self.label_arm

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

        log.info("[Hardware] Initialization complete")

    def _pose_callback(self, msg):
        self.latest_pose_msg = msg

    def _label_pose_callback_z1(self, msg):
        self.latest_label_pose_z1 = msg

    def _label_pose_callback_z2(self, msg):
        self.latest_label_pose_z2 = msg

    def compute_pick_joints(self, task_pos,
                            target_ori=np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]]),
                            executor=None, label_zone=None, max_attempts=10, yaw = None):
        pre_pos   = [task_pos[0], task_pos[1], task_pos[2] + APPROACH_DISTANCE]
        final_pos = [task_pos[0], task_pos[1], task_pos[2]]

        for attempt in range(max_attempts):
            initial_guess = [0.0] * self.chain_length
            # joint[4] always set to -1.5 (gripper pointing down); all others randomized on retries
            initial_guess[4] = -1.5
            if attempt > 0:
                # qmin = np.array([-1.57, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973])
                # qmax = np.array([ 1.57,  1.7628,  2.8973, -0.0698,  2.8973,  3.7525,  2.8973])
                window = 0.3  # radians around the original zero seed
                for i in range(self.chain_length):
                    if i == 4:
                        continue
                    # chain index i corresponds to joint i-1 (index 0 is base link)
                    # ji = i - 1
                    # if 0 <= ji < len(qmin):
                    #     lo, hi = qmin[ji], qmax[ji]
                    # else:
                    #     lo, hi = -np.pi, np.pi
                    # initial_guess[i] = random.uniform(lo, hi)
                    initial_guess[i] = random.uniform(-window, window)

            pre_angles = self.ik_chain.inverse_kinematics(
                target_position=pre_pos, target_orientation=target_ori,
                orientation_mode="all", initial_position=initial_guess)
            final_angles = self.ik_chain.inverse_kinematics(
                target_position=final_pos, target_orientation=target_ori,
                orientation_mode="all", initial_position=pre_angles)

            if executor is not None:
                pre_collision   = executor.check_collision(pre_angles[1:8],   label_zone=label_zone)
                final_collision = executor.check_collision(final_angles[1:8], label_zone=label_zone)
                if pre_collision or final_collision:
                    log.warning("[IK] attempt %d/%d in collision (pre=%s final=%s), retrying with new seed",
                                attempt + 1, max_attempts, pre_collision, final_collision)
                    continue

                log.info("[IK] Arm%d compute_pick_joints attempt %d/%d OK — target=(%s) pre=%s final=%s",
                     executor.arm_number, attempt + 1, max_attempts,
                     np.round(task_pos, 4),
                     np.round(pre_angles[1:8], 4),
                     np.round(final_angles[1:8], 4))
            if yaw is not None:
                pre_angles[7] = yaw
                final_angles[7] = yaw
            
            return pre_angles[1:8], final_angles[1:8]

        raise RuntimeError(
            f"[IK] Failed to find a collision-free configuration for target={np.round(task_pos, 4)} "
            f"after {max_attempts} attempts"
        )

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
        log.info("[compute_label_joints] Requesting label pose for zone %d, item_depth=%.4f",
                      zone_number, item_depth)

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
        task_pos = [p.position.x, p.position.y, item_depth]
        task_ori = [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]
        task_ori_Mat = R.from_quat(task_ori).as_matrix()
        rpy = R.from_quat(task_ori).as_euler('xyz', degrees=True)
        yaw = R.from_matrix(task_ori_Mat).as_euler('xyz')[2]
        log.info("[compute_label_joints] Zone %d received pose: x=%.4f y=%.4f z=%.4f  RPY r=%.2f p=%.2f y=%.2f deg  yaw=%.4f rad",
                      zone_number,
                      task_pos[0], task_pos[1], task_pos[2],
                      rpy[0], rpy[1], rpy[2], yaw)

        return self.compute_pick_joints(task_pos,
                                        executor=self.la_executor, label_zone=zone_number, yaw=1.0997)  # TEST: Change yaw = yaw


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
                log.warning(
                    "_prm_move: q_goal EE is NOT pointing straight down "
                    "(tilt=%.1f deg, ee_z=[%.3f, %.3f, %.3f])",
                    tilt_deg, *ee_z_axis
                )
            else:
                log.info(
                    "_prm_move: q_goal EE orientation OK (tilt=%.1f deg from straight-down)",
                    tilt_deg
                )

            log.info("[_prm_move] Arm%d label_zone=%s traj=%s speed=%s q_goal=%s",
                          executor.arm_number, label_zone, traj_type, spline_speed,
                          np.round(q_goal, 4))

            q_init = executor.fa.get_joints()
            log.info("[_prm_move] Arm%d q_init=%s", executor.arm_number, np.round(q_init, 4))

            plan = executor._plan_from_joints(q_init, q_goal, label_zone)
            if plan is None:
                _rpy_goal = R.from_matrix(fk[:3, :3]).as_euler('xyz', degrees=True)
                log.error(
                    "[_prm_move] Arm%d PRM planning failed  label_zone=%s  traj=%s  speed=%s\n"
                    "  q_goal: %s\n"
                    "  FK goal  xyz=(%.4f, %.4f, %.4f)  rpy=(%.2f, %.2f, %.2f) deg",
                    executor.arm_number, label_zone, traj_type, spline_speed,
                    np.round(q_goal, 4),
                    fk[0,3], fk[1,3], fk[2,3], _rpy_goal[0], _rpy_goal[1], _rpy_goal[2],
                )
                raise RuntimeError("Arm%d: PRM planning failed (label_zone=%s)" % (executor.arm_number, label_zone))
            log.info("[_prm_move] Arm%d plan has %d waypoints", executor.arm_number, len(plan))

            # executor._execute_plan(np.array(plan))
            executor._execute_plan_waypoints(np.array(plan))
            executor.fa.wait_for_skill()
            q_actual = executor.fa.get_joints()
            q_err = q_actual - q_goal
            full_q = [0.0] * self.chain_length
            full_q[1:8] = list(q_actual)
            fk = self.ik_chain.forward_kinematics(full_q)
            ee_z = fk[:3, 2]
            tilt_deg = np.degrees(np.arccos(np.clip(-ee_z[2], -1.0, 1.0)))
            log.info("[_prm_move] Arm%d done  q_err=%s (max=%.4f rad)  EE tilt=%.1f deg",
                          executor.arm_number,
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

    def set_state(self, zone, new_state):
        """Set zone state and log the transition."""
        old_state = self.states[zone]
        self.states[zone] = new_state
        log.info("[ZoneManager] Zone %s: %s → %s", zone, old_state, new_state)

# Globals
manager = ZoneManager()
hardware = None  # Will be initialized in main()

# Tracks where Robot 1 (pick-and-place arm) currently is.
# Values: 'LABEL_ZONE_1', 'LABEL_ZONE_2', 'NONE'
robot1_position = 'NONE'
# Tracks which state Robot 1 came from, to disambiguate PRM file selection.
# Values: 'DROP_TO_ZONE', 'PLACE_OUTPUT', None
robot1_prev_state = 'PLACE_OUTPUT'

# Cycle counter for throughput tracking
_items_placed_output = 0

_ZONE_TO_INT = {'LABEL_ZONE_1': 1, 'LABEL_ZONE_2': 2}

def _set_robot1_state(new_position, new_prev_state):
    """Update robot1 globals and log the transition."""
    global robot1_position, robot1_prev_state
    log.info("[Robot1State] position: %s → %s  prev_state: %s → %s",
                  robot1_position, new_position, robot1_prev_state, new_prev_state)
    robot1_position = new_position
    robot1_prev_state = new_prev_state

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
# SECTION 2: ARM 1 (LOGISTICS SM) STATES
# ==============================================================================

class TaskSelector(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['do_load', 'do_unload', 'wait'])

    def execute(self, userdata):
        log.debug("[TaskSelector] Evaluating zone states: %s  busy: %s",
                       manager.states, manager.busy)
        with manager.lock:
            for z in ['A', 'B']:
                if manager.states[z] == 'READY_FOR_PICKUP' and not manager.busy[z]:
                    log.info("[TaskSelector] → do_unload (zone %s ready)", z)
                    return 'do_unload'
            for z in ['A', 'B']:
                if manager.states[z] == 'EMPTY' and not manager.busy[z]:
                    log.info("[TaskSelector] → do_load (zone %s empty)", z)
                    return 'do_load'
        log.debug("[TaskSelector] → wait (no actionable zones)")
        rospy.sleep(0.5)
        return 'wait'

class FetchInput(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'], output_keys=['item_depth'])

    def execute(self, userdata):
        log.info("[FetchInput] Entering state")
        hardware.latest_pose_msg = None

        if not call_trigger_service('/compute_grasps'):
            log.error("[FetchInput] /compute_grasps service failed → exiting: failed")
            return 'failed'

        deadline = rospy.Time.now() + rospy.Duration(5.0)
        while hardware.latest_pose_msg is None and rospy.Time.now() < deadline:
            rospy.sleep(0.1)

        if hardware.latest_pose_msg is None or not hardware.latest_pose_msg.poses:
            log.error("[FetchInput] No grasp pose received within timeout → exiting: failed")
            return 'failed'

        p = hardware.latest_pose_msg.poses[0]
        task_pos = [p.position.x, p.position.y, p.position.z]
        hardware.current_fetch_depth = task_pos[2]
        userdata.item_depth = hardware.current_fetch_depth
        log.info("[FetchInput] Grasp pose received: x=%.4f y=%.4f z=%.4f",
                      task_pos[0], task_pos[1], task_pos[2])

        hardware.pick_up_pre_joints, hardware.pick_up_final_joints = hardware.compute_pick_joints(
            task_pos, executor=None)
        log.info("[FetchInput] IK pre_joints:   %s", np.round(hardware.pick_up_pre_joints, 4))
        log.info("[FetchInput] IK final_joints: %s", np.round(hardware.pick_up_final_joints, 4))

        prm_zone = _robot1_prm_file('FETCH_INPUT')
        log.info("[FetchInput] Moving to pre-pick via PRM (zone=%s)", prm_zone)
        hardware._prm_move(hardware.ppa_executor, hardware.pick_up_pre_joints, prm_zone, traj_type='spline', spline_speed=0.3)
        # hardware.ppa_executor.fa.goto_joints(hardware.pick_up_pre_joints, duration=1)
        _set_robot1_state('NONE', robot1_prev_state)

        log.info("[FetchInput] Descending to pick")
        hardware.ppa_executor.fa.goto_joints(hardware.pick_up_final_joints, duration=5)

        call_trigger_service('/orio/pnp_cup/on')
        rospy.sleep(1.0)

        log.info("[FetchInput] Retracting to pre-pick height")
        hardware.ppa_executor.fa.goto_joints(hardware.pick_up_pre_joints, duration=2)

        log.info("[FetchInput] Exiting: succeeded (item_depth=%.4f)", hardware.current_fetch_depth)
        return 'succeeded'

class DropToZone(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['dropped', 'wait'], input_keys=['item_depth'])

    def execute(self, userdata):
        log.info("[DropToZone] Entering state (item_depth=%.4f)", userdata.item_depth)
        target_zone = None
        with manager.lock:
            for z in ['A', 'B']:
                if manager.states[z] == 'EMPTY' and not manager.busy[z]:
                    target_zone = z
                    manager.busy[z] = True
                    break

        if target_zone:
            log.info("[DropToZone] Targeting zone %s", target_zone)
            # Transfer depth to manager so Arm 2 knows it
            manager.depths[target_zone] = userdata.item_depth

            if (target_zone == 'A'):
                pre_place_joints = hardware.pose_to_joints("LABEL_ZONE_1", depth_offset=manager.depths[target_zone] + 0.01 + 0.1787)
                place_joints = hardware.pose_to_joints("LABEL_ZONE_1", depth_offset=manager.depths[target_zone] + 0.01)
                log.info("[DropToZone] place_joints for LABEL_ZONE_1: %s", np.round(pre_place_joints, 4))
                _set_robot1_state('LABEL_ZONE_1', robot1_prev_state)
                hardware._prm_move(hardware.ppa_executor, pre_place_joints, _robot1_prm_file('DROP_TO_ZONE', 'LABEL_ZONE_1'), traj_type='spline', spline_speed=0.2)
                hardware.ppa_executor.fa.goto_joints(place_joints, duration=2)
                call_trigger_service('/orio/pnp_cup/off')
                hardware.ppa_executor.fa.goto_joints(pre_place_joints, duration=2)
            else:
                pre_place_joints = hardware.pose_to_joints("LABEL_ZONE_2", depth_offset=manager.depths[target_zone] + 0.01 + 0.1787)
                place_joints = hardware.pose_to_joints("LABEL_ZONE_2", depth_offset=manager.depths[target_zone] + 0.01)
                log.info("[DropToZone] place_joints for LABEL_ZONE_2: %s", np.round(pre_place_joints, 4))
                _set_robot1_state('LABEL_ZONE_2', robot1_prev_state)
                hardware._prm_move(hardware.ppa_executor, pre_place_joints, _robot1_prm_file('DROP_TO_ZONE', 'LABEL_ZONE_2'), traj_type='spline', spline_speed=0.2)
                hardware.ppa_executor.fa.goto_joints(place_joints, duration=2)
                call_trigger_service('/orio/pnp_cup/off')
                hardware.ppa_executor.fa.goto_joints(pre_place_joints, duration=2)
                
            _set_robot1_state(robot1_position, 'DROP_TO_ZONE')
            hardware.ppa_executor.fa.reset_joints()
            with manager.lock:
                manager.set_state(target_zone, 'NEEDS_LABEL')
                manager.busy[target_zone] = False
            log.info("[DropToZone] Exiting: dropped (zone %s)", target_zone)
            return 'dropped'

        log.debug("[DropToZone] No empty zone available → wait")
        rospy.sleep(0.5)
        return 'wait'

class RetrieveFromZone(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['picked', 'no_items'], output_keys=['retrieved_depth'])

    def execute(self, userdata):
        log.info("[RetrieveFromZone] Entering state")
        target_zone = None
        with manager.lock:
            for z in ['A', 'B']:
                if manager.states[z] == 'READY_FOR_PICKUP' and not manager.busy[z]:
                    target_zone = z
                    manager.busy[z] = True
                    break

        if target_zone:
            depth = manager.depths[target_zone]
            userdata.retrieved_depth = depth
            log.info("[RetrieveFromZone] Picking from zone %s (depth=%.4f)", target_zone, depth)

            if (target_zone == 'A'):
                pre_pick_joints = hardware.pose_to_joints("LABEL_ZONE_1", depth_offset=depth + APPROACH_DISTANCE)
                pick_joints = hardware.pose_to_joints("LABEL_ZONE_1", depth_offset=depth - 0.01) # Added small offset (0.01) for interference
                log.info("[RetrieveFromZone] A pre_pick=%s pick=%s",
                              np.round(pre_pick_joints, 4), np.round(pick_joints, 4))
                _set_robot1_state('LABEL_ZONE_1', robot1_prev_state)
                hardware._prm_move(hardware.ppa_executor, pre_pick_joints, _robot1_prm_file('RETRIEVE_FROM_ZONE', 'LABEL_ZONE_1'), traj_type='spline', spline_speed=0.2)
                hardware.ppa_executor.fa.goto_joints(pick_joints, duration=2)
                # hardware._prm_move(hardware.ppa_executor, pick_joints, _robot1_prm_file('RETRIEVE_FROM_ZONE', 'LABEL_ZONE_1'), traj_type='spline', spline_speed=0.2)
            else:
                pre_pick_joints = hardware.pose_to_joints("LABEL_ZONE_2", depth_offset=depth + APPROACH_DISTANCE)
                pick_joints = hardware.pose_to_joints("LABEL_ZONE_2", depth_offset=depth - 0.01) # Added small offset (0.01) for interference
                log.info("[RetrieveFromZone] B pre_pick=%s pick=%s",
                              np.round(pre_pick_joints, 4), np.round(pick_joints, 4))
                _set_robot1_state('LABEL_ZONE_2', robot1_prev_state)
                hardware._prm_move(hardware.ppa_executor, pre_pick_joints, _robot1_prm_file('RETRIEVE_FROM_ZONE', 'LABEL_ZONE_2'), traj_type='spline', spline_speed=0.2)
                hardware.ppa_executor.fa.goto_joints(pick_joints, duration=2)
                # hardware._prm_move(hardware.ppa_executor, pick_joints, _robot1_prm_file('RETRIEVE_FROM_ZONE', 'LABEL_ZONE_2'), traj_type='spline', spline_speed=0.2)

            call_trigger_service('/orio/pnp_cup/on')
            rospy.sleep(1.0)

            log.info("[RetrieveFromZone] Retracting to pre-pick height")
            hardware.ppa_executor.fa.goto_joints(pre_pick_joints, duration=2)

            with manager.lock:
                manager.set_state(target_zone, 'EMPTY')
                manager.busy[target_zone] = False
            log.info("[RetrieveFromZone] Exiting: picked (zone %s)", target_zone)
            return 'picked'

        log.debug("[RetrieveFromZone] No ready zones → no_items")
        return 'no_items'

class PlaceOutput(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'], input_keys=['retrieved_depth'])

    def execute(self, userdata):
        global _items_placed_output
        log.info("[PlaceOutput] Entering state (retrieved_depth=%.4f)", userdata.retrieved_depth)
        item_depth = userdata.retrieved_depth
        drop_joints = hardware.pose_to_joints("OUTPUT", depth_offset=item_depth + 0.1)
        log.info("[PlaceOutput] drop_joints: %s", np.round(drop_joints, 4))

        prm_zone = _robot1_prm_file('PLACE_OUTPUT')
        hardware._prm_move(hardware.ppa_executor, drop_joints, prm_zone, traj_type='spline', spline_speed=0.2)
        _set_robot1_state('NONE', 'PLACE_OUTPUT')
        call_trigger_service('/orio/pnp_cup/off')

        _items_placed_output += 1
        log.info("[PlaceOutput] Exiting: succeeded (total items placed: %d)", _items_placed_output)
        return 'succeeded'

# ==============================================================================
# SECTION 3: ARM 2 (LABELING SM) STATES
# ==============================================================================

class GetLabel(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

    def execute(self, userdata):
        log.info("[GetLabel] Entering state")
        hardware.la_executor.fa.goto_joints(hardware.label_arm_dispenser_pre, duration=5)
        hardware.la_executor.fa.goto_joints(hardware.label_arm_dispenser_contact, duration=5)
        call_trigger_service('/orio/lbl_cup/on')
        hardware.la_executor.fa.goto_joints(hardware.label_arm_dispenser_pre, duration=2)
        safe_joints = hardware.pose_to_joints("LABELLING_SAFE")
        hardware.la_executor.fa.goto_joints(safe_joints, duration=3)
        hardware.la_executor.fa.wait_for_skill()
        log.info("[GetLabel] Exiting: succeeded")
        return 'succeeded'

class ApplyLabel(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['labeled', 'wait'])

    def execute(self, userdata):
        log.info("[ApplyLabel] Entering state (zones: %s  busy: %s)",
                      manager.states, manager.busy)
        target_zone = None
        with manager.lock:
            for z in ['A', 'B']:
                if manager.states[z] == 'NEEDS_LABEL' and not manager.busy[z]:
                    target_zone = z
                    manager.busy[z] = True
                    break

        if target_zone:
            zone_number = 1 if target_zone == 'A' else 2
            log.info("[ApplyLabel] Applying label to zone %s (zone_number=%d, depth=%.4f)",
                          target_zone, zone_number, manager.depths[target_zone])
            pre_joints, final_joints = hardware.compute_label_joints(zone_number, manager.depths[target_zone])
            log.info("[ApplyLabel] IK pre_joints=%s  final_joints=%s",
                          np.round(pre_joints, 4), np.round(final_joints, 4))

            hardware._prm_move(hardware.la_executor, pre_joints, zone_number, traj_type='spline', spline_speed=0.2)
            hardware._prm_move(hardware.la_executor, final_joints, zone_number, traj_type='spline', spline_speed=0.2)
            call_trigger_service('/orio/lbl_cup/off')
            rospy.sleep(1.0)
            hardware._prm_move(hardware.la_executor, pre_joints, zone_number, traj_type='spline', spline_speed=0.2)

            safe_joints = hardware.pose_to_joints("LABELLING_SAFE")
            hardware._prm_move(hardware.la_executor, safe_joints, zone_number, traj_type='spline', spline_speed=0.2)

            with manager.lock:
                manager.set_state(target_zone, 'READY_FOR_PICKUP')
                manager.busy[target_zone] = False
            log.info("[ApplyLabel] Exiting: labeled (zone %s)", target_zone)
            return 'labeled'

        log.debug("[ApplyLabel] No zone needs label → wait")
        rospy.sleep(0.5)
        return 'wait'

# ==============================================================================
# SECTION 4: MAIN EXECUTION FUNCTION
# ==============================================================================

def main():
    global hardware
    rospy.init_node('dual_arm_labeling_fsm')
    log_path = _setup_file_logger()
    log.info("[Main] Node started. Log file: %s", log_path)

    random.seed(1)

    # Initialize hardware once node is running
    hardware = RobotHardware()

    sm_logistics = smach.StateMachine(outcomes=['finished'])
    with sm_logistics:
        smach.StateMachine.add('DECIDE_HUB', TaskSelector(),
                               transitions={'do_load':'FETCH_INPUT', 'do_unload':'RETRIEVE', 'wait':'DECIDE_HUB'})

        smach.StateMachine.add('FETCH_INPUT', FetchInput(),
                               transitions={'succeeded':'DROP_TO_ZONE', 'failed':'DECIDE_HUB'})
        smach.StateMachine.add('DROP_TO_ZONE', DropToZone(),
                               transitions={'dropped':'DECIDE_HUB', 'wait':'DROP_TO_ZONE'})

        smach.StateMachine.add('RETRIEVE', RetrieveFromZone(),
                               transitions={'picked':'PLACE_OUTPUT', 'no_items':'DECIDE_HUB'})
        smach.StateMachine.add('PLACE_OUTPUT', PlaceOutput(),
                               transitions={'succeeded':'DECIDE_HUB'})

    sm_labeling = smach.StateMachine(outcomes=['finished'])
    with sm_labeling:
        smach.StateMachine.add('GET_LABEL', GetLabel(), transitions={'succeeded':'APPLY'})
        smach.StateMachine.add('APPLY', ApplyLabel(), transitions={'labeled':'GET_LABEL', 'wait':'APPLY'})

    top_sm = smach.Concurrence(
        outcomes=['done', 'error'],
        default_outcome='error',
        outcome_map={'done': {'ARM1':'finished', 'ARM2':'finished'}}
    )

    with top_sm:
        smach.Concurrence.add('ARM1', sm_logistics)
        smach.Concurrence.add('ARM2', sm_labeling)

    try:
        # Introspection server for rqt_smach visualization
        sis = smach_ros.IntrospectionServer('orio_visualiser', top_sm, '/ORIO_ROOT')
        sis.start()
        log.info("[Main] Starting state machine execution")
        outcome = top_sm.execute()
        log.info("[Main] State machine finished with outcome: %s", outcome)
    except rospy.ROSInterruptException:
        log.warning("[Main] ROSInterruptException caught")
    finally:
        log.warning("[Main] Shutting down: turning off both vacuums")
        log.info("[Main] Total items placed at output: %d", _items_placed_output)
        call_trigger_service('/orio/pnp_cup/off')
        call_trigger_service('/orio/lbl_cup/off')
        if 'sis' in locals():
            sis.stop()
        rospy.signal_shutdown('Process terminated.')

if __name__ == '__main__':
    main()
