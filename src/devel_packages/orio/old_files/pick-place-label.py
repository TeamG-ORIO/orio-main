#!/usr/bin/env python3

import rospy
import json
import ikpy.chain
import numpy as np
from frankapy import FrankaArm
from autolab_core import RigidTransform
from geometry_msgs.msg import PoseArray
from std_srvs.srv import Trigger

# ── Constants ──────────────────────────────────────────────────────────────────
APPROACH_DISTANCE = 0.05   # metres — vertical (world +Z) offset from contact to PRE position
POSES_FILE = "joint_angles.json"
URDF_FILE  = "panda_arm_hand.urdf"


# ── Helpers ────────────────────────────────────────────────────────────────────

def call_trigger_service(service_name):
    """Call a ROS Trigger service. Returns True on success."""
    try:
        rospy.wait_for_service(service_name, timeout=10.0)
        proxy = rospy.ServiceProxy(service_name, Trigger)
        response = proxy()
        return response.success
    except rospy.ROSException:
        return False
    except Exception as e:
        rospy.logerr(f"Service call failed: {e}")
        return False


def compute_pick_joints(ik_chain, chain_length, task_pos):
    """
    Compute (pre_joints, final_joints) for a vision-guided pick-up.
    Mirrors pnp_test_forever.py exactly: fixed end-effector orientation,
    initial_guess[4]=-1.5, two IK calls where the first result seeds the second.
    """
    target_ori = np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]])

    initial_guess = [0.0] * chain_length
    if 4 < chain_length:
        initial_guess[4] = -1.5

    pre_pos   = [task_pos[0], task_pos[1], task_pos[2] + 0.10]
    final_pos = [task_pos[0], task_pos[1], task_pos[2] + 0.05]

    pre_angles = ik_chain.inverse_kinematics(
        target_position=pre_pos,
        target_orientation=target_ori,
        orientation_mode="all",
        initial_position=initial_guess
    )
    final_angles = ik_chain.inverse_kinematics(
        target_position=final_pos,
        target_orientation=target_ori,
        orientation_mode="all",
        initial_position=pre_angles
    )
    return pre_angles[1:8], final_angles[1:8]


# ── Controller ─────────────────────────────────────────────────────────────────

class PickPlaceLabelController:
    def __init__(self):
        # Arms
        self.ppa = FrankaArm(with_gripper=False, old_gripper=False, robot_num=1)  # pick_place_arm
        self.la  = FrankaArm(with_gripper=False, old_gripper=False, robot_num=2, init_node=False)  # label_arm

        # IK chain (both arms share the same Panda URDF kinematics)
        self.ik_chain = ikpy.chain.Chain.from_urdf_file(
            URDF_FILE, base_elements=["panda_link0"]
        )
        chain_length = len(self.ik_chain.links)
        active_mask = [False] * chain_length
        for i in range(1, 8):
            if i < chain_length:
                active_mask[i] = True
        self.ik_chain.active_links_mask = active_mask
        self.chain_length = chain_length

        # Cartesian approach / retract deltas (world frame, ±Z)
        self.approach = RigidTransform(
            translation=[0.0, 0.0, -APPROACH_DISTANCE],
            from_frame='world', to_frame='world'
        )
        self.retract = RigidTransform(
            translation=[0.0, 0.0,  APPROACH_DISTANCE],
            from_frame='world', to_frame='world'
        )

        # Load PRE joint angles directly from JSON (no IK needed for stored zones)
        with open(POSES_FILE) as f:
            raw = json.load(f)

        ppa = raw['pick_place_arm']
        la  = raw['label_arm']
        self.pre = {
            'ppa_DROP_ZONE':      ppa['DROP_ZONE'],
            'ppa_LABEL_ZONE1':    ppa['LABEL_ZONE1'],
            'ppa_LABEL_ZONE2':    ppa['LABEL_ZONE2'],
            'la_LABEL_DISPENSER': la['LABEL_DISPENSER'],
        }
        self.safe_pos = raw['SAFE_POS']

        # LABEL_DISPENSER stores the contact position; precompute its PRE joints once
        # so _init_pick_label and phases 2 & 4 all reuse the same value.
        self.la_dispenser_pre_joints = self._contact_to_pre_joints(self.pre['la_LABEL_DISPENSER'])

        # Vision subscriber (PICK_UP_ZONE comes from vision each cycle)
        self.latest_pose_msg = None
        rospy.Subscriber("/grasp_poses", PoseArray, self._pose_callback, queue_size=1)
        self.pick_up_pre_joints   = None
        self.pick_up_final_joints = None

        # Vision subscriber (label zones come from vision before each label placement)
        self.latest_label_pose_msg = None
        rospy.Subscriber("/label_grasp_poses", PoseArray, self._label_pose_callback, queue_size=1)
        self.la_label_pre_joints   = None
        self.la_label_final_joints = None

    # ── Vision ─────────────────────────────────────────────────────────────────

    def _pose_callback(self, msg):
        self.latest_pose_msg = msg

    def _get_pick_up_pre_joints(self):
        """
        Reset to home, trigger vision, wait for a grasp pose, and compute
        pre + final joint angles for PICK_UP_ZONE (mirrors pnp_test_forever.py).
        """
        rospy.loginfo("Moving to home position for scanning...")
        self.ppa.reset_joints()

        self.latest_pose_msg = None
        if not call_trigger_service('/compute_grasps'):
            rospy.logwarn("compute_grasps service call failed")
            return False

        deadline = rospy.Time.now() + rospy.Duration(5.0)
        while self.latest_pose_msg is None and rospy.Time.now() < deadline:
            rospy.sleep(0.1)

        if self.latest_pose_msg is None:
            rospy.logerr("Vision succeeded but no poses were caught by the subscriber.")
            return False

        if not self.latest_pose_msg.poses:
            rospy.logwarn("Received empty PoseArray despite vision success.")
            return False

        rospy.loginfo("Object pose received! Computing pick joints...")
        p = self.latest_pose_msg.poses[0]
        task_pos = [p.position.x, p.position.y, p.position.z]
        self.pick_up_pre_joints, self.pick_up_final_joints = compute_pick_joints(
            self.ik_chain, self.chain_length, task_pos
        )
        return True

    # ── Parallel motion helpers ────────────────────────────────────────────────

    def _par_goto_joints(self, j_ppa, j_la):
        """Move both arms to their respective joint targets simultaneously."""
        self.ppa.goto_joints(j_ppa, block=False)
        self.la.goto_joints(j_la,  block=False)
        self.ppa.wait_for_skill()
        self.la.wait_for_skill()

    def _par_delta(self, delta):
        """Apply the same Cartesian delta to both arms simultaneously."""
        self.ppa.goto_pose_delta(delta, block=False)
        self.la.goto_pose_delta(delta,  block=False)
        self.ppa.wait_for_skill()
        self.la.wait_for_skill()

    # ── Initialisation ─────────────────────────────────────────────────────────

    def _contact_to_pre_joints(self, contact_joints):
        """
        Given joint angles at the contact (grasp/pick/place) position,
        use FK to get the Cartesian pose, then IK to solve PRE joint angles
        offset APPROACH_DISTANCE above in world +Z.
        """
        full = [0.0] * self.chain_length
        full[1:8] = contact_joints
        tf = self.ik_chain.forward_kinematics(full)
        contact_pos = tf[:3, 3]
        contact_ori = tf[:3, :3]

        pre_pos = contact_pos + np.array([0.0, 0.0, APPROACH_DISTANCE])
        initial_guess = [0.0] * self.chain_length
        if 4 < self.chain_length:
            initial_guess[4] = -1.5
        pre_angles = self.ik_chain.inverse_kinematics(
            target_position=pre_pos,
            target_orientation=contact_ori,
            orientation_mode="all",
            initial_position=initial_guess
        )
        return pre_angles[1:8]

    def _init_pick_label(self):
        """label_arm picks the first label from LABEL_DISPENSER before cycles begin."""
        rospy.loginfo("Init: label_arm picking first label from LABEL_DISPENSER")
        self.la.goto_joints(self.la_dispenser_pre_joints)
        self.la.goto_joints(self.pre['la_LABEL_DISPENSER'])
        call_trigger_service('/snaak/lbl_cup/on')
        self.la.goto_joints(self.la_dispenser_pre_joints)
    

    # ── Phases ─────────────────────────────────────────────────────────────────

    def _phase1(self):
        """
        pick_place_arm : pick_up(PICK_UP_ZONE)  →  drop_off(LABEL_ZONE2)
        label_arm      : place_label(LABEL_ZONE1)  →  SAFE_POS
        """
        rospy.loginfo("--- Phase 1 start ---")
        pre = self.pre

        # Sub-task step 1 (parallel): both navigate to first pre-position
        # ppa → PICK_UP_ZONE pre (z+0.10)  |  la → LABEL_ZONE1 pre
        self._par_goto_joints(self.pick_up_pre_joints, pre['la_LABEL_ZONE1'])

        # Sub-task step 2 (parallel): ppa descends via IK to final (z+0.05), la via delta
        self.ppa.goto_joints(self.pick_up_final_joints, block=False)
        self.la.goto_pose_delta(self.approach, block=False)
        self.ppa.wait_for_skill()
        self.la.wait_for_skill()

        # Sub-task step 3: vacuum actions
        call_trigger_service('/snaak/pnp_cup/on')   # ppa: grasp object
        rospy.sleep(1.0)
        call_trigger_service('/snaak/lbl_cup/off')  # la:  release (place) label

        # Sub-task step 4 (parallel): both retract
        self._par_delta(self.retract)

        # Sub-task step 5 (parallel): ppa → LABEL_ZONE2 pre  |  la → SAFE_POS
        self._par_goto_joints(pre['ppa_LABEL_ZONE2'], self.safe_pos)
        # label_arm is now done with Phase 1

        # Sub-task step 6–8 (ppa solo): approach → release → retract at LABEL_ZONE2
        self.ppa.goto_pose_delta(self.approach)
        call_trigger_service('/snaak/pnp_cup/off')  # ppa: release object at LABEL_ZONE2
        self.ppa.goto_pose_delta(self.retract)

        rospy.loginfo("--- Phase 1 complete ---")

    def _phase2(self):
        """
        pick_place_arm : pick_up(LABEL_ZONE1)  →  drop_off(DROP_ZONE)
        label_arm      : pick_label(LABEL_DISPENSER)
        """
        rospy.loginfo("--- Phase 2 start ---")
        pre = self.pre

        # Sub-task step 1 (parallel): ppa → LABEL_ZONE1 pre  |  la → LABEL_DISPENSER pre
        self._par_goto_joints(pre['ppa_LABEL_ZONE1'], self.la_dispenser_pre_joints)

        # Sub-task step 2 (parallel): both descend
        self._par_delta(self.approach)

        # Sub-task step 3: vacuum actions
        call_trigger_service('/snaak/pnp_cup/on')   # ppa: grasp labeled object
        call_trigger_service('/snaak/lbl_cup/on')   # la:  pick next label

        # Sub-task step 4 (parallel): both retract
        self._par_delta(self.retract)
        # label_arm is now done with Phase 2 (holds the next label)

        # Sub-task step 5–8 (ppa solo): navigate → approach → release → retract at DROP_ZONE
        self.ppa.goto_joints(pre['ppa_DROP_ZONE'])
        self.ppa.goto_pose_delta(self.approach)
        call_trigger_service('/snaak/pnp_cup/off')  # ppa: release labeled object at output
        self.ppa.goto_pose_delta(self.retract)

        rospy.loginfo("--- Phase 2 complete ---")

    def _phase3(self):
        """
        pick_place_arm : pick_up(PICK_UP_ZONE)  →  drop_off(LABEL_ZONE1)
        label_arm      : place_label(LABEL_ZONE2)  →  SAFE_POS
        """
        rospy.loginfo("--- Phase 3 start ---")
        pre = self.pre

        # Sub-task step 1 (parallel): ppa → PICK_UP_ZONE pre (z+0.10)  |  la → LABEL_ZONE2 pre
        self._par_goto_joints(self.pick_up_pre_joints, pre['la_LABEL_ZONE2'])

        # Sub-task step 2 (parallel): ppa descends via IK to final (z+0.05), la via delta
        self.ppa.goto_joints(self.pick_up_final_joints, block=False)
        self.la.goto_pose_delta(self.approach, block=False)
        self.ppa.wait_for_skill()
        self.la.wait_for_skill()

        # Sub-task step 3: vacuum actions
        call_trigger_service('/snaak/pnp_cup/on')   # ppa: grasp object
        rospy.sleep(1.0)
        call_trigger_service('/snaak/lbl_cup/off')  # la:  release (place) label

        # Sub-task step 4 (parallel): both retract
        self._par_delta(self.retract)

        # Sub-task step 5 (parallel): ppa → LABEL_ZONE1 pre  |  la → SAFE_POS
        self._par_goto_joints(pre['ppa_LABEL_ZONE1'], self.safe_pos)
        # label_arm is now done with Phase 3

        # Sub-task step 6–8 (ppa solo): approach → release → retract at LABEL_ZONE1
        self.ppa.goto_pose_delta(self.approach)
        call_trigger_service('/snaak/pnp_cup/off')  # ppa: release object at LABEL_ZONE1
        self.ppa.goto_pose_delta(self.retract)

        rospy.loginfo("--- Phase 3 complete ---")

    def _phase4(self):
        """
        pick_place_arm : pick_up(LABEL_ZONE2)  →  drop_off(DROP_ZONE)
        label_arm      : pick_label(LABEL_DISPENSER)
        """
        rospy.loginfo("--- Phase 4 start ---")
        pre = self.pre

        # Sub-task step 1 (parallel): ppa → LABEL_ZONE2 pre  |  la → LABEL_DISPENSER pre
        self._par_goto_joints(pre['ppa_LABEL_ZONE2'], self.la_dispenser_pre_joints)

        # Sub-task step 2 (parallel): both descend
        self._par_delta(self.approach)

        # Sub-task step 3: vacuum actions
        call_trigger_service('/snaak/pnp_cup/on')   # ppa: grasp labeled object
        call_trigger_service('/snaak/lbl_cup/on')   # la:  pick next label

        # Sub-task step 4 (parallel): both retract
        self._par_delta(self.retract)
        # label_arm is now done with Phase 4 (holds the next label for the next cycle)

        # Sub-task step 5–8 (ppa solo): navigate → approach → release → retract at DROP_ZONE
        self.ppa.goto_joints(pre['ppa_DROP_ZONE'])
        self.ppa.goto_pose_delta(self.approach)
        call_trigger_service('/snaak/pnp_cup/off')  # ppa: release labeled object at output
        self.ppa.goto_pose_delta(self.retract)

        rospy.loginfo("--- Phase 4 complete ---")

    # ── Main loop ──────────────────────────────────────────────────────────────

    def run(self):
        # label_arm picks its first label before the cycle loop starts
        self._init_pick_label()

        while not rospy.is_shutdown():
            # Vision query #1 — for phase 1 pick-up
            rospy.loginfo("=== Querying PICK_UP_ZONE (for phase 1) ===")
            if not self._get_pick_up_pre_joints():
                rospy.logwarn("Could not acquire pick-up pose. Retrying in 3 s...")
                rospy.sleep(3.0)
                continue

            self._phase1()
            self._phase2()

            # Vision query #2 — fresh pose for phase 3 pick-up
            rospy.loginfo("=== Querying PICK_UP_ZONE (for phase 3) ===")
            if not self._get_pick_up_pre_joints():
                rospy.logwarn("Could not acquire pick-up pose for phase 3. Retrying in 3 s...")
                rospy.sleep(3.0)
                continue

            self._phase3()
            self._phase4()

            rospy.loginfo("=== Cycle complete ===")


if __name__ == "__main__":
    controller = None
    try:
        controller = PickPlaceLabelController()
        controller.run()
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        pass
    except Exception as e:
        rospy.logerr(f"Unhandled exception: {e}")
    finally:
        rospy.logwarn("Shutting down: turning off both vacuums")
        call_trigger_service('/snaak/pnp_cup/off')
        call_trigger_service('/snaak/lbl_cup/off')
