#!/usr/bin/env python3
"""Single-arm DexNet pick-and-place — no labelling.

A stripped-down version of state_machine.py for testing DexNet on the robot. It
uses only the pick-and-place arm (robot_num=1): scan the bin, ask DexNet for a
suction grasp, pick the item, and drop it in DROP_ZONE (the output box). No
label arm, no label zones, no smach, no GUI.

The reusable primitives (IK, vacuum services, pose file, safety bounds) are the
same ones state_machine.py uses, so behaviour matches the real pick path:

    grasp source : /compute_grasps (Trigger) → first pose on /grasp_poses
    arm          : FrankaArm(robot_num=1)
    vacuum       : /orio/pnp_cup/on|off  (+ orio/vacuum/pnp_has_item feedback)
    drop target  : pick_place_arm['DROP_ZONE'] joints from joint_angles.json

By default the arm honours DexNet's grasp orientation (tilted approach along the
grasp normal), refusing picks steeper than --max-tilt-deg. Pass --straight-down
to ignore the orientation and always approach vertically (the classical path).

Runs INSIDE the main docker container, like state_machine.py:
    docker exec -it <container> bash -c \
      'source /home/ros_ws/devel/setup.bash \
       && cd /home/ros_ws/src/devel_packages/orio && python3 dexnet_pnp.py'

Prereqs: roscore, robot-1 control PC, cameras, the DexNet planner container, and
the perception node with grasp_backend=dexnet publishing to /grasp_poses. The
orio_dexnet_pnp tmux layout brings all of these up. Use --no-vacuum for a dry
run without pneumatics (move the cup by hand).
"""
import argparse
import json
import os
import sys

import ikpy.chain
import numpy as np
import rospy
from frankapy import FrankaArm
from geometry_msgs.msg import PoseArray
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

# ── Constants (match state_machine.py) ──────────────────────────────────────
APPROACH_DISTANCE = 0.1                 # metres, world +Z offset contact → pre
POSES_FILE = "joint_angles.json"
TARGET_POSES_FILE = "Target_Task_Poses.json"  # OUTPUT1/2, L2_INTER (Cartesian + rotation)
URDF_FILE = "panda_arm_hand.urdf"

# Where items go. state_machine.py's PlaceOutput drops at OUTPUT1/OUTPUT2 (over the output
# box), reaching in by item_depth + this margin — NOT the DROP_ZONE joint pose, which sits
# at the box edge. We use OUTPUT1 via IK for the same reason.
OUTPUT_POSE = "OUTPUT1"
OUTPUT_TRANSIT_POSE = "L2_INTER"        # intermediate waypoint before the drop
OUTPUT_DEPTH_MARGIN = 0.15              # m added to item depth so the cup clears the rim

# Straight-down tool orientation, same matrix state_machine.py's IK defaults to.
VERTICAL_ORI = np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]])

# Same safe workspace box FetchInput enforces on the incoming grasp pose (world).
PICK_BOUNDS = {'x_min': -0.2, 'x_max': 0.55, 'y_min': -0.8, 'y_max': -0.1,
               'z_min': 0.0, 'z_max': 0.5}

# Set from --no-vacuum / ORIO_DISABLE_PNEUMATICS: skip vacuum calls + sensor checks.
disable_pneumatics = False
# When True, poll the vacuum sensor after grasp/release (needs the pneumatic node).
test_vacuum = False


def _is_vacuum_service(name):
    return '/pnp_cup/' in name or '/lbl_cup/' in name


def call_trigger_service(service_name, timeout=5.0):
    """Call a std_srvs/Trigger service. Vacuum services are no-ops (return True)
    when pneumatics are disabled; vision services are unaffected."""
    if disable_pneumatics and _is_vacuum_service(service_name):
        rospy.loginfo("[Vacuum] disabled — skipping %s", service_name)
        return True
    try:
        rospy.wait_for_service(service_name, timeout=timeout)
        proxy = rospy.ServiceProxy(service_name, Trigger)
        response = proxy()
        rospy.loginfo("[Service] %s → success=%s msg='%s'", service_name,
                      response.success, getattr(response, 'message', ''))
        return response.success
    except Exception as exc:  # noqa: BLE001 — a failed service must not crash the loop
        rospy.logerr("[Service] Call failed for %s: %s", service_name, exc)
        return False


class DexnetPickPlace:
    """Single-arm pick-and-place hardware wrapper + control loop."""

    def __init__(self, use_grasp_orientation=True, max_tilt_deg=45.0,
                 motion_timeout_margin=8.0):
        self.use_grasp_orientation = use_grasp_orientation
        self.max_tilt_deg = float(max_tilt_deg)
        # Extra seconds beyond a motion's `duration` before _move() gives up (covers
        # planning/settling); a dead control PC then errors instead of hanging forever.
        self.motion_timeout_margin = float(motion_timeout_margin)

        rospy.loginfo("[Hardware] Initialising pick-and-place arm (robot_num=1)")
        # init_node=False: main() already called rospy.init_node('dexnet_pnp'). Letting
        # FrankaArm init the node too raises "init_node called with different arguments".
        self.arm = FrankaArm(with_gripper=False, old_gripper=False, robot_num=1,
                             init_node=False)
        self._move(self.arm.reset_joints, desc="initial reset_joints")

        # IK chain (same Panda URDF + active-link mask as state_machine.py).
        self.ik_chain = ikpy.chain.Chain.from_urdf_file(
            URDF_FILE, base_elements=["panda_link0"])
        self.chain_length = len(self.ik_chain.links)
        self.ik_chain.active_links_mask = (
            [False] + [True] * 7 + [False] * (self.chain_length - 8))

        # Output-box drop poses (OUTPUT1/2, L2_INTER) from Target_Task_Poses.json —
        # Cartesian + rotation, resolved to joints via IK with a depth offset so the arm
        # reaches INTO the box (see pose_to_joints / place_at_output). This matches
        # state_machine.py's PlaceOutput; the old DROP_ZONE joint pose dropped at the rim.
        with open(TARGET_POSES_FILE) as f:
            self.target_poses = json.load(f)
        for tag in (OUTPUT_POSE, OUTPUT_TRANSIT_POSE):
            if tag not in self.target_poses:
                raise KeyError("%s missing from %s (have: %s)"
                               % (tag, TARGET_POSES_FILE, list(self.target_poses)))
        rospy.loginfo("[Hardware] Output drop at '%s' (transit via '%s')",
                      OUTPUT_POSE, OUTPUT_TRANSIT_POSE)

        # Grasp poses from perception (DexNet backend publishes here).
        self.latest_pose_msg = None
        rospy.Subscriber("/grasp_poses", PoseArray, self._pose_cb, queue_size=1)

        # Vacuum sensor feedback from the pneumatic node.
        self.pnp_has_item = False
        rospy.Subscriber("orio/vacuum/pnp_has_item", Bool,
                         lambda m: setattr(self, 'pnp_has_item', m.data),
                         queue_size=1)
        rospy.loginfo("[Hardware] Initialisation complete")

    def _pose_cb(self, msg):
        self.latest_pose_msg = msg

    # ── Motion with a timeout ─────────────────────────────────────────────────
    def _move(self, motion_fn, *args, timeout=None, desc="motion", **kwargs):
        """Run a blocking FrankaArm motion (reset_joints/goto_joints) but with a HARD
        timeout, so a dead control PC can't hang us forever.

        frankapy's wait_for_skill() busy-loops with no timeout: if franka-interface stops
        executing (e.g. control PC crash), a blocking motion never returns. We instead
        issue it non-blocking and poll is_skill_done() against a deadline, raising a clear
        error on expiry. `duration` (the motion's own time budget) sets a sensible default
        timeout so slow moves are not cut off.
        """
        duration = kwargs.get("duration", 5)
        if timeout is None:
            timeout = float(duration) + self.motion_timeout_margin
        kwargs["block"] = False
        motion_fn(*args, **kwargs)   # issue asynchronously
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        rate = rospy.Rate(20)
        while not self.arm.is_skill_done():
            if rospy.Time.now() > deadline:
                raise RuntimeError(
                    "%s did not complete in %.1fs — the robot is not executing "
                    "(control PC crashed / FCI not active / e-stop?). Aborting the pick."
                    % (desc, timeout))
            if rospy.is_shutdown():
                raise KeyboardInterrupt
            rate.sleep()

    # ── Vacuum verification (mirrors state_machine.py's assert_vacuum) ────────
    def assert_vacuum(self, expected_state, timeout=1.0):
        """Poll the pnp vacuum sensor until it matches expected_state; raise if
        it doesn't within timeout. No-op unless test_vacuum and pneumatics on."""
        if disable_pneumatics or not test_vacuum:
            return
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        while rospy.Time.now() < deadline:
            if self.pnp_has_item == expected_state:
                return
            rospy.sleep(0.05)
        action = "pick up" if expected_state else "release"
        raise RuntimeError("PNP vacuum failed to %s item (expected=%s, actual=%s)"
                           % (action, expected_state, self.pnp_has_item))

    # ── IK (mirrors state_machine.py.compute_pick_joints) ─────────────────────
    def compute_pick_joints(self, task_pos, target_ori=None):
        """(pre_joints, final_joints) for a pick at task_pos with orientation
        target_ori (defaults straight down). Waypoints are offset along the tool
        approach axis so the cup travels down its own axis, not diagonally."""
        target_ori = VERTICAL_ORI if target_ori is None else np.asarray(target_ori)
        # Tool Z is the approach direction; retreat backs off along its negation.
        retreat = -target_ori[:, 2]
        task_pos = np.asarray(task_pos, dtype=float)
        pre_pos = list(task_pos + retreat * APPROACH_DISTANCE)
        final_pos = list(task_pos)

        initial_guess = [0.0] * self.chain_length
        initial_guess[4] = -1.5   # elbow config that keeps the tool pointing down
        pre_angles = self.ik_chain.inverse_kinematics(
            target_position=pre_pos, target_orientation=target_ori,
            orientation_mode="all", initial_position=initial_guess)
        final_angles = self.ik_chain.inverse_kinematics(
            target_position=final_pos, target_orientation=target_ori,
            orientation_mode="all", initial_position=pre_angles)

        # FK residual validation, same threshold as state_machine.py.
        for name, angles, pos in (("pre", pre_angles, pre_pos),
                                  ("final", final_angles, final_pos)):
            fk = self.ik_chain.forward_kinematics(list(angles))
            err = np.linalg.norm(fk[:3, 3] - np.asarray(pos))
            if err > 0.02:
                raise RuntimeError("IK %s residual %.4f m > 0.02 m for target %s"
                                   % (name, err, np.round(task_pos, 4)))
        return pre_angles[1:8], final_angles[1:8]

    def pose_to_joints(self, tag, depth_offset=0.0):
        """Resolve a Target_Task_Poses.json tag (OUTPUT1, L2_INTER, …) to joint angles
        via IK. depth_offset is added to the z translation before IK, so a positive value
        reaches DOWN into the box. Ported from state_machine.py.pose_to_joints."""
        entry = self.target_poses[tag]
        tx, ty, tz = entry["translation"]
        target_pos = [tx, ty, tz + depth_offset]
        target_rot = np.array(entry["rotation"])
        initial_guess = [0.0] * self.chain_length
        initial_guess[4] = -1.5
        angles = self.ik_chain.inverse_kinematics(
            target_position=target_pos, target_orientation=target_rot,
            orientation_mode="all", initial_position=initial_guess)
        return angles[1:8]

    # ── Grasp acquisition ─────────────────────────────────────────────────────
    def request_grasp(self, wait_s=5.0):
        """Trigger vision and return (task_pos, target_ori) for the best grasp,
        or None if the bin is empty / no valid grasp. target_ori is None when
        approaching straight down."""
        self.latest_pose_msg = None
        if not call_trigger_service('/compute_grasps'):
            rospy.logwarn("[Grasp] /compute_grasps failed")
            return None

        deadline = rospy.Time.now() + rospy.Duration(wait_s)
        while self.latest_pose_msg is None and rospy.Time.now() < deadline:
            rospy.sleep(0.1)

        if self.latest_pose_msg is None or not self.latest_pose_msg.poses:
            rospy.loginfo("[Grasp] No grasp pose returned (bin empty?)")
            return None

        p = self.latest_pose_msg.poses[0]
        task_pos = [p.position.x, p.position.y, p.position.z]
        if any(np.isnan(v) for v in task_pos):
            rospy.logwarn("[Grasp] NaN in grasp pose %s", task_pos)
            return None

        x, y, z = task_pos
        if not (PICK_BOUNDS['x_min'] <= x <= PICK_BOUNDS['x_max'] and
                PICK_BOUNDS['y_min'] <= y <= PICK_BOUNDS['y_max'] and
                PICK_BOUNDS['z_min'] <= z <= PICK_BOUNDS['z_max']):
            rospy.logwarn("[Grasp] pose (%.3f, %.3f, %.3f) outside safe bounds %s",
                          x, y, z, PICK_BOUNDS)
            return None

        target_ori = None
        if self.use_grasp_orientation:
            q = [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]
            if abs(np.linalg.norm(q) - 1.0) > 1e-3:
                rospy.logwarn("[Grasp] quaternion not unit (%.4f); approaching "
                              "vertically instead", np.linalg.norm(q))
            else:
                ori = R.from_quat(q).as_matrix()
                tilt = np.degrees(np.arccos(np.clip(-ori[2, 2], -1.0, 1.0)))
                if tilt > self.max_tilt_deg:
                    rospy.logwarn("[Grasp] tilt %.1f deg > max_tilt_deg %.1f; "
                                  "refusing pick", tilt, self.max_tilt_deg)
                    return None
                rospy.loginfo("[Grasp] using grasp orientation, tilt %.1f deg", tilt)
                target_ori = ori
        rospy.loginfo("[Grasp] pick at (%.3f, %.3f, %.3f)", x, y, z)
        return task_pos, target_ori

    # ── One pick-and-place cycle ──────────────────────────────────────────────
    def pick_and_place(self, task_pos, target_ori):
        pre_joints, final_joints = self.compute_pick_joints(task_pos, target_ori)
        # Item depth (world z of the grasp) sets how far to reach into the output box.
        item_depth = float(task_pos[2])

        rospy.loginfo("[Pick] Moving to pre-pick")
        self._move(self.arm.goto_joints, pre_joints, duration=3, desc="goto pre-pick")
        rospy.loginfo("[Pick] Descending to contact")
        self._move(self.arm.goto_joints, final_joints, duration=2, desc="descend to contact")

        call_trigger_service('/orio/pnp_cup/on')
        rospy.sleep(1.0)

        rospy.loginfo("[Pick] Retracting to pre-pick height")
        self._move(self.arm.goto_joints, pre_joints, duration=2, desc="retract")
        self.assert_vacuum(True)

        # Transit via HOME so the arm lifts to a known safe posture before crossing to the
        # drop, rather than sweeping directly from the bin.
        rospy.loginfo("[Place] Transiting via HOME")
        self._move(self.arm.reset_joints, duration=3, desc="transit to HOME (carrying item)")
        self.assert_vacuum(True)

        # Drop at the OUTPUT box (over it, reaching in by item_depth + margin), via the
        # L2_INTER waypoint — same as state_machine.py's PlaceOutput. Not the DROP_ZONE
        # joint pose, which sat at the box edge.
        rospy.loginfo("[Place] Approaching output via %s", OUTPUT_TRANSIT_POSE)
        int_joints = self.pose_to_joints(OUTPUT_TRANSIT_POSE, depth_offset=item_depth)
        self._move(self.arm.goto_joints, int_joints, duration=3,
                   desc="goto %s" % OUTPUT_TRANSIT_POSE)

        rospy.loginfo("[Place] Moving to %s (into the box)", OUTPUT_POSE)
        drop_joints = self.pose_to_joints(
            OUTPUT_POSE, depth_offset=item_depth + OUTPUT_DEPTH_MARGIN)
        self._move(self.arm.goto_joints, drop_joints, duration=3,
                   desc="goto %s" % OUTPUT_POSE)
        self.assert_vacuum(True)
        call_trigger_service('/orio/pnp_cup/off')
        rospy.sleep(1.0)
        self.assert_vacuum(False)

        rospy.loginfo("[Place] Item dropped; resetting to home")
        self._move(self.arm.reset_joints, duration=2, desc="reset to home")

    # ── Continuous loop ───────────────────────────────────────────────────────
    def run(self, confirm=False, max_declines=5):
        """Pick-and-place until the planner declines max_declines times in a row.

        A single decline (low q, no pose, out of bounds) does not end the run: the bin
        may just need a re-scan (objects shift, a better frame helps). Only after
        max_declines consecutive declines do we conclude the bin is empty / no reliable
        grasp. A successful pick resets the counter.
        """
        rospy.loginfo("[Loop] Starting DexNet pick-and-place loop (Ctrl-C to stop)")
        picked = 0
        declines = 0
        while not rospy.is_shutdown():
            self._move(self.arm.reset_joints, desc="reset before scan")
            grasp = self.request_grasp()
            if grasp is None:
                declines += 1
                if declines >= max_declines:
                    rospy.loginfo("[Loop] Planner declined %d times in a row — bin "
                                  "empty or no reliable grasp. Stopping. Total placed: %d",
                                  declines, picked)
                    break
                rospy.logwarn("[Loop] Grasp declined (%d/%d) — re-scanning…",
                              declines, max_declines)
                rospy.sleep(0.5)
                continue
            declines = 0  # got a valid grasp; reset the streak
            task_pos, target_ori = grasp

            if confirm:
                try:
                    input("[Loop] Press Enter to execute this pick "
                          "(Ctrl-C to abort)... ")
                except (EOFError, KeyboardInterrupt):
                    rospy.loginfo("[Loop] Aborted by operator")
                    break

            try:
                self.pick_and_place(task_pos, target_ori)
                picked += 1
                rospy.loginfo("[Loop] Placed item #%d", picked)
            except Exception as exc:  # noqa: BLE001 — log, drop cup, keep looping
                rospy.logerr("[Loop] Pick/place failed: %s", exc)
                call_trigger_service('/orio/pnp_cup/off')
                rospy.loginfo("[Loop] Continuing after failure")
        return picked


def _resolve_disable_pneumatics(args):
    env = os.environ.get('ORIO_DISABLE_PNEUMATICS', '').strip().lower() in (
        '1', 'true', 'yes', 'on')
    param = bool(rospy.get_param('~disable_pneumatics', False))
    return args.no_vacuum or env or param


def main():
    global disable_pneumatics, test_vacuum

    parser = argparse.ArgumentParser(
        description="Single-arm DexNet pick-and-place (no labelling).")
    parser.add_argument('--no-vacuum', '--disable-pneumatics', dest='no_vacuum',
                        action='store_true',
                        help="Dry-run without pneumatics (skip vacuum + checks).")
    parser.add_argument('--straight-down', action='store_true',
                        help="Ignore the DexNet grasp orientation and always "
                             "approach vertically.")
    parser.add_argument('--max-tilt-deg', type=float, default=45.0,
                        help="Refuse grasps whose approach tilt exceeds this "
                             "(only when honouring grasp orientation).")
    parser.add_argument('--check-vacuum', action='store_true',
                        help="Poll the vacuum sensor after grasp/release and "
                             "fail the pick if it disagrees.")
    parser.add_argument('--confirm', action='store_true',
                        help="Wait for Enter before each pick (safe for bring-up).")
    parser.add_argument('--max-declines', type=int, default=5,
                        help="Consecutive planner declines (low q / no grasp) before the "
                             "loop concludes the bin is empty and stops.")
    parser.add_argument('--check-only', action='store_true',
                        help="Only check that robot 1 (franka-interface) is ready, then "
                             "exit 0 if ready / non-zero if not. No motion. Used as a "
                             "pre-flight so the heavy stack is not started for a locked robot.")
    args, _ = parser.parse_known_args(rospy.myargv(argv=sys.argv)[1:])

    rospy.init_node('dexnet_pnp')

    if args.check_only:
        # Constructing FrankaArm runs frankapy's wait_for_franka_interface(), which
        # raises FrankaArmCommException after ~10s if the robot is not ready (locked /
        # not activated / control PC down). No reset, no motion.
        try:
            FrankaArm(with_gripper=False, old_gripper=False, robot_num=1, init_node=False)
            rospy.loginfo("[Check] Robot 1 franka-interface is READY.")
            return 0
        except Exception as exc:  # noqa: BLE001 — report cleanly for the pre-flight
            rospy.logerr("[Check] Robot 1 NOT ready: %s", exc)
            return 1

    disable_pneumatics = _resolve_disable_pneumatics(args)
    test_vacuum = args.check_vacuum and not disable_pneumatics
    if disable_pneumatics:
        rospy.logwarn("[Main] PNEUMATICS DISABLED — vacuum is a no-op (dry run). "
                      "Move the cup by hand.")

    controller = DexnetPickPlace(
        use_grasp_orientation=not args.straight_down,
        max_tilt_deg=args.max_tilt_deg)

    try:
        placed = controller.run(confirm=args.confirm, max_declines=args.max_declines)
        rospy.loginfo("[Main] Finished. Total items placed: %d", placed)
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        rospy.logwarn("[Main] Interrupt — stopping")
    finally:
        rospy.logwarn("[Main] Turning off pnp vacuum")
        call_trigger_service('/orio/pnp_cup/off')


if __name__ == '__main__':
    sys.exit(main() or 0)
