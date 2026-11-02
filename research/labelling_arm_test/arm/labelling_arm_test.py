#!/usr/bin/env python3
"""The labelling arm (robot 2, iam-luisa) on its own: take a label from the dispenser and
put it on the item in a label zone, at the height the camera measured.

This is the dual-arm state machine's GET_LABEL and APPLY states without robot 1. The label
pose comes from /labelling_test/compute_label_pose (research/labelling_arm_test/perception:
the demo's label placement plus Fast-FoundationStereo depth), as a full world-frame pose,
so no pick height is carried over from the other arm. Runs inside orio_docker_container;
run_labelling_test.sh brings everything up and starts this in the foreground.

    --no-vacuum        skip the suction services (the cup holds nothing: a dry run)
    --confirm/--auto   Enter before each placement (default) / no prompt
    --zone 1|2|auto    which zone to label (auto: whichever has a pose, zone 1 first)
    --skip-dispenser   do not fetch a label first (place with whatever is on the cup)
    --cycles N         stop after N labels (default: until 'q' at the prompt)
    --z-offset M       added to the measured label z, metres (e.g. -0.005: 5 mm higher)
    --no-motion        perception + IK only, print the pose, no robot (no control PC needed)
    --check-only       is robot 2 ready? exit 0/1, no motion
"""

import argparse
import json
import os
import random
import sys

import ikpy.chain
import numpy as np
import rospy
from geometry_msgs.msg import PoseArray
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

# The demo's arm package: URDF, pose files, obstacle files and the collision checker.
ORIO_DIR = os.environ.get('ORIO_ARM_PKG_DIR', '/home/ros_ws/src/devel_packages/orio')
sys.path.insert(0, ORIO_DIR)
import CollisionChecker  # noqa: E402

URDF_FILE = os.path.join(ORIO_DIR, "panda_arm_hand.urdf")
POSES_FILE = os.path.join(ORIO_DIR, "joint_angles.json")          # label_arm.LABEL_DISPENSER
TARGET_POSES_FILE = os.path.join(ORIO_DIR, "Target_Task_Poses.json")  # LABELLING_SAFE
OBSTACLES_FILE = os.path.join(ORIO_DIR, "obstacles_files", "obs_arm2_free.p")

# state_machine.py's values.
APPROACH_DISTANCE = 0.1
# The XY box ApplyLabel accepts a label pose in. The z band is new: z is the item's top in
# the robot frame, and items lie directly on the table, which the camera sees at z ~ 0.004,
# so a flat pouch comes out near 0 and a box at its height. Below the table is a depth
# error, and nothing we label is taller than ~0.35 m. Outside the band the pose is refused
# before any motion.
POSE_BOUNDS = {'x_min': 0.2, 'x_max': 0.7, 'y_min': -0.6, 'y_max': 0.6,
               'z_min': -0.01, 'z_max': 0.40}

# Retries re-seed IK with this much random change (rad) per joint around the nominal seed.
# state_machine.py re-seeds over the whole joint range instead, which can converge on a
# configuration that reaches the zone from behind (base turned ~170 deg); on this arm that
# motion swings out over the dispenser side of the cell.
SEED_JITTER = 0.6
# A solution with the base (joint 1) or upper arm (joint 3) turned further than this is
# that from-behind configuration, whatever seed produced it: refused.
MAX_BASE_TURN = 2.0
# A label is the same rectangle turned by 180 deg about the cup, so the published pose and
# the pose turned by this about the tool axis place it identically. Both are solved and
# the one needing the smaller joint motion from where the arm is wins; otherwise the IK
# has to honour whichever yaw perception happened to publish, which can mean turning the
# wrist 120 deg one way when 30 deg the other way would do.
TOOL_FLIP = np.diag([-1.0, -1.0, 1.0])

SERVICE = '/labelling_test/compute_label_pose'
ZONE_TOPICS = {1: '/labelling_test/label_pose_z1', 2: '/labelling_test/label_pose_z2'}

disable_pneumatics = False


class MotionFailed(RuntimeError):
    """A commanded motion did not complete: the robot is stopped or not executing."""


def call_trigger_service(service_name, timeout=5.0):
    """Call a std_srvs/Trigger service. The cup services are no-ops (True) with
    --no-vacuum; the perception service is always called. Returns (ok, message)."""
    if disable_pneumatics and '/lbl_cup/' in service_name:
        rospy.loginfo("[Vacuum] disabled — skipping %s", service_name)
        return True, ""
    try:
        rospy.wait_for_service(service_name, timeout=timeout)
        response = rospy.ServiceProxy(service_name, Trigger)()
        rospy.loginfo("[Service] %s → success=%s msg='%s'", service_name,
                      response.success, response.message)
        return bool(response.success), response.message
    except Exception as exc:  # noqa: BLE001 — a failed service must not crash the loop
        rospy.logerr("[Service] %s failed: %s", service_name, exc)
        return False, str(exc)


class LabellingArm(object):
    """Robot 2 plus the IK, poses and perception hooks the label cycle needs."""

    def __init__(self, no_motion=False, z_offset=0.0, motion_timeout_margin=8.0):
        self.no_motion = no_motion
        self.z_offset = float(z_offset)
        self.motion_timeout_margin = float(motion_timeout_margin)

        self.arm = None
        if not no_motion:
            from frankapy import FrankaArm
            rospy.loginfo("[Hardware] Connecting to the labelling arm (robot_num=2)")
            self.arm = FrankaArm(with_gripper=False, old_gripper=False, robot_num=2,
                                 init_node=False)
            self._move(self.arm.reset_joints, desc="initial reset_joints")

        # IK chain and collision checker as state_machine.py sets them up for arm 2.
        self.ik_chain = ikpy.chain.Chain.from_urdf_file(URDF_FILE, base_elements=["panda_link0"])
        self.chain_length = len(self.ik_chain.links)
        self.ik_chain.active_links_mask = [False] + [True] * 7 + [False] * (self.chain_length - 8)
        self.collision_checker = CollisionChecker.CollisionChecker(arm_number=2)
        import pickle
        with open(OBSTACLES_FILE, 'rb') as f:
            self.obs_points = pickle.load(f)
            self.obs_axes = pickle.load(f)
        # The table is modelled as boxes whose top is z = 0, and the cup's collision block
        # reaches ~1 cm past the cup, so a target lower than that is always "in collision"
        # with the table even though touching it there is the point of the placement
        # (a flat pouch sits at z ~ 0.005). The tool block touching a table-level box is
        # therefore allowed for the final pose only; everything else is a real collision.
        self.table_obstacles = [j for j, p in enumerate(self.obs_points)
                                if np.asarray(p)[:, 2].max() <= 0.001]
        self.tool_block = len(self.collision_checker.Cidx) - 1
        rospy.loginfo("[Hardware] %d obstacles from %s (%d at table level)",
                      len(self.obs_points), OBSTACLES_FILE, len(self.table_obstacles))

        with open(POSES_FILE) as f:
            raw = json.load(f)
        self.dispenser_contact = np.array(raw['label_arm']['LABEL_DISPENSER'])
        self.dispenser_pre = self._contact_to_pre_joints(self.dispenser_contact)
        with open(TARGET_POSES_FILE) as f:
            self.target_poses = json.load(f)
        self.safe_joints = self.pose_to_joints("LABELLING_SAFE")

        self.label_poses = {}
        for zone, topic in ZONE_TOPICS.items():
            rospy.Subscriber(topic, PoseArray, self._pose_cb(zone), queue_size=1)
        self.lbl_has_item = False
        rospy.Subscriber("orio/vacuum/lbl_has_item", Bool,
                         lambda m: setattr(self, 'lbl_has_item', m.data), queue_size=1)
        rospy.loginfo("[Hardware] Ready%s", " (no motion)" if no_motion else "")

    def _pose_cb(self, zone):
        def cb(msg):
            self.label_poses[zone] = msg
        return cb

    # ── Motion with a timeout (dexnet_pnp.py's _move) ─────────────────────────
    def _move(self, motion_fn, *args, timeout=None, desc="motion", **kwargs):
        """A blocking FrankaArm motion with a HARD timeout: frankapy's wait_for_skill()
        never returns if franka-interface stops executing, so issue the motion
        non-blocking and poll is_skill_done() against a deadline instead."""
        if self.no_motion:
            rospy.loginfo("[Move] (no motion) %s", desc)
            return
        duration = kwargs.get("duration", 5)
        if timeout is None:
            timeout = float(duration) + self.motion_timeout_margin
        kwargs["block"] = False
        motion_fn(*args, **kwargs)
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        rate = rospy.Rate(20)
        faults = 0
        while True:
            try:
                # Recoverable controller faults (acceleration discontinuities) are common;
                # franka-interface recovers by itself, so log once and keep polling.
                if self.arm.is_skill_done(ignore_errors=False):
                    return
            except Exception as exc:  # noqa: BLE001 — FrankaArmException / NotReady
                faults += 1
                if faults == 1:
                    rospy.logwarn("[Move] %s: controller fault (recovering): %s",
                                  desc, str(exc).split('\n')[0])
                try:
                    self.arm.wait_for_franka_interface()
                except Exception:  # noqa: BLE001
                    pass
            if rospy.Time.now() > deadline:
                self._abort_skill()
                raise MotionFailed(
                    "%s did not complete in %.1fs%s — the robot is not executing (control "
                    "PC down / FCI not active / e-stop, or a fault it never recovered from)."
                    % (desc, timeout, "" if not faults else " after %d fault(s)" % faults))
            if rospy.is_shutdown():
                self._abort_skill()
                raise KeyboardInterrupt
            rate.sleep()

    def _abort_skill(self):
        """Cancel an in-flight skill so the next command is accepted (not stop_skill(),
        whose unbounded wait hangs on exactly the dead-control-PC case this is for)."""
        try:
            if getattr(self.arm, '_connected', False) and getattr(self.arm, '_in_skill', False):
                self.arm._client.cancel_goal()
        except Exception as exc:  # noqa: BLE001
            rospy.logwarn("[Move] cancel_goal during abort failed: %s", exc)
        try:
            self.arm._in_skill = False
        except Exception:  # noqa: BLE001
            pass
        rospy.sleep(0.5)

    def goto(self, joints, duration, desc):
        self._move(self.arm.goto_joints if self.arm else None, list(joints),
                   duration=duration, desc=desc)

    # ── IK (state_machine.py's helpers for arm 2) ─────────────────────────────
    def _seed(self, jitter=0.0):
        guess = [0.0] * self.chain_length
        guess[4] = -1.5   # tool pointing down
        if jitter:
            for i in range(1, 8):
                lo, hi = self.ik_chain.links[i].bounds
                guess[i] = float(np.clip(guess[i] + random.uniform(-jitter, jitter), lo, hi))
        return guess

    def pose_to_joints(self, tag, depth_offset=0.0):
        entry = self.target_poses[tag]
        tx, ty, tz = entry["translation"]
        angles = self.ik_chain.inverse_kinematics(
            target_position=[tx, ty, tz + depth_offset],
            target_orientation=np.array(entry["rotation"]),
            orientation_mode="all", initial_position=self._seed())
        return angles[1:8]

    def _contact_to_pre_joints(self, contact_joints):
        full = [0.0] * self.chain_length
        full[1:8] = contact_joints
        tf = self.ik_chain.forward_kinematics(full)
        pre = self.ik_chain.inverse_kinematics(
            target_position=tf[:3, 3] + np.array([0.0, 0.0, APPROACH_DISTANCE]),
            target_orientation=tf[:3, :3], orientation_mode="all",
            initial_position=self._seed())
        return pre[1:8]

    def collisions(self, q7, allow_tool_on_table=False):
        """(block, obstacle) pairs of the configuration that collide; CollisionChecker's
        DetectCollision, kept pairwise so the tool-on-table contact can be told apart."""
        import RobotUtil as rt
        cc = self.collision_checker
        cc.CompCollisionBlockPoints(q7)
        hits = []
        for i in range(len(cc.Cpoints)):
            for j in range(len(self.obs_points)):
                if allow_tool_on_table and i == self.tool_block and j in self.table_obstacles:
                    continue
                if rt.CheckBoxBoxCollision(cc.Cpoints[i], cc.Caxes[i],
                                           self.obs_points[j], self.obs_axes[j]):
                    hits.append((i, j))
        return hits

    def current_joints(self):
        """Where the arm is now, for choosing the nearer of equivalent solutions. Without
        a robot (or a state read that fails) LABELLING_SAFE stands in, the pose every
        label motion starts from."""
        if self.arm is not None:
            try:
                return np.asarray(self.arm.get_joints(), dtype=float)
            except Exception as exc:  # noqa: BLE001
                rospy.logwarn("[IK] get_joints failed (%s); using LABELLING_SAFE", exc)
        return np.asarray(self.safe_joints, dtype=float)

    def compute_label_joints(self, task_pos, target_ori, max_attempts=10):
        """(pre_joints, final_joints) for the label pose, from whichever of the two
        equivalent tool orientations (see TOOL_FLIP) needs the smaller joint motion from
        the arm's current configuration."""
        q_now = self.current_joints()
        best = None
        for flipped in (False, True):
            ori = target_ori @ TOOL_FLIP if flipped else target_ori
            try:
                pre, final = self._solve_label_joints(task_pos, ori, max_attempts)
            except RuntimeError as exc:
                rospy.logwarn("[IK] %s orientation: %s", "flipped" if flipped else "published", exc)
                continue
            travel = float(np.abs(np.asarray(pre) - q_now).sum())
            rospy.loginfo("[IK] %s orientation: joint 7 %.0f deg (%+.0f deg from now), "
                          "total joint travel to pre %.2f rad",
                          "flipped" if flipped else "published", np.degrees(pre[6]),
                          np.degrees(pre[6] - q_now[6]), travel)
            if best is None or travel < best[0]:
                best = (travel, pre, final, flipped)
        if best is None:
            raise RuntimeError("[IK] no collision-free solution for %s in either tool "
                               "orientation" % np.round(task_pos, 4))
        travel, pre, final, flipped = best
        rospy.loginfo("[IK] using the %s orientation (travel %.2f rad)",
                      "flipped" if flipped else "published", travel)
        return pre, final

    def _solve_label_joints(self, task_pos, target_ori, max_attempts):
        """(pre_joints, final_joints) for one orientation: collision-checked against the
        arm-2 obstacles and FK-validated, as compute_pick_joints does for ApplyLabel, with
        the cup allowed to meet the table model at the final pose and from-behind
        configurations refused (see SEED_JITTER / MAX_BASE_TURN)."""
        pre_pos = [task_pos[0], task_pos[1], task_pos[2] + APPROACH_DISTANCE]
        for attempt in range(max_attempts):
            pre = self.ik_chain.inverse_kinematics(
                target_position=pre_pos, target_orientation=target_ori,
                orientation_mode="all",
                initial_position=self._seed(jitter=SEED_JITTER if attempt else 0.0))
            final = self.ik_chain.inverse_kinematics(
                target_position=list(task_pos), target_orientation=target_ori,
                orientation_mode="all", initial_position=pre)
            if abs(pre[1]) > MAX_BASE_TURN or abs(pre[3]) > MAX_BASE_TURN \
                    or abs(final[1]) > MAX_BASE_TURN or abs(final[3]) > MAX_BASE_TURN:
                rospy.logwarn("[IK] attempt %d/%d reaches the zone from behind (joint 1 %.2f, "
                              "joint 3 %.2f rad), retrying with a new seed",
                              attempt + 1, max_attempts, final[1], final[3])
                continue
            hits_pre = self.collisions(pre[1:8])
            hits_final = self.collisions(final[1:8], allow_tool_on_table=True)
            if hits_pre or hits_final:
                rospy.logwarn("[IK] attempt %d/%d in collision (pre %s, final %s as "
                              "(block, obstacle)), retrying with a new seed",
                              attempt + 1, max_attempts, hits_pre, hits_final)
                continue
            pre_err = np.linalg.norm(self.ik_chain.forward_kinematics(list(pre))[:3, 3] - pre_pos)
            final_err = np.linalg.norm(self.ik_chain.forward_kinematics(list(final))[:3, 3] - task_pos)
            if pre_err > 0.02 or final_err > 0.02:
                rospy.logwarn("[IK] attempt %d/%d poor convergence (pre %.4f m, final %.4f m), retrying",
                              attempt + 1, max_attempts, pre_err, final_err)
                continue
            rospy.loginfo("[IK] OK on attempt %d: pre=%s final=%s (FK err %.4f / %.4f m)",
                          attempt + 1, np.round(pre[1:8], 4), np.round(final[1:8], 4),
                          pre_err, final_err)
            return pre[1:8], final[1:8]
        raise RuntimeError("[IK] no collision-free solution for %s after %d attempts"
                           % (np.round(task_pos, 4), max_attempts))

    # ── Perception ────────────────────────────────────────────────────────────
    def request_label_poses(self, zone_choice):
        """Call the perception service and return the usable label poses as a list of
        (zone, position, rotation matrix): the requested zone, or with 'auto' zone 1 then
        2. A zone with no pose, or one outside POSE_BOUNDS, is reported and left out."""
        self.label_poses.clear()
        ok, message = call_trigger_service(SERVICE, timeout=10.0)
        if not ok:
            raise RuntimeError("%s failed: %s" % (SERVICE, message))
        wanted = [1, 2] if zone_choice == 'auto' else [int(zone_choice)]
        # The service publishes every zone's pose before it answers, so the messages are
        # normally already here; the short wait covers the transport.
        deadline = rospy.Time.now() + rospy.Duration(2.0)
        while rospy.Time.now() < deadline and not all(z in self.label_poses for z in wanted):
            rospy.sleep(0.05)
        candidates, problems = [], []
        for zone in wanted:
            msg = self.label_poses.get(zone)
            if msg is None or not msg.poses:
                problems.append("zone %d: no pose" % zone)
                continue
            p = msg.poses[0]
            pos = np.array([p.position.x, p.position.y, p.position.z + self.z_offset])
            quat = [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]
            if np.any(np.isnan(pos)) or np.any(np.isnan(quat)):
                problems.append("zone %d: NaN in the pose" % zone)
                continue
            why = self.check_bounds(pos)
            if why:
                problems.append("zone %d: %s" % (zone, why))
                continue
            candidates.append((zone, pos, R.from_quat(quat).as_matrix()))
        for text in problems:
            rospy.logwarn("[Label] %s", text)
        if not candidates:
            raise RuntimeError("no usable label pose (perception said: %s)" % message)
        return candidates

    def plan_label(self, zone_choice):
        """(zone, pos, pre_joints, final_joints) for the first usable zone that IK can
        reach; raises with the reasons when none can."""
        failures = []
        for zone, pos, ori in self.request_label_poses(zone_choice):
            rpy = R.from_matrix(ori).as_euler('xyz', degrees=True)
            rospy.loginfo("[Label] zone %d: world (%.3f, %.3f, %.3f) m%s, tool RPY "
                          "(%.1f, %.1f, %.1f) deg", zone, pos[0], pos[1], pos[2],
                          " (z offset %+.3f)" % self.z_offset if self.z_offset else "",
                          rpy[0], rpy[1], rpy[2])
            try:
                pre_joints, final_joints = self.compute_label_joints(pos, ori)
            except RuntimeError as exc:
                rospy.logwarn("[Label] zone %d: %s", zone, exc)
                failures.append("zone %d: %s" % (zone, exc))
                continue
            return zone, pos, pre_joints, final_joints
        raise RuntimeError("; ".join(failures))

    @staticmethod
    def check_bounds(pos):
        """None if the pose is inside POSE_BOUNDS, else the reason it is refused."""
        x, y, z = pos
        b = POSE_BOUNDS
        if not (b['x_min'] <= x <= b['x_max'] and b['y_min'] <= y <= b['y_max']):
            return "pose (x=%.3f, y=%.3f) outside the zone box %s" % (x, y, b)
        if not (b['z_min'] <= z <= b['z_max']):
            return ("z=%.3f m outside [%.2f, %.2f]: the measured depth looks wrong (the "
                    "table is at z ~ 0)" % (z, b['z_min'], b['z_max']))
        return None

    # ── The two states ────────────────────────────────────────────────────────
    def get_label(self):
        """GET_LABEL: dispenser pre -> contact, cup on, back to pre, then LABELLING_SAFE."""
        rospy.loginfo("[GetLabel] Fetching a label from the dispenser")
        self.goto(self.dispenser_pre, 5, "dispenser pre")
        self.goto(self.dispenser_contact, 5, "dispenser contact")
        call_trigger_service('/orio/lbl_cup/on')
        rospy.sleep(1.0)
        self.goto(self.dispenser_pre, 2, "dispenser retreat")
        self.goto(self.safe_joints, 3, "LABELLING_SAFE")
        rospy.loginfo("[GetLabel] Done (cup sensor: %s)",
                      "has item" if self.lbl_has_item else "no item / no sensor")

    def apply_label(self, pre_joints, final_joints):
        """APPLY: pre -> final, cup off, back to pre, then LABELLING_SAFE."""
        self.goto(pre_joints, 5, "label pre")
        self.goto(final_joints, 5, "label contact")
        call_trigger_service('/orio/lbl_cup/off')
        rospy.sleep(1.0)
        self.goto(pre_joints, 2, "label retreat")
        self.goto(self.safe_joints, 3, "LABELLING_SAFE")

    # ── Loop ──────────────────────────────────────────────────────────────────
    @staticmethod
    def _confirm():
        """Enter = place, 'r' = compute the pose again, 's' = skip this cycle, 'q' = stop."""
        while True:
            try:
                sys.stderr.flush()
                print("[Loop] Press Enter to place the label, 'r' to recompute the pose, "
                      "'s' to skip, 'q' to stop...")
                sys.stdout.flush()
                answer = input().strip().lower()
            except (EOFError, KeyboardInterrupt):
                return 'quit'
            if answer == '':
                return 'place'
            if answer in ('r', 'regen', 'recompute'):
                return 'recompute'
            if answer in ('s', 'skip'):
                return 'skip'
            if answer in ('q', 'quit', 'abort'):
                return 'quit'
            print("[Loop] Unrecognised input %r" % answer)

    def run(self, zone_choice='auto', confirm=True, skip_dispenser=False, cycles=None):
        placed = 0
        cycle = 0
        # The dispenser is visited only when the cup has no label: at the start and after
        # a placement. A failed placement keeps the label (and the arm) where it is until
        # the operator asks for a new pose; a motion failure ends the run, since the robot
        # has stopped executing (user stop, fault) and must be looked at first.
        have_label = skip_dispenser
        while not rospy.is_shutdown() and (cycles is None or cycle < cycles):
            cycle += 1
            rospy.loginfo("[Loop] ── cycle %d ──", cycle)
            if not have_label:
                try:
                    self.get_label()
                except MotionFailed as exc:
                    rospy.logerr("[Loop] %s", exc)
                    break
                have_label = True

            while True:
                plan = None
                try:
                    plan = self.plan_label(zone_choice)
                except Exception as exc:  # noqa: BLE001 — let the operator decide
                    rospy.logerr("[Label] %s", exc)
                    if not confirm:
                        rospy.logwarn("[Loop] No usable label pose — trying again in 2 s")
                        rospy.sleep(2.0)
                        continue
                choice = self._confirm() if (confirm or plan is None) else 'place'
                if plan is None and choice == 'place':
                    choice = 'recompute'   # nothing to place yet: Enter tries again
                if choice == 'recompute':
                    continue
                break
            if choice == 'quit':
                rospy.loginfo("[Loop] Stopped by operator")
                break
            if choice == 'skip' or plan is None:
                continue
            zone, pos, pre_joints, final_joints = plan

            if self.no_motion:
                rospy.loginfo("[Loop] (no motion) would place at zone %d: pre=%s final=%s",
                              zone, np.round(pre_joints, 4), np.round(final_joints, 4))
                continue
            try:
                self.apply_label(pre_joints, final_joints)
                placed += 1
                have_label = False
                rospy.loginfo("[Loop] Label %d placed in zone %d", placed, zone)
            except MotionFailed as exc:
                rospy.logerr("[Loop] Placement failed: %s", exc)
                rospy.logerr("[Loop] Stopping: check the robot (user stop / Desk error), then re-run.")
                break
            except Exception as exc:  # noqa: BLE001 — log, release the cup, back to the prompt
                rospy.logerr("[Loop] Placement failed: %s", exc)
                call_trigger_service('/orio/lbl_cup/off')
                have_label = False
        return placed


def main():
    global disable_pneumatics
    parser = argparse.ArgumentParser(description="Labelling arm (robot 2) on its own.")
    parser.add_argument('--no-vacuum', '--disable-pneumatics', dest='no_vacuum',
                        action='store_true', help="dry run: skip the cup services")
    parser.add_argument('--confirm', dest='confirm', action='store_true', default=True,
                        help="ask before each placement (default)")
    parser.add_argument('--auto', dest='confirm', action='store_false', help="no prompt")
    parser.add_argument('--zone', choices=['1', '2', 'auto'], default='auto')
    parser.add_argument('--skip-dispenser', action='store_true')
    parser.add_argument('--cycles', type=int, default=None, metavar='N')
    parser.add_argument('--z-offset', type=float, default=0.0, metavar='M')
    parser.add_argument('--no-motion', action='store_true')
    parser.add_argument('--check-only', action='store_true')
    args, _ = parser.parse_known_args(rospy.myargv(argv=sys.argv)[1:])

    rospy.init_node('labelling_arm_test')

    if args.check_only:
        # Constructing FrankaArm runs frankapy's wait_for_franka_interface(), which raises
        # after ~10 s if the robot is not ready (locked / not activated / control PC down).
        from frankapy import FrankaArm
        try:
            FrankaArm(with_gripper=False, old_gripper=False, robot_num=2, init_node=False)
            rospy.loginfo("[Check] Robot 2 franka-interface is READY.")
            return 0
        except Exception as exc:  # noqa: BLE001
            rospy.logerr("[Check] Robot 2 NOT ready: %s", exc)
            return 1

    disable_pneumatics = args.no_vacuum or os.environ.get(
        'ORIO_DISABLE_PNEUMATICS', '').strip().lower() in ('1', 'true', 'yes', 'on')
    if disable_pneumatics:
        rospy.logwarn("[Main] PNEUMATICS DISABLED — the cup services are no-ops (dry run). "
                      "Put a label on the cup by hand if you want to see a placement.")
    if args.no_motion:
        rospy.logwarn("[Main] NO MOTION — perception and IK only.")

    controller = LabellingArm(no_motion=args.no_motion, z_offset=args.z_offset)
    try:
        placed = controller.run(zone_choice=args.zone, confirm=args.confirm,
                                skip_dispenser=args.skip_dispenser, cycles=args.cycles)
        rospy.loginfo("[Main] Finished. Labels placed: %d", placed)
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        rospy.logwarn("[Main] Interrupt — stopping")
    finally:
        rospy.logwarn("[Main] Turning the labelling cup off")
        call_trigger_service('/orio/lbl_cup/off')
    return 0


if __name__ == '__main__':
    sys.exit(main())
