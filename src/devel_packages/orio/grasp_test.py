#!/usr/bin/env python3
"""Grasp test: plan a suction grasp on the live Xtion frame and send the arm there. No vacuum.

Each round:
  1. reset the arm (out of the camera's view) and take the newest RGB-D frame;
  2. ask the grasp planner (the orio-grasping network, started by grasp_test.sh) for a grasp
     and draw it on the frame;
  3. Enter: move 10 cm above the grasp along its approach axis, then down to it
     ('r' plans again, 'q' quits);
  4. with the arm at the grasp, take another frame and draw where the arm went;
  5. Enter: back off and reset ('q' resets, then quits).

Everything a round produces goes to --out (grasp_test.sh: data/grasp_test/<stamp>/):
  round_NNN_plan.png  the planning frame: both cups where the planner put them, and its score
  round_NNN_arm.png   the frame with the arm at the grasp: the planned cups, the target the arm
                      was sent to (grasp + offset) and where it reached (forward kinematics of
                      the measured joints), projected through the camera TF (realsense_tf.yaml)
  rounds.jsonl        one line per round: the planner's answer, the world grasp, offsets, IK
                      joints and residuals, the measured joints and the reach error
The planner writes its own panel (masks, height map, score map) and inputs per call beside them.

Runs INSIDE the main container, like dexnet_pnp.py (grasp_test.sh does this):
    docker exec -it orio_docker_container bash -c \\
      'source /home/ros_ws/devel/setup.bash && cd /home/ros_ws/src/devel_packages/orio \\
       && python3 grasp_test.py --out /home/ros_ws/data/grasp_test/<stamp>'

Prereqs: roscore, the Xtion driver, the planner service, and (unless --no-arm) the robot-1
control PC.
"""
import argparse
import json
import os
import sys
import time

import cv2
import numpy as np
import rospy
import yaml
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import CameraInfo, Image

from custom_msgs.srv import PlanDexnetGrasp

RGB_TOPIC = "/camera/rgb/image_raw"
DEPTH_TOPIC = "/camera/depth_registered/image_raw"
INFO_TOPIC = "/camera/rgb/camera_info"
TF_YAML = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       "../manipulation/config/realsense_tf.yaml")
# Fallback when the driver publishes no camera_info, as in the perception node.
PRIMESENSE_K = np.array([[525.0, 0.0, 319.5], [0.0, 525.0, 239.5], [0.0, 0.0, 1.0]])
# The camera calibration the planner plans with (vendored orio-grasping): its focal length, not
# the driver's default camera_info, places grasps in the image (see the planner's K_cal).
CALIBRATION = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           "../orio_grasping/assets/calibration/xtion_pose.json")


def calibrated_K(width, height):
    """The planner's calibrated camera matrix for a full driver frame (turned 180 deg, as the
    planner turns it), or None without the calibration file."""
    if not os.path.isfile(CALIBRATION):
        return None
    with open(CALIBRATION) as f:
        fx = width / (2.0 * np.tan(np.radians(json.load(f)["hfov_deg"]) / 2.0))
    return np.array([[fx, 0.0, width - 1 - width / 2.0], [0.0, fx, height - 1 - height / 2.0], [0.0, 0.0, 1.0]])
# OrioNet's two cups (label_info cup_spacing / cup_radius of its checkpoints), for drawing.
CUP_SPACING = 0.03302
CUP_RADIUS = 0.015875
# Seconds of streaming before the first grab: the camera's first frames are torn.
WARMUP_S = 2.0


def load_tf(path):
    """4x4 camera-to-world transform from a manipulation/config TF yaml."""
    with open(path) as f:
        pose = yaml.safe_load(f)["pose"]
    t, r = pose["translation"], pose["rotation"]
    T = np.eye(4)
    T[:3, :3] = R.from_quat([r["x"], r["y"], r["z"], r["w"]]).as_matrix()
    T[:3, 3] = [t["x"], t["y"], t["z"]]
    return T


def rotation_from_approach(approach_world, cup_line_world):
    """Tool rotation with Z along the approach and Y along the cup line, the sign that keeps
    tool X nearest world +X. Same as the perception node's _rotation_from_approach."""
    z_axis = approach_world / np.linalg.norm(approach_world)
    ref = np.cross(np.asarray(cup_line_world, dtype=float), z_axis)
    if ref[0] < 0:
        ref = -ref
    x_axis = ref - np.dot(ref, z_axis) * z_axis
    x_axis /= np.linalg.norm(x_axis)
    return np.column_stack([x_axis, np.cross(z_axis, x_axis), z_axis])


def decode_color(msg):
    color = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)[:, :, :3]
    return np.ascontiguousarray(color[:, :, ::-1] if "bgr" in msg.encoding else color)


def decode_depth(msg):
    """Depth in metres, 0 where invalid."""
    if msg.encoding == "16UC1":
        return np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width).astype(np.float32) / 1000.0
    if msg.encoding == "32FC1":
        return np.nan_to_num(np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width))
    raise ValueError("unsupported depth encoding %s" % msg.encoding)


def image_msg(arr, encoding):
    msg = Image()
    msg.header.frame_id = "pnp_camera"
    msg.height, msg.width = arr.shape[0], arr.shape[1]
    msg.encoding = encoding
    msg.step = arr.strides[0]
    msg.data = arr.tobytes()
    return msg


def jsonable(x):
    if isinstance(x, dict):
        return {k: jsonable(v) for k, v in x.items()}
    if isinstance(x, (list, tuple, np.ndarray)):
        return [jsonable(v) for v in x]
    if isinstance(x, np.generic):
        return x.item()
    return x


def shown(path):
    """A container path as the repo-relative path it has on the host (data/..., logging/...)."""
    return path[len("/home/ros_ws/"):] if path.startswith("/home/ros_ws/") else path


def ask(prompt, choices):
    """Read one answer from the terminal: '' (Enter) or one of choices; 'q' on Ctrl-C / EOF.
    The prompt is its own flushed line, as in dexnet_pnp.py, so it shows under a pipe."""
    while True:
        try:
            sys.stderr.flush()
            print("[Test] " + prompt)
            sys.stdout.flush()
            answer = input().strip().lower()
        except (EOFError, KeyboardInterrupt):
            return "q"
        if answer == "" or answer in choices:
            return answer
        print("[Test] Unrecognised input %r." % answer)


class GraspTest(object):
    def __init__(self, args):
        self.args = args
        self.out = args.out
        os.makedirs(self.out, exist_ok=True)
        self.T_world_cam = load_tf(TF_YAML)
        self.T_cam_world = np.linalg.inv(self.T_world_cam)
        self.offset = np.array([args.offset_x, args.offset_y, 0.0])

        try:
            info = rospy.wait_for_message(INFO_TOPIC, CameraInfo, timeout=5.0)
            K_driver = np.array(info.K, dtype=np.float64).reshape(3, 3)
            size = (info.width, info.height)
        except rospy.ROSException:
            rospy.logwarn("[Test] No %s; using PrimeSense default intrinsics", INFO_TOPIC)
            K_driver, size = PRIMESENSE_K.copy(), (640, 480)
        # Sent to the planner as received; drawing uses what the planner plans with.
        self.info = CameraInfo()
        self.info.header.frame_id = "pnp_camera"
        self.info.K = list(K_driver.ravel())
        K_cal = calibrated_K(*size)
        self.K = K_driver if K_cal is None else K_cal
        rospy.loginfo("[Test] Drawing with fx %.1f px (%s; driver fx %.1f)", self.K[0, 0],
                      "driver" if K_cal is None else "calibrated", K_driver[0, 0])

        # Subscribed for the whole run, as the perception node is. The driver only streams
        # colour while something subscribes, and the first frames after it starts are torn;
        # subscribing per grab (wait_for_message) restarted the stream and got one every time.
        self._latest = {RGB_TOPIC: None, DEPTH_TOPIC: None}
        for topic in self._latest:
            rospy.Subscriber(topic, Image, self._on_image, callback_args=topic, queue_size=1,
                             buff_size=2 ** 24)
        rospy.sleep(WARMUP_S)

        rospy.loginfo("[Test] Waiting for %s", args.service)
        rospy.wait_for_service(args.service, timeout=120.0)
        self.plan_srv = rospy.ServiceProxy(args.service, PlanDexnetGrasp)

        # The arm: dexnet_pnp.py's wrapper, for its timed motions (controller-fault recovery,
        # hard timeouts) and its IK. Constructing it resets the arm.
        self.pnp = None
        if not args.no_arm:
            import dexnet_pnp
            self.dexnet_pnp = dexnet_pnp
            self.pnp = dexnet_pnp.DexnetPickPlace(use_grasp_orientation=not args.straight_down)

    # ── camera ────────────────────────────────────────────────────────────────
    def _on_image(self, msg, topic):
        self._latest[topic] = msg

    def grab(self, depth=True, timeout=5.0):
        """The first colour (and depth) frames stamped after this call, so a frame taken
        while the arm was still moving is never used."""
        topics = (RGB_TOPIC, DEPTH_TOPIC) if depth else (RGB_TOPIC,)
        since = rospy.Time.now()
        deadline = time.time() + timeout
        while any(self._latest[t] is None or self._latest[t].header.stamp <= since for t in topics):
            if time.time() > deadline or rospy.is_shutdown():
                raise RuntimeError("no new frame on %s within %.0fs" % (" / ".join(topics), timeout))
            rospy.sleep(0.02)
        color = decode_color(self._latest[RGB_TOPIC])
        return color, decode_depth(self._latest[DEPTH_TOPIC]) if depth else None

    def project(self, p_cam):
        p = np.asarray(p_cam, dtype=float)
        return (int(round(self.K[0, 0] * p[0] / p[2] + self.K[0, 2])),
                int(round(self.K[1, 1] * p[1] / p[2] + self.K[1, 2])))

    def world_to_cam(self, p_world):
        return self.T_cam_world.dot(np.append(np.asarray(p_world, dtype=float), 1.0))[:3]

    # ── planning ──────────────────────────────────────────────────────────────
    def plan(self, color, depth):
        """Call the planner; returns the round record's grasp part."""
        mask = np.full(depth.shape, 255, np.uint8)
        t0 = time.time()
        resp = self.plan_srv(image_msg(color, "rgb8"), image_msg(np.ascontiguousarray(depth, np.float32), "32FC1"),
                             self.info, image_msg(mask, "mono8"))
        g = resp.grasp
        rot_cam = R.from_quat([g.pose.orientation.x, g.pose.orientation.y,
                               g.pose.orientation.z, g.pose.orientation.w]).as_matrix()
        centre_cam = np.array([g.pose.position.x, g.pose.position.y, g.pose.position.z])
        # Camera frame -> world, as the perception node does for the pick-and-place loop.
        R_wc = self.T_world_cam[:3, :3]
        centre_world = R_wc.dot(centre_cam) + self.T_world_cam[:3, 3]
        approach_world = R_wc.dot(rot_cam[:, 0])
        approach_world /= np.linalg.norm(approach_world)
        if approach_world[2] > 0:
            approach_world = -approach_world
        cup_line_world = R_wc.dot(rot_cam[:, 1])
        tilt = float(np.degrees(np.arccos(np.clip(-approach_world[2], -1.0, 1.0))))
        target = centre_world + self.offset
        rotation = None if self.args.straight_down else rotation_from_approach(approach_world, cup_line_world)
        return {
            "planner_ok": bool(resp.success),
            "planner_message": resp.message,
            "score": float(g.q_value),
            "center_px": [float(v) for v in g.center_px],
            "cup_angle_deg": float(np.degrees(g.angle)),
            "plan_time_s": time.time() - t0,
            "centre_cam": centre_cam,
            "cup_line_cam": rot_cam[:, 1],
            "centre_world": centre_world,
            "approach_world": approach_world,
            "cup_line_world": cup_line_world,
            "tilt_deg": tilt,
            "offset": self.offset,
            "target_world": target,
            "target_rotation": rotation,
        }

    def check(self, grasp):
        """Why the arm must not go to this grasp, or None."""
        if not grasp["planner_ok"]:
            return "planner rejected it: %s" % grasp["planner_message"]
        if not self.args.straight_down and grasp["tilt_deg"] > self.args.max_tilt_deg:
            return "tilt %.1f deg > --max-tilt-deg %.1f" % (grasp["tilt_deg"], self.args.max_tilt_deg)
        if self.pnp is not None:
            b = self.dexnet_pnp.PICK_BOUNDS
            x, y, z = grasp["target_world"]
            if not (b["x_min"] <= x <= b["x_max"] and b["y_min"] <= y <= b["y_max"]
                    and b["z_min"] <= z <= b["z_max"]):
                return "target (%.3f, %.3f, %.3f) outside the safe box %s" % (x, y, z, b)
        return None

    # ── drawing ───────────────────────────────────────────────────────────────
    def draw_cups(self, img, grasp, colour, width=2):
        half = 0.5 * CUP_SPACING * np.asarray(grasp["cup_line_cam"])
        for c in (grasp["centre_cam"] + half, grasp["centre_cam"] - half):
            u, v = self.project(c)
            cv2.circle(img, (u, v), max(2, int(round(CUP_RADIUS * self.K[0, 0] / c[2]))), colour, width)
        cv2.drawMarker(img, self.project(grasp["centre_cam"]), colour, cv2.MARKER_CROSS, 10, 1)

    @staticmethod
    def label(img, lines, colour=(255, 255, 255)):
        for i, text in enumerate(lines):
            y = 18 + 18 * i
            cv2.putText(img, text, (6, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(img, text, (6, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, colour, 1, cv2.LINE_AA)

    def save_plan_image(self, n, color, grasp, refusal):
        img = cv2.cvtColor(color, cv2.COLOR_RGB2BGR)
        self.draw_cups(img, grasp, (0, 255, 0) if refusal is None else (0, 0, 255))
        x, y, z = grasp["target_world"]
        self.label(img, ["round %d  score %.3f  tilt %.1f deg  cups %.0f deg"
                         % (n, grasp["score"], grasp["tilt_deg"], grasp["cup_angle_deg"]),
                         "target world (%.3f, %.3f, %.3f)" % (x, y, z),
                         "OK" if refusal is None else "NOT SENT: " + refusal[:70]])
        path = os.path.join(self.out, "round_%03d_plan.png" % n)
        cv2.imwrite(path, img)
        return path

    def save_arm_image(self, n, color, grasp, reached_world, goal):
        """The planned cups (green), the arm's target (yellow) and where it reached (magenta)."""
        img = cv2.cvtColor(color, cv2.COLOR_RGB2BGR)
        self.draw_cups(img, grasp, (0, 255, 0), 1)
        tgt = self.project(self.world_to_cam(goal))
        cv2.drawMarker(img, tgt, (0, 255, 255), cv2.MARKER_TILTED_CROSS, 14, 2)
        lines = ["round %d  arm at %s" % (n, "pre-grasp (hover)" if self.args.hover else "grasp"),
                 "yellow x: target  green: planned cups"]
        if reached_world is not None:
            cv2.drawMarker(img, self.project(self.world_to_cam(reached_world)), (255, 0, 255),
                           cv2.MARKER_SQUARE, 12, 2)
            err = 1000 * (np.asarray(reached_world) - np.asarray(goal))
            lines.append("magenta: reached (FK)  error (%+.0f, %+.0f, %+.0f) mm" % tuple(err))
        self.label(img, lines)
        path = os.path.join(self.out, "round_%03d_arm.png" % n)
        cv2.imwrite(path, img)
        return path

    # ── arm ───────────────────────────────────────────────────────────────────
    def reached(self):
        """Measured joints, their forward kinematics (the IK chain's tool frame, world) and
        frankapy's own end-effector position."""
        joints = [float(j) for j in self.pnp.arm.get_joints()]
        chain = [0.0] + joints + [0.0] * (self.pnp.chain_length - 8)
        fk = self.pnp.ik_chain.forward_kinematics(chain)[:3, 3]
        try:
            ee = self.pnp.arm.get_pose().translation
        except Exception as exc:  # noqa: BLE001 - only a cross-check
            rospy.logwarn("[Test] get_pose failed: %s", exc)
            ee = None
        return {"joints": joints, "fk_world": fk, "frankapy_ee_world": ee}

    def reset(self, desc):
        self.pnp._move(self.pnp.arm.reset_joints, duration=4, desc=desc)

    # ── loop ──────────────────────────────────────────────────────────────────
    def run(self):
        n = 0
        while not rospy.is_shutdown():
            n += 1
            rec = {"round": n, "time": time.strftime("%Y-%m-%dT%H:%M:%S")}
            try:
                quit_after = self.round(n, rec)
            except Exception as exc:  # noqa: BLE001 - log it, get home, keep testing
                rec["error"] = str(exc)
                rospy.logerr("[Test] round %d failed: %s", n, exc)
                quit_after = False
                if self.pnp is not None:
                    try:
                        self.pnp._abort_skill()
                        self.reset("reset after failure")
                    except Exception as reset_exc:  # noqa: BLE001
                        rospy.logerr("[Test] Cannot reset: %s - stopping.", reset_exc)
                        quit_after = True
            finally:
                with open(os.path.join(self.out, "rounds.jsonl"), "a") as f:
                    f.write(json.dumps(jsonable(rec)) + "\n")
            if quit_after:
                break

    def round(self, n, rec):
        """One round; returns True when the operator asked to quit."""
        if self.pnp is not None:
            self.reset("reset before scan")
        color, depth = self.grab()
        grasp = self.plan(color, depth)
        refusal = self.check(grasp)
        rec["grasp"] = grasp
        rec["refused"] = refusal
        rec["plan_image"] = self.save_plan_image(n, color, grasp, refusal)
        x, y, z = grasp["target_world"]
        print("[Test] round %d: score %.3f, tilt %.1f deg, target (%.3f, %.3f, %.3f)%s"
              % (n, grasp["score"], grasp["tilt_deg"], x, y, z,
                 "" if refusal is None else " - NOT SENT: " + refusal))
        print("[Test]   image: %s" % shown(rec["plan_image"]))
        if grasp["planner_message"].startswith("panel: "):
            print("[Test]   planner panel: %s" % shown(grasp["planner_message"][len("panel: "):]))

        if self.pnp is None or refusal is not None:
            return ask("Enter = plan again, q = quit", ("q",)) == "q"

        answer = ask("Enter = move the arm to this grasp, r = plan again, q = quit", ("r", "q"))
        rec["operator"] = {"": "move", "r": "replan", "q": "quit"}[answer]
        if answer != "":
            return answer == "q"

        pre, final = self.pnp.compute_pick_joints(grasp["target_world"], grasp["target_rotation"])
        rec["ik"] = {"pre_joints": pre, "final_joints": final}
        self.pnp._move(self.pnp.arm.goto_joints, pre, duration=3, desc="goto pre-grasp")
        goal = np.asarray(grasp["target_world"])
        if self.args.hover:
            ori = grasp["target_rotation"] if grasp["target_rotation"] is not None else self.dexnet_pnp.VERTICAL_ORI
            goal = goal - ori[:, 2] * self.dexnet_pnp.APPROACH_DISTANCE
        else:
            self.pnp._move(self.pnp.arm.goto_joints, final, duration=2, desc="descend to grasp")
        rospy.sleep(0.5)  # let the arm settle before measuring and photographing it

        reach = self.reached()
        reach["goal_world"] = goal
        reach["error_mm"] = 1000 * (reach["fk_world"] - goal)
        rec["reached"] = reach
        arm_color, _ = self.grab(depth=False)
        rec["arm_image"] = self.save_arm_image(n, arm_color, grasp, reach["fk_world"], goal)
        print("[Test]   arm reached (FK) %s, error %s mm"
              % (np.round(reach["fk_world"], 3), np.round(reach["error_mm"], 0)))
        print("[Test]   image: %s" % shown(rec["arm_image"]))

        answer = ask("Enter = reset the arm, q = reset and quit", ("q",))
        if not self.args.hover:
            self.pnp._move(self.pnp.arm.goto_joints, pre, duration=2, desc="back off")
        self.reset("reset after grasp")
        return answer == "q"


def main():
    parser = argparse.ArgumentParser(description="Plan a suction grasp, draw it, send the arm there (no vacuum).")
    parser.add_argument("--out", required=True, help="Folder for the images and rounds.jsonl.")
    parser.add_argument("--service", default="/grasp_test_planner/plan_grasp")
    parser.add_argument("--no-arm", action="store_true", help="Plan and draw only; no robot.")
    parser.add_argument("--hover", action="store_true",
                        help="Stop 10 cm above the grasp, along its approach, instead of touching it.")
    parser.add_argument("--straight-down", action="store_true", help="Ignore the grasp tilt and approach vertically.")
    parser.add_argument("--max-tilt-deg", type=float, default=35.0,
                        help="Do not send grasps tilted more than this (the network's labels stop at 35).")
    parser.add_argument("--offset-x", type=float, default=0.0, metavar="M",
                        help="World-frame X correction (metres) added to the grasp before the arm moves.")
    parser.add_argument("--offset-y", type=float, default=0.0, metavar="M", help="As --offset-x, for Y.")
    args, _ = parser.parse_known_args(rospy.myargv(argv=sys.argv)[1:])

    rospy.init_node("grasp_test")
    test = GraspTest(args)
    rospy.loginfo("[Test] Writing to %s", args.out)
    try:
        test.run()
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        rospy.logwarn("[Test] Interrupted")
    return 0


if __name__ == "__main__":
    sys.exit(main())
