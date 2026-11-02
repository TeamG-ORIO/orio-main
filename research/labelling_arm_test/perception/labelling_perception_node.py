#!/usr/bin/env python3
"""Label placement from the ZED with Fast-FoundationStereo depth (labelling arm test).

The demo's /compute_grasps_labelling finds the label spot in the ZED image but cannot use
the ZED's own depth (too noisy), so the state machine carries the pick height over from the
input bin instead. This node runs the same label placement (GroundingDINO + SAM, then
opt_label_loc) and reads the depth at the chosen pixel from /ffs/depth, so the pose it
publishes has a measured z and the labelling arm needs nothing from the other arm.

    /labelling_test/compute_label_pose   std_srvs/Trigger
    /labelling_test/label_pose_z1        geometry_msgs/PoseArray in panda_link0, one pose
    /labelling_test/label_pose_z2        the same for zone 2

Position: the chosen pixel back-projected with the camera_info intrinsics at the measured
depth and taken through the ZED-to-robot TF (zed_to_label_tf.yaml). Orientation: that TF's
rotation turned by the label angle about the camera's z, as handle_labelling publishes it.

~depth_source   'ffs' (default): ask /ffs/compute for a fresh depth and segment the left
                frame it was computed from, so pixels line up.
                'zed': the wrapper's own depth_registered, for a side-by-side comparison.
~text_prompt    GroundingDINO prompt (default: the demo's TEXT_PROMPT). A pale, flat
                packet scores ~0.19 on "bright box on black background" and is dropped;
                "packet" scores it ~0.44.
~box_threshold  lowest GroundingDINO score kept (default: the demo's BOX_THRESHOLD, 0.25).
Debug images of each call go to $ORIO_LBL_IMAGE_DIR (masks, boxes, chosen points, depth).

Runs in the orio/perception container (run_lbl_perception.sh) with the perception scripts
on PYTHONPATH. The prompt, thresholds, crop, model paths and TF are imported from
perception_control_combined_pass_through.py, so the two stay in step; that import also
creates its scratch image dirs under ORIO_PERCEPTION_ASSETS, as the demo does.
"""

import os
import sys
import time
import types

import cv2
import numpy as np
import rospy
from geometry_msgs.msg import Pose, PoseArray
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import CameraInfo, Image
from std_srvs.srv import Trigger, TriggerResponse

sys.path.insert(0, os.environ.get('ORIO_PERCEPTION_SCRIPTS',
                                  '/home/ros_ws/src/devel_packages/orio_perception/scripts'))
import perception_control_combined_pass_through as perc  # noqa: E402
from opt_label_location import is_safe_oriented_placement, opt_label_loc  # noqa: E402
import groundingdino.datasets.transforms as T  # noqa: E402
from groundingdino.util.inference import load_model  # noqa: E402
from segment_anything import SamPredictor, sam_model_registry  # noqa: E402

# Half-width of the window around the chosen pixel whose median depth is used: a single
# pixel can be a hole or an outlier, 7x7 at 1080p is still well inside a label's footprint.
DEPTH_WINDOW = 3

ZONE_TOPICS = {1: '/labelling_test/label_pose_z1', 2: '/labelling_test/label_pose_z2'}

# The label footprint opt_label_loc tests a placement with (pixels, at the crop's full
# resolution): the defaults of is_safe_oriented_placement, so the drawing matches the check.
_FOOTPRINT_PARAMS = __import__('inspect').signature(is_safe_oriented_placement).parameters
LABEL_W_PX = _FOOTPRINT_PARAMS['label_w'].default
LABEL_H_PX = _FOOTPRINT_PARAMS['label_h'].default


def normalise_label_angle(angle_deg):
    """The angle in (-90, 90]. A label is the same rectangle turned by 180 deg, and
    get_object_orientation's `angle % 180` jumps between ~0 and ~180 for a nearly
    axis-aligned item, so without this the commanded tool yaw flips by 180 deg from one
    frame to the next for an identical placement."""
    a = angle_deg % 180.0
    return a - 180.0 if a > 90.0 else a


def label_footprint(u, v, angle_deg):
    """Corners of the label's footprint at (u, v). opt_label_loc returns the footprint's
    angle + 90, so take the 90 back off to draw the box it actually checked."""
    rect = ((float(u), float(v)), (float(LABEL_W_PX), float(LABEL_H_PX)), float(angle_deg) - 90.0)
    return np.int32(cv2.boxPoints(rect)).reshape(-1, 1, 2)

ENCODING_CHANNELS = {'bgra8': 4, 'rgba8': 4, 'bgr8': 3, 'rgb8': 3}


def decode_color(msg):
    """ROS Image -> RGB uint8 HxWx3 (ZED bgra8, or the FFS node's rgb8)."""
    channels = ENCODING_CHANNELS.get(msg.encoding.lower())
    if channels is None:
        raise RuntimeError("unsupported colour encoding %r" % msg.encoding)
    rows = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
    arr = rows[:, :msg.width * channels].reshape(msg.height, msg.width, channels)[:, :, :3]
    if msg.encoding.lower().startswith('bgr'):
        arr = arr[:, :, ::-1]
    return np.ascontiguousarray(arr)


def decode_depth(msg):
    """ROS depth Image -> float32 metres, NaN where invalid."""
    if msg.encoding == '32FC1':
        depth = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width).copy()
    elif msg.encoding == '16UC1':
        depth = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
        depth = depth.astype(np.float32) / perc.DEPTH_UNIT_LBL
    else:
        raise RuntimeError("unsupported depth encoding %r" % msg.encoding)
    depth[~np.isfinite(depth) | (depth <= 0)] = np.nan
    return depth


def sample_depth(depth, u, v, r=DEPTH_WINDOW):
    """(median depth in the window around (u, v), number of valid pixels used)."""
    win = depth[max(v - r, 0):v + r + 1, max(u - r, 0):u + r + 1]
    vals = win[np.isfinite(win)]
    if vals.size == 0:
        return None, 0
    return float(np.median(vals)), int(vals.size)


class LabellingPerceptionNode(object):
    def __init__(self):
        rospy.init_node('labelling_test_perception')
        self.depth_source = rospy.get_param('~depth_source', 'ffs')
        if self.depth_source not in ('ffs', 'zed'):
            raise ValueError("~depth_source must be ffs or zed, got %r" % self.depth_source)
        self.image_dir = os.environ.get('ORIO_LBL_IMAGE_DIR') or None
        if self.image_dir:
            os.makedirs(self.image_dir, exist_ok=True)
        self.text_prompt = str(rospy.get_param('~text_prompt', perc.TEXT_PROMPT)).strip() or perc.TEXT_PROMPT
        # _detect_and_segment reads the threshold from its module; this process's copy of
        # the module is the only one changed.
        perc.BOX_THRESHOLD = float(rospy.get_param('~box_threshold', perc.BOX_THRESHOLD))
        rospy.loginfo("Detection prompt %r, box threshold %.2f, text threshold %.2f",
                      self.text_prompt, perc.BOX_THRESHOLD, perc.TEXT_THRESHOLD)

        rospy.loginfo("Loading SAM & GroundingDINO (as the demo's perception node)…")
        sam = sam_model_registry[perc.MODEL_TYPE](checkpoint=perc.SAM_CHECKPOINT)
        sam.to(device=perc.device)
        # CombinedPerceptionNode._detect_and_segment only reads these four attributes, so it
        # runs unchanged on this stand-in instead of a copy of its code.
        self.seg = types.SimpleNamespace(
            predictor=SamPredictor(sam),
            gdino_model=load_model(perc.CONFIG_PATH, perc.WEIGHTS_PATH),
            gdino_transform=T.Compose([
                T.RandomResize([800], max_size=1333),
                T.ToTensor(),
                T.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225]),
            ]),
            _inference_lock=__import__('threading').RLock())
        rospy.loginfo("Models loaded.")

        self.tf_lbl = perc.CombinedPerceptionNode._load_tf(None, perc.LBL_TF_YAML)
        rospy.loginfo("Labelling camera TF from %s", perc.LBL_TF_YAML)

        # Latest frames. For 'ffs' the left frame and camera_info are kept by stamp so the
        # ones belonging to a depth message can be found; the depth's arrival time tells a
        # fresh result from the one before the request.
        self.depth_msg = None
        self.depth_rx = rospy.Time(0)
        self.colors = {}
        self.infos = {}
        if self.depth_source == 'ffs':
            color_topic, depth_topic, info_topic = '/ffs/left', '/ffs/depth', '/ffs/camera_info'
        else:
            color_topic = perc.LBL_RGB_TOPIC
            depth_topic = perc.LBL_DEPTH_TOPIC
            info_topic = '/zedm/zed_node/rgb/camera_info'
        rospy.Subscriber(color_topic, Image, self._keep(self.colors), queue_size=1,
                         buff_size=2 ** 24)
        rospy.Subscriber(info_topic, CameraInfo, self._keep(self.infos), queue_size=1)
        rospy.Subscriber(depth_topic, Image, self._on_depth, queue_size=1, buff_size=2 ** 24)
        rospy.loginfo("Depth source '%s': colour %s, depth %s, intrinsics %s",
                      self.depth_source, color_topic, depth_topic, info_topic)

        self.pubs = {z: rospy.Publisher(t, PoseArray, queue_size=1, latch=True)
                     for z, t in ZONE_TOPICS.items()}
        rospy.Service('/labelling_test/compute_label_pose', Trigger, self.handle)
        rospy.loginfo("/labelling_test/compute_label_pose ready (images -> %s)",
                      self.image_dir or "not saved")

    @staticmethod
    def _keep(store, keep=30):
        def cb(msg):
            store[msg.header.stamp.to_nsec()] = msg
            while len(store) > keep:
                del store[min(store)]
        return cb

    def _on_depth(self, msg):
        self.depth_msg = msg
        self.depth_rx = rospy.Time.now()

    # ── Frames ────────────────────────────────────────────────────────────────
    def _frames(self):
        """(colour msg, depth msg, camera_info) of one frame, per ~depth_source."""
        if self.depth_source == 'ffs':
            t_req = rospy.Time.now()
            try:
                rospy.wait_for_service('/ffs/compute', timeout=5.0)
                resp = rospy.ServiceProxy('/ffs/compute', Trigger)()
            except Exception as exc:  # noqa: BLE001 — reported in the response
                raise RuntimeError("/ffs/compute unavailable: %s" % exc)
            if not resp.success:
                raise RuntimeError("/ffs/compute: %s" % resp.message)
            rospy.loginfo("[FFS] %s", resp.message)
            deadline = rospy.Time.now() + rospy.Duration(10.0)
            while rospy.Time.now() < deadline:
                depth = self.depth_msg
                if depth is not None and self.depth_rx > t_req:
                    key = depth.header.stamp.to_nsec()
                    if key in self.colors and key in self.infos:
                        return self.colors[key], depth, self.infos[key]
                rospy.sleep(0.05)
            raise RuntimeError("no /ffs/depth with its /ffs/left + /ffs/camera_info within 10 s")

        deadline = rospy.Time.now() + rospy.Duration(5.0)
        while rospy.Time.now() < deadline:
            if self.depth_msg is not None and self.colors and self.infos:
                return (self.colors[max(self.colors)], self.depth_msg,
                        self.infos[max(self.infos)])
            rospy.sleep(0.05)
        raise RuntimeError("no ZED colour/depth/camera_info within 5 s")

    # ── Service ───────────────────────────────────────────────────────────────
    def handle(self, _req):
        t0 = time.time()
        try:
            color_msg, depth_msg, info = self._frames()
            color = decode_color(color_msg)
            depth = decode_depth(depth_msg)
        except RuntimeError as exc:
            rospy.logerr("[Label] %s", exc)
            return TriggerResponse(success=False, message=str(exc))
        if depth.shape != color.shape[:2]:
            msg = "depth %s and colour %s differ in size" % (depth.shape, color.shape[:2])
            rospy.logerr("[Label] %s", msg)
            return TriggerResponse(success=False, message=msg)
        fx, fy, cx, cy = info.K[0], info.K[4], info.K[2], info.K[5]
        stamp = color_msg.header.stamp

        # The label placement as handle_labelling runs it: on the cropped frame.
        crop = color[perc.LBL_CROP_Y1:perc.LBL_CROP_Y2, perc.LBL_CROP_X1:perc.LBL_CROP_X2]
        masks, boxes, phrases, logits = perc.CombinedPerceptionNode._detect_and_segment(
            self.seg, crop, caption=self.text_prompt)
        if masks is None:
            self._save(stamp, color, depth, crop, None, None, {})
            msg = "GroundingDINO found 0 objects (prompt %r, threshold %.2f)" % (
                self.text_prompt, perc.BOX_THRESHOLD)
            rospy.logwarn("[Label] %s", msg)
            return TriggerResponse(success=False, message=msg)
        joint_mask = np.zeros_like(masks[0])
        for m in masks:
            joint_mask = (joint_mask | m).astype(np.uint8)

        results = {}
        for zone in (1, 2):
            opt_x, opt_y, angle_deg = opt_label_loc(crop, joint_mask, label_zone_num=zone,
                                                    visualize=False)
            if opt_x is None:
                results[zone] = dict(error="no candidate placement")
                continue
            angle_deg = normalise_label_angle(angle_deg)
            u, v = int(opt_x) + perc.LBL_CROP_X1, int(opt_y) + perc.LBL_CROP_Y1
            z, n = sample_depth(depth, u, v)
            if z is None:
                results[zone] = dict(px=(u, v), angle=angle_deg,
                                     error="no valid depth within %d px" % DEPTH_WINDOW)
                continue
            cam = np.array([(u - cx) * z / fx, (v - cy) * z / fy, z, 1.0])
            world = self.tf_lbl @ cam
            quat = R.from_matrix(
                self.tf_lbl[:3, :3] @ R.from_euler('z', np.radians(angle_deg)).as_matrix()
            ).as_quat()
            results[zone] = dict(px=(u, v), angle=angle_deg, depth=z, n=n,
                                 cam=cam[:3], world=world[:3], quat=quat)

            msg = PoseArray()
            msg.header.stamp = stamp
            msg.header.frame_id = "panda_link0"
            p = Pose()
            p.position.x, p.position.y, p.position.z = world[:3]
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = quat
            msg.poses.append(p)
            self.pubs[zone].publish(msg)
            rospy.loginfo("[Label] zone %d: px (%d, %d) depth %.3f m (%d px) -> camera "
                          "(%.3f, %.3f, %.3f) -> world (%.3f, %.3f, %.3f), angle %.1f deg",
                          zone, u, v, z, n, cam[0], cam[1], cam[2],
                          world[0], world[1], world[2], angle_deg)

        saved = self._save(stamp, color, depth, crop, masks, boxes, results,
                           phrases=phrases, logits=logits, intrinsics=(fx, fy, cx, cy))
        ok = any('world' in r for r in results.values())
        parts = []
        for zone, r in sorted(results.items()):
            if 'world' in r:
                parts.append("zone %d: world (%.3f, %.3f, %.3f) from depth %.3f m at px %s"
                             % (zone, r['world'][0], r['world'][1], r['world'][2],
                                r['depth'], r['px']))
            else:
                parts.append("zone %d: %s" % (zone, r['error']))
        summary = "%s [%s depth, fx %.1f, %.1f s%s]" % (
            "; ".join(parts), self.depth_source, fx, time.time() - t0,
            (", images " + saved) if saved else "")
        rospy.loginfo("[Label] %s", summary)
        return TriggerResponse(success=ok, message=summary)

    # ── Debug images ──────────────────────────────────────────────────────────
    def _save(self, stamp, color, depth, crop, masks, boxes, results, phrases=None,
              logits=None, intrinsics=None):
        """Colour with masks/boxes/points, the depth as a colour map, and the raw arrays
        (so a call can be rerun offline). Returns the stem of the files, or None."""
        if not self.image_dir:
            return None
        stem = os.path.join(self.image_dir, "label_%s" % time.strftime("%H%M%S"))
        colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0), (0, 255, 255)]
        vis = color.copy()
        x0, y0 = perc.LBL_CROP_X1, perc.LBL_CROP_Y1
        if masks is not None:
            overlay = vis[y0:perc.LBL_CROP_Y2, x0:perc.LBL_CROP_X2].copy()
            for i, m in enumerate(masks):
                overlay[m] = colors[i % len(colors)]
            vis[y0:perc.LBL_CROP_Y2, x0:perc.LBL_CROP_X2] = (
                0.5 * crop + 0.5 * overlay).astype(np.uint8)
            for i, (x1, y1, x2, y2) in enumerate(boxes):
                c = colors[i % len(colors)]
                cv2.rectangle(vis, (int(x1) + x0, int(y1) + y0), (int(x2) + x0, int(y2) + y0), c, 2)
                cv2.putText(vis, "%s %.2f" % (phrases[i], float(logits[i])),
                            (int(x1) + x0, max(int(y1) + y0 - 6, 10)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, c, 2)
        cv2.rectangle(vis, (x0, y0), (perc.LBL_CROP_X2, perc.LBL_CROP_Y2), (255, 255, 255), 1)

        finite = np.isfinite(depth)
        if finite.any():
            lo, hi = np.percentile(depth[finite], [2, 98])
            norm = np.clip((depth - lo) / max(hi - lo, 1e-6), 0, 1)
            depth_vis = cv2.applyColorMap((norm * 255).astype(np.uint8), cv2.COLORMAP_TURBO)[..., ::-1]
            depth_vis[~finite] = 0
        else:
            depth_vis = np.zeros_like(color)
        depth_vis = np.ascontiguousarray(depth_vis)

        for zone, r in results.items():
            if 'px' not in r:
                continue
            u, v = r['px']
            c = (255, 128, 0) if zone == 1 else (128, 0, 255)
            text = ("Z%d %.3f m  %.1f deg" % (zone, r['depth'], r['angle']) if 'depth' in r
                    else "Z%d NO DEPTH" % zone)
            box = label_footprint(u, v, r['angle'])
            for img in (vis, depth_vis):
                cv2.polylines(img, [box], isClosed=True, color=c, thickness=2)
                cv2.circle(img, (u, v), 4, c, -1)
                cv2.putText(img, text, (u + 14, v - LABEL_H_PX), cv2.FONT_HERSHEY_SIMPLEX, 0.7, c, 2)
            if 'world' in r:
                cv2.putText(vis, "world (%.3f, %.3f, %.3f)" % tuple(r['world']),
                            (u + 14, v + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, c, 2)
        footer = "%s depth, stamp %.3f, prompt %r >= %.2f" % (
            self.depth_source, stamp.to_sec(), self.text_prompt, perc.BOX_THRESHOLD)
        if intrinsics:
            footer += ", fx %.1f fy %.1f cx %.1f cy %.1f" % intrinsics
        cv2.putText(vis, footer, (10, color.shape[0] - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (255, 255, 255), 2)

        cv2.imwrite(stem + ".png", np.concatenate([vis, depth_vis], axis=1)[..., ::-1])
        cv2.imwrite(stem + "_color.png", color[..., ::-1])
        np.save(stem + "_depth.npy", depth)
        return stem

if __name__ == '__main__':
    LabellingPerceptionNode()
    rospy.spin()
