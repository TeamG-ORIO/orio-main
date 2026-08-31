# This file directly passes throught he optical label location in the camera frame without transforming to the 
# robot world frame. This is a stop gap measure for the zed depth issues. The intent is for
# the calling script to handle the trasnformation using the depth from the Asus camera.


#!/usr/bin/env python3

import numpy as np
import open3d as o3d
import torch
import rospy
import os
import yaml
import threading
from PIL import Image
from scipy.spatial.transform import Rotation as R_scipy

# ROS Imports
from sensor_msgs.msg import Image as RosImage
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Float32
from std_srvs.srv import Trigger, TriggerResponse

# SAM & GroundingDINO
from segment_anything import sam_model_registry, SamPredictor
from groundingdino.util.inference import load_model, predict
import groundingdino.datasets.transforms as T

# Custom modules
from grasp_solver import optimize_grasp_pose
from opt_label_location import opt_label_loc

# ==============================================================================
# SHARED PARAMETERS
# ==============================================================================
WEIGHTS_NAME   = "groundingdino_swint_ogc.pth"
MODEL_TYPE     = "vit_b"
TEXT_PROMPT    = "bright box on black background"
BOX_THRESHOLD  = 0.25
TEXT_THRESHOLD = 0.4
device         = "cuda" if torch.cuda.is_available() else "cpu"

# Asset roots (env-overridable; default to CWD): weights + image dirs under
# ORIO_PERCEPTION_ASSETS, GroundingDINO checkout under ORIO_GROUNDINGDINO_DIR.
HOME         = os.environ.get("ORIO_PERCEPTION_ASSETS", os.getcwd())
GDINO_DIR    = os.environ.get("ORIO_GROUNDINGDINO_DIR", os.path.join(HOME, "GroundingDINO"))
CONFIG_PATH  = os.path.join(GDINO_DIR, "groundingdino/config/GroundingDINO_SwinT_OGC.py")
WEIGHTS_PATH = os.path.join(HOME, "weights", WEIGHTS_NAME)
SAM_CHECKPOINT = os.path.join(HOME, "SAM_weights", "sam_vit_b_01ec64.pth")

# Scratch image-dump dirs (debug PNGs, git-ignored) — ensure they exist so the
# node can write regardless of launch dir.
for _d in ("Xtion_imgs", "ZED_imgs", "Segmented_imgs"):
    os.makedirs(os.path.join(HOME, _d), exist_ok=True)

# ==============================================================================
# DEPTH SAMPLING PARAMETERS
# ==============================================================================
DEPTH_SAMPLE_N          = 50    # number of random points to sample around grasp pixel
DEPTH_SAMPLE_RADIUS_M   = 0.025 # sampling radius in metres (2.5 cm)

# ==============================================================================
# PICK-AND-PLACE PARAMETERS  (Xtion camera)
# ==============================================================================
PNP_RGB_TOPIC   = "/camera/rgb/image_raw"
PNP_DEPTH_TOPIC = "/camera/depth/image_raw"
PNP_POSE_TOPIC  = "/grasp_poses"
# Repo root, env-anchored (ORIO_REPO), for the manipulation TF yamls.
_REPO_ROOT      = os.environ.get("ORIO_REPO",
                                 os.path.abspath(os.path.join(os.getcwd(), "../../..", "16662_RobotAutonomy")))
PNP_TF_YAML     = os.path.join(_REPO_ROOT, "src/devel_packages/manipulation/config/realsense_tf.yaml")

PNP_CROP_Y1, PNP_CROP_Y2 = 40, 405
PNP_CROP_X1, PNP_CROP_X2 = 115, 515

PNP_INTRINSICS = o3d.camera.PinholeCameraIntrinsic(
    o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault)

CUP_RADIUS_CM       = 0.02
DIST_BETW_CUPS_CM   = 0.05
MIN_POINTS_FOR_SEAL = 15
THETA_ALLOWED_RADIANS = np.radians(15)
DEPTH_UNIT_PNP      = 1000.0
MAX_DEPTH_M         = 1.5
MIN_DEPTH_M         = 0.0
OUTLIER_NEIGHBORS   = 10
OUTLIER_STD_RATIO   = 5.0

# Final grasp offsets in metres
OFFSET_X = 0.015
OFFSET_Y = -0.05

# ==============================================================================
# LABELLING PARAMETERS  (ZED camera)
# ==============================================================================
LBL_RGB_TOPIC   = "/zedm/zed_node/rgb/image_rect_color"
LBL_DEPTH_TOPIC = "/zedm/zed_node/depth/depth_registered"
LBL_ZONE1_TOPIC = "/grasp_poses_labelling_z1"
LBL_ZONE2_TOPIC = "/grasp_poses_labelling_z2"
LBL_TF_YAML     = os.path.join(_REPO_ROOT, "src/devel_packages/manipulation/config/zed_to_label_tf.yaml")

LBL_WIDTH, LBL_HEIGHT = 1920, 1080
LBL_FX, LBL_FY        = 1509.65, 1509.65
LBL_CX, LBL_CY        = 964.33, 559.28
DEPTH_UNIT_LBL        = 1000.0

LBL_CROP_Y1, LBL_CROP_Y2 = 180, 1050
LBL_CROP_X1, LBL_CROP_X2 = 160, 1860

# LBL_CROP_Y1, LBL_CROP_Y2 = 0, 1080
# LBL_CROP_X1, LBL_CROP_X2 = 0, 1920

# ==============================================================================
# COMBINED NODE
# ==============================================================================
class CombinedPerceptionNode:
    def __init__(self):
        rospy.init_node('combined_perception_node')

        # ── Load models once ──────────────────────────────────────────────────
        rospy.loginfo("Loading AI Models (SAM & GroundingDINO)...")
        self.sam = sam_model_registry[MODEL_TYPE](checkpoint=SAM_CHECKPOINT)
        self.sam.to(device=device)
        self.predictor = SamPredictor(self.sam)
        self._inference_lock = threading.Lock()
        self.gdino_model = load_model(CONFIG_PATH, WEIGHTS_PATH)
        self.gdino_transform = T.Compose([
            T.RandomResize([800], max_size=1333),
            T.ToTensor(),
            T.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225]),
        ])
        rospy.loginfo("Models loaded.")

        # ── Pick-and-place TF ─────────────────────────────────────────────────
        self.tf_pnp = self._load_tf(PNP_TF_YAML)
        rospy.loginfo(f"Loaded PnP camera TF from {PNP_TF_YAML}")

        # ── Labelling TF ──────────────────────────────────────────────────────
        self.tf_lbl = self._load_tf(LBL_TF_YAML)
        rospy.loginfo(f"Loaded labelling camera TF from {LBL_TF_YAML}")

        # ── Stored grasp pixel coords from last compute_grasps call ──────────
        self._last_grasp_pixel_x = None
        self._last_grasp_pixel_y = None
        self._last_pnp_depth_msg = None  # depth image snapshot at grasp time

        # ── Camera subscribers ────────────────────────────────────────────────
        self.pnp_rgb_msg   = None
        self.pnp_depth_msg = None
        rospy.Subscriber(PNP_RGB_TOPIC,   RosImage, lambda m: setattr(self, 'pnp_rgb_msg',   m))
        rospy.Subscriber(PNP_DEPTH_TOPIC, RosImage, lambda m: setattr(self, 'pnp_depth_msg', m))

        self.lbl_rgb_msg   = None
        self.lbl_depth_msg = None
        rospy.Subscriber(LBL_RGB_TOPIC,   RosImage, lambda m: setattr(self, 'lbl_rgb_msg',   m))
        rospy.Subscriber(LBL_DEPTH_TOPIC, RosImage, lambda m: setattr(self, 'lbl_depth_msg', m))

        # ── Publishers ────────────────────────────────────────────────────────
        self.target_frame = "panda_link0"
        self.pnp_pub        = rospy.Publisher(PNP_POSE_TOPIC,       PoseArray, queue_size=1)
        self.lbl_pub1       = rospy.Publisher(LBL_ZONE1_TOPIC,      PoseArray, queue_size=1)
        self.lbl_pub2       = rospy.Publisher(LBL_ZONE2_TOPIC,      PoseArray, queue_size=1)
        self.depth_query_pub = rospy.Publisher('/grasp_point_depth', Float32,   queue_size=1)

        # ── Services ──────────────────────────────────────────────────────────
        rospy.Service('/compute_grasps',           Trigger, self.handle_pnp)
        rospy.Service('/compute_grasps_labelling', Trigger, self.handle_labelling)
        rospy.Service('/get_depth_at_grasp',       Trigger, self.handle_depth_at_grasp)
        rospy.loginfo("Both perception services ready.")

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _load_tf(self, yaml_path):
        with open(yaml_path, 'r') as f:
            data = yaml.safe_load(f)
        t = data['pose']['translation']
        r = data['pose']['rotation']
        rot_mat = R_scipy.from_quat([r['x'], r['y'], r['z'], r['w']]).as_matrix()
        tf = np.eye(4)
        tf[:3, :3] = rot_mat
        tf[:3, 3]  = [t['x'], t['y'], t['z']]
        return tf

    def _detect_and_segment(self, color_img):
        """Run GroundingDINO + SAM on a uint8 RGB image.

        Returns (best_masks, boxes_xyxy, phrases, logits) where best_masks is a
        list of boolean masks (one per detected object), boxes_xyxy is an int
        array of shape (N, 4) in [x1, y1, x2, y2] pixel coords, phrases and
        logits are the corresponding GroundingDINO labels and confidence scores.
        Returns (None, None, None, None) if no objects found.

        Serialised by _inference_lock so that labelling and PnP calls never
        share gdino_model, gdino_transform, or the SAM predictor concurrently.
        """
        if color_img.shape[2] == 4:
            color_img = color_img[:, :, :3]

        with self._inference_lock:
            image_pil = Image.fromarray(color_img)
            image_transformed, _ = self.gdino_transform(image_pil, None)

            with torch.no_grad():
                boxes, logits, phrases = predict(
                    model=self.gdino_model, image=image_transformed, caption=TEXT_PROMPT,
                    box_threshold=BOX_THRESHOLD, text_threshold=TEXT_THRESHOLD,
                )

            h, w = color_img.shape[:2]
            keep = [i for i, c in enumerate(boxes) if (c[2]*w * c[3]*h) < 0.7*h*w]
            boxes_xyxy = np.array([
                [int((boxes[i][0]-boxes[i][2]/2)*w), int((boxes[i][1]-boxes[i][3]/2)*h),
                 int((boxes[i][0]+boxes[i][2]/2)*w), int((boxes[i][1]+boxes[i][3]/2)*h)]
                for i in keep
            ])
            kept_logits  = [logits[i]  for i in keep]
            kept_phrases = [phrases[i] for i in keep]

            if len(boxes_xyxy) == 0:
                return None, None, None, None

            with torch.no_grad():
                self.predictor.set_image(color_img)
                best_masks = []
                for box in boxes_xyxy:
                    masks, scores, _ = self.predictor.predict(box=box, multimask_output=True)
                    best_masks.append(masks[np.argmax(scores)])

        return best_masks, boxes_xyxy, kept_phrases, kept_logits

    def decode_ros_images(self, rgb_msg, depth_msg):
        """Decode ROS Image messages to numpy arrays."""
        try:
            color = np.frombuffer(rgb_msg.data, dtype=np.uint8).reshape(rgb_msg.height, rgb_msg.width, -1)
            if "bgr" in rgb_msg.encoding:
                color = color[:, :, ::-1].copy()
            if depth_msg.encoding == '16UC1':
                depth = np.frombuffer(depth_msg.data, dtype=np.uint16).reshape(depth_msg.height, depth_msg.width)
            elif depth_msg.encoding == '32FC1':
                depth = np.frombuffer(depth_msg.data, dtype=np.float32).reshape(depth_msg.height, depth_msg.width)
            else:
                return None, None
            return color, depth
        except Exception:
            return None, None

    # ── Pick-and-place service ────────────────────────────────────────────────

    def handle_pnp(self, req):
        res = TriggerResponse()
        rospy.loginfo("PnP grasp computation triggered!")

        if self.pnp_rgb_msg is None or self.pnp_depth_msg is None:
            res.success = False
            res.message = "No images received from PnP camera."
            return res

        color_base, depth_base = self.decode_ros_images(self.pnp_rgb_msg, self.pnp_depth_msg)
        if color_base is None:
            res.success = False
            res.message = "Failed to decode PnP camera images."
            return res

        color_base = color_base[PNP_CROP_Y1:PNP_CROP_Y2, PNP_CROP_X1:PNP_CROP_X2]
        depth_base = depth_base[PNP_CROP_Y1:PNP_CROP_Y2, PNP_CROP_X1:PNP_CROP_X2]

        Image.fromarray(color_base).save(
            os.path.join(HOME, "Xtion_imgs", f"input_color_{rospy.Time.now().secs}.png"))

        best_masks, boxes_xyxy, phrases, logits = self._detect_and_segment(color_base)
        if best_masks is None:
            res.success = False
            res.message = "GroundingDINO found 0 objects."
            return res

        # Save debug image with SAM masks and GroundingDINO bounding boxes
        import cv2
        colors_list = [(255, 0, 0, 255), (0, 255, 0, 255), (0, 0, 255, 255), (255, 255, 0, 255), (0, 255, 255, 255)]
        n_ch = color_base.shape[2]
        debug_img = color_base.copy()
        overlay   = debug_img.copy()
        for i, mask in enumerate(best_masks):
            overlay[mask] = colors_list[i % len(colors_list)][:n_ch]
        debug_img = (debug_img * 0.5 + overlay * 0.5).astype(np.uint8)
        for i, (x1, y1, x2, y2) in enumerate(boxes_xyxy):
            color_rgb = tuple(int(v) for v in colors_list[i % len(colors_list)][:3])
            cv2.rectangle(debug_img, (int(x1), int(y1)), (int(x2), int(y2)), color_rgb, 2)
            label = f"{phrases[i]} {float(logits[i]):.2f}"
            cv2.putText(debug_img, label, (int(x1), max(int(y1) - 6, 10)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color_rgb, 1)
        Image.fromarray(debug_img).save(
            os.path.join(HOME, "Segmented_imgs", f"debug_masks_{rospy.Time.now().secs}.png"))

        # Build cropped intrinsics
        intr = PNP_INTRINSICS
        cropped_intr = o3d.camera.PinholeCameraIntrinsic(
            width  = PNP_CROP_X2 - PNP_CROP_X1,
            height = PNP_CROP_Y2 - PNP_CROP_Y1,
            fx=intr.get_focal_length()[0],   fy=intr.get_focal_length()[1],
            cx=intr.get_principal_point()[0] - PNP_CROP_X1,
            cy=intr.get_principal_point()[1] - PNP_CROP_Y1,
        )

        pose_array_msg = PoseArray()
        pose_array_msg.header.stamp    = rospy.Time.now()
        pose_array_msg.header.frame_id = self.target_frame

        for mask in best_masks:
            color_masked, depth_masked = color_base.copy(), depth_base.copy()
            color_masked[~mask], depth_masked[~mask] = 0, 0

            rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
                o3d.geometry.Image(color_masked), o3d.geometry.Image(depth_masked),
                depth_scale=DEPTH_UNIT_PNP, depth_trunc=4.0, convert_rgb_to_intensity=False,
            )
            pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, cropped_intr)

            points = np.asarray(pcd.points)
            colors = np.asarray(pcd.colors)
            valid  = (points[:, 2] >= MIN_DEPTH_M) & (points[:, 2] <= MAX_DEPTH_M)
            pcd.points = o3d.utility.Vector3dVector(points[valid])
            pcd.colors = o3d.utility.Vector3dVector(colors[valid])
            pcd.transform(self.tf_pnp)

            grasp_data, _ = optimize_grasp_pose(
                pcd, CUP_RADIUS_CM, DIST_BETW_CUPS_CM, THETA_ALLOWED_RADIANS, MIN_POINTS_FOR_SEAL)

            if grasp_data:
                grasp_depth = abs(grasp_data["center"][2])
                if not (MIN_DEPTH_M <= grasp_depth <= MAX_DEPTH_M):
                    rospy.logwarn(f"Grasp depth ({grasp_depth:.2f}m) out of bounds, skipping.")
                    grasp_data = None

            if grasp_data:
                quat = R_scipy.from_matrix(grasp_data["rotation"]).as_quat()
                p = Pose()
                p.position.x, p.position.y, p.position.z = grasp_data["center"]
                # X and Y Offsets
                p.position.x += OFFSET_X
                p.position.y += OFFSET_Y
                p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = quat
                pose_array_msg.poses.append(p)
                rospy.loginfo(p)
                # Store the pixel-space grasp center for the depth-query service.
                # grasp_data["center"] is in the cropped-image coordinate system before
                # tf_pnp is applied, so we record the 2-D pixel coords directly from the
                # point cloud center projected back: use the pre-transform centroid pixel.
                # We store the depth image snapshot here so handle_depth_at_grasp can
                # sample it after the item has been removed.
                mask_pixels = np.argwhere(mask)  # (row, col)
                if len(mask_pixels):
                    cy_px = float(np.mean(mask_pixels[:, 0]))
                    cx_px = float(np.mean(mask_pixels[:, 1]))
                    # Store in cropped-image coords so handle_depth_at_grasp can
                    # query the cropped depth directly without re-shifting.
                    self._last_grasp_pixel_x = cx_px  # col in cropped image
                    self._last_grasp_pixel_y = cy_px  # row in cropped image
                    self._last_pnp_depth_msg = self.pnp_depth_msg

        if pose_array_msg.poses:
            self.pnp_pub.publish(pose_array_msg)
            res.success = True
            res.message = f"Published {len(pose_array_msg.poses)} PnP poses."
        else:
            res.success = False
            res.message = "Found objects but could not compute valid grasp poses."
        return res

    # ── Depth-at-grasp service ────────────────────────────────────────────────

    def handle_depth_at_grasp(self, req):
        """Return the current depth at the pixel location of the last grasp point.

        Grabs a fresh depth frame from the PnP camera and samples it at the
        (pixel_x, pixel_y) stored during the most recent handle_pnp call.
        The depth is published on /grasp_point_depth (Float32, metres) and also
        returned in the Trigger response message as a float string so the caller
        can use it directly without a separate subscriber.
        """
        res = TriggerResponse()

        if self._last_grasp_pixel_x is None or self._last_grasp_pixel_y is None:
            res.success = False
            res.message = "No grasp pixel stored — call /compute_grasps first."
            return res

        if self.pnp_depth_msg is None:
            res.success = False
            res.message = "No depth image available from PnP camera."
            return res

        _, depth_img = self.decode_ros_images(self.pnp_rgb_msg, self.pnp_depth_msg)
        if depth_img is None:
            res.success = False
            res.message = "Failed to decode PnP depth image."
            return res

        # Crop to the same ROI used during the PnP call so pixel coords align.
        depth_cropped = depth_img[PNP_CROP_Y1:PNP_CROP_Y2, PNP_CROP_X1:PNP_CROP_X2]

        # _last_grasp_pixel_x/y are already in cropped-image coords.
        cx_px = self._last_grasp_pixel_x
        cy_px = self._last_grasp_pixel_y

        h, w = depth_cropped.shape[:2]
        intr = PNP_INTRINSICS
        cropped_intr = o3d.camera.PinholeCameraIntrinsic(
            width  = PNP_CROP_X2 - PNP_CROP_X1,
            height = PNP_CROP_Y2 - PNP_CROP_Y1,
            fx=intr.get_focal_length()[0],   fy=intr.get_focal_length()[1],
            cx=intr.get_principal_point()[0] - PNP_CROP_X1,
            cy=intr.get_principal_point()[1] - PNP_CROP_Y1,
        )
        fx, fy = cropped_intr.get_focal_length()
        cx_crop, cy_crop = cropped_intr.get_principal_point()

        # Convert the metric radius to pixels using the mean focal length.
        # We sample a nominal depth from the center pixel to do this conversion;
        # fall back to a fixed 0.5 m estimate if the center pixel is invalid.
        center_raw = float(depth_cropped[int(round(cy_px)), int(round(cx_px))])
        center_depth_cam = (center_raw / DEPTH_UNIT_PNP) if center_raw > 0 else 0.5
        radius_px = DEPTH_SAMPLE_RADIUS_M * fx / center_depth_cam

        # Draw N random (dx, dy) offsets uniformly within a circle of radius_px.
        rng = np.random.default_rng()
        angles = rng.uniform(0, 2 * np.pi, DEPTH_SAMPLE_N)
        radii  = np.sqrt(rng.uniform(0, 1, DEPTH_SAMPLE_N)) * radius_px
        sample_cols = (cx_px + radii * np.cos(angles)).astype(int)
        sample_rows = (cy_px + radii * np.sin(angles)).astype(int)

        # Clamp to image bounds and collect valid (non-zero) depth readings.
        robot_z_values = []
        for sc, sr in zip(sample_cols, sample_rows):
            if not (0 <= sr < h and 0 <= sc < w):
                continue
            raw = float(depth_cropped[sr, sc])
            if raw <= 0:
                continue
            depth_cam = raw / DEPTH_UNIT_PNP
            X_cam = (sc - cx_crop) * depth_cam / fx
            Y_cam = (sr - cy_crop) * depth_cam / fy
            pt_robot = self.tf_pnp @ np.array([X_cam, Y_cam, depth_cam, 1.0])
            robot_z_values.append(float(pt_robot[2]))

        if not robot_z_values:
            res.success = False
            res.message = "All sampled depth pixels were invalid (zero or out of bounds)."
            return res

        depth_m = float(np.median(robot_z_values))

        rospy.loginfo(
            f"[depth_at_grasp] center_px=({cx_px:.1f},{cy_px:.1f})  "
            f"samples={DEPTH_SAMPLE_N}  valid={len(robot_z_values)}  "
            f"depth_robot_z={depth_m:.4f} m")

        self.depth_query_pub.publish(Float32(data=depth_m))
        res.success = True
        res.message = str(depth_m)
        return res

    # ── Labelling service ─────────────────────────────────────────────────────

    def handle_labelling(self, req):
        res = TriggerResponse()
        rospy.loginfo("Labelling grasp computation triggered!")

        try:
            rgb_msg   = rospy.wait_for_message(LBL_RGB_TOPIC,   RosImage, timeout=3.0)
            depth_msg = rospy.wait_for_message(LBL_DEPTH_TOPIC, RosImage, timeout=3.0)
        except rospy.ROSException:
            res.success = False
            res.message = "Timeout: Could not grab images from labelling camera."
            return res

        color_base, depth_base = self.decode_ros_images(rgb_msg, depth_msg)
        if color_base is None:
            res.success = False
            res.message = "Failed to decode labelling camera images."
            return res

        color_base = color_base[LBL_CROP_Y1:LBL_CROP_Y2, LBL_CROP_X1:LBL_CROP_X2]
        depth_base = depth_base[LBL_CROP_Y1:LBL_CROP_Y2, LBL_CROP_X1:LBL_CROP_X2]

        Image.fromarray(color_base).save(
            os.path.join(HOME, "ZED_imgs", f"input_color_{rospy.Time.now().secs}.png"))

        # Principal point adjusted for crop offset
        lbl_cx = LBL_CX - LBL_CROP_X1
        lbl_cy = LBL_CY - LBL_CROP_Y1

        best_masks, boxes_xyxy, phrases, logits = self._detect_and_segment(color_base)
        if best_masks is None:
            res.success = False
            res.message = "GroundingDINO found 0 objects."
            return res

        # Build joint segmentation mask for opt_label_loc
        joint_mask = np.zeros_like(best_masks[0])
        for mask in best_masks:
            joint_mask = (joint_mask | mask).astype(np.uint8)

        opt_x1, opt_y1, opt_ang_deg1 = opt_label_loc(color_base, joint_mask, label_zone_num=1, visualize=False)
        opt_x2, opt_y2, opt_ang_deg2 = opt_label_loc(color_base, joint_mask, label_zone_num=2, visualize=False)

        # Save debug image for labelling with SAM masks and GroundingDINO bounding boxes
        import cv2
        colors_list = [(255, 0, 0, 255), (0, 255, 0, 255), (0, 0, 255, 255), (255, 255, 0, 255), (0, 255, 255, 255)]
        n_ch = color_base.shape[2]
        debug_lbl = color_base.copy()
        overlay_lbl = debug_lbl.copy()
        for i, mask in enumerate(best_masks):
            overlay_lbl[mask] = colors_list[i % len(colors_list)][:n_ch]
        debug_lbl = (debug_lbl * 0.5 + overlay_lbl * 0.5).astype(np.uint8)
        for i, (x1, y1, x2, y2) in enumerate(boxes_xyxy):
            color_rgb = tuple(int(v) for v in colors_list[i % len(colors_list)][:3])
            cv2.rectangle(debug_lbl, (int(x1), int(y1)), (int(x2), int(y2)), color_rgb, 2)
            label = f"{phrases[i]} {float(logits[i]):.2f}"
            cv2.putText(debug_lbl, label, (int(x1), max(int(y1) - 6, 10)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color_rgb, 1)
        if opt_x1 is not None:
            cv2.circle(debug_lbl, (opt_x1, opt_y1), 8, (255, 128, 0), -1)
            cv2.putText(debug_lbl, f"Z1 {opt_ang_deg1:.1f}deg", (opt_x1 + 10, opt_y1),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 128, 0), 1)
        if opt_x2 is not None:
            cv2.circle(debug_lbl, (opt_x2, opt_y2), 8, (128, 0, 255), -1)
            cv2.putText(debug_lbl, f"Z2 {opt_ang_deg2:.1f}deg", (opt_x2 + 10, opt_y2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (128, 0, 255), 1)
        cx, cy = int(lbl_cx), int(lbl_cy)
        cv2.drawMarker(debug_lbl, (cx, cy), (0, 0, 0), cv2.MARKER_CROSS, 40, 4)
        cv2.drawMarker(debug_lbl, (cx, cy), (255, 255, 255), cv2.MARKER_CROSS, 36, 2)
        cv2.putText(debug_lbl, f"CX={cx} CY={cy}", (cx + 14, cy - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3)
        cv2.putText(debug_lbl, f"CX={cx} CY={cy}", (cx + 14, cy - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        Image.fromarray(debug_lbl).save(
            os.path.join(HOME, "Segmented_imgs", f"debug_label_{rospy.Time.now().secs}.png"))

        time = rospy.Time.now()
        pose_array_msg1 = PoseArray()
        pose_array_msg1.header.stamp    = time
        pose_array_msg1.header.frame_id = self.target_frame

        pose_array_msg2 = PoseArray()
        pose_array_msg2.header.stamp    = time
        pose_array_msg2.header.frame_id = self.target_frame


        if opt_x1 is not None:
            depth_val1 = depth_base[opt_y1, opt_x1]

            if depth_val1 == 0:
                rospy.logerr(f"Depth is 0 at Zone 1 pixel ({opt_x1}, {opt_y1}). Aborting zone 1.")
            else:
                Z1 = depth_val1 # / DEPTH_UNIT_LBL
                X1 = (opt_x1 - LBL_CX) * Z1 / LBL_FX
                Y1 = (opt_y1 - LBL_CY) * Z1 / LBL_FY
                print("Zone 1 Optx1: ", opt_x1)
                print("Zone 1 Opty1: ", opt_y1)
                print("Zone 1 X1: ", X1)
                print("Zone 1 Y1: ", Y1)
                print("Zone 1 Z1: ", Z1)
                opt_x_adj = opt_x1 + LBL_CROP_X1
                opt_y_adj = opt_y1 + LBL_CROP_Y1
                pos1 = np.array([opt_x_adj, opt_y_adj, 0.0, 1.0])
                ang1 = np.radians(opt_ang_deg1)
                quat1 = R_scipy.from_matrix(
                    self.tf_lbl[:3, :3] @ R_scipy.from_euler('z', ang1).as_matrix()
                ).as_quat()
                p1 = Pose()
                p1.position.x, p1.position.y, p1.position.z = pos1[0], pos1[1], pos1[2]
                p1.orientation.x, p1.orientation.y, p1.orientation.z, p1.orientation.w = quat1
                print("Zone 1 x: ",p1.position.x, " y: ", p1.position.y)
                pose_array_msg1.poses.append(p1)
                self.lbl_pub1.publish(pose_array_msg1)
        else:
            depth_val1 = 0

        if opt_x2 is not None:
            depth_val2 = depth_base[opt_y2, opt_x2]
            if depth_val2 == 0:
                rospy.logerr(f"Depth is 0 at Zone 2 pixel ({opt_x2}, {opt_y2}). Aborting zone 2.")
            else:
                Z2 = depth_val2 # / DEPTH_UNIT_LBL
                X2 = (opt_x2 - lbl_cx) * Z2 / LBL_FX
                Y2 = (opt_y2 - lbl_cy) * Z2 / LBL_FY
                print("Zone 2 Optx1: ", opt_x2)
                print("Zone 2 Opty1: ", opt_y2)
                print("Zone 2 X1: ", X2)
                print("Zone 2 Y1: ", Y2)
                print("Zone 2 Z1: ", Z2)
                opt_x_adj = opt_x2 + LBL_CROP_X1
                opt_y_adj = opt_y2 + LBL_CROP_Y1
                pos2 = np.array([opt_x_adj, opt_y_adj, 0.0, 1.0])
                ang2 = np.radians(opt_ang_deg2)
                quat2 = R_scipy.from_matrix(
                    self.tf_lbl[:3, :3] @ R_scipy.from_euler('z', ang2).as_matrix()
                ).as_quat()
                p2 = Pose()
                p2.position.x, p2.position.y, p2.position.z = pos2[0], pos2[1], pos2[2]
                p2.orientation.x, p2.orientation.y, p2.orientation.z, p2.orientation.w = quat2
                print("Zone 2 x: ", p2.position.x, " y: ", p2.position.y)
                pose_array_msg2.poses.append(p2)
                self.lbl_pub2.publish(pose_array_msg2)
        else:
            depth_val2 = 0

        res.success = bool((depth_val1 != 0) or (depth_val2 != 0))
        res.message = f"Zone1={'OK' if depth_val1 != 0 else 'FAIL'}, Zone2={'OK' if depth_val2 != 0 else 'FAIL'}"
        return res


if __name__ == "__main__":
    try:
        node = CombinedPerceptionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
