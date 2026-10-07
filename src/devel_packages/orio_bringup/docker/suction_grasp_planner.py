#!/usr/bin/env python
"""Suction grasp planner service: the orio-grasping height-map network.

Advertises ~plan_grasp (custom_msgs/PlanDexnetGrasp), the same contract the DexNet planner
serves, so the perception node calls either one the same way. Runs inside the orio/suction
container (Python 3.11 + PyTorch, Dockerfile.suction) with the orio-grasping repo mounted.

The network was trained on simulated top-down height maps (orio-grasping/orio_grasping/
heightmap.py), so the planner rebuilds that exact input: the depth image is back-projected
with the calibrated camera pose from orio-grasping's config.toml into its workcell frame and
binned onto the checkpoint's grid. The model scores every cell at each cup-line angle
(seal x hold x access); the best one becomes the grasp. The two cups make yaw matter: the
returned pose carries it, unlike DexNet's rotationally symmetric single cup.

Response, in the camera frame of the request (like DexNet's):
    pose         position on the surface; rotation X = approach (into the surface),
                 Y = cup line, Z = X x Y
    q_value      seal x hold x access of the chosen cell and angle
    center_px    the grasp point projected into the request image
    angle        cup-line angle in the workcell frame, radians from +x toward +y
    depth        camera-frame z of the grasp point

The segmask and colour image are ignored by the model (the colour image is drawn in the
affordance panel only).
"""

import os
import sys
import time

import numpy as np


# Xtion frames from the driver are upside down relative to the calibration captures the
# network's camera pose was fitted on (see orio-grasping/scripts/snap_and_predict.sh).
# Rotating an image by 180 deg is the same as rotating the camera frame about its z axis,
# so it is folded into the camera pose instead of touching the pixels.
ROT180 = np.diag([-1.0, -1.0, 1.0, 1.0])


class SuctionModel(object):
    """The network plus everything needed to turn one depth image into one grasp.

    No ROS here, so it can be checked offline against orio-grasping's predict_capture.py.
    """

    def __init__(self, repo, checkpoint, config=None, device=None, rot180=True):
        if repo not in sys.path:
            sys.path.insert(0, repo)
        import torch
        from orio_grasping.camera_model import extrinsics, table_inner_crop
        from orio_grasping.config import load_config
        from orio_grasping.heightmap import Grid, build_heightmap, network_input
        from orio_grasping.model import load_checkpoint, predict

        self._build_heightmap, self._network_input, self._predict = build_heightmap, network_input, predict
        self.cfg = load_config(config)
        self.device = device or ("cuda" if torch.cuda.is_available() else "cpu")
        self.model, ck = load_checkpoint(checkpoint, self.device)
        li = ck["label_info"]
        g = li["grid"]
        self.grid = Grid(tuple(g["x_range"]), tuple(g["y_range"]), g["cell"])
        self.angles_deg = list(ck["angles_deg"])
        self.cup_radius = float(li["cup_radius"])
        self.cup_spacing = float(li["cup_spacing"])
        self.table_z = float(self.cfg.table.top_z)
        self.checkpoint_info = "%s (epoch %d, %s)" % (checkpoint, ck["epoch"], ck["train_cfg"]["encoder"])

        cam = self.cfg.camera
        T = extrinsics(self.cfg)
        self.rot180 = bool(rot180)
        self.T_world_cam = T.dot(ROT180) if self.rot180 else T
        self.T_cam_world = np.linalg.inv(self.T_world_cam)
        # The training depth only covered the table window (camera.crop = "table"); blank
        # the rest of a full frame the same way so off-table geometry never lands in the map.
        self.full_shape = (int(cam.height), int(cam.width))
        self.crop = None
        if cam.crop == "table":
            x0, y0, x1, y1 = table_inner_crop(self.cfg, self.full_shape, cam.crop_inset_px)
            if self.rot180:
                h, w = self.full_shape
                x0, y0, x1, y1 = w - x1, h - y1, w - x0, h - y0
            self.crop = (x0, y0, x1, y1)

    def project(self, xyz_world, K):
        """World point to (u, v, z) in the request image."""
        p = self.T_cam_world.dot(np.append(np.asarray(xyz_world, dtype=np.float64), 1.0))
        return K[0, 0] * p[0] / p[2] + K[0, 2], K[1, 1] * p[1] / p[2] + K[1, 2], p[2]

    def plan(self, depth_m, K):
        """Best grasp for one depth image (metres; NaN or <= 0 invalid) and its 3x3 K.

        Returns a dict: score, seal, hold, access, angle_deg, cell (row, col), world_xyz,
        normal_world, cup_line_world, position_cam, rotation_cam, center_px, depth_cam,
        tilt_deg, and the maps (heightmap, prediction) for the diagnostic panel.
        """
        depth = np.asarray(depth_m, dtype=np.float32)
        depth = np.where(np.isfinite(depth) & (depth > 0), depth, 0.0).astype(np.float32)
        if self.crop is not None and depth.shape == self.full_shape:
            x0, y0, x1, y1 = self.crop
            windowed = np.zeros_like(depth)
            windowed[y0:y1, x0:x1] = depth[y0:y1, x0:x1]
            depth = windowed
        K = np.asarray(K, dtype=np.float64)

        hm = self._build_heightmap(depth, K, self.T_world_cam, self.table_z, self.grid, self.cfg.heightmap)
        p = self._predict(self.model, self._network_input(hm), self.device)
        a, r, c = np.unravel_index(int(np.argmax(p["score"])), p["score"].shape)

        x, y = self.grid.cell_to_world(np.array([r, c], dtype=np.float64))
        world = np.array([x, y, self.table_z + float(hm["height"][r, c])])
        normal = np.asarray(hm["normals"][:, r, c], dtype=np.float64)
        normal /= np.linalg.norm(normal)
        theta = np.radians(self.angles_deg[a])
        line = np.array([np.cos(theta), np.sin(theta), 0.0])
        line -= line.dot(normal) * normal  # cups sit on the surface, not in the table plane
        line /= np.linalg.norm(line)

        R_cw = self.T_cam_world[:3, :3]
        approach_cam = R_cw.dot(-normal)
        line_cam = R_cw.dot(line)
        rotation_cam = np.column_stack([approach_cam, line_cam, np.cross(approach_cam, line_cam)])
        position_cam = self.T_cam_world.dot(np.append(world, 1.0))[:3]
        u, v, _ = self.project(world, K)
        return {
            "score": float(p["score"][a, r, c]),
            "seal": float(p["seal"][a, r, c]),
            "hold": float(p["hold"][a, r, c]),
            "access": float(p["access"][r, c]),
            "angle_deg": float(self.angles_deg[a]),
            "angle_index": int(a),
            "cell": (int(r), int(c)),
            "unseen": bool(hm["unseen"][r, c]),
            "world_xyz": world,
            "normal_world": normal,
            "cup_line_world": line,
            "position_cam": position_cam,
            "rotation_cam": rotation_cam,
            "center_px": (float(u), float(v)),
            "depth_cam": float(position_cam[2]),
            "tilt_deg": float(np.degrees(np.arccos(np.clip(normal[2], -1.0, 1.0)))),
            "heightmap": hm,
            "prediction": p,
        }

    def cup_centres_world(self, g):
        half = 0.5 * self.cup_spacing * g["cup_line_world"]
        return [g["world_xyz"] + half, g["world_xyz"] - half]

    def save_panel(self, path, g, color, K, title):
        """Three-panel diagnostic: the cups on the camera image, on the height map the
        network saw, and on its best score per cell over all cup angles."""
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib.patches import Circle

        hm, p = g["heightmap"], g["prediction"]
        col = "cyan" if g.get("accepted", True) else "red"
        fig, ax = plt.subplots(1, 3, figsize=(20, 6.5))

        # Everything is drawn as the camera delivers it, like the DexNet panels, so the
        # same reading holds for both (COMMANDS.md: up is robot +Y, right is robot +X).
        # The maps are laid out in the calibration orientation, so with rot180 they are
        # turned to match.
        img = np.asarray(color)
        ax[0].imshow(img)
        for centre in self.cup_centres_world(g):
            u, v, z = self.project(centre, K)
            ax[0].add_patch(Circle((u, v), self.cup_radius * K[0, 0] / z, fill=False, ec=col, lw=2.5))
        if self.crop is not None and img.shape[:2] == self.full_shape:
            x0, y0, x1, y1 = self.crop
            ax[0].add_patch(plt.Rectangle((x0, y0), x1 - x0, y1 - y0, fill=False, ec="yellow", lw=1, ls="--"))
        ax[0].set_title("camera (dataset window dashed)")

        def shown(a):
            return a[::-1, ::-1] if self.rot180 else a

        rows, cols = self.grid.shape
        cells = []
        for c in self.cup_centres_world(g):
            r, cc = self.grid.world_to_cell(c[:2])
            cells.append((rows - 1 - r, cols - 1 - cc) if self.rot180 else (r, cc))
        r_cells = self.cup_radius / self.grid.cell
        im = ax[1].imshow(shown(hm["height"]) * 1000, cmap="gray")
        unseen = shown(hm["unseen"])
        ax[1].imshow(np.ma.masked_where(~unseen, np.zeros(unseen.shape)), cmap="autumn", alpha=0.5, vmin=0, vmax=1)
        fig.colorbar(im, ax=ax[1], fraction=0.046, label="mm above table")
        ax[1].set_title("height map (unseen red)")
        im = ax[2].imshow(shown(p["score"].max(axis=0)), cmap="inferno", vmin=0, vmax=1)
        fig.colorbar(im, ax=ax[2], fraction=0.046)
        ax[2].set_title("best score per cell over cup angles")
        for a_ in ax[1:]:
            for rc in cells:
                a_.add_patch(Circle((rc[1], rc[0]), r_cells, fill=False, ec=col, lw=2.5))
        for a_ in ax:
            a_.axis("off")
        fig.suptitle(title, fontsize=13)
        fig.tight_layout(rect=[0, 0, 1, 0.94])
        fig.savefig(path, dpi=90)
        plt.close(fig)


class SuctionGraspPlanner(object):
    def __init__(self):
        import rospy
        from std_msgs.msg import String
        from custom_msgs.srv import PlanDexnetGrasp

        rospy.init_node("suction_grasp_planner")
        repo = rospy.get_param("~grasping_repo", "/opt/orio-grasping")
        checkpoint = rospy.get_param("~checkpoint")
        config = rospy.get_param("~config", "") or None
        rot180 = bool(rospy.get_param("~rot180", True))
        rospy.loginfo("Loading suction network %s", checkpoint)
        self._model = SuctionModel(repo, checkpoint, config=config, rot180=rot180)
        rospy.loginfo("Model %s on %s; grid %dx%d cells of %.1f mm, cup angles %s, frame %s",
                      self._model.checkpoint_info, self._model.device,
                      self._model.grid.shape[1], self._model.grid.shape[0],
                      1000 * self._model.grid.cell, self._model.angles_deg,
                      "rotated 180 deg" if rot180 else "as calibrated")
        self._min_score = float(rospy.get_param("~min_score", 0.0))

        # Diagnostic panels, opt-in like the DexNet planner's affordance panels (same env
        # var, so run_dexnet_pnp_single.sh --affordance drives both).
        self._panel_dir = os.environ.get("ORIO_AFFORDANCE_DIR", "").strip()
        self._panel_n = 0
        if self._panel_dir:
            try:
                if not os.path.isdir(self._panel_dir):
                    os.makedirs(self._panel_dir)
                rospy.loginfo("Grasp panels -> %s", self._panel_dir)
            except OSError as exc:
                rospy.logwarn("Cannot use panel dir %s: %s", self._panel_dir, exc)
                self._panel_dir = ""

        self._events = None
        try:
            from orio_core.events import TOPIC as EVENTS_TOPIC, EventSink
        except ImportError:  # orio_core not mounted: no run events, service still works
            EventSink = None
        if EventSink and os.environ.get("ORIO_LOGGING", "1") != "0":
            ev_pub = rospy.Publisher(EVENTS_TOPIC, String, queue_size=20)
            self._events = EventSink(lambda s: ev_pub.publish(String(data=s)), src="suction")

        self._srv = rospy.Service("~plan_grasp", PlanDexnetGrasp, self._handle)
        rospy.loginfo("Suction grasp planner ready on %s (min_score=%.3f)",
                      rospy.resolve_name("~plan_grasp"), self._min_score)

    @staticmethod
    def _decode(msg, dtype, channels=1):
        """Decode a sensor_msgs/Image without cv_bridge."""
        arr = np.frombuffer(msg.data, dtype=dtype)
        shape = (msg.height, msg.width) if channels == 1 else (msg.height, msg.width, channels)
        return arr.reshape(shape)

    def _handle(self, req):
        import rospy
        from custom_msgs.msg import DexnetGrasp
        from custom_msgs.srv import PlanDexnetGraspResponse
        from scipy.spatial.transform import Rotation

        if req.depth_image.encoding not in ("32FC1", "passthrough"):
            raise rospy.ServiceException(
                "depth_image encoding must be 32FC1, got %s" % req.depth_image.encoding)
        depth = self._decode(req.depth_image, np.float32)
        K = np.array(req.camera_info.K, dtype=np.float64).reshape(3, 3)

        start = time.time()
        g = self._model.plan(depth, K)
        elapsed = time.time() - start

        msg = DexnetGrasp()
        msg.q_value = g["score"]
        msg.grasp_type = DexnetGrasp.SUCTION
        msg.center_px = list(g["center_px"])
        msg.angle = float(np.radians(g["angle_deg"]))
        msg.depth = g["depth_cam"]
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = g["position_cam"]
        qx, qy, qz, qw = Rotation.from_matrix(g["rotation_cam"]).as_quat()
        msg.pose.orientation.x, msg.pose.orientation.y = qx, qy
        msg.pose.orientation.z, msg.pose.orientation.w = qz, qw

        rejected = g["score"] < self._min_score
        g["accepted"] = not rejected
        summary = ("score %.3f (seal %.2f hold %.2f access %.2f) cups at %.0f deg, tilt %.1f deg"
                   % (g["score"], g["seal"], g["hold"], g["access"], g["angle_deg"], g["tilt_deg"]))

        if self._panel_dir and req.color_image.height:
            try:
                self._panel_n += 1
                stamp = time.strftime("%H%M%S", time.localtime())
                out = os.path.join(self._panel_dir, "grasp_%04d_%s.png" % (self._panel_n, stamp))
                color = self._decode(req.color_image, np.uint8, channels=3)
                self._model.save_panel(out, g, color, K, "call %04d @ %s  %s  %s  (%.2fs)" % (
                    self._panel_n, stamp, "ACCEPTED" if not rejected else "REJECTED", summary, elapsed))
                rospy.loginfo("Grasp panel: %s", out)
            except Exception as exc:  # noqa: BLE001 - a diagnostic must never fail a pick
                rospy.logwarn("Could not write grasp panel: %s", exc)

        if self._events:
            self._events.emit("suction", "plan", q=g["score"], seal=g["seal"], hold=g["hold"],
                              access=g["access"], angle_deg=g["angle_deg"], px=list(msg.center_px),
                              world=[float(v) for v in g["world_xyz"]], tilt_deg=g["tilt_deg"],
                              plan_time_s=elapsed, ok=not rejected)
        if rejected:
            rospy.logwarn("Rejected grasp: %s < min_score %.3f (%.3fs)", summary, self._min_score, elapsed)
            return PlanDexnetGraspResponse(
                msg, False, "score %.4f below threshold %.3f" % (g["score"], self._min_score))
        rospy.loginfo("Planned suction grasp %s at px (%.0f, %.0f) in %.3fs",
                      summary, msg.center_px[0], msg.center_px[1], elapsed)
        return PlanDexnetGraspResponse(msg, True, "")


if __name__ == "__main__":
    import rospy
    SuctionGraspPlanner()
    rospy.spin()
