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

The request segmask is ignored. A checkpoint trained with the "seg" input channels (e.g.
seg_v1) also needs the colour image: orio-grasping's RGB-D segmentation model runs on it and the
depth first, as in orio-grasping/scripts/real_grasp_sheet.py, and its instance masks are
projected into the height map. Depth-only checkpoints use the colour image for the panel only.
"""

import collections
import json
import os
import sys
import time

import numpy as np


# Xtion frames from the driver are upside down relative to the calibration captures the
# network's camera pose was fitted on (see orio-grasping/scripts/snap_and_predict.sh).
# Rotating an image by 180 deg is the same as rotating the camera frame about its z axis,
# so it is folded into the camera pose instead of touching the pixels.
ROT180 = np.diag([-1.0, -1.0, 1.0, 1.0])


def align_hires(img, x0, y0, shape):
    """The part of a 1280x1024 Xtion colour frame on the 640x480 grid at twice its
    resolution: hires[2v + i, 2u + j] lies in 640x480 pixel (u, v). (x0, y0) is where that
    grid starts in the frame (measured: 5, 35); columns past the frame's edge repeat it."""
    h, w = 2 * shape[0], 2 * shape[1]
    out = img[y0:y0 + h, x0:x0 + w]
    if out.shape[0] < h or out.shape[1] < w:
        out = np.pad(out, ((0, h - out.shape[0]), (0, w - out.shape[1]), (0, 0)), mode="edge")
    return np.ascontiguousarray(out)


class SuctionModel(object):
    """The network plus everything needed to turn one depth image into one grasp.

    No ROS here, so it can be checked offline against orio-grasping's predict_capture.py.
    """

    def __init__(self, repo, checkpoint, config=None, device=None, rot180=True,
                 seg_checkpoint=None, seg_model="RFDETRSegLarge", seg_threshold=0.5,
                 seg_trim_top=16, seg_trim_sides=16, calibrated_intrinsics=True,
                 seg_max_mask_frac=0.5):
        for path in (repo, os.path.join(repo, "scripts")):
            if path not in sys.path:
                sys.path.insert(0, path)
        import torch
        from orio_grasping.camera_model import extrinsics, intrinsics, table_inner_crop
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
        self.channels = tuple(self.model.channels)
        self.checkpoint_info = "%s (epoch %d, %s, channels %s)" % (
            checkpoint, ck["epoch"], ck["train_cfg"]["encoder"], "+".join(self.channels))

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
            self.seg_crop = (x0, y0, x1, y1)  # calibration orientation, where the seg model looks
            if self.rot180:
                h, w = self.full_shape
                x0, y0, x1, y1 = w - x1, h - y1, w - x0, h - y0
            self.crop = (x0, y0, x1, y1)

        # The camera pose was calibrated together with a focal length (hfov_deg in
        # xtion_pose.json: fx 549.7 px, from the table width), not the driver's default
        # camera_info (570.3 px), and orio-grasping's real-capture scripts use it. Back-projecting
        # with the driver's K would scale every point ~4 % about the image centre (up to ~1 cm
        # at the bin edges): a different height map for the network, and a different 3D grasp.
        # So full frames use the calibrated K, turned like the pose when rot180 is set.
        self.K_cal = None
        if calibrated_intrinsics:
            h, w = self.full_shape
            K_cal = intrinsics(self.cfg)
            if self.rot180:
                K_cal[0, 2], K_cal[1, 2] = w - 1 - K_cal[0, 2], h - 1 - K_cal[1, 2]
            self.K_cal = K_cal

        self.seg = None
        # A mask bigger than this share of the segmentation window is the table itself (on
        # an empty table the model returns one mask covering ~100% of it; items are a few
        # %), so it is dropped. 0 or None keeps every mask.
        self.seg_max_mask_frac = float(seg_max_mask_frac or 0)
        if "seg" in self.channels:
            self._load_seg(T, seg_checkpoint, seg_model, seg_threshold, seg_trim_top, seg_trim_sides)

    def _load_seg(self, T_cal, checkpoint, model_name, threshold, trim_top, trim_sides):
        """The RGB-D segmentation model and the window and bin region it is fed, as
        orio-grasping's real_grasp_sheet.py sets them up."""
        if not checkpoint:
            raise ValueError("checkpoint reads segmentation masks (channels %s): set the seg "
                             "checkpoint (SUCTION_SEG_CHECKPOINT)" % "+".join(self.channels))
        if self.crop is None:
            raise ValueError("segmentation needs camera.crop = \"table\" in the orio-grasping config")
        import rfdetr
        from orio_grasping import rgbd_seg
        from predict_seg_masks import instance_image
        from segment_captures import bin_region

        self._depth_channels, self._instance_image = rgbd_seg.depth_channels, instance_image
        b = self.cfg.bin
        meta = {"camera": {"crop": {"box_xyxy_full": list(self.seg_crop)}},
                "workcell": {"bin_inner": {"x": [b.x_min, b.x_max], "y": [b.y_min, b.y_max]},
                             "table_top_z": self.table_z}}
        self.seg_keep, _ = bin_region(meta, self.cfg, trim_top, trim_sides)
        self.T_cal = T_cal
        self.seg_threshold = float(threshold)
        self.seg = rgbd_seg.load_model(getattr(rfdetr, model_name), checkpoint)
        self.checkpoint_info += "; seg %s (%s, threshold %.2f)" % (checkpoint, model_name, self.seg_threshold)

    def segment(self, color, depth, K, color_hires=None):
        """Instance image (request orientation and shape, 0 = no object; k = the k-th most
        confident mask) from the RGB-D segmentation model, and each mask's confidence in id
        order. The model sees the frame in the calibration orientation, cut to the table
        window, black and with invalid depth outside the bin region.

        color_hires, when given, is the colour at exactly twice the request's resolution
        on the same pixel grid (see align_hires). The model then sees that sharper crop,
        with the depth channels repeated 2x2 to match, and its masks are brought back to
        the depth's resolution (a pixel is in a mask when most of its 2x2 block is)."""
        import torch

        h, w = depth.shape
        K_cal = np.array(K, dtype=np.float64)
        if self.rot180:  # turn the pixels, not the camera: the model was trained upright
            color, depth = color[::-1, ::-1], depth[::-1, ::-1]
            if color_hires is not None:
                color_hires = color_hires[::-1, ::-1]
            K_cal[0, 2], K_cal[1, 2] = w - 1 - K_cal[0, 2], h - 1 - K_cal[1, 2]
        x0, y0, x1, y1 = self.seg_crop
        keep = np.zeros((h, w), bool)
        keep[y0:y1, x0:x1] = self.seg_keep
        ch = self._depth_channels(np.ascontiguousarray(depth), K_cal, self.T_cal, self.table_z, keep=keep)
        ch = np.ascontiguousarray(ch[:, y0:y1, x0:x1])
        seg_keep = self.seg_keep
        if color_hires is not None:
            up = lambda a, ax: np.repeat(np.repeat(a, 2, axis=ax), 2, axis=ax + 1)  # noqa: E731
            rgb = np.array(color_hires[2 * y0:2 * y1, 2 * x0:2 * x1, :3], dtype=np.uint8)
            seg_keep, ch = up(seg_keep, 0), np.ascontiguousarray(up(ch, 1))
        else:
            rgb = np.array(color[y0:y1, x0:x1, :3], dtype=np.uint8)
        rgb[~seg_keep] = 0
        x = torch.cat([torch.from_numpy(rgb).permute(2, 0, 1).float() / 255.0,
                       torch.from_numpy(ch)])
        det = self.seg.predict(x, threshold=self.seg_threshold, include_source_image=False)
        masks = det.mask if det.mask is not None else np.zeros((0,) + seg_keep.shape, bool)
        conf = np.asarray(det.confidence if det.mask is not None else [], dtype=np.float64)
        if color_hires is not None and len(masks):
            n, mh, mw = masks.shape
            masks = masks.reshape(n, mh // 2, 2, mw // 2, 2).mean(axis=(2, 4)) >= 0.5
        elif color_hires is not None:
            masks = np.zeros((0, y1 - y0, x1 - x0), bool)
        dropped = []
        if self.seg_max_mask_frac > 0 and len(masks):
            frac = masks.reshape(len(masks), -1).sum(axis=1) / float(self.seg_keep.sum())
            big = frac > self.seg_max_mask_frac
            dropped = [{"frac": round(float(f), 3), "conf": round(float(c), 3)}
                       for f, c in zip(frac[big], conf[big])]
            masks, conf = masks[~big], conf[~big]
        inst, records = self._instance_image(masks, conf, (y1 - y0, x1 - x0))
        full = np.zeros((h, w), np.uint16)
        full[y0:y1, x0:x1] = inst
        return (np.ascontiguousarray(full[::-1, ::-1] if self.rot180 else full),
                [r["score"] for r in records], dropped)

    def project(self, xyz_world, K):
        """World point to (u, v, z) in the request image."""
        p = self.T_cam_world.dot(np.append(np.asarray(xyz_world, dtype=np.float64), 1.0))
        return K[0, 0] * p[0] / p[2] + K[0, 2], K[1, 1] * p[1] / p[2] + K[1, 2], p[2]

    def plan(self, depth_m, K, color=None, color_hires=None):
        """Best grasp for one depth image (metres; NaN or <= 0 invalid) and its 3x3 K, plus
        the registered RGB image when the checkpoint reads segmentation masks.

        Returns a dict: score, seal, hold, access, angle_deg, cell (row, col), world_xyz,
        normal_world, cup_line_world, position_cam, rotation_cam, center_px, depth_cam,
        tilt_deg, n_masks (None without segmentation), and the maps (heightmap, prediction,
        instances) for the diagnostic panel.
        """
        depth = np.asarray(depth_m, dtype=np.float32)
        depth = np.where(np.isfinite(depth) & (depth > 0), depth, 0.0).astype(np.float32)
        K = np.asarray(K, dtype=np.float64)
        if self.K_cal is not None and depth.shape == self.full_shape:
            K = self.K_cal  # the request's K is the driver default, see __init__
        instances, mask_scores, dropped = None, None, []
        if self.seg is not None:
            if color is None or color.shape[:2] != depth.shape or depth.shape != self.full_shape:
                raise ValueError("segmentation needs a %dx%d colour image registered to the depth"
                                 % (self.full_shape[1], self.full_shape[0]))
            if color_hires is not None and color_hires.shape[:2] != (2 * depth.shape[0], 2 * depth.shape[1]):
                raise ValueError("the high-res colour must be exactly 2x the depth (%dx%d), got %dx%d"
                                 % (2 * depth.shape[1], 2 * depth.shape[0],
                                    color_hires.shape[1], color_hires.shape[0]))
            instances, mask_scores, dropped = self.segment(color, depth, K, color_hires)
        if self.crop is not None and depth.shape == self.full_shape:
            x0, y0, x1, y1 = self.crop
            windowed = np.zeros_like(depth)
            windowed[y0:y1, x0:x1] = depth[y0:y1, x0:x1]
            depth = windowed

        hm = self._build_heightmap(depth, K, self.T_world_cam, self.table_z, self.grid, self.cfg.heightmap,
                                   instances=instances)
        p = self._predict(self.model, self._network_input(hm, self.channels), self.device)
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
            "n_masks": None if instances is None else int(instances.max()),
            "heightmap": hm,
            "prediction": p,
            "instances": instances,
            "mask_scores": mask_scores,
            "dropped_masks": dropped,
            "K": K,
        }

    def cup_centres_world(self, g):
        half = 0.5 * self.cup_spacing * g["cup_line_world"]
        return [g["world_xyz"] + half, g["world_xyz"] - half]

    def save_panel(self, path, g, color, K, title):
        """Three-panel diagnostic: the cups on the camera image (with the segmentation
        masks, if any), on the height map the network saw, and on its best score per cell
        over all cup angles."""
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
        self._draw_camera(ax[0], g, color, K, col)
        ax[0].set_title("camera (dataset window: yellow dashed)" if g.get("n_masks") is None
                        else "camera, %d masks: id: confidence (dataset window: yellow dashed)" % g["n_masks"])

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

    def save_camera(self, path, g, color, K, min_score):
        """Just the camera view of save_panel (masks + the chosen cups), titled with the
        score against the planner's threshold: what the run dashboard shows. Later checks
        (e.g. perception's tilt limit) can still turn a grasp down; the dashboard says so."""
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        accepted = g.get("accepted", True)
        h, w = np.asarray(color).shape[:2]
        fig, ax = plt.subplots(figsize=(8, 8 * h / w + 0.9))
        self._draw_camera(ax, g, color, K, "cyan" if accepted else "red",
                          cups=not g.get("reject_reason"))   # no item: no grasp to show
        ax.axis("off")
        head = ("REJECTED: %s" % g["reject_reason"] if g.get("reject_reason") else
                "score %.3f %s %.2f%s" % (g["score"], "≥" if accepted else "<", min_score,
                                          "" if accepted else "  REJECTED"))
        ax.set_title("%s\nseal %.2f  hold %.2f  access %.2f  ·  tilt %.0f°"
                     % (head, g["seal"], g["hold"], g["access"], g["tilt_deg"]),
                     fontsize=15, color="#1a7f37" if accepted else "#c62828", loc="center")
        fig.savefig(path, dpi=110, bbox_inches="tight", pad_inches=0.1)
        plt.close(fig)

    def _draw_camera(self, ax, g, color, K, col, cups=True):
        """The camera image with the segmentation masks (if any), the table window and
        the two cups of grasp g."""
        import matplotlib.pyplot as plt
        from matplotlib.patches import Circle

        img = np.asarray(color)
        ax.imshow(img)
        inst = g.get("instances")
        if inst is not None and inst.shape == img.shape[:2]:
            # tab20's even entries (blue, orange, green, red, ...): neighbouring ids get clearly
            # different colours, not two shades of one.
            shade = (2 * (inst.astype(np.int32) - 1)) % 20
            ax.imshow(np.ma.masked_where(inst == 0, shade), cmap="tab20", vmin=0, vmax=19, alpha=0.45)
            # Each mask in its own colour: outline, dashed bounding box, and its id (1 = most
            # confident) with the segmentation confidence.
            tab20 = plt.get_cmap("tab20")
            scores = g.get("mask_scores") or []
            for k in range(1, int(inst.max()) + 1):
                on = inst == k
                if not on.any():
                    continue
                c = tab20(((2 * (k - 1)) % 20) / 19.0)
                ys, xs = np.nonzero(on)
                ax.contour(on, levels=[0.5], colors=[c], linewidths=1.2)
                ax.add_patch(plt.Rectangle((xs.min() - 0.5, ys.min() - 0.5), xs.max() - xs.min() + 1,
                                              ys.max() - ys.min() + 1, fill=False, ec=c, lw=1.5, ls="--"))
                text = "%d" % k if k > len(scores) else "%d: %.2f" % (k, scores[k - 1])
                ax.text(xs.min(), ys.min() - 2, text, color="black", fontsize=9, va="bottom",
                           bbox=dict(boxstyle="square,pad=0.15", fc=c, ec="none", alpha=0.9))
        for centre in (self.cup_centres_world(g) if cups else []):
            u, v, z = self.project(centre, K)
            ax.add_patch(Circle((u, v), self.cup_radius * K[0, 0] / z, fill=False, ec=col, lw=2.5))
        if self.crop is not None and img.shape[:2] == self.full_shape:
            x0, y0, x1, y1 = self.crop
            ax.add_patch(plt.Rectangle((x0, y0), x1 - x0, y1 - y0, fill=False, ec="yellow", lw=1, ls="--"))


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
        self._model = SuctionModel(
            repo, checkpoint, config=config, rot180=rot180,
            seg_checkpoint=rospy.get_param("~seg_checkpoint", "") or None,
            seg_model=rospy.get_param("~seg_model", "RFDETRSegLarge"),
            seg_threshold=float(rospy.get_param("~seg_threshold", 0.5)),
            seg_trim_top=int(rospy.get_param("~seg_trim_top", 16)),
            seg_trim_sides=int(rospy.get_param("~seg_trim_sides", 16)),
            calibrated_intrinsics=bool(rospy.get_param("~calibrated_intrinsics", True)),
            seg_max_mask_frac=float(rospy.get_param("~seg_max_mask_frac", 0.5)))
        if self._model.K_cal is not None:
            rospy.loginfo("Intrinsics: calibrated fx %.1f px (the request's K is used only for other "
                          "frame sizes)", self._model.K_cal[0, 0])
        rospy.loginfo("Model %s on %s; grid %dx%d cells of %.1f mm, cup angles %s, frame %s",
                      self._model.checkpoint_info, self._model.device,
                      self._model.grid.shape[1], self._model.grid.shape[0],
                      1000 * self._model.grid.cell, self._model.angles_deg,
                      "rotated 180 deg" if rot180 else "as calibrated")
        self._min_score = float(rospy.get_param("~min_score", 0.0))
        # Everything a later offline rerun needs besides the frame itself (see _save_inputs).
        self._settings = {
            "grasping_repo": repo, "checkpoint": checkpoint, "checkpoint_info": self._model.checkpoint_info,
            "config": config, "rot180": rot180, "min_score": self._min_score,
            "seg_checkpoint": rospy.get_param("~seg_checkpoint", "") or None,
            "seg_model": rospy.get_param("~seg_model", "RFDETRSegLarge"),
            "seg_threshold": float(rospy.get_param("~seg_threshold", 0.5)),
            "seg_trim_top": int(rospy.get_param("~seg_trim_top", 16)),
            "seg_trim_sides": int(rospy.get_param("~seg_trim_sides", 16)),
            "calibrated_intrinsics": self._model.K_cal is not None,
        }

        # High-res segmentation: with the Xtion in high-res mode (xtion_only.launch
        # hires:=true) the colour in a request is rebuilt from a 1280x1024 frame with the
        # same stamp; the model segments that frame instead. Without one (640x480 mode, or
        # an old request without a stamp) it segments the request's colour as before.
        self._hires = bool(rospy.get_param("~seg_hires", True)) and self._model.seg is not None
        self._hires_frames = collections.deque(maxlen=12)
        if self._hires:
            from sensor_msgs.msg import Image
            topic = rospy.get_param("~hires_topic", "/camera/rgb_hires/image_raw")
            rospy.Subscriber(topic, Image, self._hires_frames.append, queue_size=1, buff_size=2 ** 24)
            rospy.loginfo("High-res segmentation: frames from %s when the Xtion is in high-res mode", topic)
        self._settings["seg_hires"] = self._hires

        # Diagnostic panels, opt-in like the DexNet planner's affordance panels (same env
        # var, so run_pickup-only.sh --affordance drives both). Each call also keeps
        # its raw inputs beside the panel, so the models can be rerun on it offline.
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

    def _hires_for(self, req, shape):
        """(colour at twice `shape` on the same grid, None) from the high-res frame whose
        stamp the request's colour carries, or (None, why not)."""
        import rospy
        if not self._hires:
            return None, "high-res segmentation off"
        st = req.color_image.header.stamp
        if st.is_zero():
            return None, "the request's colour has no stamp"
        frames = list(self._hires_frames)
        if not frames:
            return None, "no high-res frames (Xtion in 640x480 mode?)"
        m = min(frames, key=lambda f: abs((f.header.stamp - st).to_sec()))
        dt = abs((m.header.stamp - st).to_sec())
        if dt > 1e-3:
            return None, "no high-res frame with the colour's stamp (nearest %.3f s off)" % dt
        img = np.frombuffer(m.data, np.uint8).reshape(m.height, m.step // 3, 3)[:, :m.width]
        if m.encoding == "bgr8":
            img = img[:, :, ::-1]
        x0 = int(rospy.get_param("/camera/hires_align/x0", 5))
        y0 = int(rospy.get_param("/camera/hires_align/y0", 35))
        return align_hires(img, x0, y0, shape), None

    def _save_inputs(self, base, req, depth, K, color, color_hires=None):
        """The request as an Xtion capture folder (color.npy, depth.npy, K.npy, the layout
        snap_xtion.sh writes to logging/xtion) plus meta.json with the planner settings.
        Depth is saved exactly as received (float32 metres, before the table window).
        color_hires.npy, when there is one, is the 1280x960 colour on the same grid at
        twice the resolution (pixel (u, v) of the others is its 2x2 block at (2u, 2v))."""
        os.makedirs(base)
        np.save(os.path.join(base, "depth.npy"), depth)
        np.save(os.path.join(base, "K.npy"), K)
        if color is not None:
            np.save(os.path.join(base, "color.npy"), color)
        if color_hires is not None:
            np.save(os.path.join(base, "color_hires.npy"), color_hires)
        h = req.depth_image.header
        meta = {"call": self._panel_n, "time": time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime()),
                "frame_id": h.frame_id, "stamp": h.stamp.to_sec(), "depth_encoding": req.depth_image.encoding,
                "planner": self._settings}
        with open(os.path.join(base, "meta.json"), "w") as f:
            json.dump(meta, f, indent=1)

    @staticmethod
    def _save_result(base, result):
        """Add the planner's answer to the call's meta.json."""
        path = os.path.join(base, "meta.json")
        with open(path) as f:
            meta = json.load(f)
        meta["result"] = result
        with open(path, "w") as f:
            json.dump(meta, f, indent=1)

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
        color = self._decode(req.color_image, np.uint8, channels=3) if req.color_image.height else None
        color_hires, no_hires = (None, "no colour") if color is None else self._hires_for(req, depth.shape)
        if color_hires is None and self._hires:
            rospy.logwarn_throttle(60, "Segmenting the 640x480 colour: %s" % no_hires)

        # Panel and inputs share one name per call: grasp_NNNN_hhmmss(.png | /).
        base = None
        if self._panel_dir:
            self._panel_n += 1
            stamp = time.strftime("%H%M%S", time.localtime())
            base = os.path.join(self._panel_dir, "grasp_%04d_%s" % (self._panel_n, stamp))
            try:
                self._save_inputs(base, req, depth, K, color, color_hires)
            except Exception as exc:  # noqa: BLE001 - a diagnostic must never fail a pick
                rospy.logwarn("Could not save grasp inputs: %s", exc)
                base = None

        start = time.time()
        try:
            g = self._model.plan(depth, K, color, color_hires)
        except ValueError as exc:
            if base:
                try:
                    self._save_result(base, {"error": str(exc)})
                except Exception as save_exc:  # noqa: BLE001
                    rospy.logwarn("Could not save grasp result: %s", save_exc)
            raise rospy.ServiceException(str(exc))
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

        # With segmentation, no mask left (none found, or only the table-sized one) means
        # no item: the network would still score a spot on the bare table.
        no_item = g["n_masks"] == 0
        g["reject_reason"] = None
        if no_item:
            g["reject_reason"] = ("no item: only a table-sized mask (dropped)" if g["dropped_masks"]
                                  else "no item: no masks")
        rejected = no_item or g["score"] < self._min_score
        g["accepted"] = not rejected
        summary = ("score %.3f (seal %.2f hold %.2f access %.2f) cups at %.0f deg, tilt %.1f deg"
                   % (g["score"], g["seal"], g["hold"], g["access"], g["angle_deg"], g["tilt_deg"]))
        if g["n_masks"] is not None:
            summary += ", %d masks" % g["n_masks"]

        if base:
            try:
                self._save_result(base, {
                    "score": g["score"], "seal": g["seal"], "hold": g["hold"], "access": g["access"],
                    "angle_deg": g["angle_deg"], "tilt_deg": g["tilt_deg"], "cell": list(g["cell"]),
                    "world_xyz": [float(v) for v in g["world_xyz"]], "center_px": list(g["center_px"]),
                    "unseen": g["unseen"], "n_masks": g["n_masks"], "mask_scores": g["mask_scores"], "accepted": not rejected,
                    "dropped_masks": g["dropped_masks"], "reject_reason": g["reject_reason"],
                    "seg_input": "1280x960" if color_hires is not None else "640x480",
                    "plan_time_s": elapsed})
            except Exception as exc:  # noqa: BLE001
                rospy.logwarn("Could not save grasp result: %s", exc)

        panel = ""
        if base and color is not None:
            try:
                out = base + ".png"
                self._model.save_panel(out, g, color, g["K"], "call %04d @ %s  %s  %s  (%.2fs)" % (
                    self._panel_n, stamp, "ACCEPTED" if not rejected else "REJECTED", summary, elapsed))
                rospy.loginfo("Grasp panel: %s", out)
                panel = out
            except Exception as exc:  # noqa: BLE001 - a diagnostic must never fail a pick
                rospy.logwarn("Could not write grasp panel: %s", exc)
            try:   # the camera view alone, for the run dashboard
                self._model.save_camera(base + "_camera.png", g, color, g["K"], self._min_score)
            except Exception as exc:  # noqa: BLE001
                rospy.logwarn("Could not write grasp camera view: %s", exc)

        if self._events:
            self._events.emit("suction", "plan", q=g["score"], seal=g["seal"], hold=g["hold"],
                              access=g["access"], angle_deg=g["angle_deg"], px=list(msg.center_px),
                              world=[float(v) for v in g["world_xyz"]], tilt_deg=g["tilt_deg"],
                              n_masks=g["n_masks"], plan_time_s=elapsed, ok=not rejected)
        if no_item:
            rospy.logwarn("Rejected grasp: %s (dropped masks %s; %.3fs)", g["reject_reason"],
                          g["dropped_masks"], elapsed)
            return PlanDexnetGraspResponse(msg, False, g["reject_reason"])
        if rejected:
            rospy.logwarn("Rejected grasp: %s < min_score %.3f (%.3fs)", summary, self._min_score, elapsed)
            return PlanDexnetGraspResponse(
                msg, False, "score %.4f below threshold %.3f" % (g["score"], self._min_score))
        rospy.loginfo("Planned suction grasp %s at px (%.0f, %.0f) in %.3fs",
                      summary, msg.center_px[0], msg.center_px[1], elapsed)
        # On success the message names the panel, if one was written (grasp_test.py prints it).
        return PlanDexnetGraspResponse(msg, True, panel and "panel: " + panel)


if __name__ == "__main__":
    import rospy
    SuctionGraspPlanner()
    rospy.spin()
