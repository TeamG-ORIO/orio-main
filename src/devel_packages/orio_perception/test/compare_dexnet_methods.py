#!/usr/bin/env python3
"""Side-by-side of the old and new DexNet methods on saved Xtion captures. No arm.

Old: depth-band segmask, DexNet's own approach axis.
New: item mask (GroundingDINO+SAM OR above-table) and the plane-fit approach axis.

Both plan on the same frame through the perception node's own _plan_grasp_dexnet,
so the comparison is the code that runs on the robot. Each scene is a directory
written by capture_xtion.py (color.npy, depth.npy, K.npy). Per scene this writes
<out>/<scene>_compare.png plus the six panels as <scene>_<old|new>_<panel>.png,
and <out>/summary.json covers all scenes.

With --live it captures from the running Xtion instead: every Enter grabs a few
fresh frames (--attempts), saves them under <out>/scenes/ and compares them, each
method keeping its highest-scoring attempt. The planner's own score threshold is
the wrapper's business (it starts the planner with none).

Runs inside the perception container IN PLACE OF the perception node (it creates
the node itself), with roscore and the DexNet planner up. Use the wrapper:

    bash src/devel_packages/orio_bringup/docker/run_dexnet_compare.sh <scene_dir> [...]
    bash src/devel_packages/orio_bringup/docker/run_dexnet_compare.sh --live
"""
import argparse
import json
import os
import re
import sys
import time
import textwrap
import types

import numpy as np
import yaml

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Circle, Patch, Rectangle

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(PKG, "scripts"))

NODE_NS = "/combined_perception_node"
CUP_RADIUS_M = 0.02

# Mask-source colours (fixed order) and grasp markers.
C_ABOVE, C_ITEM, C_BOTH, C_BAND = "#2a78d6", "#eb6834", "#1baf7a", "#eda100"
C_GRASP, C_DEXNET_AXIS = "#00e5ff", "#ffffff"

METHODS = [
    ("old", "Old: depth band, DexNet axis", {"enabled": False}, {"mode": "off"}),
    ("new", "New: item mask, plane-fit axis", {"enabled": True}, {"mode": "override"}),
]


def tilt_deg(approach):
    return float(np.degrees(np.arccos(np.clip(-approach[2], -1.0, 1.0))))


def floor_cut_row(depth, plane, fx, fy, cx, cy):
    """First row of the floor strip at the bottom of the image, or the image height.

    The crop reaches past the table edge, and the floor ~1 m further away there
    wrecks the depth colour scale. Display only; planning uses the full crop.
    """
    import dexnet_geometry as dg
    h = depth.shape[0]
    valid = np.isfinite(depth) & (depth > 0)
    if plane is not None:
        floor = np.nan_to_num(dg.height_above_plane(depth, fx, fy, cx, cy, plane), nan=0.0) < -0.10
    elif valid.any():
        floor = valid & (depth > np.median(depth[valid]) + 0.15)
    else:
        return h
    bad = floor.mean(axis=1) > 0.15
    cut, gap = h, 0
    for row in range(h - 1, -1, -1):
        if bad[row]:
            cut, gap = row, 0
        else:
            gap += 1
            if gap > 3:
                break
    if h - cut < 3:
        return h
    return max(cut - 4, 1)


def plan(node, color, depth, item_over, plane_over, base_item, base_plane):
    """One planning pass with the given item-mask / plane-fit overrides."""
    node.item_mask_cfg = dict(base_item, **item_over)
    node.plane_fit_cfg = dict(base_plane, **plane_over)
    node._table_plane = None
    fake_msg = types.SimpleNamespace(encoding="32FC1")
    grasp, message = node._plan_grasp_dexnet(color, depth, fake_msg)
    dbg = dict(node._dexnet_debug)
    dbg["accepted"] = grasp is not None
    dbg["message"] = message
    return dbg


def record(dbg):
    """JSON-friendly summary of one planning pass."""
    rec = {"accepted": dbg["accepted"], "message": dbg["message"],
           "q_value": dbg.get("q_value"),
           "mask_px": int(dbg["mask"].sum()) if "mask" in dbg else 0}
    if "items" in dbg:
        rec.update(detections=0 if dbg.get("boxes") is None else int(len(dbg["boxes"])),
                   item_px=int(dbg["items"].sum()), above_table_px=int(dbg["above"].sum()))
    if "approach" in dbg:
        rec.update(center_px=list(dbg["center_px"]),
                   center_world=[float(v) for v in dbg["centre_world"]],
                   tilt_deg=tilt_deg(dbg["approach"]),
                   tilt_dexnet_deg=tilt_deg(dbg["approach_dexnet"]),
                   axis_source=dbg["approach_source"])
    return rec


def overlay(color, layers, dim=0.45, alpha=0.65):
    """RGB dimmed, with (mask, hex colour) layers blended on top."""
    out = color.astype(np.float32) / 255.0 * dim
    for mask, hex_colour in layers:
        rgb = np.array(matplotlib.colors.to_rgb(hex_colour), dtype=np.float32)
        out[mask] = (1 - alpha) * out[mask] + alpha * rgb
    return np.clip(out, 0, 1)


def draw_axis(ax, centre, approach_world, rot_world_to_cam, colour, dashed=False):
    """Arrow along the approach axis as seen in the image; length grows with tilt."""
    d = rot_world_to_cam.dot(approach_world)[:2] * 90.0
    if np.linalg.norm(d) < 2.0:
        return
    end = (centre[0] + d[0], centre[1] + d[1])
    for col, lw in (("black", 4.0), (colour, 2.0)):
        ax.annotate("", xy=end, xytext=centre, zorder=6,
                    arrowprops=dict(arrowstyle="-|>", color=col, lw=lw,
                                    linestyle="--" if dashed else "-",
                                    shrinkA=0, shrinkB=0))


PANELS = ("depth", "segmask", "grasp")
PANEL_TITLES = {"depth": "Depth inside the mask", "segmask": "Segmask", "grasp": "Chosen grasp"}


def depth_range(runs, cut):
    """Colour limits shared by every depth panel of a scene."""
    ref = runs["old"]
    depth, band = ref["depth"][:cut], ref["band"][:cut]
    in_band = depth[band & np.isfinite(depth)]
    return tuple(np.percentile(in_band, [1, 99])) if in_band.size else (0.6, 1.2)


def draw_panel(fig, ax, panel, dbg, cut, fx, rot_world_to_cam, vlim):
    """Draw one panel for one method; returns the text describing it."""
    color, depth = dbg["color"][:cut], dbg["depth"][:cut]
    mask = dbg["mask"][:cut]
    info = ""

    if panel == "depth":
        ax.imshow(np.full(color.shape, 0.82, dtype=np.float32))
        im = ax.imshow(np.ma.masked_where(~mask | ~np.isfinite(depth), depth),
                       cmap="viridis", vmin=vlim[0], vmax=vlim[1], interpolation="nearest")
        fig.colorbar(im, ax=ax, fraction=0.043, pad=0.02).set_label("depth (m)", fontsize=8)

    elif panel == "segmask":
        if "items" in dbg:
            items, above = dbg["items"][:cut], dbg["above"][:cut]
            ax.imshow(overlay(color, [(above & ~items, C_ABOVE), (items & ~above, C_ITEM),
                                      (items & above, C_BOTH)]))
            for box in (dbg.get("boxes") if dbg.get("boxes") is not None else []):
                x1, y1, x2, y2 = [float(v) for v in box]
                ax.add_patch(Rectangle((x1, y1), x2 - x1, min(y2, cut - 1) - y1, fill=False,
                                       edgecolor="white", linewidth=0.9, linestyle=":"))
            handles = [Patch(color=C_ITEM, label="detected item"),
                       Patch(color=C_ABOVE, label="above table"),
                       Patch(color=C_BOTH, label="both"),
                       Line2D([], [], color="white", linestyle=":", label="detection box")]
        else:
            ax.imshow(overlay(color, [(mask, C_BAND)]))
            handles = [Patch(color=C_BAND, label="inside depth band")]
        leg = ax.legend(handles=handles, loc="upper center", bbox_to_anchor=(0.5, -0.02),
                        ncol=len(handles), frameon=True, fontsize=7.5, handlelength=1.4,
                        columnspacing=1.0, facecolor="#555555", edgecolor="none")
        for text in leg.get_texts():
            text.set_color("white")
        info = "%d px" % int(dbg["mask"].sum())

    else:
        ax.imshow(color)
        if "approach" in dbg:
            centre = dbg["center_px"]
            radius = fx * CUP_RADIUS_M / float(dbg["centre_cam"][2])
            for col, lw in (("black", 4.0), (C_GRASP, 2.0)):
                ax.add_patch(Circle(centre, radius, fill=False, edgecolor=col, linewidth=lw,
                                    zorder=5))
            ax.plot([centre[0]], [centre[1]], "o", color=C_GRASP, markeredgecolor="black",
                    markersize=6, zorder=7)
            overridden = dbg["approach_source"] != "dexnet"
            if overridden:
                draw_axis(ax, centre, dbg["approach_dexnet"], rot_world_to_cam,
                          C_DEXNET_AXIS, dashed=True)
            draw_axis(ax, centre, dbg["approach"], rot_world_to_cam, C_GRASP)
            info = "q=%.3f   tilt %.1f deg   axis: %s" % (
                dbg["q_value"], tilt_deg(dbg["approach"]), dbg["approach_source"])
            if overridden:
                info += "\nDexNet's own axis: %.1f deg" % tilt_deg(dbg["approach_dexnet"])
            if not dbg["accepted"]:
                info = "REJECTED: " + textwrap.fill(dbg["message"], 46) + "\n" + info
        else:
            info = "NO GRASP: " + textwrap.fill(dbg["message"] or "", 46)

    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_xlim(-0.5, color.shape[1] - 0.5)
    ax.set_ylim(color.shape[0] - 0.5, -0.5)
    return info


def render(out_dir, scene, runs, cut, fx, rot_world_to_cam, note):
    """Write the combined 2x3 figure and the six panels as separate images."""
    vlim = depth_range(runs, cut)
    files = {}

    fig, axes = plt.subplots(2, 3, figsize=(13.5, 8.6))
    for r, (key, row_label, _, _) in enumerate(METHODS):
        for c, panel in enumerate(PANELS):
            info = draw_panel(fig, axes[r, c], panel, runs[key], cut, fx, rot_world_to_cam, vlim)
            head = PANEL_TITLES[panel] if r == 0 else ""
            axes[r, c].set_title("\n".join(t for t in (head, info) if t),
                                 fontsize=10 if panel == "grasp" else 11)
        axes[r, 0].set_ylabel(row_label, fontsize=11, fontweight="bold")
    fig.suptitle("DexNet old vs new: %s%s" % (scene, note), fontsize=13)
    fig.text(0.5, 0.012, "Ring: suction cup footprint. Arrow: approach direction seen from "
             "the camera, longer for a larger tilt.", ha="center", fontsize=8, color="#444444")
    fig.tight_layout(rect=(0, 0.025, 1, 0.96))
    files["combined"] = scene + "_compare.png"
    fig.savefig(os.path.join(out_dir, files["combined"]), dpi=200)
    plt.close(fig)

    for key, row_label, _, _ in METHODS:
        for panel in PANELS:
            fig, ax = plt.subplots(figsize=(5.6, 5.4))
            info = draw_panel(fig, ax, panel, runs[key], cut, fx, rot_world_to_cam, vlim)
            title = "%s\n%s" % (row_label, PANEL_TITLES[panel])
            ax.set_title(title + ("\n" + info if info else ""), fontsize=10)
            fig.tight_layout()
            files["%s_%s" % (key, panel)] = "%s_%s_%s.png" % (scene, key, panel)
            fig.savefig(os.path.join(out_dir, files["%s_%s" % (key, panel)]), dpi=200,
                        bbox_inches="tight")
            plt.close(fig)
    return files


def load_frames(scene_dir):
    """Frames of a scene: color.npy/depth.npy, then color_1.npy/depth_1.npy, ..."""
    frames, k = [], 0
    while True:
        suffix = "" if k == 0 else "_%d" % k
        c = os.path.join(scene_dir, "color%s.npy" % suffix)
        d = os.path.join(scene_dir, "depth%s.npy" % suffix)
        if not (os.path.exists(c) and os.path.exists(d)):
            return frames
        frames.append((np.load(c)[:, :, :3], np.load(d).astype(np.float32)))
        k += 1


def compare_scene(node, pcp, scene_dir, out_dir, base_item, base_plane):
    """Plan with both methods on a saved scene, render it, return its summary row.

    Every frame of the scene is planned with both methods and each method keeps
    its highest-scoring attempt.
    """
    import open3d as o3d
    import rospy
    scene = os.path.basename(os.path.normpath(scene_dir))
    frames = load_frames(scene_dir)
    if not frames:
        raise IOError("no color.npy/depth.npy in %s" % scene_dir)
    K = np.load(os.path.join(scene_dir, "K.npy"))
    node.pnp_intrinsics = o3d.camera.PinholeCameraIntrinsic(
        width=frames[0][0].shape[1], height=frames[0][0].shape[0],
        fx=K[0, 0], fy=K[1, 1], cx=K[0, 2], cy=K[1, 2])

    runs = {}
    for key, _, item_over, plane_over in METHODS:
        attempts = []
        for index, (color, depth) in enumerate(frames):
            color = np.ascontiguousarray(
                color[pcp.PNP_CROP_Y1:pcp.PNP_CROP_Y2, pcp.PNP_CROP_X1:pcp.PNP_CROP_X2])
            depth = np.ascontiguousarray(
                depth[pcp.PNP_CROP_Y1:pcp.PNP_CROP_Y2, pcp.PNP_CROP_X1:pcp.PNP_CROP_X2])
            dbg = plan(node, color, depth, item_over, plane_over, base_item, base_plane)
            dbg.update(color=color, depth=depth, frame=index)
            attempts.append(dbg)
        best = max(attempts, key=lambda d: d.get("q_value", -1.0))
        best["attempt_q"] = [d.get("q_value") for d in attempts]
        runs[key] = best

    intr = node._cropped_intrinsics()
    fx, fy = intr.get_focal_length()
    cx, cy = intr.get_principal_point()
    ref = runs["new"]
    cut = floor_cut_row(ref["depth"], ref.get("table_plane"), fx, fy, cx, cy)
    # Never crop a chosen grasp out of the figure.
    if any("center_px" in d and d["center_px"][1] >= cut - 1 for d in runs.values()):
        cut = ref["depth"].shape[0]

    note = "" if len(frames) == 1 else "  (best of %d frames per method)" % len(frames)
    files = render(out_dir, scene, runs, cut, fx, node.tf_pnp[:3, :3].T, note)
    rospy.loginfo("Compared %s -> %s", scene, os.path.join(out_dir, files["combined"]))
    row = {"scene": scene, "figure": files["combined"], "panels": files,
           "frames": len(frames), "floor_rows_cropped": int(ref["depth"].shape[0] - cut)}
    for key in ("old", "new"):
        row[key] = record(runs[key])
        row[key].update(best_frame=runs[key]["frame"], attempt_q=runs[key]["attempt_q"])
    return row


def capture_scene(node, scene_dir, frames=3, timeout=5.0):
    """Save the next Xtion frames as a capture_xtion.py-style scene directory.

    Frame 0 is color.npy/depth.npy; further frames are color_1.npy/depth_1.npy, ...
    """
    os.makedirs(scene_dir, exist_ok=True)
    for stale in os.listdir(scene_dir):
        if re.match(r"(color|depth)(_\d+)?\.npy$", stale):
            os.remove(os.path.join(scene_dir, stale))
    for k in range(frames):
        node.pnp_rgb_msg = node.pnp_depth_msg = None  # only frames from after the keypress
        deadline = time.time() + timeout
        while node.pnp_rgb_msg is None or node.pnp_depth_msg is None:
            if time.time() > deadline:
                return "no camera frames within %.0f s" % timeout
            time.sleep(0.05)
        depth_msg = node.pnp_depth_msg
        color, depth = node.decode_ros_images(node.pnp_rgb_msg, depth_msg)
        if color is None:
            return "could not decode the camera images"
        suffix = "" if k == 0 else "_%d" % k
        np.save(os.path.join(scene_dir, "color%s.npy" % suffix),
                np.ascontiguousarray(color[:, :, :3]))
        np.save(os.path.join(scene_dir, "depth%s.npy" % suffix),
                node._depth_to_metres(depth, depth_msg))
    np.save(os.path.join(scene_dir, "K.npy"), np.asarray(node.pnp_intrinsics.intrinsic_matrix))
    return ""


def write_summary(out_dir, summary):
    with open(os.path.join(out_dir, "summary.json"), "w") as f:
        json.dump(summary, f, indent=2)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("scenes", nargs="*", help="capture directories (color.npy, depth.npy, K.npy)")
    ap.add_argument("--live", action="store_true",
                    help="capture from the running Xtion on each Enter instead")
    ap.add_argument("--attempts", type=int, default=3,
                    help="--live: frames captured per Enter; each method keeps its best score")
    ap.add_argument("--out", default=os.path.join(os.getcwd(), "dexnet_compare"))
    args = ap.parse_args()
    if not args.live and not args.scenes:
        ap.error("give scene directories or --live")
    os.makedirs(args.out, exist_ok=True)

    import rosgraph
    import rospy
    if not rosgraph.is_master_online():
        sys.exit("No ROS master. Start roscore and the DexNet planner first.")
    try:
        rosgraph.Master("/dexnet_compare").lookupService("/compute_grasps")
        sys.exit("The perception node is running (/compute_grasps exists). Stop it first: "
                 "this script creates the node itself and would replace the live one.")
    except rosgraph.MasterError:
        pass

    # Same config as perception.launch, with the models loaded. Replay takes its
    # intrinsics from each scene's K.npy, so only --live waits for camera_info.
    with open(os.path.join(PKG, "config", "grasp.yaml")) as f:
        cfg = yaml.safe_load(f)
    cfg["grasp_backend"] = "dexnet"
    cfg["dexnet"].setdefault("item_mask", {})["enabled"] = True
    cfg["camera"]["use_camera_info"] = bool(args.live)
    rospy.set_param(NODE_NS, cfg)

    import perception_control_combined_pass_through as pcp
    try:
        node = pcp.CombinedPerceptionNode()
        if node._dexnet_srv is None:
            sys.exit("DexNet planner service not reachable.")
        base_item, base_plane = dict(node.item_mask_cfg), dict(node.plane_fit_cfg)
        live_intrinsics = node.pnp_intrinsics

        summary = []
        for scene_dir in args.scenes:
            summary.append(compare_scene(node, pcp, scene_dir, args.out, base_item, base_plane))
            write_summary(args.out, summary)

        while args.live:
            try:
                typed = input("\nEnter = capture and compare (optionally type a scene name "
                              "first), q = quit: ").strip()
            except EOFError:
                break
            if typed.lower() in ("q", "quit", "exit"):
                break
            name = re.sub(r"[^A-Za-z0-9_.-]+", "_", typed) or time.strftime("scene_%Y%m%d_%H%M%S")
            scene_dir = os.path.join(args.out, "scenes", name)
            node.pnp_intrinsics = live_intrinsics
            error = capture_scene(node, scene_dir, frames=max(args.attempts, 1))
            if error:
                print("Capture failed: %s. Is the Xtion publishing?" % error)
                continue
            summary = [row for row in summary if row["scene"] != name]
            summary.append(compare_scene(node, pcp, scene_dir, args.out, base_item, base_plane))
            write_summary(args.out, summary)
            print("Saved %s" % os.path.join(args.out, summary[-1]["figure"]))

        print("Wrote %d figure(s) and summary.json to %s" % (len(summary), args.out))
    finally:
        try:
            rospy.delete_param(NODE_NS)
        except Exception:
            pass


if __name__ == "__main__":
    main()
