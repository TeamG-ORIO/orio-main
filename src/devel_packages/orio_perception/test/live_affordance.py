#!/usr/bin/env python3
"""Capture from the Xtion, run DexNet, and render the affordance map + grasp pose.

Runs inside the DexNet container (it needs gqcnn), reading a capture written by
capture_xtion.py. Produces a per-run PNG and a JSON record.

    python live_affordance.py <workdir> <run_label>
"""
import json
import os
import sys

import numpy as np
from scipy import ndimage
import logging
logging.getLogger().setLevel(logging.ERROR)

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle

from autolab_core import (YamlConfig, CameraIntrinsics, DepthImage, BinaryImage,
                          RgbdImage, ColorImage)
import gqcnn.model.tf.fc_network_tf as _fcmod
from gqcnn.grasping import FullyConvolutionalGraspingPolicySuction, RgbdImageState

# See dexnet_grasp_planner.py: _parse_config ignores batch_size, leaving it at the
# model's training value of 64 and computing 64x the work for one image.
_orig = _fcmod.FCGQCNNTF._parse_config


def _patched(self, cfg):
    _orig(self, cfg)
    if "batch_size" in cfg:
        self._batch_size = cfg["batch_size"]


_fcmod.FCGQCNNTF._parse_config = _patched

G = "/opt/gqcnn"
# Black-background workspace only: no desk (x<110), no table edge or floor (y>375).
CROP = (20, 375, 110, 455)
DEPTH_LO, DEPTH_HI = 0.80, 0.94      # table sits at ~0.956 m
MIN_Q = 0.30
# FC-GQCNN's receptive field is 96 px and Berkeley's training objects span ~30-70 px.
# At this camera height ours span ~100-130 px, which depresses q. Downscaling brings
# them back into the range the network expects.
RESCALE = 1.0


def main():
    global RESCALE
    work, label = sys.argv[1], sys.argv[2]
    if len(sys.argv) > 3:
        RESCALE = float(sys.argv[3])
    colour_full = np.load(work + "/color.npy")
    depth_full = np.load(work + "/depth.npy").astype(np.float32)
    K = np.load(work + "/K.npy")

    y0, y1, x0, x1 = CROP
    c = np.ascontiguousarray(colour_full[y0:y1, x0:x1, :3])
    d = np.ascontiguousarray(depth_full[y0:y1, x0:x1]).copy()
    d[~np.isfinite(d)] = 0.0

    fx, fy = K[0, 0], K[1, 1]
    cx, cy = K[0, 2] - x0, K[1, 2] - y0
    if RESCALE != 1.0:
        c = ndimage.zoom(c, (RESCALE, RESCALE, 1), order=1).astype(np.uint8)
        d = np.ascontiguousarray(ndimage.zoom(d, (RESCALE, RESCALE), order=0))
        fx, fy = fx * RESCALE, fy * RESCALE
        cx, cy = cx * RESCALE, cy * RESCALE
        print("    rescaled to %dx%d (scale %.2f)" % (d.shape[1], d.shape[0], RESCALE))

    intr = CameraIntrinsics(frame="camera", fx=fx, fy=fy, cx=cx, cy=cy,
                            height=d.shape[0], width=d.shape[1])

    cfg = YamlConfig(G + "/cfg/examples/fc_gqcnn_suction.yaml")
    pc = cfg["policy"]
    pc["metric"]["gqcnn_model"] = G + "/models/FC-GQCNN-4.0-SUCTION"
    fc = pc["metric"]["fully_conv_gqcnn_config"]
    fc["batch_size"] = 1
    fc["im_height"], fc["im_width"] = d.shape
    policy = FullyConvolutionalGraspingPolicySuction(pc)

    depth_im = DepthImage(d[:, :, None], frame=intr.frame)
    colour_im = ColorImage(c, frame=intr.frame)
    seg_arr = (((d >= DEPTH_LO) & (d <= DEPTH_HI)).astype(np.uint8) * 255)
    seg = BinaryImage(seg_arr, frame=intr.frame)
    seg = seg.mask_binary(depth_im.invalid_pixel_mask().inverse())
    depth_im = depth_im.inpaint(rescale_factor=cfg["inpaint_rescale_factor"])
    state = RgbdImageState(RgbdImage.from_color_and_depth(colour_im, depth_im),
                           intr, segmask=seg)

    # Dense affordance map, the same tensor the policy takes its argmax from.
    wrapped, raw_depth, raw_seg, ci = policy._unpack_state(state)
    images, depths = policy._gen_images_and_depths(raw_depth, raw_seg)
    preds = policy._grasp_quality_fn.quality(images, depths)
    succ = policy._mask_predictions(preds[:, :, :, 1::2], raw_seg)[0, :, :, 0]

    action = policy(state)
    g = action.grasp
    approach = g.pose().rotation[:, 0]          # X column is the approach axis
    if approach[2] < 0:
        approach = -approach
    tilt = float(np.degrees(np.arccos(np.clip(approach[2], -1.0, 1.0))))
    px, py = float(g.center[0]), float(g.center[1])
    q = float(action.q_value)

    rec = {"label": label, "q": q, "accepted": bool(q >= MIN_Q), "tilt_deg": tilt,
           "depth_m": float(g.depth), "center_px_crop": [px, py],
           "center_px_full": [px / RESCALE + x0, py / RESCALE + y0],
           "rescale": RESCALE,
           "approach_camera": [float(v) for v in approach],
           "crop": list(CROP), "mask_px": int((seg_arr > 0).sum()),
           "pose_camera": [float(v) for v in g.pose().translation]}
    json.dump(rec, open("%s/run_%s.json" % (work, label), "w"), indent=2)
    np.save("%s/afford_%s.npy" % (work, label), succ)

    print("RUN %s: q=%.4f %s  tilt=%.1f deg  depth=%.3f m  px_full=(%.0f,%.0f)"
          % (label, q, "ACCEPT" if q >= MIN_Q else "REJECT (q<%.2f)" % MIN_Q,
             tilt, g.depth, px / RESCALE + x0, py / RESCALE + y0))
    print("    approach (camera frame) = [%.3f, %.3f, %.3f]" % tuple(approach))

    render(work, label, c, d, seg_arr, succ, rec, policy)


def render(work, label, c, d, seg_arr, succ, rec, policy):
    px, py = rec["center_px_crop"]
    q, tilt = rec["q"], rec["tilt_deg"]
    ok = rec["accepted"]
    col = "cyan" if ok else "red"
    ax_col = "black" if ok else "darkred"
    stride = policy._gqcnn_stride
    recep = policy._gqcnn_recep_h

    fig, ax = plt.subplots(2, 3, figsize=(17, 10.5))

    ax[0, 0].imshow(c)
    ax[0, 0].set_title("workspace %dx%d%s\n(black background only)"
                       % (c.shape[1], c.shape[0],
                          "" if rec["rescale"] == 1.0 else "  scale %.2f" % rec["rescale"]),
                       fontweight="bold")

    valid = d > 0
    im = ax[0, 1].imshow(np.where(valid, d, np.nan), cmap="viridis")
    ax[0, 1].set_title("depth (m)")
    fig.colorbar(im, ax=ax[0, 1], fraction=0.046)

    ov = c.copy()
    ov[seg_arr > 0] = [0, 255, 0]
    ax[0, 2].imshow(ov)
    ax[0, 2].set_title("segmask: depth %.2f-%.2f m\n%d px"
                       % (DEPTH_LO, DEPTH_HI, rec["mask_px"]))

    im = ax[1, 0].imshow(succ, cmap="inferno", vmin=0, vmax=1)
    ax[1, 0].set_title("affordance map %dx%d\nstride %d, receptive field %d"
                       % (succ.shape[1], succ.shape[0], stride, recep))
    fig.colorbar(im, ax=ax[1, 0], fraction=0.046)

    # Upsample the map back to image pixels: index i covers pixel i*stride + recep/2.
    up = np.full(d.shape, np.nan)
    ys = (np.arange(succ.shape[0]) * stride + recep // 2)
    xs = (np.arange(succ.shape[1]) * stride + recep // 2)
    keep_y = ys < d.shape[0]
    keep_x = xs < d.shape[1]
    blk = np.kron(succ[np.ix_(keep_y, keep_x)], np.ones((stride, stride)))
    oy, ox = recep // 2, recep // 2
    hh = min(blk.shape[0], d.shape[0] - oy)
    ww = min(blk.shape[1], d.shape[1] - ox)
    up[oy:oy + hh, ox:ox + ww] = blk[:hh, :ww]
    ax[1, 1].imshow(c)
    ax[1, 1].imshow(up, cmap="inferno", alpha=0.6, vmin=0, vmax=1)
    ax[1, 1].set_title("affordance over scene")

    Z = 60
    xs0, ys0 = max(int(px) - Z, 0), max(int(py) - Z, 0)
    ax[1, 2].imshow(c[ys0:ys0 + 2 * Z, xs0:xs0 + 2 * Z])
    nx, ny = rec["approach_camera"][0], rec["approach_camera"][1]
    L = 34
    ax[1, 2].arrow(px - xs0 - nx * L, py - ys0 - ny * L, nx * L, ny * L,
                   width=2, head_width=9, length_includes_head=True,
                   color=col, ec="black", lw=0.5, zorder=3)
    ax[1, 2].add_patch(Circle((px - xs0, py - ys0), 14, fill=False, ec=col, lw=3, zorder=4))
    ax[1, 2].add_patch(Circle((px - xs0, py - ys0), 3, color=col, zorder=5))
    ax[1, 2].set_title("chosen grasp\nq=%.3f  tilt=%.1f deg  depth %.3f m"
                       % (q, tilt, rec["depth_m"]), color=ax_col, fontweight="bold")

    for a in ax.ravel():
        a.axis("off")
    fig.suptitle("DexNet 4.0 suction - run '%s'%s   %s   q=%.3f, tilt %.1f deg"
                 % (label, "" if rec["rescale"] == 1.0 else "  (rescale %.2f)" % rec["rescale"],
                    "ACCEPTED" if ok else "REJECTED", q, tilt),
                 fontsize=14, color=ax_col)
    fig.tight_layout(rect=[0, 0, 1, 0.94])
    out = "%s/run_%s.png" % (work, label)
    fig.savefig(out, dpi=105)
    print("    wrote %s" % out)


if __name__ == "__main__":
    main()
