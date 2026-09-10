#!/usr/bin/env python3
"""Render planned grasps over the sample scenes as a PNG contact sheet.

    venv/bin/python test/visualize_grasps.py --json results.json --out grasps.png

Reads the JSON written by test_dexnet_service.py --json.
"""
import argparse
import json
import os

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../.."))
SAMPLES = os.path.join(REPO,
                       "src/devel_packages/gqcnn/data/examples/clutter/phoxi/fcgqcnn")
ZOOM_HALF_WIDTH = 90  # pixels either side of the grasp, in colour-image coordinates


def q_colour(q):
    """Red-to-green by grasp quality, over the range the policy actually returns."""
    return plt.cm.RdYlGn(np.clip((q - 0.5) / 0.5, 0.0, 1.0))


def draw_grasp(ax, cx, cy, normal, colour, radius, arrow_len, lw):
    """Grasp point plus the approach direction projected onto the image plane."""
    nx, ny, _ = normal
    ax.arrow(cx - nx * arrow_len, cy - ny * arrow_len, nx * arrow_len, ny * arrow_len,
             width=arrow_len * 0.05, head_width=arrow_len * 0.24,
             length_includes_head=True, color=colour, ec="black", lw=0.5, zorder=3)
    ax.add_patch(Circle((cx, cy), radius, fill=False, ec=colour, lw=lw, zorder=4))
    ax.add_patch(Circle((cx, cy), radius * 0.2, color=colour, zorder=5))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--json", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--samples", default=SAMPLES)
    ap.add_argument("--dpi", type=int, default=125,
                    help="125 for working renders, ~70 for the committed doc image")
    args = ap.parse_args()

    with open(args.json) as f:
        results = json.load(f)

    n = len(results)
    fig, axes = plt.subplots(3, n, figsize=(4.0 * n, 11.4))
    if n == 1:
        axes = axes.reshape(3, 1)

    for col, r in enumerate(results):
        i = r["scene"]
        depth = np.load(os.path.join(args.samples, "depth_%d.npy" % i)).astype(float)
        if depth.ndim == 3:
            depth = depth[:, :, 0]
        colour = plt.imread(os.path.join(args.samples, "color_%d.png" % i))
        segmask = plt.imread(os.path.join(args.samples, "segmask_%d.png" % i))
        mask = segmask if segmask.ndim == 2 else segmask[:, :, 0]

        px, py = r["center_px"]
        q, tilt = r["q_value"], r["tilt_deg"]
        c = q_colour(q)
        # Colour images are twice the depth resolution; grasp pixels are in depth
        # coordinates, so scale them to overlay the colour frame.
        scale = colour.shape[1] / float(depth.shape[1])
        cx, cy = px * scale, py * scale

        # Row 0: full colour scene.
        ax = axes[0, col]
        ax.imshow(colour)
        draw_grasp(ax, cx, cy, r["normal"], c, radius=26, arrow_len=70, lw=2.6)
        ax.set_title("scene %d\nq = %.3f     tilt = %.1f°" % (i, q, tilt),
                     fontsize=12, fontweight="bold")
        ax.axis("off")

        # Row 1: zoomed crop around the grasp.
        ax = axes[1, col]
        h, w = colour.shape[:2]
        x0 = int(np.clip(cx - ZOOM_HALF_WIDTH, 0, w - 2 * ZOOM_HALF_WIDTH))
        y0 = int(np.clip(cy - ZOOM_HALF_WIDTH, 0, h - 2 * ZOOM_HALF_WIDTH))
        ax.imshow(colour[y0:y0 + 2 * ZOOM_HALF_WIDTH, x0:x0 + 2 * ZOOM_HALF_WIDTH])
        draw_grasp(ax, cx - x0, cy - y0, r["normal"], c,
                   radius=20, arrow_len=52, lw=3.0)
        ax.set_title("zoom ±%d px" % ZOOM_HALF_WIDTH, fontsize=10)
        ax.axis("off")

        # Row 2: depth with the segmask outline.
        ax = axes[2, col]
        valid = depth > 0
        vmin = np.percentile(depth[valid], 2) if valid.any() else 0.0
        vmax = np.percentile(depth[valid], 98) if valid.any() else 1.0
        im = ax.imshow(np.where(valid, depth, np.nan), cmap="viridis",
                       vmin=vmin, vmax=vmax)
        ax.contour(mask > 0.5, levels=[0.5], colors="white", linewidths=0.8)
        ax.add_patch(Circle((px, py), 13, fill=False, ec="red", lw=2.4, zorder=4))
        ax.set_title("depth %.2f m     px (%d, %d)"
                     % (r["center"][2], int(px), int(py)), fontsize=10)
        ax.axis("off")
        fig.colorbar(im, ax=ax, fraction=0.046, pad=0.02).ax.tick_params(labelsize=7)

    qs = [r["q_value"] for r in results]
    tilts = [r["tilt_deg"] for r in results]
    fig.suptitle(
        "FC-GQCNN-4.0-SUCTION planned grasps      "
        "q %.3f–%.3f      tilt %.1f–%.1f°\n"
        "circle = suction point, arrow = approach direction; "
        "bottom row shows depth with the bin segmask outlined"
        % (min(qs), max(qs), min(tilts), max(tilts)), fontsize=13)
    fig.tight_layout(rect=[0, 0, 1, 0.945])
    fig.savefig(args.out, dpi=args.dpi)
    print("wrote %s" % args.out)


if __name__ == "__main__":
    main()
