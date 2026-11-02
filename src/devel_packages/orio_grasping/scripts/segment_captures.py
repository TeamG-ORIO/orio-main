"""Run a fine-tuned RF-DETR segmentation model on real Xtion captures and draw the masks.

Run in the ``orio-grasping`` env::

    conda run -n orio-grasping python scripts/segment_captures.py misc/real-piles/* \\
        --checkpoint /mnt/Shared/orio-grasping/runs/seg/full_v0/checkpoint_best_total.pth --rot180 --trim-top 16 --trim-sides 16

Each capture folder is a packed Xtion capture (``color.png``, ``depth_mm.png``, ``K.json``;
``scripts/pack_xtion_capture.py``). Frames from ``orio-main/snap_xtion.sh`` are upside down
relative to the calibration: pass ``--rot180``. The frame is then cut to the dataset's table
window (``camera.crop`` of a scene's ``meta.json``, ``--meta``), so objects appear at the scale
the model was trained on, and with ``--region bin`` (the default) every pixel outside the bin
region (``workcell.bin_inner`` of that scene, projected with the calibrated camera) is painted
black: in simulation nothing ever lay there, and on the real cell it is table edge, floor and
clutter. The saved images are cut to the bin region. ``--trim-top N`` moves the region's far (top)
edge N pixels down and ``--trim-sides N`` its left and right edges N pixels in: the table top
shifts a little between captures (in the 2026-10-06 ones it sits about 14 px lower than the
calibration puts it), so floor and the table's edges show inside the region otherwise.

Writes into ``--out`` (default ``<checkpoint folder>/real``): per capture ``<stamp>.png`` (the
model's input | the masks over it with outlines and scores) and ``<stamp>.json`` (score, box in
crop-window pixels and
pixel count of each mask), and ``contact_sheet.png`` with every capture. No scores against ground
truth: the captures have no labelled masks yet.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

from orio_grasping.cuda_env import drop_isaac_libraries  # noqa: E402
from orio_grasping.xtion import load_capture, reference_frame  # noqa: E402

MODELS = {
    "nano": "RFDETRSegNano",
    "small": "RFDETRSegSmall",
    "medium": "RFDETRSegMedium",
    "large": "RFDETRSegLarge",
    "xlarge": "RFDETRSegXLarge",
    "2xlarge": "RFDETRSeg2XLarge",
}


def parse_args():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("captures", type=Path, nargs="+", help="Packed capture folders (color.png, ...).")
    p.add_argument("--checkpoint", type=Path, required=True, help="Fine-tuned weights (checkpoint_best_total.pth).")
    p.add_argument("--model", choices=MODELS, default="large", help="Size the checkpoint was trained as.")
    p.add_argument("--threshold", type=float, default=0.5, help="Minimum mask confidence drawn.")
    p.add_argument("--rot180", action="store_true", help="Rotate the frame by 180 deg first (snap_xtion.sh frames).")
    p.add_argument("--meta", type=Path, default=None, help="A scene meta.json for the crop and bin (default: first scene under data/).")
    p.add_argument("--region", choices=("bin", "crop"), default="bin",
                   help="bin: black out everything outside the bin region; crop: the whole training window.")
    p.add_argument("--trim-top", type=int, default=0, help="Move the bin region's top edge this many pixels down (floor showing above the table).")
    p.add_argument("--trim-sides", type=int, default=0, help="Move the bin region's left and right edges this many pixels in.")
    p.add_argument("--config", type=Path, default=None, help="Settings file for the calibrated camera (default: config.toml).")
    p.add_argument("--out", type=Path, default=None, help="Output folder (default <checkpoint folder>/real).")
    return p.parse_args()


def bin_region(meta: dict, cfg, trim_top: int = 0, trim_sides: int = 0) -> tuple[np.ndarray, tuple[int, int, int, int]]:
    """Boolean mask of the bin region inside the dataset crop window, and the window's pixel box
    ``(x0, y0, x1, y1)`` around that region (where the saved images are cut). ``trim_top`` moves
    the far edge (world y_min, the image top) that many pixels down, ``trim_sides`` the two side
    edges (world x_max on the image left, x_min on the right) that many pixels in."""
    from orio_grasping.camera_model import extrinsics, intrinsics, project

    crop = meta["camera"]["crop"]
    cx0, cy0, cx1, cy1 = crop["box_xyxy_full"]
    b, z = meta["workcell"]["bin_inner"], meta["workcell"]["table_top_z"]
    (xa, xb), (ya, yb) = b["x"], b["y"]
    corners = project(np.array([[xa, ya, z], [xb, ya, z], [xb, yb, z], [xa, yb, z]]), intrinsics(cfg), extrinsics(cfg)) - (cx0, cy0)
    corners[:2, 1] += trim_top  # the two y_min corners
    corners[[0, 3], 0] -= trim_sides  # x_min, image right
    corners[[1, 2], 0] += trim_sides  # x_max, image left
    img = Image.new("1", (cx1 - cx0, cy1 - cy0), 0)
    ImageDraw.Draw(img).polygon([tuple(p) for p in corners.tolist()], fill=1)
    mask = np.asarray(img, dtype=bool)
    ys, xs = np.nonzero(mask)
    return mask, (int(xs.min()), int(ys.min()), int(xs.max()) + 1, int(ys.max()) + 1)


def palette(n: int) -> np.ndarray:
    """``n`` distinct bright colours (golden-ratio hues), the same for every image."""
    import colorsys

    return np.array([colorsys.hsv_to_rgb((k * 0.618034) % 1.0, 0.75, 1.0) for k in range(n)]) * 255


def draw_masks(rgb: np.ndarray, masks: np.ndarray, scores: np.ndarray, boxes: np.ndarray) -> Image.Image:
    """Masks tinted over ``rgb`` (most confident drawn last), white outlines, and each score."""
    out = rgb.astype(np.float32)
    colours = palette(len(masks))
    order = np.argsort(scores)
    for k in order:
        m = masks[k]
        out[m] = 0.45 * out[m] + 0.55 * colours[k]
    img = Image.fromarray(out.clip(0, 255).astype(np.uint8))
    edge = np.zeros(rgb.shape[:2], bool)
    for k in order:
        m = masks[k]
        inner = m.copy()
        inner[1:] &= m[:-1]
        inner[:-1] &= m[1:]
        inner[:, 1:] &= m[:, :-1]
        inner[:, :-1] &= m[:, 1:]
        edge |= m & ~inner
    px = np.asarray(img).copy()
    px[edge] = 255
    img = Image.fromarray(px)
    d = ImageDraw.Draw(img)
    for k in order:
        x0, y0 = boxes[k][:2]
        d.rectangle([x0, y0, x0 + 26, y0 + 11], fill=(0, 0, 0))
        d.text((x0 + 2, y0), f"{scores[k]:.2f}", fill=tuple(int(c) for c in colours[k]))
    return img


def main():
    drop_isaac_libraries()
    args = parse_args()
    meta_path = args.meta or next(iter(sorted((REPO_ROOT / "data").glob("*/scene_*/meta.json"))), None)
    if meta_path is None:
        sys.exit("no scene meta.json under data/; pass --meta")
    meta = json.loads(Path(meta_path).read_text())
    crop = meta["camera"]["crop"]
    x0, y0, x1, y1 = crop["box_xyxy_full"]
    if args.region == "bin":
        from orio_grasping.config import load_config

        keep, (vx0, vy0, vx1, vy1) = bin_region(meta, load_config(args.config), args.trim_top, args.trim_sides)
    else:
        keep, (vx0, vy0, vx1, vy1) = np.ones((y1 - y0, x1 - x0), bool), (0, 0, x1 - x0, y1 - y0)
    out = args.out or args.checkpoint.parent / "real"
    out.mkdir(parents=True, exist_ok=True)

    import rfdetr

    model = getattr(rfdetr, MODELS[args.model])(pretrain_weights=str(args.checkpoint))
    rows, summary = [], []
    for folder in args.captures:
        color = load_capture(folder)["color"]
        if color.shape[:2] != (crop["full_height"], crop["full_width"]):
            sys.exit(f"{folder}: frame is {color.shape[1]}x{color.shape[0]}, the dataset crop expects {crop['full_width']}x{crop['full_height']}")
        if args.rot180:
            color = reference_frame(color)
        rgb = np.ascontiguousarray(color[y0:y1, x0:x1])
        rgb[~keep] = 0
        det = model.predict(rgb, threshold=args.threshold, include_source_image=False)
        masks = det.mask if det.mask is not None else np.zeros((0, *rgb.shape[:2]), bool)
        overlay = np.asarray(draw_masks(rgb, masks, det.confidence, det.xyxy))
        row = np.concatenate([rgb[vy0:vy1, vx0:vx1], overlay[vy0:vy1, vx0:vx1]], axis=1)
        Image.fromarray(row).save(out / f"{folder.name}.png")
        records = [
            {"score": round(float(det.confidence[k]), 3), "bbox_xyxy": [round(float(v), 1) for v in det.xyxy[k]], "pixels": int(masks[k].sum())}
            for k in np.argsort(-det.confidence)
        ]
        (out / f"{folder.name}.json").write_text(json.dumps({"capture": str(folder), "rot180": args.rot180, "crop_xyxy_full": [x0, y0, x1, y1],
                                                             "region": args.region, "trim_top": args.trim_top, "trim_sides": args.trim_sides, "view_xyxy_in_crop": [vx0, vy0, vx1, vy1],
                                                             "checkpoint": str(args.checkpoint), "threshold": args.threshold, "masks": records}, indent=1))
        labelled = Image.fromarray(row)
        ImageDraw.Draw(labelled).text((4, 4), f"{folder.name}: {len(records)} masks", fill=(255, 255, 0))
        rows.append(np.asarray(labelled))
        summary.append(len(records))
        print(f"{folder.name}: {len(records)} masks >= {args.threshold}")
    Image.fromarray(np.concatenate(rows, axis=0)).save(out / "contact_sheet.png")
    print(f"wrote {out} ({sum(summary)} masks over {len(summary)} captures)")


if __name__ == "__main__":
    main()
