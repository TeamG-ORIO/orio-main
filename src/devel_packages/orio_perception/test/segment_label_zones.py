#!/usr/bin/env python3
"""Run the label-zone segmentation (scripts/label_zone_seg.py) on saved snapshots.

An offline check of what the labelling path will see. Reads the snapshots of
snap_label_zones.sh (or a demo run's label records: any folder with color.npy, depth.npy
and meta.json), finds the items in each zone exactly as label_seg_node.py does (six passes
of the RGB-D segmentation model, see label_zone_seg.py) and marks the top one.

Writes, under <out>:

    <mode>/<item>/setup_NN_zoneK.png   colour with the masks (cross = top item's pick
                                       point) | height above the table
    sheet_<mode>_<item>.png            all of a group's zones on one sheet
    results.json                       per zone: the items, each pass's count, the time

Needs torch and rfdetr, so it runs in the suction planner's image:

    docker run --rm --gpus all -v <repo>:/repo orio/suction:latest \\
        python /repo/src/devel_packages/orio_perception/test/segment_label_zones.py \\
        /repo/logging/label_zones --out /repo/logging/label_zones_seg/run \\
        --checkpoint /repo/src/devel_packages/orio_grasping/runs/seg/rgbd_v3_deformables/checkpoint_best_total.pth
"""
import argparse
import json
import os
import sys
import time

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(PKG, "scripts"))
import label_zone_seg as lz  # noqa: E402

GRASPING = os.path.join(os.path.dirname(PKG), "orio_grasping")
COLOURS = [(60, 220, 60), (60, 140, 255), (255, 90, 200), (0, 220, 255), (255, 200, 0), (180, 120, 255)]


def make_predict(checkpoint, model_name="RFDETRSegLarge"):
    """The model as label_zone_seg.segment_zone wants it. Loads torch and rfdetr."""
    sys.path.insert(0, GRASPING)
    import rfdetr
    import torch
    from orio_grasping import rgbd_seg
    model = rgbd_seg.load_model(getattr(rfdetr, model_name), checkpoint)

    def predict(rgb, height):
        x = torch.cat([torch.from_numpy(rgb).permute(2, 0, 1).float() / 255.0,
                       torch.from_numpy(rgbd_seg.encode_height(height))])
        det = model.predict(x, threshold=lz.SCORE_MIN, include_source_image=False)
        if det.mask is None or len(det.mask) == 0:
            return np.zeros((0,) + rgb.shape[:2], bool), np.zeros(0)
        return np.asarray(det.mask, bool), np.asarray(det.confidence, dtype=np.float64)
    return predict


def draw(rgb, height, seg, title):
    """Colour with the masks next to the height map, as one BGR picture."""
    left = np.ascontiguousarray(rgb[:, :, ::-1])
    hv = np.clip(np.nan_to_num(height, nan=0.0) / 0.03, 0.0, 1.0)   # 0 to 30 mm
    right = cv2.applyColorMap((hv * 255).astype(np.uint8), cv2.COLORMAP_VIRIDIS)
    for k, (m, it) in enumerate(zip(seg["masks"], seg["items"])):
        col = COLOURS[k % len(COLOURS)]
        left[m] = (0.6 * left[m] + 0.4 * np.array(col)).astype(np.uint8)
        cs, _ = cv2.findContours(m.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for pic in (left, right):
            cv2.drawContours(pic, cs, -1, col, 3)
        cx, cy = (int(round(c)) for c in it["centroid_px"])
        if it["top"]:
            cv2.drawMarker(left, tuple(it["pick_px"]), (255, 255, 255), cv2.MARKER_CROSS, 40, 4)
        txt = "%d%s %.2f" % (k + 1, " TOP" if it["top"] and seg["n_items"] > 1 else "", it["score"])
        cv2.putText(left, txt, (cx - 60, cy - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 5, cv2.LINE_AA)
        cv2.putText(left, txt, (cx - 60, cy - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, col, 2, cv2.LINE_AA)
    pic = np.concatenate([left, right], axis=1)
    cv2.rectangle(pic, (0, 0), (pic.shape[1], 40), (0, 0, 0), -1)
    cv2.putText(pic, title, (10, 29), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2, cv2.LINE_AA)
    return pic


def sheet(pics, cols=4, width=640):
    """Pictures scaled to one width and laid out in a grid."""
    small = [cv2.resize(p, (width, int(round(p.shape[0] * width / p.shape[1])))) for p in pics]
    h = max(s.shape[0] for s in small)
    rows = []
    for i in range(0, len(small), cols):
        row = [np.pad(s, ((0, h - s.shape[0]), (0, 0), (0, 0))) for s in small[i:i + cols]]
        row += [np.zeros((h, width, 3), np.uint8)] * (cols - len(row))
        rows.append(np.concatenate(row, axis=1))
    return np.concatenate(rows, axis=0)


def find_records(root):
    out = []
    for d, _dirs, files in os.walk(root):
        if {"color.npy", "depth.npy", "meta.json"} <= set(files):
            out.append(d)
    return sorted(out)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("root", help="folder of snapshots (searched recursively)")
    ap.add_argument("--out", required=True)
    ap.add_argument("--checkpoint", required=True)
    ap.add_argument("--model", default="RFDETRSegLarge")
    args = ap.parse_args()
    predict = make_predict(args.checkpoint, args.model)

    out_root = os.path.abspath(args.out)
    records = [r for r in find_records(args.root) if not os.path.abspath(r).startswith(out_root)]
    print("%d snapshots under %s" % (len(records), args.root), flush=True)
    results, sheets = [], {}
    for rec in records:
        meta = json.load(open(os.path.join(rec, "meta.json")))
        color = np.load(os.path.join(rec, "color.npy"))
        depth = np.load(os.path.join(rec, "depth.npy")).astype(np.float32)
        zones = meta.get("zones") or {str(z): list(b) for z, b in lz.zone_boxes(
            color.shape, meta.get("crop_xyxy", lz.LBL_CROP_XYXY)).items()}
        rel = os.path.relpath(rec, args.root)
        group = os.path.dirname(rel) or "records"
        for zone, (x1, y1, x2, y2) in sorted(zones.items()):
            rgb = np.ascontiguousarray(color[y1:y2, x1:x2, :3])
            t0 = time.time()
            seg = lz.segment_zone(predict, rgb, depth[y1:y2, x1:x2])
            dt = time.time() - t0
            expected = meta.get("items_per_zone")
            title = "%s zone %s: %d item%s%s  (%s)" % (
                rel, zone, seg["n_items"], "" if seg["n_items"] == 1 else "s",
                "" if expected is None else ", expected %d" % expected,
                " ".join(str(v) for v in seg["votes"].values()))
            pic = draw(rgb, lz.height_map(depth[y1:y2, x1:x2])[0], seg, title)
            path = os.path.join(args.out, group, "%s_zone%s.png" % (os.path.basename(rel), zone))
            os.makedirs(os.path.dirname(path), exist_ok=True)
            cv2.imwrite(path, pic)
            sheets.setdefault(group, []).append(pic)
            results.append({"record": rel, "group": group, "zone": int(zone), "expected": expected,
                            "seconds": round(dt, 3), **{k: v for k, v in seg.items() if k != "masks"}})
            print("%-40s zone %s: %d item(s)%s  passes %s  %.2f s" % (
                rel, zone, seg["n_items"], "" if expected is None else " / expected %d" % expected,
                list(seg["votes"].values()), dt), flush=True)
    for group, pics in sheets.items():
        cv2.imwrite(os.path.join(args.out, "sheet_%s.png" % group.replace(os.sep, "_")), sheet(pics))
    with open(os.path.join(args.out, "results.json"), "w") as fh:
        json.dump({"args": vars(args), "zones": results}, fh, indent=1)

    secs = sorted(r["seconds"] for r in results[1:]) or [0.0]   # the first call warms the model up
    print("\nTime per zone (six passes): median %.2f s, slowest %.2f s" % (secs[len(secs) // 2], secs[-1]))
    print("Items found per zone, by group (count: zones):")
    for group in sheets:
        rs = [r for r in results if r["group"] == group]
        counts = {}
        for r in rs:
            counts[r["n_items"]] = counts.get(r["n_items"], 0) + 1
        exp = rs[0]["expected"]
        line = "  ".join("%d: %d" % kv for kv in sorted(counts.items()))
        if exp == 1:
            line += "   -> %d/%d zones with 1 item" % (counts.get(1, 0), len(rs))
        elif exp is not None:
            line += "   -> %d/%d zones with 2 or more" % (sum(n for c, n in counts.items() if c >= 2), len(rs))
        print("  %-28s %s" % (group, line))


if __name__ == "__main__":
    main()
