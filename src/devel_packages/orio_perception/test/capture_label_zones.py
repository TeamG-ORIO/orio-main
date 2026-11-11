#!/usr/bin/env python3
"""Collect RGB + depth snapshots of the two label zones, one per press of Enter.

For each item type you place items in BOTH zones by hand, press Enter, and the script
saves what the label camera sees: the ZED left image and the Fast-FoundationStereo depth
of the same moment, exactly what /compute_grasps_labelling works from. Then you rearrange
and press Enter again. Two modes, saved apart:

    --mode double (default)  two overlapping items in each zone, as after a double pick
    --mode single            one item in each zone: any items, no names asked, saved as
                             <outdir>/single/setup_NN/ until you quit

    <outdir>/<mode>/<item>/setup_01/color.npy   ZED left RGB, full frame (uint8)
                                    depth.npy   depth in metres, NaN where invalid (float32)
                                    K.npy       3x3 intrinsics
                                    meta.json   item, mode, items per zone, time, the crop
                                                and the zone boxes
                                    zone1.png   zone 1 (A) as the labelling crop shows it
                                    zone2.png   zone 2 (B)
                                    preview.png the crop, RGB above depth

color/depth/K/meta are laid out like the label records a demo run saves
(logging/demo/<run>.labels/label_NNNN_hhmmss/), so one offline tool reads both.

Needs rospy, a ROS master, the ZED and /ffs/compute. From the repo root (starts what is
missing):

    bash snap_label_zones.sh

or by hand inside the perception container:

    python3 test/capture_label_zones.py <outdir> [--mode single] [--setups 10] [--items a,b,c]

Giving an outdir that already has snapshots carries on where it stopped.
"""
import argparse
import json
import os
import re
import shutil
import sys
import time

import cv2
import numpy as np

# The labelling crop of the ZED frame and how it is split into zones: zone 1 is the left
# half, zone 2 the right half. Keep in step with LBL_CROP_* in
# scripts/perception_control_combined_pass_through.py and opt_label_loc.
CROP_X1, CROP_Y1, CROP_X2, CROP_Y2 = 160, 180, 1860, 1050
MIN_VALID_FRAC = 0.80   # warn when less of a zone's depth is valid
# mode -> (items in each zone, what the operator is asked to place)
MODES = {"double": (2, "TWO overlapping items in EACH zone"),
         "single": (1, "ONE item in EACH zone")}

FFS_COMPUTE_SRV = "/ffs/compute"
FFS_LEFT_TOPIC = "/ffs/left"
FFS_DEPTH_TOPIC = "/ffs/depth"
FFS_INFO_TOPIC = "/ffs/camera_info"


def zone_boxes(shape):
    """{zone: (x1, y1, x2, y2)} in full-frame pixels, clipped to the frame."""
    h, w = shape[:2]
    x1, y1, x2, y2 = max(CROP_X1, 0), max(CROP_Y1, 0), min(CROP_X2, w), min(CROP_Y2, h)
    mid = x1 + (x2 - x1) // 2
    return {1: (x1, y1, mid, y2), 2: (mid, y1, x2, y2)}


def depth_picture(depth):
    """Depth as a viridis BGR picture scaled to its own valid range; invalid = black."""
    valid = np.isfinite(depth) & (depth > 0)
    if valid.any():
        lo, hi = np.percentile(depth[valid], [1, 99])
        hi = max(hi, lo + 1e-3)
    else:
        lo, hi = 0.0, 1.0
    t = np.clip((np.where(valid, depth, lo) - lo) / (hi - lo), 0.0, 1.0)
    # near = bright: an item on the table stands out from it
    pic = cv2.applyColorMap(((1.0 - t) * 255).astype(np.uint8), cv2.COLORMAP_VIRIDIS)
    pic[~valid] = 0
    return pic, float(lo), float(hi)


def save_setup(path, color, depth, K, meta):
    """Write one snapshot into `path` (created). Returns {zone: valid depth fraction}."""
    os.makedirs(path)
    boxes = zone_boxes(color.shape)
    np.save(os.path.join(path, "color.npy"), color)
    np.save(os.path.join(path, "depth.npy"), depth)
    np.save(os.path.join(path, "K.npy"), np.asarray(K, dtype=np.float64).reshape(3, 3))

    valid = {}
    for zone, (x1, y1, x2, y2) in boxes.items():
        cv2.imwrite(os.path.join(path, "zone%d.png" % zone), color[y1:y2, x1:x2, ::-1])
        valid[zone] = float(np.isfinite(depth[y1:y2, x1:x2]).mean())

    x1, y1, _, y2 = boxes[1]
    x2 = boxes[2][2]
    rgb = np.ascontiguousarray(color[y1:y2, x1:x2, ::-1])
    dep, lo, hi = depth_picture(depth[y1:y2, x1:x2])
    mid = boxes[1][2] - x1
    for pic in (rgb, dep):
        cv2.line(pic, (mid, 0), (mid, pic.shape[0] - 1), (255, 255, 255), 2)
        cv2.putText(pic, "zone 1", (12, 34), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(pic, "zone 2", (mid + 12, 34), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2, cv2.LINE_AA)
    cv2.putText(dep, "depth %.3f (bright) to %.3f m" % (lo, hi), (12, dep.shape[0] - 14),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2, cv2.LINE_AA)
    cv2.imwrite(os.path.join(path, "preview.png"), np.concatenate([rgb, dep], axis=0))

    meta = dict(meta, crop_xyxy=[x1, y1, x2, y2],
                zones={str(z): list(b) for z, b in boxes.items()},
                depth_valid_frac={str(z): round(v, 4) for z, v in valid.items()})
    with open(os.path.join(path, "meta.json"), "w") as fh:
        json.dump(meta, fh, indent=1)
    return valid


def setups_done(item_dir):
    """Numbers of the setups already saved for an item, sorted."""
    if not os.path.isdir(item_dir):
        return []
    return sorted(int(m.group(1)) for m in (re.fullmatch(r"setup_(\d+)", e) for e in os.listdir(item_dir)) if m)


def folder_name(item):
    return re.sub(r"[^A-Za-z0-9._-]+", "_", item.strip()).strip("_")


class Camera:
    """One fresh ZED left + Fast-FoundationStereo depth pair per grab()."""

    def __init__(self):
        import rospy
        from sensor_msgs.msg import CameraInfo, Image
        from std_srvs.srv import Trigger
        self.rospy, self.Trigger = rospy, Trigger
        self.lefts, self.infos, self.depth, self.depth_rx = {}, {}, None, None
        rospy.Subscriber(FFS_LEFT_TOPIC, Image, self._keep(self.lefts), queue_size=2, buff_size=2 ** 26)
        rospy.Subscriber(FFS_INFO_TOPIC, CameraInfo, self._keep(self.infos), queue_size=2)
        rospy.Subscriber(FFS_DEPTH_TOPIC, Image, self._on_depth, queue_size=2, buff_size=2 ** 26)

    @staticmethod
    def _keep(store, keep=10):
        def cb(msg):
            store[msg.header.stamp.to_nsec()] = msg
            while len(store) > keep:
                del store[min(store)]
        return cb

    def _on_depth(self, msg):
        self.depth, self.depth_rx = msg, self.rospy.Time.now()

    def grab(self):
        """(RGB uint8, depth float32 metres with NaN invalid, CameraInfo, FFS's message).
        Raises RuntimeError with the reason."""
        rospy = self.rospy
        t_req = rospy.Time.now()
        try:
            rospy.wait_for_service(FFS_COMPUTE_SRV, timeout=10.0)
            resp = rospy.ServiceProxy(FFS_COMPUTE_SRV, self.Trigger)()
        except Exception as exc:  # noqa: BLE001 - reported to the operator
            raise RuntimeError("%s unavailable: %s" % (FFS_COMPUTE_SRV, exc))
        if not resp.success:
            raise RuntimeError("%s: %s" % (FFS_COMPUTE_SRV, resp.message))
        deadline = time.time() + 10.0
        while time.time() < deadline:
            d = self.depth
            if d is not None and self.depth_rx > t_req:
                key = d.header.stamp.to_nsec()
                if key in self.lefts and key in self.infos:
                    left = self.lefts[key]
                    color = np.frombuffer(left.data, np.uint8).reshape(left.height, left.step)
                    color = color[:, :left.width * (left.step // left.width)].reshape(left.height, left.width, -1)[:, :, :3]
                    if "bgr" in left.encoding.lower():
                        color = color[:, :, ::-1]
                    if d.encoding == "32FC1":
                        depth = np.frombuffer(d.data, np.float32).reshape(d.height, d.width).copy()
                    elif d.encoding == "16UC1":
                        depth = np.frombuffer(d.data, np.uint16).reshape(d.height, d.width).astype(np.float32) / 1000.0
                    else:
                        raise RuntimeError("unsupported depth encoding %r" % d.encoding)
                    depth[~np.isfinite(depth) | (depth <= 0)] = np.nan
                    if depth.shape != color.shape[:2]:
                        raise RuntimeError("depth %s and colour %s differ in size" % (depth.shape, color.shape[:2]))
                    return np.ascontiguousarray(color), depth, self.infos[key], resp.message
            time.sleep(0.05)
        raise RuntimeError("no %s with its %s + %s within 10 s" % (FFS_DEPTH_TOPIC, FFS_LEFT_TOPIC, FFS_INFO_TOPIC))


def ask(prompt):
    try:
        return input(prompt).strip()
    except EOFError:
        return "q"


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("outdir")
    ap.add_argument("--mode", choices=sorted(MODES), default="double",
                    help="double (default): two overlapping items per zone; single: one item per zone")
    ap.add_argument("--setups", type=int, default=10, help="snapshots per item type (default 10)")
    ap.add_argument("--items", default="", help="comma-separated item type names; without it the "
                                                "script asks for a name before each block")
    args = ap.parse_args()
    per_zone, what = MODES[args.mode]
    root = os.path.join(args.outdir, args.mode)
    os.makedirs(root, exist_ok=True)

    import rosgraph
    import rospy
    if not rosgraph.is_master_online():
        sys.exit("no ROS master at %s - start the stack first (or use snap_label_zones.sh, "
                 "which starts it for you)" % rosgraph.get_master_uri())
    # disable_signals: Ctrl-C reaches input() as KeyboardInterrupt instead of rospy's handler
    rospy.init_node("capture_label_zones", anonymous=True, disable_signals=True)
    cam = Camera()

    queue = [n for n in (folder_name(i) for i in args.items.split(",")) if n]
    ask_names = not queue
    mixed = args.mode == "single"   # any items, no names, no limit: one run of setups
    existing = sorted(e for e in os.listdir(root) if os.path.isdir(os.path.join(root, e)))
    if mixed:
        queue, ask_names = ["mixed"], False
        if setups_done(root):
            print("Already in %s: %d snapshot(s)" % (root, len(setups_done(root))))
    elif existing:
        print("Already in %s: %s" % (root, ", ".join(
            "%s (%d)" % (e, len(setups_done(os.path.join(root, e)))) for e in existing)))
    print("\n%s mode: place %s for every snapshot." % (args.mode.upper(), what))
    print("Both arms must be out of the label camera's view. Each snapshot covers BOTH zones.")

    total = 0
    try:
        while True:
            if ask_names:
                item = folder_name(ask("\nItem type name (empty = finished): "))
                if not item or item == "q":
                    break
            elif queue:
                item = queue.pop(0)
            else:
                break
            item_dir = root if mixed else os.path.join(root, item)
            quit_all = False
            while True:
                done = setups_done(item_dir)
                if not mixed and len(done) >= args.setups:
                    print("[%s] %d of %d setups saved." % (item, len(done), args.setups))
                    break
                n = (done[-1] if done else 0) + 1
                if mixed:
                    key = ask("\n[single %d] place %s (any items), then press Enter "
                              "(u = undo the last snapshot, q = quit): " % (len(done) + 1, what)).lower()
                    key = "q" if key == "n" else key
                else:
                    key = ask("\n[%s  %s %d/%d] place %s, then press Enter "
                              "(u = undo the last snapshot, n = next item type, q = quit): "
                              % (item, args.mode, len(done) + 1, args.setups, what)).lower()
                if key == "q":
                    quit_all = True
                    break
                if key == "n":
                    break
                if key == "u":
                    if done:
                        shutil.rmtree(os.path.join(item_dir, "setup_%02d" % done[-1]))
                        print("  removed setup_%02d" % done[-1])
                    else:
                        print("  nothing to undo")
                    continue
                if key:
                    print("  not understood: %r" % key)
                    continue
                try:
                    color, depth, info, ffs_msg = cam.grab()
                except RuntimeError as exc:
                    print("  SNAPSHOT FAILED, nothing saved: %s" % exc)
                    continue
                path = os.path.join(item_dir, "setup_%02d" % n)
                valid = save_setup(path, color, depth, info.K, {
                    "item": item, "mode": args.mode, "items_per_zone": per_zone, "setup": n, "time": time.strftime("%Y-%m-%dT%H:%M:%S"),
                    "frame_id": info.header.frame_id, "stamp": info.header.stamp.to_sec(),
                    "ffs": ffs_msg})
                total += 1
                print("  saved %s  (depth valid: zone 1 %.0f%%, zone 2 %.0f%%)"
                      % (path, 100 * valid[1], 100 * valid[2]))
                for zone, frac in valid.items():
                    if frac < MIN_VALID_FRAC:
                        print("  WARNING: zone %d has little valid depth; look at preview.png and "
                              "press u to retake if it is bad" % zone)
            if quit_all:
                break
    except KeyboardInterrupt:
        print()
    print("\n%d snapshot(s) taken this session. Everything is under %s" % (total, root))


if __name__ == "__main__":
    main()
