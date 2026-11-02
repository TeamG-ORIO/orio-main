#!/usr/bin/env python3
"""Grab the ZED-M's current frame with its Fast-FoundationStereo depth.

Asks the FFS node (/ffs/compute) for depth on the newest stereo pair and writes into
<outdir>: color.png (the left frame), right.png (the right frame of the same pair),
preview.png (colour and depth side by side), color.npy, depth.npy (metres, NaN where
invalid), K.npy, and meta.json (stamp, the FFS summary and the ZED's exposure, gain and
white-balance settings at capture time).

Needs rospy and a running ZED + FFS node, so it runs inside the perception container.
From the repo root:

    bash research/labelling_arm_test/snap_zed.sh [outdir]

or by hand inside a container with rospy:

    python3 ffs/capture_ffs.py <outdir>
"""
import json
import os
import sys
import threading
import time
import cv2
import numpy as np
import rospy
import rosgraph
from sensor_msgs.msg import Image, CameraInfo
from std_srvs.srv import Trigger

ZED_NODE = "/zedm/zed_node"
RIGHT_TOPIC = ZED_NODE + "/right/image_rect_color"
CAMERA_PARAMS = ("auto_exposure_gain", "exposure", "gain", "auto_whitebalance",
                 "whitebalance_temperature", "brightness", "contrast", "gamma",
                 "saturation", "sharpness", "general/grab_resolution")

if len(sys.argv) < 2:
    sys.exit(__doc__)
OUT = sys.argv[1]
os.makedirs(OUT, exist_ok=True)
# init_node retries against a missing master forever, silently, so check first.
if not rosgraph.is_master_online():
    sys.exit("no ROS master at %s - start the stack first "
             "(research/labelling_arm_test/run_labelling_test.sh)" % rosgraph.get_master_uri())
rospy.init_node("capture_ffs", anonymous=True)


def to_rgb(msg):
    """ZED bgra8 or the FFS node's rgb8 -> RGB uint8 HxWx3."""
    a = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, -1)
    a = a[:, :, 2::-1] if msg.encoding.startswith("bgr") else a[:, :, :3]
    return np.ascontiguousarray(a)


# Everything is kept by stamp: the FFS node gives /ffs/left, /ffs/camera_info and /ffs/depth
# of one computation the left frame's stamp, and the ZED stamps a left/right pair alike.
lock = threading.Lock()
rights, lefts, infos, depths = {}, {}, {}, {}


def keep(store, limit):
    def cb(msg):
        with lock:
            store[msg.header.stamp.to_nsec()] = msg
            while len(store) > limit:
                del store[min(store)]
    return cb


subs = [rospy.Subscriber("/ffs/left", Image, keep(lefts, 3), queue_size=1, buff_size=2**25),
        rospy.Subscriber("/ffs/camera_info", CameraInfo, keep(infos, 3), queue_size=1),
        rospy.Subscriber("/ffs/depth", Image, keep(depths, 3), queue_size=1, buff_size=2**25)]
rospy.Subscriber(RIGHT_TOPIC, Image, keep(rights, 15), queue_size=2, buff_size=2**25)

try:
    rospy.wait_for_service("/ffs/compute", timeout=5.0)
except rospy.ROSException:
    sys.exit("no /ffs/compute - is the FFS depth node up? "
             "research/labelling_arm_test/run_labelling_test.sh starts it")
# The FFS topics are not latched: connect before asking, or the result goes past us.
t0 = time.time()
while (any(s.get_num_connections() == 0 for s in subs) or not rights) \
        and time.time() - t0 < 5.0:
    rospy.sleep(0.05)
if not rights:
    sys.exit("no frames on %s within 5 s - is the ZED publishing?" % RIGHT_TOPIC)

resp = rospy.ServiceProxy("/ffs/compute", Trigger)()
if not resp.success:
    sys.exit("/ffs/compute failed: %s" % resp.message)
key = None
t0 = time.time()
while key is None and time.time() - t0 < 10.0:
    with lock:
        ready = [k for k in depths if k in lefts and k in infos]
    key = max(ready) if ready else None
    rospy.sleep(0.05)
if key is None:
    sys.exit("no /ffs/depth with its /ffs/left and /ffs/camera_info within 10 s")

with lock:
    left_msg, info, depth_msg = lefts[key], infos[key], depths[key]
    near = min(rights, key=lambda k: abs(k - key))
    right_msg = rights[near] if abs(near - key) < 10e6 else None
color = to_rgb(left_msg)
d = np.frombuffer(depth_msg.data, np.float32).reshape(depth_msg.height, depth_msg.width)

# color is RGB; OpenCV writes BGR, so swap back for the PNGs.
cv2.imwrite(os.path.join(OUT, "color.png"), color[:, :, ::-1])
np.save(os.path.join(OUT, "color.npy"), color)
np.save(os.path.join(OUT, "depth.npy"), d)
np.save(os.path.join(OUT, "K.npy"), np.array(info.K).reshape(3, 3))
if right_msg is not None:
    cv2.imwrite(os.path.join(OUT, "right.png"), to_rgb(right_msg)[:, :, ::-1])
camera = {p: rospy.get_param(ZED_NODE + "/" + p, None) for p in CAMERA_PARAMS}
with open(os.path.join(OUT, "meta.json"), "w") as f:
    json.dump({"stamp": key / 1e9, "ffs": resp.message, "right_saved": right_msg is not None,
               "camera": camera}, f, indent=2)
v = np.isfinite(d) & (d > 0)

# preview.png: RGB | viridis depth | colour bar. The colour scale spans the closest 30 cm of
# valid depth (the items and the table top); anything farther saturates, invalid is black.
if v.any():
    lo = float(np.percentile(d[v], 0.5))
    near_d = d[v & (d < lo + 0.3)]
    hi = max(float(np.percentile(near_d, 99.5)) if near_d.size else lo + 0.3, lo + 1e-3)
else:
    lo, hi = 0.0, 1.0
t = np.clip((np.where(v, d, lo) - lo) / (hi - lo), 0.0, 1.0)
depth_bgr = cv2.applyColorMap((t * 255).astype(np.uint8), cv2.COLORMAP_VIRIDIS)
depth_bgr[~v] = 0
# Colour bar with depth ticks in metres, far (top) to near (bottom).
h = d.shape[0]
bar = np.full((h, 110, 3), 255, np.uint8)
top, bot = 30, h - 20
ramp = np.linspace(1.0, 0.0, bot - top)[:, None].repeat(20, axis=1)
bar[top:bot, 10:30] = cv2.applyColorMap((ramp * 255).astype(np.uint8), cv2.COLORMAP_VIRIDIS)
font = cv2.FONT_HERSHEY_SIMPLEX
cv2.putText(bar, "depth (m)", (6, 18), font, 0.45, (0, 0, 0), 1, cv2.LINE_AA)
for i, val in enumerate(np.linspace(hi, lo, 6)):
    y = int(round(top + i * (bot - top - 1) / 5))
    cv2.line(bar, (30, y), (36, y), (0, 0, 0), 1)
    label = ("%.3f+" if i == 0 else "%.3f") % val  # farther than the top saturates
    cv2.putText(bar, label, (40, y + 5), font, 0.45, (0, 0, 0), 1, cv2.LINE_AA)
cv2.imwrite(os.path.join(OUT, "preview.png"),
            np.concatenate([color[:, :, ::-1], depth_bgr, bar], axis=1))
print("captured %dx%d to %s: depth valid %.1f%%, median %.3f m, fx=%.2f; exposure %s, gain %s%s"
      % (color.shape[1], color.shape[0], OUT, 100 * v.mean(),
         np.median(d[v]) if v.any() else float("nan"), info.K[0], camera["exposure"],
         camera["gain"], "" if right_msg is not None else "; right frame not found"))
