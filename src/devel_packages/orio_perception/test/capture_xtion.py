#!/usr/bin/env python3
"""Grab one time-synced RGB + registered-depth frame from the Xtion.

Writes color.png (the RGB snapshot), preview.png (RGB and colourised depth side by
side), plus color.npy, depth.npy and K.npy for the offline tools (live_affordance.py, compare_dexnet_methods.py).

Needs rospy, so it runs inside the perception container rather than the host venv.
From the repo root (starts the camera stack itself if nothing is running):

    bash snap_xtion.sh [outdir]

or by hand inside the container:

    python3 test/capture_xtion.py <outdir>
"""
import os
import sys
import cv2
import numpy as np
import rospy
import rosgraph
import message_filters
from sensor_msgs.msg import Image, CameraInfo

if len(sys.argv) < 2:
    sys.exit(__doc__)
OUT = sys.argv[1]
os.makedirs(OUT, exist_ok=True)
# init_node retries against a missing master forever, silently, so check first.
if not rosgraph.is_master_online():
    sys.exit("no ROS master at %s - start the camera stack first (or use snap_xtion.sh, "
             "which starts it for you)" % rosgraph.get_master_uri())
rospy.init_node("capture_xtion", anonymous=True)
got = {}


def cb(rgb, dep):
    if got:
        return
    c = np.frombuffer(rgb.data, np.uint8).reshape(rgb.height, rgb.width, -1)
    if "bgr" in rgb.encoding:
        c = c[:, :, ::-1]
    got["c"] = np.ascontiguousarray(c)
    got["d"] = np.ascontiguousarray(
        np.frombuffer(dep.data, np.float32).reshape(dep.height, dep.width))


rs = message_filters.Subscriber("/camera/rgb/image_raw", Image)
ds = message_filters.Subscriber("/camera/depth_registered/image_raw", Image)
message_filters.ApproximateTimeSynchronizer([rs, ds], 10, 0.05).registerCallback(cb)

try:
    info = rospy.wait_for_message("/camera/rgb/camera_info", CameraInfo, timeout=30)
except rospy.ROSException:
    sys.exit("no /camera/rgb/camera_info within 30 s - is the Xtion running?")
t0 = rospy.Time.now()
while not got and (rospy.Time.now() - t0).to_sec() < 30:
    rospy.sleep(0.1)
if not got:
    sys.exit("failed to sync RGB and depth")

# got["c"] is RGB; OpenCV writes BGR, so swap back for the PNG.
cv2.imwrite(os.path.join(OUT, "color.png"), got["c"][:, :, ::-1])
np.save(os.path.join(OUT, "color.npy"), got["c"])
np.save(os.path.join(OUT, "depth.npy"), got["d"])
np.save(os.path.join(OUT, "K.npy"), np.array(info.K).reshape(3, 3))
d = got["d"]
v = np.isfinite(d) & (d > 0)

# preview.png: RGB | viridis depth | colour bar. The colour scale spans the closest 30 cm of valid
# depth (the pile and the table top); anything farther saturates, invalid pixels are black.
if v.any():
    lo = float(np.percentile(d[v], 0.5))
    near = d[v & (d < lo + 0.3)]
    hi = max(float(np.percentile(near, 99.5)) if near.size else lo + 0.3, lo + 1e-3)
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
            np.concatenate([got["c"][:, :, 2::-1], depth_bgr, bar], axis=1))
print("captured %dx%d to %s: depth valid %.1f%%, median %.3f m, fx=%.2f"
      % (got["c"].shape[1], got["c"].shape[0], OUT,
         100 * v.mean(), np.median(d[v]) if v.any() else float("nan"), info.K[0]))
