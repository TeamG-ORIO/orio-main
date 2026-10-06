#!/usr/bin/env python3
"""Grab one time-synced RGB + registered-depth frame from the Xtion.

Writes color.png (the RGB snapshot), plus color.npy, depth.npy and K.npy for the
offline tools (live_affordance.py, compare_dexnet_methods.py).

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
print("captured %dx%d to %s: depth valid %.1f%%, median %.3f m, fx=%.2f"
      % (got["c"].shape[1], got["c"].shape[0], OUT,
         100 * v.mean(), np.median(d[v]) if v.any() else float("nan"), info.K[0]))
