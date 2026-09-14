#!/usr/bin/env python3
"""Grab one time-synced RGB + registered-depth frame from the Xtion.

    venv/bin/python test/capture_xtion.py <outdir>
"""
import sys
import numpy as np
import rospy
import message_filters
from sensor_msgs.msg import Image, CameraInfo

OUT = sys.argv[1]
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

info = rospy.wait_for_message("/camera/rgb/camera_info", CameraInfo, timeout=30)
t0 = rospy.Time.now()
while not got and (rospy.Time.now() - t0).to_sec() < 30:
    rospy.sleep(0.1)
if not got:
    sys.exit("failed to sync RGB and depth")

np.save(OUT + "/color.npy", got["c"])
np.save(OUT + "/depth.npy", got["d"])
np.save(OUT + "/K.npy", np.array(info.K).reshape(3, 3))
d = got["d"]
v = np.isfinite(d) & (d > 0)
print("captured: depth valid %.1f%%, median %.3f m, fx=%.2f"
      % (100 * v.mean(), np.median(d[v]), info.K[0]))
