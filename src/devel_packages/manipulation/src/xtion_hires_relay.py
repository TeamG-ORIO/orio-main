#!/usr/bin/env python3
"""Rebuilds the Xtion's 640x480 colour stream from its 1280x1024 one.

In high-res mode (xtion_only.launch hires:=true) the driver publishes 1280x1024 colour
on <camera>/rgb_hires; depth stays 640x480 and registered to the usual 640x480 colour
grid. This republishes <camera>/rgb/image_raw and camera_info on that grid, as the
driver does in 640x480 mode, so everything downstream is unchanged. Each frame keeps
the high-res frame's header, so a node can find the high-res frame behind it by stamp.

The 640x480 grid is a 2x2-binned window of the 1280x1024 frame starting at (x0, y0)
(measured: 5, 35, params ~x0/~y0, also read by the suction planner from
/camera/hires_align). The few columns past the frame's right edge repeat the edge.
"""
import numpy as np
import rospy
from sensor_msgs.msg import CameraInfo, Image

W, H = 640, 480


def main():
    rospy.init_node('xtion_hires_relay')
    x0 = int(rospy.get_param('/camera/hires_align/x0', 5))
    y0 = int(rospy.get_param('/camera/hires_align/y0', 35))
    # What the driver publishes in 640x480 mode (its default intrinsics; the planner
    # uses its own calibration).
    f = float(rospy.get_param('~fx', 570.3422241210938))
    cx, cy = float(rospy.get_param('~cx', 319.5)), float(rospy.get_param('~cy', 239.5))
    img_pub = rospy.Publisher('rgb/image_raw', Image, queue_size=2)
    info_pub = rospy.Publisher('rgb/camera_info', CameraInfo, queue_size=2)

    def on_frame(msg):
        if msg.encoding not in ('rgb8', 'bgr8'):
            rospy.logerr_throttle(10, 'xtion_hires_relay: unsupported encoding %s' % msg.encoding)
            return
        if msg.width < 2 * W or msg.height < y0 + 2 * H:
            rospy.logerr_throttle(10, 'xtion_hires_relay: expected a 1280x1024 frame, got %dx%d '
                                  '(is the Xtion in color_mode 1?)' % (msg.width, msg.height))
            return
        img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step // 3, 3)[:, :msg.width]
        win = img[y0:y0 + 2 * H, x0:x0 + 2 * W]
        if win.shape[1] < 2 * W:
            win = np.pad(win, ((0, 0), (0, 2 * W - win.shape[1]), (0, 0)), mode='edge')
        small = win.reshape(H, 2, W, 2, 3).sum(axis=(1, 3), dtype=np.uint16)
        small = ((small + 2) // 4).astype(np.uint8)

        out = Image(header=msg.header, height=H, width=W, encoding=msg.encoding,
                    is_bigendian=0, step=3 * W, data=small.tobytes())
        info = CameraInfo(header=msg.header, height=H, width=W, distortion_model='plumb_bob',
                          D=[0.0] * 5, K=[f, 0.0, cx, 0.0, f, cy, 0.0, 0.0, 1.0],
                          R=[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
                          P=[f, 0.0, cx, 0.0, 0.0, f, cy, 0.0, 0.0, 0.0, 1.0, 0.0])
        img_pub.publish(out)
        info_pub.publish(info)

    rospy.Subscriber('rgb_hires/image_raw', Image, on_frame, queue_size=1, buff_size=2 ** 24)
    rospy.loginfo('xtion_hires_relay: rgb_hires (1280x1024) -> rgb (640x480), window at (%d, %d)',
                  x0, y0)
    rospy.spin()


if __name__ == '__main__':
    main()
