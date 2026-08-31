#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class SaveOneFrame:
    def __init__(self):
        self.bridge = CvBridge()
        self.rgb_image = None
        self.depth_image = None

        rospy.Subscriber("/camera/rgb/image_raw", Image, self.rgb_callback)
        rospy.Subscriber("/camera/depth/image_raw", Image, self.depth_callback)

    def rgb_callback(self, msg):
        if self.rgb_image is None:
            self.rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def depth_callback(self, msg):
        if self.depth_image is None:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def save_images(self):
        rospy.loginfo("Waiting for images...")
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            if self.rgb_image is not None and self.depth_image is not None:
                break
            rate.sleep()

        cwd = os.getcwd()

        # Save RGB
        rgb_path = os.path.join(cwd, "rgb_image.png")
        cv2.imwrite(rgb_path, self.rgb_image)

        # Save depth (normalized for visualization)
        depth_norm = cv2.normalize(self.depth_image, None, 0, 255, cv2.NORM_MINMAX)
        depth_norm = depth_norm.astype('uint8')
        depth_path = os.path.join(cwd, "depth_image.png")
        cv2.imwrite(depth_path, depth_norm)

        # Save raw depth
        raw_path = os.path.join(cwd, "depth_raw.npy")
        import numpy as np
        np.save(raw_path, self.depth_image)

        rospy.loginfo(f"Saved images to {cwd}")
        rospy.signal_shutdown("Done")


if __name__ == "__main__":
    rospy.init_node("save_one_frame")
    node = SaveOneFrame()
    node.save_images()
