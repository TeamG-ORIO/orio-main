#!/usr/bin/env python3

import rospy
import sys
import argparse
from datetime import datetime
from sensor_msgs.msg import Image
import cv2
import os
import numpy as np

class SaveOneFrame:
    def __init__(self, file_prefix, output_dir):
        self.file_prefix = file_prefix
        self.output_dir = output_dir
        self.rgb_image = None
        self.depth_image = None

        rospy.Subscriber("/zedm/zed_node/rgb/image_rect_color", Image, self.rgb_callback)
        rospy.Subscriber("/zedm/zed_node/depth/depth_registered", Image, self.depth_callback)

    def rgb_callback(self, msg):
        if self.rgb_image is None:
            # Decode as 4-channel (BGRA), then slice [:, :, :3] to drop the Alpha channel and keep BGR
            self.rgb_image = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 4)[:, :, :3]
            
    def depth_callback(self, msg):
        if self.depth_image is None:
            # Decode the raw ROS 32FC1 depth bytes directly into a float32 array
            self.depth_image = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)

    def save_images(self):
        rospy.loginfo(f"Waiting for ZED Mini images to save as '{self.file_prefix}'...")
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            if self.rgb_image is not None and self.depth_image is not None:
                break
            rate.sleep()

        # Create the subfolder path and make sure the directory exists
        save_dir = os.path.join(os.getcwd(), self.output_dir)
        os.makedirs(save_dir, exist_ok=True)

        # Save RGB
        rgb_path = os.path.join(save_dir, f"{self.file_prefix}_rgb.png")
        cv2.imwrite(rgb_path, self.rgb_image)

        # Clean the depth array before visualizing
        valid_depth = np.nan_to_num(self.depth_image, nan=0.0, posinf=0.0, neginf=0.0)

        # Save depth (normalized for visualization)
        depth_norm = cv2.normalize(valid_depth, None, 0, 255, cv2.NORM_MINMAX)
        depth_norm = depth_norm.astype('uint8')
        depth_path = os.path.join(save_dir, f"{self.file_prefix}_depth.png")
        cv2.imwrite(depth_path, depth_norm)

        # Save raw depth
        raw_path = os.path.join(save_dir, f"{self.file_prefix}_depth_raw.npy")
        np.save(raw_path, self.depth_image)

        rospy.loginfo(f"Successfully saved {self.file_prefix} images to {save_dir}")
        rospy.signal_shutdown("Done")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Capture and save a single frame from the ZED Mini.")
    parser.add_argument('--name', '-n', type=str, default=None, 
                        help='Base name for the saved files (e.g., item_1).')
    parser.add_argument('--dir', '-d', type=str, default='dataset', 
                        help='Subfolder to save the images in. Defaults to "dataset".')
    
    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])

    if args.name:
        file_prefix = args.name
    else:
        file_prefix = datetime.now().strftime("frame_%Y%m%d_%H%M%S")

    rospy.init_node("save_one_frame", anonymous=True)
    node = SaveOneFrame(file_prefix, args.dir)
    node.save_images()