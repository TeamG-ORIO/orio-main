#!/usr/bin/env python3
"""
orbslam_video.py - record what ORB-SLAM3 "sees" into a video: the camera image with the map points it is
tracking in each frame drawn on it (green marks), plus ORB-SLAM3's status line at the bottom.
Started by `orbslam_run.sh --video`; writes ~out (WebM / VP8) when the run ends.

It records /orb_slam3/tracking_image. The head camera is mounted rolled by 90 degrees, so the image is
rotated upright (~rotate, default true). The tracking state of each frame is written in the top corner:
green OK, red LOST / not initialised.
"""
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Header

STATES = {1: 'NOT INITIALISED', 2: 'OK', 3: 'RECENTLY LOST', 4: 'LOST', 5: 'OK'}


class Video(object):
    def __init__(self):
        self.out = rospy.get_param('~out')
        self.fps = float(rospy.get_param('~fps', 15.0))
        self.rotate = rospy.get_param('~rotate', True)
        self.writer = None
        self.state = {}
        self.t0 = None
        self.n = 0
        rospy.Subscriber('/orb_slam3/tracking_state', Header, self.on_state, queue_size=500)
        rospy.Subscriber('/orb_slam3/tracking_image', Image, self.on_image, queue_size=200, buff_size=1 << 26)
        rospy.on_shutdown(self.close)

    def on_state(self, msg):
        try:
            self.state[msg.stamp.to_nsec()] = int(msg.frame_id)
        except ValueError:
            pass
        if len(self.state) > 2000:
            for k in sorted(self.state)[:1000]:
                del self.state[k]

    def on_image(self, msg):
        img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3).copy()      # wrapper publishes bgr8
        if self.rotate:
            # ORB-SLAM3 appends its status text below the camera image: rotate the image part only
            h_img = msg.height - 20 if msg.height in (500, 380, 260) else msg.height
            body = cv2.rotate(img[:h_img], cv2.ROTATE_90_CLOCKWISE)
            img = body
        ns = msg.header.stamp.to_nsec()
        if self.t0 is None:
            self.t0 = ns
        st = self.state.get(ns)
        label = '%5.1f s   %s' % ((ns - self.t0) * 1e-9, STATES.get(st, '?'))
        colour = (0, 220, 0) if st in (2, 5) else (0, 0, 255)
        cv2.putText(img, label, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 4, cv2.LINE_AA)
        cv2.putText(img, label, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, colour, 2, cv2.LINE_AA)
        if self.writer is None:
            self.writer = cv2.VideoWriter(self.out, cv2.VideoWriter_fourcc(*'VP80'), round(self.fps), (img.shape[1], img.shape[0]))
            self.size = (img.shape[1], img.shape[0])
        if (img.shape[1], img.shape[0]) == self.size:
            self.writer.write(img)
            self.n += 1

    def close(self):
        if self.writer is not None:
            self.writer.release()
        rospy.loginfo('orbslam_video: %d frames -> %s', self.n, self.out)


if __name__ == '__main__':
    rospy.init_node('orbslam_video')
    Video()
    rospy.spin()
