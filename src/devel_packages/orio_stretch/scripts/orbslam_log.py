#!/usr/bin/env python3
"""
orbslam_log.py - log what ORB-SLAM3 did with every frame it was offered. Writes ~out (frames.csv) at shutdown:

    stamp, state, n_points, x, y, z, qx, qy, qz, qw

one row per frame published on /orb_feed/image. state is the ORB-SLAM3 tracking state reported by the
patched wrapper on /orb_slam3/tracking_state (empty = the node never processed that frame, i.e. it was
dropped): 1 not initialised, 2 OK, 3 recently lost, 4 lost. n_points = map points tracked in that frame.
x..qw = the LIVE camera pose (not the final optimised trajectory; only meaningful while state is 2).

Also keeps the last /orb_slam3/all_points message (every point of the active map, ORB-SLAM3 world frame)
and writes it next to frames.csv as map_points.npy (N x 3 float32) for orbslam_map.py.
"""
import os
import struct

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header


class Log(object):
    def __init__(self):
        self.out = rospy.get_param('~out')
        self.offered = []
        self.state = {}
        self.points = {}
        self.pose = {}
        # AnyMsg: only the header stamp of the image is needed, do not deserialise 30 images/s twice
        rospy.Subscriber('/orb_feed/image', rospy.AnyMsg, self.on_image, queue_size=500, buff_size=1 << 26)
        rospy.Subscriber('/orb_slam3/tracking_state', Header, self.on_state, queue_size=500)
        rospy.Subscriber('/orb_slam3/camera_pose', PoseStamped, self.on_pose, queue_size=500)
        rospy.Subscriber('/orb_slam3/tracked_points', PointCloud2, self.on_points, queue_size=50, buff_size=1 << 24)
        self.map_msg = None
        rospy.Subscriber('/orb_slam3/all_points', PointCloud2, self.on_map, queue_size=2, buff_size=1 << 26)
        rospy.on_shutdown(self.write)

    def on_image(self, msg):
        # std_msgs/Header on the wire: uint32 seq, uint32 secs, uint32 nsecs, ...
        _, secs, nsecs = struct.unpack_from('<III', msg._buff, 0)
        self.offered.append(secs * 1000000000 + nsecs)

    def on_state(self, msg):
        try:
            self.state[msg.stamp.to_nsec()] = int(msg.frame_id)
        except ValueError:
            pass

    def on_pose(self, msg):
        p, q = msg.pose.position, msg.pose.orientation
        self.pose[msg.header.stamp.to_nsec()] = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)

    @staticmethod
    def xyz(msg):
        """N x 3 points of the wrapper's clouds; slots without a map point are left at exactly (0, 0, 0)"""
        pts = np.frombuffer(msg.data, np.float32).reshape(-1, 3)
        return pts[np.any(pts != 0, axis=1)]

    def on_points(self, msg):
        # the cloud has one slot per image keypoint; only the non-zero ones are tracked map points
        self.points[msg.header.stamp.to_nsec()] = len(self.xyz(msg))

    def on_map(self, msg):
        self.map_msg = msg

    def write(self):
        if self.map_msg is not None:
            np.save(os.path.join(os.path.dirname(self.out), 'map_points.npy'), self.xyz(self.map_msg))
        with open(self.out, 'w') as f:
            f.write('stamp,state,n_points,x,y,z,qx,qy,qz,qw\n')
            for ns in sorted(set(self.offered)):
                st = self.state.get(ns)
                row = ['%.9f' % (ns * 1e-9), '' if st is None else str(st), str(self.points.get(ns, ''))]
                row += ['%.6f' % v for v in self.pose[ns]] if ns in self.pose else [''] * 7
                f.write(','.join(row) + '\n')


if __name__ == '__main__':
    rospy.init_node('orbslam_log')
    Log()
    rospy.spin()
