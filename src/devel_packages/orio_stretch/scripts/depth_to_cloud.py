#!/usr/bin/env python3
"""
depth_to_cloud.py - a light obstacle cloud for the costmaps from the D435i depth image (no lidar needed).

Takes /camera/depth/image_rect_raw (+ camera_info), keeps only projector-ON frames when the projector is
alternating (passive-stereo frames are full of holes and noise), uses every ~stride-th pixel and a range
window, and publishes a PointCloud2 in the depth camera's optical frame at ~rate Hz. Cheap enough to run
next to ORB-SLAM3 on the NUC (about 19k points per cloud at stride 4).

Floor fit (~floor_fit, default true): the points are moved into ~base_frame (base_link), a floor plane
z = a x + b y + c is fitted to the low points of THIS frame, and every point's z becomes its height above that
plane. The head camera's TF is ~2.5 deg off in pitch, so without it the floor appears to rise ~4-5 cm per metre
and a 10 cm obstacle threshold marks distant floor as obstacle. The cloud is then published in ~base_frame.

Params: ~stride 4   ~rate 5.0   ~min_range 0.3   ~max_range 2.6   ~laser on|any   ~floor_fit true   ~base_frame base_link
Topics: in /camera/depth/image_rect_raw, /camera/depth/camera_info, /camera/depth/metadata   out ~points (default /camera/points_nav)
"""
import bisect
import json
import struct

import numpy as np
import rospy
import tf2_ros
import tf.transformations as tft
from realsense2_camera.msg import Metadata
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField


class DepthToCloud(object):
    def __init__(self):
        self.stride = int(rospy.get_param('~stride', 4))
        self.period = 1.0 / float(rospy.get_param('~rate', 5.0))
        self.zmin, self.zmax = float(rospy.get_param('~min_range', 0.3)), float(rospy.get_param('~max_range', 2.6))
        self.laser = rospy.get_param('~laser', 'on')
        if isinstance(self.laser, bool):
            self.laser = 'on' if self.laser else 'any'
        self.info = None
        self.last = rospy.Time(0)
        self.meta_t, self.meta_mode = [], []
        self.pub = rospy.Publisher('~points', PointCloud2, queue_size=2)
        self.floor_fit = bool(rospy.get_param('~floor_fit', True))
        self.base_frame = rospy.get_param('~base_frame', 'base_link')
        self.buf = tf2_ros.Buffer(rospy.Duration(10.0)); tf2_ros.TransformListener(self.buf)
        self.n_fit = self.n_nofit = 0
        rospy.Subscriber('/camera/depth/camera_info', CameraInfo, self.on_info, queue_size=1)
        if self.laser == 'on':
            rospy.Subscriber('/camera/depth/metadata', Metadata, self.on_meta, queue_size=50)
        rospy.Subscriber('/camera/depth/image_rect_raw', Image, self.on_depth, queue_size=1, buff_size=1 << 24)

    def on_info(self, msg):
        self.info = msg

    def on_meta(self, msg):
        try:
            mode = json.loads(msg.json_data).get('frame_laser_power_mode')
        except ValueError:
            return
        if mode is None:
            return
        self.meta_t.append(msg.header.stamp.to_nsec())
        self.meta_mode.append(mode)
        if len(self.meta_t) > 300:
            del self.meta_t[:150], self.meta_mode[:150]

    def laser_on(self, ns):
        if self.laser != 'on' or not self.meta_t:
            return True
        i = bisect.bisect_left(self.meta_t, ns)
        for j in (i - 1, i):
            if 0 <= j < len(self.meta_t) and abs(self.meta_t[j] - ns) < 2000000:
                return self.meta_mode[j] == 1
        return len(set(self.meta_mode)) == 1        # no metadata for this frame: pass only if the projector is constant

    def correct(self, pb):
        """z -> height above this frame's floor plane (base_link, z up); unchanged if no credible floor"""
        low = pb[np.abs(pb[:, 2]) < 0.25]
        if len(low) >= 150:
            A = np.column_stack([low[:, 0], low[:, 1], np.ones(len(low))])
            sel = np.ones(len(low), bool); coef = None
            for _ in range(3):
                if sel.sum() < 80:
                    coef = None; break
                coef = np.linalg.lstsq(A[sel], low[sel, 2], rcond=None)[0]
                sel = np.abs(low[:, 2] - A.dot(coef)) < 0.03
            if coef is not None and np.hypot(coef[0], coef[1]) < 0.15 and sel.mean() > 0.3:
                out = pb.copy(); out[:, 2] = pb[:, 2] - (coef[0] * pb[:, 0] + coef[1] * pb[:, 1] + coef[2])
                self.n_fit += 1
                return out
        self.n_nofit += 1
        if self.n_nofit % 50 == 1:
            rospy.logwarn('depth_to_cloud: no floor plane in view (%d frames so far, %d fitted): heights from base_link z', self.n_nofit, self.n_fit)
        return pb

    def on_depth(self, msg):
        if self.info is None or (msg.header.stamp - self.last).to_sec() < self.period:
            return
        if not self.laser_on(msg.header.stamp.to_nsec()):
            return
        self.last = msg.header.stamp
        s = self.stride
        z = np.frombuffer(msg.data, np.uint16).reshape(msg.height, msg.width)[::s, ::s].astype(np.float32) * 0.001
        fx, fy, cx, cy = self.info.K[0], self.info.K[4], self.info.K[2], self.info.K[5]
        v, u = np.mgrid[0:msg.height:s, 0:msg.width:s]
        ok = (z > self.zmin) & (z < self.zmax)
        pts = np.stack([(u[ok] - cx) / fx * z[ok], (v[ok] - cy) / fy * z[ok], z[ok]], axis=1).astype(np.float32)
        header = msg.header
        if self.floor_fit:
            try:
                tr = self.buf.lookup_transform(self.base_frame, msg.header.frame_id, msg.header.stamp, rospy.Duration(0.2))
            except (tf2_ros.LookupException, tf2_ros.ExtrapolationException, tf2_ros.ConnectivityException):
                return                                  # no TF yet: skip the frame rather than publish uncorrected heights
            q, t = tr.transform.rotation, tr.transform.translation
            R = tft.quaternion_matrix((q.x, q.y, q.z, q.w))[:3, :3]
            pb = pts.dot(R.T) + np.array([t.x, t.y, t.z], np.float32)
            pts = self.correct(pb).astype(np.float32)
            header = type(msg.header)(seq=msg.header.seq, stamp=msg.header.stamp, frame_id=self.base_frame)
        cloud = PointCloud2()
        cloud.header = header
        cloud.height, cloud.width = 1, len(pts)
        cloud.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1) for i, n in enumerate('xyz')]
        cloud.is_bigendian, cloud.point_step, cloud.row_step, cloud.is_dense = False, 12, 12 * len(pts), True
        cloud.data = pts.tobytes()
        self.pub.publish(cloud)


if __name__ == '__main__':
    rospy.init_node('depth_to_cloud')
    DepthToCloud()
    rospy.spin()
