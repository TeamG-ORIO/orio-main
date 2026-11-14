#!/usr/bin/env python3
"""
orbslam_tf_bridge.py - turn ORB-SLAM3's camera pose into the map -> odom transform the navigation stack expects.

ORB-SLAM3 (orb_slam3_ros) publishes the camera pose in its own world frame. This node
  1. rotates it into the upright map frame the 2D map was drawn in (frame_upright.json from marks_report.py),
  2. moves it from the camera to base_link with the robot's TF,
  3. divides out the wheel odometry (odom -> base_link) and publishes map -> odom,
exactly the way AMCL does. Between visual fixes, and whenever ORB-SLAM3 is lost, the last map -> odom
is held and wheel odometry carries the robot: the pose never jumps to garbage.

Only poses from the SAVED map count: with a loaded atlas ORB-SLAM3 starts a fresh map and merges it into
the saved one once it recognises the place; poses before that merge are in an unrelated frame. The
patched wrapper reports the number of maps in the atlas in tracking_state.seq (seq // 1000): while the new
session is a separate map there are 2, after the merge 1. The bridge waits for ~expected_maps (default 1).
If ORB-SLAM3 later gets lost and starts yet another map, the count rises again and the bridge holds the
last map -> odom until that map has merged too.

Known start (~initial_pose "x y yaw", map frame, e.g. the taped cross the robot is parked on): map -> odom is
published from that pose and wheel odometry straight away, so navigation can start before ORB-SLAM3 has
recognised the map; the first visual fix then replaces it. /orbslam/seeded says the seed is in use,
/orbslam/localized stays false until a real visual fix.

Params:  ~frame_file  (frame_upright.json)   ~map_frame map   ~odom_frame odom   ~base_frame base_link
         ~expected_maps 1   ~rate 20   ~future 0.1 (s: stamp the TF slightly ahead, as AMCL does)   ~initial_pose ""
Topics:  in  /orb_slam3/camera_pose, /orb_slam3/tracking_state      out  /orbslam/pose (base_link in map), /orbslam/localized, /orbslam/seeded
"""
import json
import threading

import numpy as np
import rospy
import tf2_ros
import tf.transformations as tft
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import Bool, Header

OK_STATES = (2, 5)


def T_from(pos, quat):
    T = tft.quaternion_matrix(quat)
    T[:3, 3] = pos
    return T


def T_from_tf(tr):
    t, q = tr.transform.translation, tr.transform.rotation
    return T_from((t.x, t.y, t.z), (q.x, q.y, q.z, q.w))


class Bridge(object):
    def __init__(self):
        fr = json.load(open(rospy.get_param('~frame_file')))
        self.T_up = np.eye(4)
        self.T_up[:3, :3] = np.array(fr['R_map_orb'])
        self.T_up[:3, 3] = np.array(fr['t_map_orb'])
        self.cam_frame = rospy.get_param('~camera_frame', fr.get('camera_frame', 'camera_infra1_optical_frame'))
        self.map_frame = rospy.get_param('~map_frame', 'map')
        self.odom_frame = rospy.get_param('~odom_frame', 'odom')
        self.base_frame = rospy.get_param('~base_frame', 'base_link')
        self.expected_maps = int(rospy.get_param('~expected_maps', 1))
        self.future = rospy.Duration(float(rospy.get_param('~future', 0.1)))
        self.buf = tf2_ros.Buffer(rospy.Duration(30.0))
        tf2_ros.TransformListener(self.buf)
        self.br = tf2_ros.TransformBroadcaster()
        self.lock = threading.Lock()
        self.T_map_odom = None
        self.state = {}             # stamp ns -> (state, number of maps)
        self.localized = False
        self.n_fix = 0
        self.last_fix = None
        self.pose_pub = rospy.Publisher('/orbslam/pose', PoseStamped, queue_size=10)
        self.loc_pub = rospy.Publisher('/orbslam/localized', Bool, queue_size=1, latch=True)
        self.loc_pub.publish(Bool(False))
        self.seed_pub = rospy.Publisher('/orbslam/seeded', Bool, queue_size=1, latch=True)
        self.seed_pub.publish(Bool(False))
        seed = str(rospy.get_param('~initial_pose', '')).split()
        if len(seed) == 3:
            self.seed = [float(v) for v in seed]
            rospy.Timer(rospy.Duration(0.5), self.try_seed)
        rospy.Subscriber('/orb_slam3/tracking_state', Header, self.on_state, queue_size=100)
        rospy.Subscriber('/orb_slam3/camera_pose', PoseStamped, self.on_pose, queue_size=100)
        rospy.Timer(rospy.Duration(1.0 / float(rospy.get_param('~rate', 20.0))), self.publish)
        rospy.Timer(rospy.Duration(5.0), self.report)

    def try_seed(self, ev):
        """until the first visual fix: map -> odom from the given start pose and the current wheel odometry"""
        with self.lock:
            if self.n_fix or self.T_map_odom is not None:
                return
        try:
            T_odom_base = T_from_tf(self.buf.lookup_transform(self.odom_frame, self.base_frame, rospy.Time(0)))
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException, tf2_ros.ConnectivityException):
            return
        x, y, yaw = self.seed
        T_map_base = T_from((x, y, 0.0), tft.quaternion_from_euler(0, 0, yaw))
        with self.lock:
            if self.n_fix:
                return
            self.T_map_odom = T_map_base.dot(np.linalg.inv(T_odom_base))
        self.seed_pub.publish(Bool(True))
        rospy.loginfo('orbslam_tf_bridge: seeded from the given start pose x=%.2f y=%.2f yaw=%.0f deg; wheel odometry until ORB-SLAM3 recognises the map', x, y, np.degrees(yaw))

    def on_state(self, msg):
        self.state[msg.stamp.to_nsec()] = (int(msg.frame_id) if msg.frame_id.lstrip('-').isdigit() else -1, int(msg.seq) // 1000)
        if len(self.state) > 400:
            for k in sorted(self.state)[:200]:
                del self.state[k]

    def on_pose(self, msg):
        st = self.state.get(msg.header.stamp.to_nsec())
        if st is None:
            rospy.sleep(0.01)                               # the state message is sent just before the pose
            st = self.state.get(msg.header.stamp.to_nsec())
        if st is None or st[0] not in OK_STATES or st[1] != self.expected_maps:
            rospy.loginfo_throttle(10.0, 'orbslam_tf_bridge: state %s, maps in atlas %s (need %d) - not using this pose', st and st[0], st and st[1], self.expected_maps)
            return
        try:
            T_cam_base = T_from_tf(self.buf.lookup_transform(self.cam_frame, self.base_frame, msg.header.stamp, rospy.Duration(0.2)))
            T_odom_base = T_from_tf(self.buf.lookup_transform(self.odom_frame, self.base_frame, msg.header.stamp, rospy.Duration(0.2)))
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException, tf2_ros.ConnectivityException) as e:
            rospy.logwarn_throttle(5.0, 'orbslam_tf_bridge: no TF for the camera pose: %s', e)
            return
        p, q = msg.pose.position, msg.pose.orientation
        T_map_cam = self.T_up.dot(T_from((p.x, p.y, p.z), (q.x, q.y, q.z, q.w)))
        T_map_base = T_map_cam.dot(T_cam_base)
        with self.lock:
            self.T_map_odom = T_map_base.dot(np.linalg.inv(T_odom_base))
            self.n_fix += 1
            self.last_fix = msg.header.stamp
            if not self.localized:
                self.localized = True
                self.loc_pub.publish(Bool(True))
                rospy.loginfo('orbslam_tf_bridge: localised in the saved map (visual fix); robot at x=%.2f y=%.2f', T_map_base[0, 3], T_map_base[1, 3])
        out = PoseStamped()
        out.header.stamp, out.header.frame_id = msg.header.stamp, self.map_frame
        out.pose.position.x, out.pose.position.y, out.pose.position.z = T_map_base[:3, 3]
        qq = tft.quaternion_from_matrix(T_map_base)
        out.pose.orientation.x, out.pose.orientation.y, out.pose.orientation.z, out.pose.orientation.w = qq
        self.pose_pub.publish(out)

    def publish(self, _):
        with self.lock:
            T = self.T_map_odom
        if T is None:
            return
        tr = TransformStamped()
        tr.header.stamp = rospy.Time.now() + self.future
        tr.header.frame_id, tr.child_frame_id = self.map_frame, self.odom_frame
        tr.transform.translation.x, tr.transform.translation.y, tr.transform.translation.z = T[:3, 3]
        q = tft.quaternion_from_matrix(T)
        tr.transform.rotation.x, tr.transform.rotation.y, tr.transform.rotation.z, tr.transform.rotation.w = q
        self.br.sendTransform(tr)

    def report(self, _):
        if not self.localized:
            rospy.loginfo_throttle(10.0, 'orbslam_tf_bridge: waiting for ORB-SLAM3 to recognise the saved map ...')
        elif self.last_fix is not None and (rospy.Time.now() - self.last_fix).to_sec() > 2.0:
            rospy.logwarn_throttle(10.0, 'orbslam_tf_bridge: no visual fix for %.0f s, running on wheel odometry', (rospy.Time.now() - self.last_fix).to_sec())


if __name__ == '__main__':
    rospy.init_node('orbslam_tf_bridge')
    Bridge()
    rospy.spin()
