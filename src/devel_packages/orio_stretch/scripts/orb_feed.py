#!/usr/bin/env python3
"""
orb_feed.py - sits between `rosbag play` and the ORB-SLAM3 node and picks the frames a mode may use.

A bag recorded with the IR projector in "alternate" mode has two kinds of frames interleaved:
projector ON (good depth, IR images full of dots) and projector OFF (clean IR, passive depth).
The per-frame state is in /camera/*/metadata (frame_laser_power_mode), keyed by the frame stamp.
This node republishes
    ~image_in  -> /orb_feed/image     (colour or left IR)
    ~second_in -> /orb_feed/second    (aligned depth, native depth or right IR)
and, with ~laser = on|off, only the frames whose projector state matches. ~laser = any passes
everything (bags recorded with the projector constantly on or off, or without metadata).

Frames are matched to metadata by nearest stamp (within 2 ms). Colour frames carry no projector
state of their own: the camera driver stamps synchronised colour + depth alike, so they are matched
the same way; if colour stamps never match, all colour frames pass and the ORB-SLAM node pairs them
with the selected depth frames by time.
"""
import bisect
import collections

import rospy
from realsense2_camera.msg import Metadata
from sensor_msgs.msg import Image

import slam_bag as sb


class Feed(object):
    def __init__(self):
        self.laser = rospy.get_param('~laser', 'any')
        if isinstance(self.laser, bool):       # `_laser:=on` / `_laser:=off` arrive as YAML booleans
            self.laser = 'on' if self.laser else 'off'
        self.want = {'on': 1, 'off': 0}.get(self.laser)
        image_in = rospy.get_param('~image_in')
        second_in = rospy.get_param('~second_in')
        # projector state per frame; metadata stamps differ from the image stamps by some microseconds
        self.meta_t, self.meta_mode = [], []        # stamp [ns] (ascending), 0/1
        self.pending = collections.deque()          # (which, msg) waiting for their metadata
        self.n = collections.Counter()
        self.color_loose = False                    # colour stamps do not match depth stamps: pass all colour
        self.is_color = (image_in == sb.COLOR)
        self.pub = {'image': rospy.Publisher('/orb_feed/image', Image, queue_size=50),
                    'second': rospy.Publisher('/orb_feed/second', Image, queue_size=50)}
        if self.want is not None:
            for t in sb.META:
                rospy.Subscriber(t, Metadata, self.on_meta, queue_size=200)
        rospy.Subscriber(image_in, Image, self.on_image, 'image', queue_size=50, buff_size=1 << 26)
        rospy.Subscriber(second_in, Image, self.on_image, 'second', queue_size=50, buff_size=1 << 26)
        rospy.on_shutdown(self.report)

    def on_meta(self, msg):
        mode = sb.laser_mode_of(msg)
        if mode is None:
            return
        ns = msg.header.stamp.to_nsec()
        if self.meta_t and ns <= self.meta_t[-1]:
            if ns < self.meta_t[-1] - 5e9:          # the bag was restarted
                self.meta_t, self.meta_mode = [], []
            else:
                return                              # same frame reported by another stream's metadata
        self.meta_t.append(ns)
        self.meta_mode.append(mode)
        if len(self.meta_t) > 1200:
            del self.meta_t[:600], self.meta_mode[:600]
        self.flush()

    def mode_at(self, ns, tol=2000000):
        i = bisect.bisect_left(self.meta_t, ns)
        best = None
        for j in (i - 1, i):
            if 0 <= j < len(self.meta_t) and abs(self.meta_t[j] - ns) <= tol and \
                    (best is None or abs(self.meta_t[j] - ns) < abs(self.meta_t[best] - ns)):
                best = j
        return None if best is None else self.meta_mode[best]

    def on_image(self, msg, which):
        self.n[which + '_in'] += 1
        if self.want is None or (which == 'image' and self.is_color and self.color_loose):
            self.emit(which, msg)
            return
        self.pending.append((which, msg))
        self.flush()

    def flush(self):
        keep = collections.deque()
        newest = self.meta_t[-1] if self.meta_t else 0
        while self.pending:
            which, msg = self.pending.popleft()
            ns = msg.header.stamp.to_nsec()
            mode = self.mode_at(ns)
            if mode is not None:
                if mode == self.want:
                    self.emit(which, msg)
                continue
            if newest - ns > 300e6:
                # metadata is well past this frame and never mentioned its stamp
                if which == 'image' and self.is_color:
                    self.n['color_unmatched'] += 1
                    if self.n['color_unmatched'] >= 10 and not self.n['image_out']:
                        rospy.logwarn('colour stamps do not match the depth stamps: passing every colour frame')
                        self.color_loose = True
                    if self.color_loose:
                        self.emit(which, msg)
                else:
                    self.n[which + '_no_metadata'] += 1
                continue
            keep.append((which, msg))
        self.pending = keep

    def emit(self, which, msg):
        self.n[which + '_out'] += 1
        self.pub[which].publish(msg)

    def report(self):
        rospy.loginfo('orb_feed (laser=%s): %s', self.laser, dict(self.n))


if __name__ == '__main__':
    rospy.init_node('orb_feed')
    Feed()
    rospy.spin()
