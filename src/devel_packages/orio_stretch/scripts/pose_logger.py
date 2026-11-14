#!/usr/bin/env python3
"""
pose_logger.py - log the robot's live map pose (TF map -> base_link, whatever localiser publishes it) as a TUM file:
    stamp x y z qx qy qz qw
so rfid_pose_join.py --traj can give every RFID read its position. Runs until SIGINT/SIGTERM.

    pose_logger.py <out.tum> [--rate 20] [--map-frame map] [--base-frame base_link]
"""
import argparse
import signal

import rospy
import tf2_ros


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('out'); ap.add_argument('--rate', type=float, default=20.0)
    ap.add_argument('--map-frame', default='map'); ap.add_argument('--base-frame', default='base_link')
    args = ap.parse_args(rospy.myargv()[1:])
    rospy.init_node('pose_logger', anonymous=True, disable_signals=True)
    buf = tf2_ros.Buffer(rospy.Duration(10.0)); tf2_ros.TransformListener(buf)
    f = open(args.out, 'w', buffering=1)
    stop = {'now': False}
    signal.signal(signal.SIGINT, lambda *a: stop.update(now=True)); signal.signal(signal.SIGTERM, lambda *a: stop.update(now=True))
    last, n = None, 0
    rate = rospy.Rate(args.rate)
    while not stop['now'] and not rospy.is_shutdown():
        try:
            tr = buf.lookup_transform(args.map_frame, args.base_frame, rospy.Time(0))
            if tr.header.stamp != last:
                last = tr.header.stamp; t, q = tr.transform.translation, tr.transform.rotation
                f.write('%.6f %.4f %.4f %.4f %.6f %.6f %.6f %.6f\n' % (rospy.Time.now().to_sec(), t.x, t.y, t.z, q.x, q.y, q.z, q.w)); n += 1
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException, tf2_ros.ConnectivityException):
            pass
        try:
            rate.sleep()
        except rospy.ROSInterruptException:
            break
    f.close(); print('pose_logger: %d poses -> %s' % (n, args.out))


if __name__ == '__main__':
    main()
