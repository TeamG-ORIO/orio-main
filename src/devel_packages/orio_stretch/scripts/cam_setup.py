#!/usr/bin/env python3
"""
cam_setup.py - set + log the D435i settings for a SLAM test recording.

Run it WHILE `roslaunch orio_stretch record.launch infra:=true ...` is up. It sets, by dynamic_reconfigure,
    colour sensor (/camera/rgb_camera):     exposure [units of 100 us], gain [0..128]
    IR / depth sensor (/camera/stereo_module): exposure [us], gain [16..248], IR projector mode
and publishes what the camera really ended up with on /slam_test/config (latched, JSON), so it is
in the bag. The same JSON is printed on stdout as one line starting with "CAMCFG ".

Projector (emitter) modes:
    on         dots in every IR image, best depth                (what the robot normally runs)
    off        clean IR images, depth from passive stereo only   (worse on blank surfaces)
    alternate  the projector toggles every frame: half the frames have clean IR, the other half
               have good depth. Per-frame state is in /camera/*/metadata (frame_laser_power_mode).
               With auto exposure the IR brightness can flicker in this mode - prefer a manual
               --ir-exposure.

    python3 cam_setup.py --emitter alternate
    python3 cam_setup.py --emitter off --ir-exposure 4000 --ir-gain 60 --color-exposure 50 --color-gain 128
"""
import argparse
import json
import sys

import rospy
from std_msgs.msg import String

RGB_NS = '/camera/rgb_camera'
IR_NS = '/camera/stereo_module'
RGB_KEYS = ('enable_auto_exposure', 'exposure', 'gain', 'auto_exposure_priority')
IR_KEYS = ('enable_auto_exposure', 'exposure', 'gain', 'emitter_enabled', 'emitter_on_off', 'laser_power')


def set_exposure(client, exposure, gain):
    """exposure 'auto' = leave the sensor as launched; otherwise manual exposure (+ gain)."""
    if exposure == 'auto':
        return
    # auto exposure must be off before the sensor accepts a manual exposure
    client.update_configuration({'enable_auto_exposure': False})
    rospy.sleep(0.5)
    upd = {'exposure': int(exposure)}
    if gain is not None:
        upd['gain'] = int(gain)
    client.update_configuration(upd)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--color-exposure', default='auto', help="'auto' or manual, units of 100 us (50 = 5 ms)")
    ap.add_argument('--color-gain', type=int, default=None, help='0..128, only with a manual --color-exposure')
    ap.add_argument('--ir-exposure', default='auto', help="'auto' or manual, in MICROSECONDS (4000 = 4 ms)")
    ap.add_argument('--ir-gain', type=int, default=None, help='16..248, only with a manual --ir-exposure')
    ap.add_argument('--emitter', choices=['on', 'off', 'alternate', 'keep'], default='keep')
    ap.add_argument('--no-ir', action='store_true', help='the IR streams are not running: only touch the colour sensor')
    args = ap.parse_args(rospy.myargv()[1:])
    for name in ('color_exposure', 'ir_exposure'):
        v = getattr(args, name)
        if v != 'auto':
            int(v)

    rospy.init_node('cam_setup')
    import dynamic_reconfigure.client
    out = {'requested': vars(args)}
    ok = True

    rgb = dynamic_reconfigure.client.Client(RGB_NS, timeout=15)
    set_exposure(rgb, args.color_exposure, args.color_gain)

    ir = dynamic_reconfigure.client.Client(IR_NS, timeout=15)
    if not args.no_ir:
        set_exposure(ir, args.ir_exposure, args.ir_gain)
    if args.emitter == 'on':
        ir.update_configuration({'emitter_on_off': False})
        ir.update_configuration({'emitter_enabled': 1})
    elif args.emitter == 'off':
        ir.update_configuration({'emitter_on_off': False})
        ir.update_configuration({'emitter_enabled': 0})
    elif args.emitter == 'alternate':
        ir.update_configuration({'emitter_enabled': 1})
        ir.update_configuration({'emitter_on_off': True})

    rospy.sleep(1.0)
    cfg = rgb.get_configuration(timeout=5)
    out['color'] = {k: cfg.get(k) for k in RGB_KEYS}
    cfg = ir.get_configuration(timeout=5)
    out['ir'] = {k: cfg.get(k) for k in IR_KEYS}

    # did the camera take it?
    if args.color_exposure != 'auto' and (out['color']['enable_auto_exposure'] or out['color']['exposure'] != int(args.color_exposure)):
        rospy.logerr('colour sensor did not accept exposure %s: %s', args.color_exposure, out['color'])
        ok = False
    if not args.no_ir and args.ir_exposure != 'auto' and (out['ir']['enable_auto_exposure'] or out['ir']['exposure'] != int(args.ir_exposure)):
        rospy.logerr('IR sensor did not accept exposure %s: %s', args.ir_exposure, out['ir'])
        ok = False
    want = {'on': (1, False), 'off': (0, False), 'alternate': (1, True)}.get(args.emitter)
    if want and (out['ir']['emitter_enabled'], out['ir']['emitter_on_off']) != want:
        rospy.logerr('IR projector mode %s was not accepted: %s', args.emitter, out['ir'])
        ok = False
    out['ok'] = ok
    out['stamp'] = rospy.Time.now().to_sec()

    pub = rospy.Publisher('/slam_test/config', String, queue_size=1, latch=True)
    text = json.dumps(out, sort_keys=True)
    # latched topics only reach rosbag while the publisher is alive: publish, then stay up a moment
    for _ in range(6):
        pub.publish(String(data=text))
        rospy.sleep(0.5)
    print('CAMCFG ' + text)
    sys.stdout.flush()
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
