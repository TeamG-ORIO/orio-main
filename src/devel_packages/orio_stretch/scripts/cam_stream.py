#!/usr/bin/env python3
"""
cam_stream.py - the head camera's colour picture for the demo window's manual mode: newest frame as a JPEG file.

    python3 scripts/cam_stream.py [--out /dev/shm/orio_cam.jpg] [--fps 8] [--width 424 --height 240] [--quality 70]

Opens the RealSense directly (pyrealsense2, colour stream only, no ROS), turns the picture upright (the head camera
is mounted on its side: 90 degrees clockwise) and rewrites the file about --fps times a second, each time in one step
(write to a temporary name, then rename), so a reader never sees half a picture. demo_agent.py starts it with manual
mode and serves the file at GET /camera.jpg. Runs until SIGINT / SIGTERM.

The camera takes one program at a time: this cannot run next to a navigation run, a recording or ~/teleop_server's
camera_server.py. demo_agent.py stops it before it starts anything else.
"""
import argparse
import os
import signal
import sys
import time

import cv2
import numpy as np
import pyrealsense2 as rs


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--out', default='/dev/shm/orio_cam.jpg'); ap.add_argument('--fps', type=float, default=8.0)
    ap.add_argument('--width', type=int, default=424); ap.add_argument('--height', type=int, default=240)
    ap.add_argument('--quality', type=int, default=70)
    a = ap.parse_args()
    stop = {'now': False}
    signal.signal(signal.SIGINT, lambda *x: stop.update(now=True)); signal.signal(signal.SIGTERM, lambda *x: stop.update(now=True))
    pipe, cfg = rs.pipeline(), rs.config()
    cfg.enable_stream(rs.stream.color, a.width, a.height, rs.format.bgr8, 15)
    try:
        pipe.start(cfg)
    except RuntimeError as e:
        sys.exit('cam_stream: the camera did not start (%s). Is something else using it?' % e)
    print('cam_stream: %dx%d colour -> %s at %.0f fps' % (a.width, a.height, a.out, a.fps), flush=True)
    tmp, n, last = a.out + '.tmp', 0, 0.0
    try:
        while not stop['now']:
            try:
                frames = pipe.wait_for_frames(2000)
            except RuntimeError:
                continue                         # no frame for 2 s: keep trying, the agent reports the feed as stale
            if time.time() - last < 1.0 / a.fps:
                continue
            c = frames.get_color_frame()
            if not c:
                continue
            img = cv2.rotate(np.asanyarray(c.get_data()), cv2.ROTATE_90_CLOCKWISE)
            ok, jpg = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, a.quality])
            if ok:
                with open(tmp, 'wb') as f:
                    f.write(jpg.tobytes())
                os.replace(tmp, a.out); last = time.time(); n += 1
    finally:
        pipe.stop()
        for p in (tmp, a.out):
            try:
                os.remove(p)
            except OSError:
                pass
        print('cam_stream: %d frames written' % n, flush=True)


if __name__ == '__main__':
    main()
