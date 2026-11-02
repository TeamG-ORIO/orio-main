#!/usr/bin/env python
"""Fast-FoundationStereo depth for the ZED-M, as a ROS node.

Subscribes to the ZED wrapper's rectified left/right images and camera infos, runs
Fast-FoundationStereo on a pair and publishes metric depth in the left camera frame at the
input resolution, so a pixel of the left image indexes the depth image directly:

    /ffs/depth        sensor_msgs/Image 32FC1, metres, NaN where invalid
    /ffs/left         sensor_msgs/Image rgb8: the left frame the depth was computed from
    /ffs/camera_info  sensor_msgs/CameraInfo of that left frame
    /ffs/disp_vis     sensor_msgs/Image rgb8: left frame beside the disparity colour map
    /ffs/compute      std_srvs/Trigger: run on the newest pair now (also at ~rate Hz)

The four messages of one computation carry the left frame's stamp, so a consumer can match
them (research/labelling_arm_test/perception does). The focal length comes from the left
camera_info and the baseline from the right one (P[3] = -fx * baseline), so this follows
whatever resolution the wrapper runs at; nothing is hard-coded.

Runs inside the orio/ffs-ros container (ffs/Dockerfile) with the fast-foundationstereo
repo mounted at $ORIO_FFS_DIR. The inference mirrors reference/ffs_orio_infer.py, the
offline script this was checked against.

Params:
    ~model        checkpoint .pth (its pickled args carry the architecture)
    ~scale        image scale before inference (default 0.5: the authors' operating point is
                  width < 1000 px, and it is ~4x faster than full HD1080; the depth is
                  resized back to full resolution, nearest neighbour)
    ~valid_iters  GRU refinement iterations (default 8)
    ~max_disp     (default 192)
    ~rate         continuous computation in Hz, 0 = only on /ffs/compute (default 0)
    ~max_depth    metres; further is invalid (default 3.0)
    ~left ~right ~left_info ~right_info   input topics (ZED wrapper defaults)
"""

import os
import sys
import threading
import time

import cv2
import numpy as np
import rospy
import torch
from omegaconf import OmegaConf
from sensor_msgs.msg import CameraInfo, Image
from std_srvs.srv import Trigger, TriggerResponse

FFS_DIR = os.environ.get('ORIO_FFS_DIR', '/opt/fast-foundationstereo')
sys.path.insert(0, FFS_DIR)
from core.utils.utils import InputPadder  # noqa: E402
from Utils import AMP_DTYPE, vis_disparity  # noqa: E402

ENCODING_CHANNELS = {'bgra8': 4, 'rgba8': 4, 'bgr8': 3, 'rgb8': 3, 'mono8': 1}


def decode_image(msg):
    """ROS Image -> RGB uint8 HxWx3 (the ZED wrapper publishes bgra8)."""
    enc = msg.encoding.lower()
    channels = ENCODING_CHANNELS.get(enc)
    if channels is None:
        raise ValueError("unsupported image encoding %r" % msg.encoding)
    rows = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
    arr = rows[:, :msg.width * channels].reshape(msg.height, msg.width, channels)
    if enc == 'mono8':
        return np.repeat(arr, 3, axis=2)
    arr = arr[:, :, :3]
    if enc.startswith('bgr'):
        arr = arr[:, :, ::-1]
    return np.ascontiguousarray(arr)


def to_image_msg(arr, encoding, header):
    msg = Image()
    msg.header = header
    msg.height, msg.width = arr.shape[:2]
    msg.encoding = encoding
    msg.is_bigendian = 0
    msg.step = arr.strides[0]
    msg.data = arr.tobytes()
    return msg


class FastFoundationStereo(object):
    """The network, turning one rectified pair into disparity and metric depth."""

    def __init__(self, checkpoint, scale=0.5, valid_iters=8, max_disp=192):
        self.scale = float(scale)
        self.valid_iters = int(valid_iters)
        t = time.time()
        model = torch.load(checkpoint, map_location='cpu', weights_only=False)
        # Some released checkpoints (the HF c-fast-foundationstereo one) pickle an args
        # struct missing keys the code reads. Backfill with the code's defaults.
        OmegaConf.set_struct(model.args, False)
        if 'normalize' not in model.args:
            model.args.normalize = True
        model.args.valid_iters = self.valid_iters
        model.args.max_disp = int(max_disp)
        self.model = model.cuda().eval()
        torch.cuda.synchronize()
        self.params_m = sum(p.numel() for p in model.parameters()) / 1e6
        self.load_s = time.time() - t

    @torch.no_grad()
    def disparity(self, left_rgb, right_rgb):
        """Disparity (pixels) at the scaled resolution, >= 0."""
        left, right = left_rgb, right_rgb
        if self.scale != 1.0:
            left = cv2.resize(left_rgb, None, fx=self.scale, fy=self.scale)
            right = cv2.resize(right_rgb, dsize=(left.shape[1], left.shape[0]))
        H, W = left.shape[:2]
        t0 = torch.as_tensor(left).cuda().float()[None].permute(0, 3, 1, 2)
        t1 = torch.as_tensor(right).cuda().float()[None].permute(0, 3, 1, 2)
        padder = InputPadder(t0.shape, divis_by=32, force_square=False)
        t0, t1 = padder.pad(t0, t1)
        with torch.amp.autocast('cuda', enabled=True, dtype=AMP_DTYPE):
            disp = self.model.forward(t0, t1, iters=self.valid_iters, test_mode=True,
                                      optimize_build_volume='pytorch1')
        torch.cuda.synchronize()
        return padder.unpad(disp.float()).cpu().numpy().reshape(H, W).clip(0, None)

    def depth(self, left_rgb, right_rgb, fx, baseline, max_depth):
        """(depth at the input resolution in metres with NaN where invalid, disparity)."""
        disp = self.disparity(left_rgb, right_rgb)
        H, W = disp.shape
        # No disparity, or a match that would fall outside the right image (the reference
        # script's remove_invisible), is not a measurement.
        xx = np.arange(W, dtype=np.float32)[None, :]
        invalid = (disp <= 0) | ((xx - disp) < 0)
        depth = np.full(disp.shape, np.nan, dtype=np.float32)
        depth[~invalid] = (fx * self.scale) * baseline / disp[~invalid]
        depth[depth > max_depth] = np.nan
        if depth.shape != left_rgb.shape[:2]:
            depth = cv2.resize(depth, (left_rgb.shape[1], left_rgb.shape[0]),
                               interpolation=cv2.INTER_NEAREST)
        return depth, disp


class FfsDepthNode(object):
    RIGHT_BUFFER = 30            # right frames kept, keyed by stamp (2 s at 15 Hz)
    PAIR_TOLERANCE_NS = 20000000  # a right frame this close in time still counts as the pair

    def __init__(self):
        rospy.init_node('ffs_depth')
        p = rospy.get_param
        checkpoint = p('~model', '/opt/ffs_checkpoint.pth')
        self.max_depth = float(p('~max_depth', 3.0))
        rate = float(p('~rate', 0.0))
        self.left_topic = p('~left', '/zedm/zed_node/left/image_rect_color')
        self.right_topic = p('~right', '/zedm/zed_node/right/image_rect_color')
        left_info_topic = p('~left_info', '/zedm/zed_node/left/camera_info')
        right_info_topic = p('~right_info', '/zedm/zed_node/right/camera_info')

        rospy.loginfo("[FFS] loading %s (scale %.2f, iters %d)…", checkpoint,
                      float(p('~scale', 0.5)), int(p('~valid_iters', 8)))
        self.net = FastFoundationStereo(checkpoint, scale=p('~scale', 0.5),
                                        valid_iters=p('~valid_iters', 8),
                                        max_disp=p('~max_disp', 192))
        rospy.loginfo("[FFS] model on GPU in %.1f s (%.1fM params)", self.net.load_s,
                      self.net.params_m)

        # One computation at a time: the timer and the service share the GPU model.
        self._lock = threading.Lock()
        self.left = None
        self.rights = {}
        self.left_info = None
        self.right_info = None
        self._warm = False

        rospy.Subscriber(self.left_topic, Image, self._on_left, queue_size=1,
                         buff_size=2 ** 24)
        rospy.Subscriber(self.right_topic, Image, self._on_right, queue_size=1,
                         buff_size=2 ** 24)
        rospy.Subscriber(left_info_topic, CameraInfo,
                         lambda m: setattr(self, 'left_info', m), queue_size=1)
        rospy.Subscriber(right_info_topic, CameraInfo,
                         lambda m: setattr(self, 'right_info', m), queue_size=1)

        self.pub_depth = rospy.Publisher('/ffs/depth', Image, queue_size=1)
        self.pub_left = rospy.Publisher('/ffs/left', Image, queue_size=1)
        self.pub_info = rospy.Publisher('/ffs/camera_info', CameraInfo, queue_size=1)
        self.pub_vis = rospy.Publisher('/ffs/disp_vis', Image, queue_size=1)
        rospy.Service('/ffs/compute', Trigger, self._on_compute)

        if rate > 0:
            rospy.Timer(rospy.Duration(1.0 / rate), lambda _e: self.compute())
            rospy.loginfo("[FFS] computing continuously at %.2f Hz", rate)
        rospy.loginfo("[FFS] ready: /ffs/compute -> /ffs/depth (waiting for %s + %s)",
                      self.left_topic, self.right_topic)

    # ── Inputs ────────────────────────────────────────────────────────────────
    def _on_left(self, msg):
        self.left = msg
        if not self._warm:
            # The first forward pass compiles kernels for this image size (seconds); do it
            # on the first real pair so the first /ffs/compute call is fast.
            self._warm = True
            threading.Thread(target=self._warmup, daemon=True).start()

    def _on_right(self, msg):
        self.rights[msg.header.stamp.to_nsec()] = msg
        while len(self.rights) > self.RIGHT_BUFFER:
            del self.rights[min(self.rights)]

    def _warmup(self):
        rospy.sleep(0.5)   # let the right frame of the same grab arrive
        ok, message = self.compute()
        rospy.loginfo("[FFS] warm-up: %s", message if ok else "skipped (%s)" % message)

    def _pair(self):
        """The newest left frame and the right frame of the same grab, or a reason."""
        left = self.left
        if left is None:
            return None, None, "no image yet on %s" % self.left_topic
        if not self.rights:
            return None, None, "no image yet on %s" % self.right_topic
        key = left.header.stamp.to_nsec()
        right = self.rights.get(key)
        if right is None:
            nearest = min(self.rights, key=lambda k: abs(k - key))
            if abs(nearest - key) > self.PAIR_TOLERANCE_NS:
                return None, None, ("no right frame within %d ms of the left frame's stamp"
                                    % (self.PAIR_TOLERANCE_NS // 1000000))
            right = self.rights[nearest]
        return left, right, None

    # ── Computation ───────────────────────────────────────────────────────────
    def compute(self):
        """Run on the newest pair and publish. Returns (ok, message)."""
        with self._lock:
            left_msg, right_msg, why = self._pair()
            if why:
                return False, why
            if self.left_info is None or self.right_info is None:
                return False, "no camera_info yet (left %s, right %s)" % (
                    self.left_info is not None, self.right_info is not None)
            fx = float(self.left_info.K[0])
            baseline = -float(self.right_info.P[3]) / float(self.right_info.P[0])
            if not (fx > 0 and baseline > 0):
                return False, "bad intrinsics: fx=%.3f baseline=%.4f" % (fx, baseline)

            t = time.time()
            left = decode_image(left_msg)
            right = decode_image(right_msg)
            depth, disp = self.net.depth(left, right, fx, baseline, self.max_depth)
            dt = time.time() - t

            header = left_msg.header
            info = self.left_info
            info.header = header
            # Depth last: a consumer that waits for it then finds the others already there.
            self.pub_left.publish(to_image_msg(left, 'rgb8', header))
            self.pub_info.publish(info)
            vis = vis_disparity(disp)
            if vis.shape[:2] != left.shape[:2]:
                vis = cv2.resize(vis, (left.shape[1], left.shape[0]))
            self.pub_vis.publish(to_image_msg(np.ascontiguousarray(
                np.concatenate([left, vis], axis=1)), 'rgb8', header))
            self.pub_depth.publish(to_image_msg(np.ascontiguousarray(depth), '32FC1', header))

            valid = np.isfinite(depth)
            median = float(np.median(depth[valid])) if valid.any() else float('nan')
            message = ("depth %dx%d @ %.3f: %.1f%% valid, median %.3f m, %.0f ms "
                       "(fx %.1f px, baseline %.4f m)"
                       % (depth.shape[1], depth.shape[0], header.stamp.to_sec(),
                          100.0 * valid.mean(), median, 1000 * dt, fx, baseline))
            rospy.loginfo("[FFS] %s", message)
            return True, message

    def _on_compute(self, _req):
        ok, message = self.compute()
        if not ok:
            rospy.logwarn("[FFS] compute failed: %s", message)
        return TriggerResponse(success=ok, message=message)


if __name__ == '__main__':
    FfsDepthNode()
    rospy.spin()
