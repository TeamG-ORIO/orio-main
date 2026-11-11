#!/usr/bin/env python3
"""Segment the items in the two label zones with the RGB-D segmentation model.

The perception node (perception_control_combined_pass_through.py) is the only caller. It
has Fast-FoundationStereo compute a fresh ZED left image + depth (/ffs/compute), then asks
this node to segment that frame:

  request  /label_seg/request  std_msgs/String, JSON {"id": any, "stamp_ns": the frame's
           stamp}. This node keeps the last few /ffs/left and /ffs/depth frames by stamp.
  answer   /label_seg/result   std_msgs/String, JSON {"id", "ok", "error", "stamp_ns",
           "seconds", "zones": {"1": {"box", "n_items", "top", "votes", "used",
           "table_frac", "items": [{"id", "score", "top", "centroid_px", "pick_px",
           "area_px", "height_mm", "edge_height_mm"}]}, "2": ...}}, pixels in the full
           frame. The items of a zone come from label_zone_seg.segment_zone: six passes of
           the grasp planner's segmentation model, the top item of a pile marked.
  masks    /label_seg/instances  mono16 image the size of the ZED frame, with the frame's
           stamp, published before the answer: 0 = no item, zone * 100 + k for the k-th
           item of a zone (k from 1, most confident first; the "id" of the answer's items).

Topics, not a service: this node holds what needs torch and rfdetr, so it runs in the
suction planner's image (orio_bringup/docker/run_label_seg.sh, whichever grasp planner
the run uses), and that image has std_msgs and sensor_msgs but no std_srvs.
/label_seg/result is advertised only once the model is loaded, so its presence means ready.

Params: ~checkpoint (the RGB-D segmentation weights), ~model (default RFDETRSegLarge),
~grasping_repo (orio_grasping, default /opt/orio-grasping).
"""
import json
import os
import sys
import time

import numpy as np
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import label_zone_seg as lz  # noqa: E402

FFS_LEFT_TOPIC = "/ffs/left"
FFS_DEPTH_TOPIC = "/ffs/depth"
REQUEST_TOPIC = "/label_seg/request"
RESULT_TOPIC = "/label_seg/result"
INSTANCES_TOPIC = "/label_seg/instances"
FRAME_WAIT_S = 5.0   # how long a request waits for its frame to arrive here


def keep_by_stamp(store, keep=10):
    def cb(msg):
        store[msg.header.stamp.to_nsec()] = msg
        while len(store) > keep:
            del store[min(store)]
    return cb


class LabelSegNode:
    def __init__(self):
        rospy.init_node("label_seg")
        checkpoint = rospy.get_param("~checkpoint")
        model_name = rospy.get_param("~model", "RFDETRSegLarge")
        sys.path.insert(0, rospy.get_param("~grasping_repo", "/opt/orio-grasping"))
        import rfdetr
        import torch
        from orio_grasping import rgbd_seg
        self._torch, self._encode = torch, rgbd_seg.encode_height
        t0 = time.time()
        self._model = rgbd_seg.load_model(getattr(rfdetr, model_name), checkpoint)
        # The first prediction builds the model's buffers and is slow: do it now.
        self._predict(np.zeros((640, 640, 3), np.uint8), np.zeros((640, 640), np.float32))
        rospy.loginfo("[LabelSeg] %s loaded from %s in %.1f s", model_name, checkpoint, time.time() - t0)

        self._lefts, self._depths = {}, {}
        rospy.Subscriber(FFS_LEFT_TOPIC, Image, keep_by_stamp(self._lefts), queue_size=2, buff_size=2 ** 26)
        rospy.Subscriber(FFS_DEPTH_TOPIC, Image, keep_by_stamp(self._depths), queue_size=2, buff_size=2 ** 26)
        self._pub_inst = rospy.Publisher(INSTANCES_TOPIC, Image, queue_size=2)
        self._pub_result = rospy.Publisher(RESULT_TOPIC, String, queue_size=5)
        # One callback thread for this topic: requests are handled one at a time, in order.
        rospy.Subscriber(REQUEST_TOPIC, String, self._on_request, queue_size=5)
        rospy.loginfo("[LabelSeg] ready: requests on %s, answers on %s", REQUEST_TOPIC, RESULT_TOPIC)

    def _predict(self, rgb, height):
        torch = self._torch
        x = torch.cat([torch.from_numpy(rgb).permute(2, 0, 1).float() / 255.0,
                       torch.from_numpy(self._encode(height))])
        det = self._model.predict(x, threshold=lz.SCORE_MIN, include_source_image=False)
        if det.mask is None or len(det.mask) == 0:
            return np.zeros((0,) + rgb.shape[:2], bool), np.zeros(0)
        return np.asarray(det.mask, bool), np.asarray(det.confidence, dtype=np.float64)

    def _frame(self, key):
        """(RGB uint8, depth float32 metres with NaN invalid, header) of the frame with
        this stamp. Raises RuntimeError with the reason."""
        deadline = time.time() + FRAME_WAIT_S
        while not (key in self._lefts and key in self._depths):
            if time.time() > deadline:
                raise RuntimeError("the frame to segment (stamp %d) did not arrive on %s and %s within %.0f s"
                                   % (key, FFS_LEFT_TOPIC, FFS_DEPTH_TOPIC, FRAME_WAIT_S))
            time.sleep(0.01)
        left, d = self._lefts[key], self._depths[key]
        ch = left.step // left.width
        color = np.frombuffer(left.data, np.uint8).reshape(left.height, left.step)
        color = color[:, :left.width * ch].reshape(left.height, left.width, ch)[:, :, :3]
        if "bgr" in left.encoding.lower():
            color = color[:, :, ::-1]
        if d.encoding != "32FC1":
            raise RuntimeError("unsupported depth encoding %r" % d.encoding)
        depth = np.frombuffer(d.data, np.float32).reshape(d.height, d.width).copy()
        depth[~np.isfinite(depth) | (depth <= 0)] = np.nan
        if depth.shape != color.shape[:2]:
            raise RuntimeError("depth %s and colour %s differ in size" % (depth.shape, color.shape[:2]))
        return np.ascontiguousarray(color), depth, d.header

    def _segment(self, key):
        color, depth, header = self._frame(key)
        instances = np.zeros(depth.shape, np.uint16)
        zones = {}
        for zone, (x1, y1, x2, y2) in lz.zone_boxes(depth.shape).items():
            seg = lz.segment_zone(self._predict, np.ascontiguousarray(color[y1:y2, x1:x2]),
                                  depth[y1:y2, x1:x2])
            # least confident first, so a more confident mask wins a shared pixel
            for k in reversed(range(seg["n_items"])):
                instances[y1:y2, x1:x2][seg["masks"][k]] = zone * 100 + k + 1
            items = [dict(it, id=zone * 100 + k + 1,
                          centroid_px=[it["centroid_px"][0] + x1, it["centroid_px"][1] + y1],
                          pick_px=[it["pick_px"][0] + x1, it["pick_px"][1] + y1])
                     for k, it in enumerate(seg["items"])]
            zones[str(zone)] = {"box": [x1, y1, x2, y2], "n_items": seg["n_items"],
                                "top": seg["top"], "votes": seg["votes"], "used": seg["used"],
                                "table_frac": seg["table_frac"], "items": items}
        msg = Image()
        msg.header = header
        msg.height, msg.width = instances.shape
        msg.encoding, msg.is_bigendian, msg.step = "mono16", 0, instances.shape[1] * 2
        msg.data = instances.tobytes()
        self._pub_inst.publish(msg)
        return zones

    def _on_request(self, msg):
        t0 = time.time()
        answer = {"id": None, "ok": False, "error": ""}
        try:
            req = json.loads(msg.data)
            answer["id"] = req.get("id")
            key = int(req["stamp_ns"])
            answer["stamp_ns"] = key
            answer["zones"] = self._segment(key)
            answer["ok"] = True
            rospy.loginfo("[LabelSeg] zone 1: %d item(s) %s, zone 2: %d item(s) %s; %.2f s",
                          answer["zones"]["1"]["n_items"], list(answer["zones"]["1"]["votes"].values()),
                          answer["zones"]["2"]["n_items"], list(answer["zones"]["2"]["votes"].values()),
                          time.time() - t0)
        except Exception as exc:  # noqa: BLE001 - the requester gets the reason
            rospy.logerr("[LabelSeg] %s", exc)
            answer["error"] = str(exc)
        answer["seconds"] = round(time.time() - t0, 3)
        self._pub_result.publish(String(data=json.dumps(answer)))


if __name__ == "__main__":
    LabelSegNode()
    rospy.spin()
