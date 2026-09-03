#!/usr/bin/env python
"""DexNet 4.0 suction grasp planner service.

Advertises ~plan_grasp (custom_msgs/PlanDexnetGrasp). Runs inside the NVIDIA TF1
container; ROS clients call it normally over the shared master. The interface is
custom_msgs rather than gqcnn's own srv because importing gqcnn.srv pulls in gqcnn's
Python package, and with it TensorFlow, which host-side clients do not have.
"""

import os

import numpy as np
import rospy

from autolab_core import CameraIntrinsics, DepthImage, BinaryImage, ColorImage, RgbdImage, YamlConfig
from gqcnn.grasping import FullyConvolutionalGraspingPolicySuction, RgbdImageState
from custom_msgs.srv import PlanDexnetGrasp, PlanDexnetGraspResponse
from custom_msgs.msg import DexnetGrasp

import gqcnn.model.tf.fc_network_tf as _fcmod

# FCGQCNNTF._parse_config overrides only im_width/im_height, so _batch_size keeps the
# model's *training* value of 64. _predict then feeds a full 64-slot buffer to the
# session with one real image in it, computing 64x the work and discarding 63/64.
# Patch _parse_config to honour batch_size; must be applied before the policy is built,
# since initialize_network sizes the placeholder from _batch_size.
_orig_parse_config = _fcmod.FCGQCNNTF._parse_config


def _parse_config_with_batch_size(self, cfg):
    _orig_parse_config(self, cfg)
    if "batch_size" in cfg:
        self._batch_size = cfg["batch_size"]


_fcmod.FCGQCNNTF._parse_config = _parse_config_with_batch_size


class DexnetGraspPlanner(object):
    def __init__(self):
        rospy.init_node("dexnet_grasp_planner")

        model_dir = rospy.get_param("~model_dir",
                                    "/opt/gqcnn/models/FC-GQCNN-4.0-SUCTION")
        config_file = rospy.get_param(
            "~config_file", "/opt/gqcnn/cfg/examples/fc_gqcnn_suction.yaml")

        cfg = YamlConfig(config_file)
        self._inpaint_rescale_factor = cfg["inpaint_rescale_factor"]
        policy_cfg = cfg["policy"]
        policy_cfg["metric"]["gqcnn_model"] = model_dir
        policy_cfg["metric"]["fully_conv_gqcnn_config"]["batch_size"] = 1

        rospy.loginfo("Loading FC-GQCNN-4.0-SUCTION from %s", model_dir)
        self._policy = FullyConvolutionalGraspingPolicySuction(policy_cfg)

        net = self._policy._grasp_quality_fn._fcgqcnn
        rospy.loginfo("Inference batch size: %d", net._batch_size)
        if net._batch_size != 1:
            rospy.logwarn("batch_size is %d, not 1 - the _parse_config patch did not "
                          "take effect and inference will be ~30x slower",
                          net._batch_size)

        self._min_q_value = float(rospy.get_param("~min_q_value", 0.0))

        self._srv = rospy.Service("~plan_grasp", PlanDexnetGrasp, self._handle)
        rospy.loginfo("DexNet grasp planner ready on %s (min_q_value=%.3f)",
                      rospy.resolve_name("~plan_grasp"), self._min_q_value)

    @staticmethod
    def _decode(msg, dtype, channels=1):
        """Decode a sensor_msgs/Image without cv_bridge, which needs host boost libs."""
        arr = np.frombuffer(msg.data, dtype=dtype)
        shape = (msg.height, msg.width) if channels == 1 else (msg.height, msg.width,
                                                               channels)
        return arr.reshape(shape)

    def _handle(self, req):
        intr = CameraIntrinsics(
            frame=req.camera_info.header.frame_id or "camera",
            fx=req.camera_info.K[0], fy=req.camera_info.K[4],
            cx=req.camera_info.K[2], cy=req.camera_info.K[5],
            height=req.camera_info.height, width=req.camera_info.width)

        # Depth must arrive as 32FC1 in metres; the orchestrator converts.
        if req.depth_image.encoding not in ("32FC1", "passthrough"):
            raise rospy.ServiceException(
                "depth_image encoding must be 32FC1, got %s" % req.depth_image.encoding)
        depth_arr = self._decode(req.depth_image, np.float32).astype(np.float32)
        depth_im = DepthImage(depth_arr[:, :, None], frame=intr.frame)

        color_arr = self._decode(req.color_image, np.uint8, channels=3)
        color_im = ColorImage(np.ascontiguousarray(color_arr), frame=intr.frame)

        segmask_arr = self._decode(req.segmask, np.uint8)
        segmask = BinaryImage(np.ascontiguousarray(segmask_arr), frame=intr.frame)

        # Both steps are mandatory: without inpainting q_value collapses to ~0.
        segmask = segmask.mask_binary(depth_im.invalid_pixel_mask().inverse())
        depth_im = depth_im.inpaint(rescale_factor=self._inpaint_rescale_factor)

        state = RgbdImageState(RgbdImage.from_color_and_depth(color_im, depth_im),
                               intr, segmask=segmask)

        start = rospy.Time.now()
        action = self._policy(state)
        elapsed = (rospy.Time.now() - start).to_sec()

        grasp = action.grasp
        pose = grasp.pose()
        msg = DexnetGrasp()
        msg.q_value = float(action.q_value)
        msg.grasp_type = DexnetGrasp.SUCTION
        msg.center_px = [float(grasp.center[0]), float(grasp.center[1])]
        msg.angle = float(getattr(grasp, "angle", 0.0))
        msg.depth = float(grasp.depth)
        msg.pose.position.x = pose.translation[0]
        msg.pose.position.y = pose.translation[1]
        msg.pose.position.z = pose.translation[2]
        quat = pose.quaternion  # (w, x, y, z)
        msg.pose.orientation.w = quat[0]
        msg.pose.orientation.x = quat[1]
        msg.pose.orientation.y = quat[2]
        msg.pose.orientation.z = quat[3]

        if msg.q_value < self._min_q_value:
            rospy.logwarn("Rejected grasp: q=%.4f < min_q_value=%.3f (%.3fs)",
                          msg.q_value, self._min_q_value, elapsed)
            return PlanDexnetGraspResponse(
                msg, False,
                "q_value %.4f below threshold %.3f" % (msg.q_value, self._min_q_value))

        rospy.loginfo("Planned suction grasp q=%.4f at px=(%.0f, %.0f) in %.3fs",
                      msg.q_value, msg.center_px[0], msg.center_px[1], elapsed)
        return PlanDexnetGraspResponse(msg, True, "")


if __name__ == "__main__":
    DexnetGraspPlanner()
    rospy.spin()
