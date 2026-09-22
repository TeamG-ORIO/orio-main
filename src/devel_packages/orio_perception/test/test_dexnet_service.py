#!/usr/bin/env python3
"""Exercise the real orchestrator DexNet path against a live planner service.

Needs a ROS master, the DexNet container (orio_bringup/docker/run_dexnet.sh) and the
perception venv. Only the local modules needing weights on disk are stubbed; the
orchestrator's own code is imported and called for real.

    source /opt/ros/noetic/setup.bash && source devel/setup.bash
    src/devel_packages/orio_perception/venv/bin/python \
        src/devel_packages/orio_perception/test/test_dexnet_service.py [--json OUT]
"""
import argparse
import json
import os
import sys
import types

import numpy as np
import rospy

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../.."))
SCRIPTS = os.path.join(REPO, "src/devel_packages/orio_perception/scripts")
SAMPLES = os.path.join(REPO,
                       "src/devel_packages/gqcnn/data/examples/clutter/phoxi/fcgqcnn")
# Berkeley's PhoXi calibration, matching the sample images.
PHOXI = dict(width=516, height=386, fx=552.5, fy=552.5, cx=255.5, cy=191.75)


def stub_weight_loading_modules():
    for name in ("grasp_solver", "opt_label_location"):
        sys.modules.setdefault(name, types.ModuleType(name))
    sys.modules["grasp_solver"].optimize_grasp_pose = lambda *a, **k: (None, "")
    sys.modules["opt_label_location"].opt_label_loc = lambda *a, **k: None


class FakeDepthMsg(object):
    encoding = "32FC1"


def build_node(module, log_path):
    node = module.CombinedPerceptionNode.__new__(module.CombinedPerceptionNode)
    node.dexnet_cfg = {
        "service": "/dexnet_grasp_planner/plan_grasp", "service_timeout": 30.0,
        "bin_depth_min": 0.60, "bin_depth_max": 1.20, "max_tilt_deg": 45.0,
        "offset_x": 0.0, "offset_y": 0.0,
    }
    node.camera_cfg = {}
    node.tf_pnp = np.eye(4)          # identity: camera frame == world for the test
    # The orchestrator's A/B attempt log is optional here and has been removed from
    # some revisions; set it up only if this build still has it.
    if hasattr(node, "_init_ab_log"):
        node.ab_log_path = log_path
        node._init_ab_log()
    node._dexnet_srv = node._connect_dexnet()
    node.pnp_intrinsics = module.o3d.camera.PinholeCameraIntrinsic(**PHOXI)
    # The samples are full frames; disable cropping.
    module.PNP_CROP_X1 = module.PNP_CROP_Y1 = 0
    module.PNP_CROP_X2, module.PNP_CROP_Y2 = PHOXI["width"], PHOXI["height"]
    return node


def load_sample(index):
    depth = np.load(os.path.join(SAMPLES, "depth_%d.npy" % index)).astype(np.float32)
    if depth.ndim == 3:
        depth = depth[:, :, 0]
    colour = np.zeros((depth.shape[0], depth.shape[1], 3), dtype=np.uint8)
    return colour, depth


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--json", help="write results here for the visualiser")
    ap.add_argument("--log", default="/tmp/dexnet_test_attempts.csv")
    args = ap.parse_args()

    stub_weight_loading_modules()
    sys.path.insert(0, SCRIPTS)
    import perception_control_combined_pass_through as module

    rospy.init_node("test_dexnet_service", anonymous=True)
    node = build_node(module, args.log)
    if node._dexnet_srv is None:
        print("FAIL: DexNet service unavailable. Start run_dexnet.sh first.")
        return 1

    results, failures = [], []
    print("planning on %d sample scenes" % 5)
    for i in range(5):
        colour, depth = load_sample(i)
        grasp, message = node._plan_grasp_dexnet(colour, depth, FakeDepthMsg())
        if hasattr(node, "_log_attempt"):
            node._log_attempt("dexnet", grasp is not None, grasp, 0.0, message)
        if grasp is None:
            print("  scene %d  REJECTED  %s" % (i, message))
            failures.append("scene %d" % i)
            continue
        rot = np.asarray(grasp["rotation"])
        orthonormal = np.allclose(rot.T @ rot, np.eye(3), atol=1e-9)
        if not orthonormal:
            failures.append("scene %d rotation not orthonormal" % i)
        print("  scene %d  q=%.4f  tilt=%5.1f deg  centre=(%.3f, %.3f, %.3f)"
              % (i, grasp["score"], grasp["tilt_deg"], *grasp["center"]))
        results.append({
            "scene": i, "q_value": grasp["score"], "tilt_deg": grasp["tilt_deg"],
            "center": list(map(float, grasp["center"])),
            "center_px": list(grasp["center_px"]),
            "normal": list(map(float, grasp["normal"])),
        })

    print("guard paths")
    empty = np.full((PHOXI["height"], PHOXI["width"]), 5.0, dtype=np.float32)
    grasp, message = node._plan_grasp_dexnet(
        np.zeros((PHOXI["height"], PHOXI["width"], 3), np.uint8), empty, FakeDepthMsg())
    ok = grasp is None and "Bin mask empty" in message
    print("  empty bin declined: %s" % ("PASS" if ok else "FAIL - " + str(message)))
    if not ok:
        failures.append("empty bin")

    node.dexnet_cfg["max_tilt_deg"] = 5.0
    _, depth = load_sample(0)
    grasp, message = node._plan_grasp_dexnet(
        np.zeros((PHOXI["height"], PHOXI["width"], 3), np.uint8), depth, FakeDepthMsg())
    ok = grasp is None and "tilt" in message.lower()
    print("  tilt gate at 5 deg rejects: %s" % ("PASS" if ok else "FAIL - " + str(message)))
    if not ok:
        failures.append("tilt gate")

    if args.json and results:
        with open(args.json, "w") as f:
            json.dump(results, f, indent=2)
        print("wrote %s" % args.json)

    print()
    if failures:
        print("%d FAILED: %s" % (len(failures), ", ".join(failures)))
        return 1
    print("all service checks passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
