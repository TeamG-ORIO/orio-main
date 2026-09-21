#!/usr/bin/env python3
"""Visualise a DexNet suction grasp in RViz — no arm involved.

Grabs one RGB-D frame from the Xtion, calls the DexNet planner service, and
publishes four things so they line up in RViz:

  * /dexnet_viz/cloud   sensor_msgs/PointCloud2  — the depth image as a 3D XYZRGB
                        cloud (the scene the policy saw)
  * /dexnet_viz/markers visualization_msgs/MarkerArray — a sphere at the grasp
                        contact point and an arrow along the suction approach
                        vector (the direction the cup travels onto the surface)
  * /dexnet_viz/pose    geometry_msgs/PoseStamped — the raw grasp pose
  * a static TF (default: world -> dexnet_camera) so RViz always has a valid
    Fixed Frame even with no live camera TF tree. The cloud, markers and pose
    all live in that one frame, so they are mutually consistent by construction.

Everything is published latched, so it stays put in RViz; the node keeps
spinning so you can leave RViz open. Nothing here moves the robot.

This host is 22.04 / ROS2 and has no host-side rospy, so run this INSIDE a ROS1
container (the orio/perception image carries rospy, custom_msgs, numpy and rviz).
The devel_packages tree is mounted at /home/ros_ws/src/devel_packages there.

Prereqs:
    - a ROS master (roscore)
    - the DexNet container running (orio_bringup/docker/run_dexnet.sh)
    - the Xtion driver publishing (or pass --depth-npy/--rgb-npy for an offline test)

Easiest: use the bundled launcher, which starts the container, this node and rviz:
    bash src/devel_packages/orio_bringup/docker/run_dexnet_viz.sh

Or by hand, inside a running ROS1 container:
    source /opt/ros/noetic/setup.bash && source /home/ros_ws/devel/setup.bash
    python3 /home/ros_ws/src/devel_packages/orio_perception/test/visualize_dexnet_rviz.py
    rviz -d /home/ros_ws/src/devel_packages/orio_perception/test/dexnet_grasp.rviz
"""
import argparse
import sys

import numpy as np
import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped, Point, TransformStamped
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from custom_msgs.srv import PlanDexnetGrasp

# Xtion topics, matching the orchestrator (perception_control_combined_pass_through.py).
RGB_TOPIC   = "/camera/rgb/image_raw"
DEPTH_TOPIC = "/camera/depth/image_raw"
INFO_TOPIC  = "/camera/rgb/camera_info"
DEPTH_UNIT_MM = 1000.0  # 16UC1 millimetres -> metres


# ── sensor_msgs/Image encode/decode without cv_bridge ────────────────────────
# cv_bridge needs host boost libs; the orchestrator and planner both avoid it, so
# we do too and stay dependency-light (numpy + rospy only).
def decode_image(msg, dtype, channels=1):
    arr = np.frombuffer(msg.data, dtype=dtype)
    shape = (msg.height, msg.width) if channels == 1 else (msg.height, msg.width, channels)
    return arr.reshape(shape)


def to_image_msg(arr, encoding, frame_id):
    msg = Image()
    msg.header.frame_id = frame_id
    msg.height, msg.width = arr.shape[0], arr.shape[1]
    msg.encoding = encoding
    msg.is_bigendian = 0
    msg.step = int(arr.strides[0])
    msg.data = arr.tobytes()
    return msg


def depth_to_metres(depth, encoding):
    """Return depth as float32 metres. 16UC1 is millimetres; 32FC1 already metres."""
    d = np.asarray(depth, dtype=np.float32)
    if encoding == "16UC1":
        d = d / DEPTH_UNIT_MM
    return d


def bin_mask(depth_m, lo, hi):
    """Depth-band segmask (mono8, 0/255), same idea as the orchestrator's _build_bin_mask.

    DexNet needs a mask so it does not rank the bin floor as a great target; a depth
    band is enough for a standalone test with no semantic segmentation.
    """
    m = (depth_m >= lo) & (depth_m <= hi) & np.isfinite(depth_m)
    return (m.astype(np.uint8) * 255)


# ── point cloud from depth + intrinsics ──────────────────────────────────────
def build_cloud(depth_m, rgb, K, frame_id, stride, dmin, dmax):
    """XYZRGB PointCloud2 by deprojecting depth with the same K sent to the planner.

    Using one set of intrinsics for both the cloud and the grasp deprojection keeps
    them mutually consistent, so the grasp marker sits on the cloud surface in RViz.
    """
    fx, fy, cx, cy = K[0], K[4], K[2], K[5]
    h, w = depth_m.shape
    vs, us = np.mgrid[0:h:stride, 0:w:stride]
    z = depth_m[vs, us]
    valid = np.isfinite(z) & (z >= dmin) & (z <= dmax)
    us, vs, z = us[valid], vs[valid], z[valid]
    x = (us - cx) / fx * z
    y = (vs - cy) / fy * z

    cols = rgb[vs, us] if rgb is not None else np.full((z.size, 3), 200, np.uint8)
    r, g, b = cols[:, 0].astype(np.uint32), cols[:, 1].astype(np.uint32), cols[:, 2].astype(np.uint32)
    rgb_packed = (r << 16) | (g << 8) | b

    pts = np.zeros(z.size, dtype=[("x", np.float32), ("y", np.float32),
                                  ("z", np.float32), ("rgb", np.uint32)])
    pts["x"], pts["y"], pts["z"], pts["rgb"] = x, y, z, rgb_packed

    cloud = PointCloud2()
    cloud.header.frame_id = frame_id
    cloud.header.stamp = rospy.Time.now()
    cloud.height = 1
    cloud.width = pts.size
    cloud.is_dense = True
    cloud.is_bigendian = False
    cloud.fields = [
        PointField("x", 0, PointField.FLOAT32, 1),
        PointField("y", 4, PointField.FLOAT32, 1),
        PointField("z", 8, PointField.FLOAT32, 1),
        PointField("rgb", 12, PointField.UINT32, 1),
    ]
    cloud.point_step = 16
    cloud.row_step = cloud.point_step * pts.size
    cloud.data = pts.tobytes()
    return cloud


# ── grasp markers ────────────────────────────────────────────────────────────
def grasp_markers(grasp, frame_id, arrow_len):
    """Sphere at the contact point + arrow along the approach vector.

    The suction approach axis is the X column of the rotation gqcnn returns (see
    SuctionPoint2D.pose() and the orchestrator), NOT Z. It points from the surface
    toward the camera; the cup travels along its negation, onto the object. The arrow
    is drawn tail-at-standoff, head-at-contact so it reads as the cup coming down.
    """
    p = grasp.pose.position
    q = grasp.pose.orientation
    contact = np.array([p.x, p.y, p.z])
    rot = quat_to_matrix(q.x, q.y, q.z, q.w)
    approach = rot[:, 0]
    approach /= np.linalg.norm(approach)
    if approach[2] > 0:          # make it point toward the camera (optical -Z)
        approach = -approach
    standoff = contact + approach * arrow_len   # a point backed off toward the camera

    markers = MarkerArray()

    arrow = Marker()
    arrow.header.frame_id = frame_id
    arrow.header.stamp = rospy.Time.now()
    arrow.ns = "dexnet_grasp"
    arrow.id = 0
    arrow.type = Marker.ARROW
    arrow.action = Marker.ADD
    arrow.scale.x = arrow_len * 0.08   # shaft diameter
    arrow.scale.y = arrow_len * 0.16   # head diameter
    arrow.scale.z = arrow_len * 0.25   # head length
    arrow.color = ColorRGBA(0.1, 0.9, 0.2, 1.0)
    arrow.points = [Point(*standoff), Point(*contact)]
    markers.markers.append(arrow)

    sphere = Marker()
    sphere.header.frame_id = frame_id
    sphere.header.stamp = rospy.Time.now()
    sphere.ns = "dexnet_grasp"
    sphere.id = 1
    sphere.type = Marker.SPHERE
    sphere.action = Marker.ADD
    sphere.pose.position = Point(*contact)
    sphere.pose.orientation.w = 1.0
    sphere.scale.x = sphere.scale.y = sphere.scale.z = arrow_len * 0.18
    sphere.color = ColorRGBA(1.0, 0.85, 0.0, 1.0)
    markers.markers.append(sphere)

    text = Marker()
    text.header.frame_id = frame_id
    text.header.stamp = rospy.Time.now()
    text.ns = "dexnet_grasp"
    text.id = 2
    text.type = Marker.TEXT_VIEW_FACING
    text.action = Marker.ADD
    text.pose.position = Point(contact[0], contact[1], contact[2] - arrow_len * 0.4)
    text.pose.orientation.w = 1.0
    text.scale.z = arrow_len * 0.22
    text.color = ColorRGBA(1.0, 1.0, 1.0, 1.0)
    text.text = "q=%.3f" % grasp.q_value
    markers.markers.append(text)
    return markers


def save_grasp_image(rgb, depth_m, grasp, out_path):
    """Write a PNG of the scene with the grasp drawn on it (suction point + approach arrow).

    Same visual language as test/visualize_grasps.py: a circle at the suction contact
    pixel and an arrow along the approach direction projected into the image plane,
    coloured red->green by grasp quality. Uses the live frame this node actually planned
    on, so the image matches what RViz shows. Returns True on success.
    """
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle

    cx, cy = float(grasp.center_px[0]), float(grasp.center_px[1])
    q = grasp.q_value

    # Approach direction in camera frame is the X column of the grasp rotation (see
    # grasp_markers); its (x, y) projected onto the image plane gives the 2D arrow.
    o = grasp.pose.orientation
    approach = quat_to_matrix(o.x, o.y, o.z, o.w)[:, 0]
    nx, ny = approach[0], approach[1]
    n = np.hypot(nx, ny)
    nx, ny = (nx / n, ny / n) if n > 1e-6 else (0.0, -1.0)

    colour = plt.cm.RdYlGn(float(np.clip((q - 0.5) / 0.5, 0.0, 1.0)))
    arrow_len = 70.0
    radius = 26.0

    fig, ax = plt.subplots(figsize=(8, 6))
    if rgb is not None:
        ax.imshow(rgb)
    else:
        valid = depth_m > 0
        vmin = np.percentile(depth_m[valid], 2) if valid.any() else 0.0
        vmax = np.percentile(depth_m[valid], 98) if valid.any() else 1.0
        ax.imshow(np.where(valid, depth_m, np.nan), cmap="viridis", vmin=vmin, vmax=vmax)

    # Arrow drawn tail-back, head-at-contact so it reads as the cup coming down.
    ax.arrow(cx - nx * arrow_len, cy - ny * arrow_len, nx * arrow_len, ny * arrow_len,
             width=arrow_len * 0.05, head_width=arrow_len * 0.24,
             length_includes_head=True, color=colour, ec="black", lw=0.5, zorder=3)
    ax.add_patch(Circle((cx, cy), radius, fill=False, ec=colour, lw=2.6, zorder=4))
    ax.add_patch(Circle((cx, cy), radius * 0.2, color=colour, zorder=5))
    ax.set_title("DexNet grasp   q = %.3f   px (%d, %d)   depth %.3f m"
                 % (q, int(cx), int(cy), grasp.pose.position.z), fontsize=11)
    ax.axis("off")
    fig.tight_layout()
    try:
        fig.savefig(out_path, dpi=125)
    except Exception as exc:  # noqa: BLE001 - writing the image must never crash the viz node
        print("WARN: could not write grasp image %s: %s" % (out_path, exc))
        plt.close(fig)
        return False
    plt.close(fig)
    return True


def quat_to_matrix(x, y, z, w):
    n = np.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ])


def load_frame(args):
    """Return (rgb HxWx3 uint8 or None, depth_metres HxW float32, camera_info K, frame_id)."""
    if args.depth_npy:
        depth = np.load(args.depth_npy).astype(np.float32)
        if depth.ndim == 3:
            depth = depth[:, :, 0]
        rgb = None
        if args.rgb_npy:
            rgb = np.load(args.rgb_npy).astype(np.uint8)[:, :, :3]
        h, w = depth.shape
        # Berkeley PhoXi defaults, only used offline when no camera_info is available.
        K = [552.5, 0, w / 2.0 - 0.5, 0, 552.5, h / 2.0 - 0.5, 0, 0, 1]
        return rgb, depth, K

    rospy.loginfo("Waiting for %s, %s, %s ...", INFO_TOPIC, DEPTH_TOPIC, RGB_TOPIC)
    info = rospy.wait_for_message(INFO_TOPIC, CameraInfo, timeout=args.timeout)
    depth_msg = rospy.wait_for_message(DEPTH_TOPIC, Image, timeout=args.timeout)
    rgb_msg = rospy.wait_for_message(RGB_TOPIC, Image, timeout=args.timeout)

    depth = depth_to_metres(decode_image(depth_msg, np.uint16 if depth_msg.encoding == "16UC1"
                                         else np.float32), depth_msg.encoding)
    rgb = None
    if rgb_msg.encoding in ("rgb8", "bgr8"):
        rgb = decode_image(rgb_msg, np.uint8, 3)
        if rgb_msg.encoding == "bgr8":
            rgb = rgb[:, :, ::-1]
    return np.ascontiguousarray(rgb) if rgb is not None else None, \
        np.ascontiguousarray(depth), list(info.K)


def broadcast_static_tf(parent, child):
    """Identity transform so RViz has a valid tree. The data all lives in `child`,
    so a fresh dedicated child frame never collides with the live driver's TF."""
    br = tf2_ros.StaticTransformBroadcaster()
    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = parent
    t.child_frame_id = child
    t.transform.rotation.w = 1.0
    br.sendTransform(t)
    return br  # keep a reference alive for the node's lifetime


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--service", default="/dexnet_grasp_planner/plan_grasp",
                    help="DexNet planner service name")
    ap.add_argument("--bin-depth-min", type=float, default=0.60,
                    help="segmask lower depth bound, metres")
    ap.add_argument("--bin-depth-max", type=float, default=1.20,
                    help="segmask upper depth bound, metres")
    ap.add_argument("--arrow-len", type=float, default=0.08,
                    help="approach arrow length in metres (visual only)")
    ap.add_argument("--stride", type=int, default=1,
                    help="point-cloud pixel stride (>1 downsamples for speed)")
    ap.add_argument("--cloud-max-depth", type=float, default=2.0,
                    help="clip cloud points beyond this depth, metres")
    ap.add_argument("--frame", default="dexnet_camera",
                    help="frame id for all published data (a dedicated frame by default, "
                         "so it never collides with the live camera's TF tree)")
    ap.add_argument("--tf-parent", default="world",
                    help="parent frame for the static TF (RViz Fixed Frame)")
    ap.add_argument("--no-static-tf", action="store_true",
                    help="do not broadcast the static TF (use if the frame already exists in TF)")
    ap.add_argument("--timeout", type=float, default=15.0,
                    help="seconds to wait for each camera message")
    ap.add_argument("--grasp-image", default="/tmp/dexnet_grasp.png",
                    help="write a PNG of the scene with the grasp drawn on it; "
                         "set empty to disable. The launcher (run_dexnet_viz.sh) points "
                         "this at logging/dexnet_viz/grasp_<timestamp>.png")
    ap.add_argument("--depth-npy", help="offline: load depth from a .npy instead of the Xtion")
    ap.add_argument("--rgb-npy", help="offline: optional colour .npy to match --depth-npy")
    args = ap.parse_args()

    rospy.init_node("visualize_dexnet_rviz", anonymous=True)
    frame = args.frame

    try:
        rgb, depth_m, K = load_frame(args)
    except rospy.ROSException as exc:
        print("FAIL: could not get a camera frame: %s" % exc)
        print("Is the Xtion driver publishing? Try --depth-npy for an offline test.")
        return 1

    print("frame: %s   depth %dx%d   range [%.3f, %.3f] m"
          % (frame, depth_m.shape[1], depth_m.shape[0],
             np.nanmin(depth_m[np.isfinite(depth_m) & (depth_m > 0)]) if np.any(depth_m > 0) else 0.0,
             np.nanmax(depth_m[np.isfinite(depth_m)]) if np.any(np.isfinite(depth_m)) else 0.0))

    segmask = bin_mask(depth_m, args.bin_depth_min, args.bin_depth_max)
    if not segmask.any():
        print("FAIL: bin mask empty - no depth in [%.2f, %.2f] m. Adjust --bin-depth-min/max."
              % (args.bin_depth_min, args.bin_depth_max))
        return 1

    rospy.loginfo("Waiting for DexNet service %s ...", args.service)
    try:
        rospy.wait_for_service(args.service, timeout=args.timeout)
    except rospy.ROSException:
        print("FAIL: DexNet service %s unavailable. Start run_dexnet.sh first." % args.service)
        return 1
    plan = rospy.ServiceProxy(args.service, PlanDexnetGrasp)

    colour_for_srv = np.ascontiguousarray(
        rgb[:, :, :3] if rgb is not None
        else np.zeros((depth_m.shape[0], depth_m.shape[1], 3), np.uint8), dtype=np.uint8)
    info_msg = CameraInfo()
    info_msg.header.frame_id = frame
    info_msg.height, info_msg.width = depth_m.shape
    info_msg.K = K

    try:
        resp = plan(
            to_image_msg(colour_for_srv, "rgb8", frame),
            to_image_msg(np.ascontiguousarray(depth_m, np.float32), "32FC1", frame),
            info_msg,
            to_image_msg(np.ascontiguousarray(segmask), "mono8", frame))
    except rospy.ServiceException as exc:
        print("FAIL: service call failed: %s" % exc)
        return 1

    g = resp.grasp
    print("grasp: q=%.4f  centre_px=(%.0f, %.0f)  pose=(%.3f, %.3f, %.3f) m  success=%s"
          % (g.q_value, g.center_px[0], g.center_px[1],
             g.pose.position.x, g.pose.position.y, g.pose.position.z, resp.success))
    if not resp.success:
        print("note: planner reported failure (%s) - visualising the returned pose anyway."
              % resp.message)

    if args.grasp_image and save_grasp_image(rgb, depth_m, g, args.grasp_image):
        print("wrote grasp image: %s" % args.grasp_image)

    _tf = None if args.no_static_tf else broadcast_static_tf(args.tf_parent, frame)
    fixed_frame = frame if args.no_static_tf else args.tf_parent

    cloud_pub = rospy.Publisher("/dexnet_viz/cloud", PointCloud2, queue_size=1, latch=True)
    marker_pub = rospy.Publisher("/dexnet_viz/markers", MarkerArray, queue_size=1, latch=True)
    pose_pub = rospy.Publisher("/dexnet_viz/pose", PoseStamped, queue_size=1, latch=True)

    cloud = build_cloud(depth_m, rgb, K, frame, max(1, args.stride), 0.05, args.cloud_max_depth)
    markers = grasp_markers(g, frame, args.arrow_len)
    pose = PoseStamped()
    pose.header.frame_id = frame
    pose.header.stamp = rospy.Time.now()
    pose.pose = g.pose

    cloud_pub.publish(cloud)
    marker_pub.publish(markers)
    pose_pub.publish(pose)

    print()
    print("Published %d cloud points + grasp markers on /dexnet_viz/* (latched)." % cloud.width)
    print("In RViz: set Fixed Frame to '%s', add PointCloud2 (/dexnet_viz/cloud)," % fixed_frame)
    print("MarkerArray (/dexnet_viz/markers) and Pose (/dexnet_viz/pose),")
    print("or: rviz -d .../orio_perception/test/dexnet_grasp.rviz  (Fixed Frame preset to 'world')")
    print("Ctrl-C to stop.")

    # Re-publish periodically so RViz instances started later still pick it up.
    rate = rospy.Rate(1.0)
    while not rospy.is_shutdown():
        cloud.header.stamp = rospy.Time.now()
        for m in markers.markers:
            m.header.stamp = cloud.header.stamp
        pose.header.stamp = cloud.header.stamp
        cloud_pub.publish(cloud)
        marker_pub.publish(markers)
        pose_pub.publish(pose)
        rate.sleep()
    return 0


if __name__ == "__main__":
    sys.exit(main())
