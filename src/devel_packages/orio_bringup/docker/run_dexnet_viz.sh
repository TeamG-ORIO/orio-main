#!/usr/bin/env bash
# Visualise a DexNet suction grasp in RViz — no arm involved.
#
# This host is 22.04/ROS2 with no host-side rospy, so the ROS1 node and rviz run inside
# the orio/perception image (it carries rospy, custom_msgs, numpy and rviz). This script
# starts a throwaway container on the host network, runs visualize_dexnet_rviz.py, and
# opens rviz with the bundled config. Closing rviz stops the container.
#
# Prereqs: a ROS master (roscore), the DexNet planner container
# (run_dexnet.sh), and the Xtion driver publishing — or pass --depth-npy for an
# offline test with the bundled Berkeley samples.
#
# Usage:
#   bash run_dexnet_viz.sh                         # live Xtion
#   bash run_dexnet_viz.sh --bin-depth-min 0.4 --bin-depth-max 1.4
#   bash run_dexnet_viz.sh --depth-npy \
#       /home/ros_ws/src/devel_packages/gqcnn/data/examples/clutter/phoxi/fcgqcnn/depth_0.npy
# Any extra args are passed straight through to visualize_dexnet_rviz.py.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)"
NAME="${DEXNET_VIZ_CONTAINER:-orio_dexnet_viz}"
IMAGE="${ORIO_PERCEPTION_IMAGE:-orio/perception:latest}"

NODE=/home/ros_ws/src/devel_packages/orio_perception/test/visualize_dexnet_rviz.py
RVIZ_CFG=/home/ros_ws/src/devel_packages/orio_perception/test/dexnet_grasp.rviz

# Where the annotated grasp PNG lands. Saved under logging/ (the repo's run-artifact
# home, gitignored) with a timestamped filename so repeat runs accumulate. The dir is
# mounted into the throwaway container so the PNG survives it exiting.
OUT_DIR="${DEXNET_VIZ_OUT_DIR:-$REPO/logging/dexnet_viz}"
mkdir -p "$OUT_DIR"
GRASP_IMAGE_NAME="grasp_$(date +%Y%m%d_%H%M%S).png"
GRASP_IMAGE_HOST="$OUT_DIR/$GRASP_IMAGE_NAME"
GRASP_IMAGE_CONT="/home/ros_ws/dexnet_viz_out/$GRASP_IMAGE_NAME"

if [ ! -d "$REPO/devel/lib/python3/dist-packages/custom_msgs" ]; then
    echo "custom_msgs not built. Run catkin_make first." >&2
    exit 1
fi

xhost +local:root >/dev/null 2>&1 || true
docker rm -f "$NAME" >/dev/null 2>&1 || true

# Node runs in the background; rviz in the foreground. When rviz closes the container
# (--rm) exits and takes the node with it. The host-built custom_msgs is mounted and put
# ahead on PYTHONPATH so PlanDexnetGrasp is always the current definition.
docker run --rm -it --name "$NAME" \
    --gpus all --ipc=host --net=host \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --env="XAUTHORITY=${XAUTH:-}" \
    --env="ROS_MASTER_URI=${ROS_MASTER_URI:-http://localhost:11311}" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    ${XAUTH:+--volume="$XAUTH:$XAUTH"} \
    --volume="$REPO/src/devel_packages:/home/ros_ws/src/devel_packages" \
    --volume="$REPO/devel/lib/python3/dist-packages:/home/ros_ws/devel_host:ro" \
    --volume="$OUT_DIR:/home/ros_ws/dexnet_viz_out:rw" \
    "$IMAGE" \
    bash -lc "source /opt/ros/noetic/setup.bash \
        && source /home/ros_ws/devel/setup.bash \
        && export PYTHONPATH=/home/ros_ws/devel_host:\$PYTHONPATH \
        && python3 $NODE --grasp-image $GRASP_IMAGE_CONT $* & \
        NODE_PID=\$!; \
        rviz -d $RVIZ_CFG; \
        kill \$NODE_PID 2>/dev/null || true"

echo "Grasp image (if planning succeeded): $GRASP_IMAGE_HOST"
