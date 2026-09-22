#!/usr/bin/env bash
# Start the DexNet grasp planner container. Long-running: the TF graph load takes ~25 s,
# so start it once at bringup rather than per pick.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)"
NAME="${DEXNET_CONTAINER_NAME:-orio_dexnet}"
IMAGE="${DEXNET_IMAGE:-orio/dexnet:latest}"
MIN_Q="${DEXNET_MIN_Q_VALUE:-0.30}"
# Input downscale before inference; lifts q_value at this camera height (see the planner).
RESCALE="${DEXNET_RESCALE:-0.5}"

if [ ! -d "$REPO/devel/lib/python3/dist-packages/custom_msgs" ]; then
    echo "custom_msgs not built. Run catkin_make first." >&2
    exit 1
fi

if [ ! -f "$REPO/src/devel_packages/gqcnn/models/FC-GQCNN-4.0-SUCTION/model.ckpt.index" ]; then
    echo "Model weights missing. Run orio_perception/scripts/download_dexnet_model.sh" >&2
    exit 1
fi

# rospy/std_msgs/sensor_msgs/geometry_msgs are baked into the orio/dexnet image
# (Dockerfile.dexnet installs ros-noetic-*), so no ROS Noetic mount is needed. The image
# is self-contained; only repo artefacts (custom_msgs, orio_core, model weights) mount in.
docker rm -f "$NAME" >/dev/null 2>&1 || true

exec docker run -d --name "$NAME" --restart unless-stopped \
    --gpus all --ipc=host --net=host \
    -e ROS_MASTER_URI="${ROS_MASTER_URI:-http://localhost:11311}" \
    -e ORIO_LOGGING="${ORIO_LOGGING:-1}" \
    -v "$REPO/devel/lib/python3/dist-packages:/catkin_ws/devel/lib/python3/dist-packages:ro" \
    -v "$REPO/src/devel_packages/orio_core:$REPO/src/devel_packages/orio_core:ro" \
    -v "$REPO/src/devel_packages/gqcnn/models:/opt/gqcnn/models:ro" \
    -v "$REPO/src/devel_packages/orio_bringup/docker/dexnet_grasp_planner.py:/opt/dexnet_grasp_planner.py:ro" \
    "$IMAGE" \
    python /opt/dexnet_grasp_planner.py _min_q_value:="$MIN_Q" _rescale:="$RESCALE"
