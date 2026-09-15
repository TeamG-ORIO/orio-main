#!/usr/bin/env bash
# Start the DexNet grasp planner container. Long-running: the TF graph load takes ~25 s,
# so start it once at bringup rather than per pick.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)"
NAME="${DEXNET_CONTAINER_NAME:-orio_dexnet}"
IMAGE="${DEXNET_IMAGE:-orio/dexnet:latest}"
MIN_Q="${DEXNET_MIN_Q_VALUE:-0.30}"

if [ ! -d "$REPO/devel/lib/python3/dist-packages/custom_msgs" ]; then
    echo "custom_msgs not built. Run catkin_make first." >&2
    exit 1
fi

if [ ! -f "$REPO/src/devel_packages/gqcnn/models/FC-GQCNN-4.0-SUCTION/model.ckpt.index" ]; then
    echo "Model weights missing. Run orio_perception/scripts/download_dexnet_model.sh" >&2
    exit 1
fi

docker rm -f "$NAME" >/dev/null 2>&1 || true

exec docker run -d --name "$NAME" --restart unless-stopped \
    --gpus all --ipc=host --net=host \
    -e ROS_MASTER_URI="${ROS_MASTER_URI:-http://localhost:11311}" \
    -e ORIO_LOGGING="${ORIO_LOGGING:-1}" \
    -v /opt/ros/noetic/lib/python3/dist-packages:/opt/ros/noetic/lib/python3/dist-packages:ro \
    -v "$REPO/devel/lib/python3/dist-packages:/catkin_ws/devel/lib/python3/dist-packages:ro" \
    -v "$REPO/src/devel_packages/orio_core:$REPO/src/devel_packages/orio_core:ro" \
    -v "$REPO/src/devel_packages/gqcnn/models:/opt/gqcnn/models:ro" \
    "$IMAGE" \
    python /opt/dexnet_grasp_planner.py _min_q_value:="$MIN_Q"
