#!/usr/bin/env bash
# Start the DexNet grasp planner on a host that has NO ROS Noetic installed.
#
# The committed run_dexnet.sh bind-mounts rospy from the host's
# /opt/ros/noetic/lib/python3/dist-packages. That works on a 20.04 host with
# ros-noetic installed, but this machine is 22.04/ROS2 (no host Noetic), so that
# mount is empty and the container crash-loops on `import rospy`.
#
# This variant provisions the ROS1 Python packages by extracting them once from the
# orio_docker image (which carries a full Noetic) into a host cache, then mounts that
# cache where the dexnet image's PYTHONPATH expects it. Everything else matches
# run_dexnet.sh. Use this instead of run_dexnet.sh on this host.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)"
NAME="${DEXNET_CONTAINER_NAME:-orio_dexnet}"
IMAGE="${DEXNET_IMAGE:-orio/dexnet:latest}"
ROS1_IMAGE="${ORIO_ROS1_IMAGE:-orio_docker:latest}"   # source of the Noetic py packages
MIN_Q="${DEXNET_MIN_Q_VALUE:-0.30}"
CACHE="${ORIO_NOETIC_PY_CACHE:-$HOME/.cache/orio/noetic_dist_packages}"

if [ ! -d "$REPO/devel/lib/python3/dist-packages/custom_msgs" ]; then
    echo "custom_msgs not built. Run catkin_make first." >&2
    exit 1
fi

if [ ! -f "$REPO/src/devel_packages/gqcnn/models/FC-GQCNN-4.0-SUCTION/model.ckpt.index" ]; then
    echo "Model weights missing. Run orio_perception/scripts/download_dexnet_model.sh" >&2
    exit 1
fi

# Provision the Noetic Python packages once (rospy, genpy, std/sensor/geometry_msgs, …).
if [ ! -d "$CACHE/rospy" ]; then
    echo "Provisioning ROS1 Python packages from $ROS1_IMAGE into $CACHE ..."
    mkdir -p "$CACHE"
    cid="$(docker create "$ROS1_IMAGE")"
    docker cp "$cid":/opt/ros/noetic/lib/python3/dist-packages/. "$CACHE"/
    docker rm "$cid" >/dev/null
    echo "Provisioned $(ls "$CACHE" | wc -l) packages."
fi

docker rm -f "$NAME" >/dev/null 2>&1 || true

exec docker run -d --name "$NAME" --restart unless-stopped \
    --gpus all --ipc=host --net=host \
    -e ROS_MASTER_URI="${ROS_MASTER_URI:-http://localhost:11311}" \
    -e ORIO_LOGGING="${ORIO_LOGGING:-1}" \
    -v "$CACHE:/opt/ros/noetic/lib/python3/dist-packages:ro" \
    -v "$REPO/devel/lib/python3/dist-packages:/catkin_ws/devel/lib/python3/dist-packages:ro" \
    -v "$REPO/src/devel_packages/orio_core:$REPO/src/devel_packages/orio_core:ro" \
    -v "$REPO/src/devel_packages/gqcnn/models:/opt/gqcnn/models:ro" \
    "$IMAGE" \
    python /opt/dexnet_grasp_planner.py _min_q_value:="$MIN_Q"
