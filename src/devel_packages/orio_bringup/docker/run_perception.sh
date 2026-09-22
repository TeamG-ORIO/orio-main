#!/usr/bin/env bash
# Start the ORIO perception node inside the ROS1 perception container (Option B).
# The host is 22.04/ROS2 and has no rospy, so perception runs here instead of on the
# host venv. A ROS master must be up before this starts (launch_demo gates on it).
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)"
NAME="${ORIO_PERCEPTION_CONTAINER:-orio_perception}"
IMAGE="${ORIO_PERCEPTION_IMAGE:-orio/perception:latest}"

# Optional grasp backend (dexnet|classical), via $1 or ORIO_GRASP_BACKEND. When set,
# perception starts through perception.launch (which loads grasp.yaml and the dexnet
# sub-config) with grasp_backend overridden; otherwise it runs the node directly, the
# historic classical path. dexnet needs this because running the node bare loads no
# params, so ~dexnet stays empty and the planner is never wired up.
BACKEND="${1:-${ORIO_GRASP_BACKEND:-}}"

if [ ! -f "$REPO/src/devel_packages/orio_perception/weights/groundingdino_swint_ogc.pth" ] || \
   [ ! -f "$REPO/src/devel_packages/orio_perception/SAM_weights/sam_vit_b_01ec64.pth" ]; then
    echo "Perception weights missing under orio_perception/{weights,SAM_weights}." >&2
    echo "  Fetch: bash src/devel_packages/orio_perception/scripts/download_dexnet_model.sh is DexNet;" >&2
    echo "  GroundingDINO + SAM weights are separate (see orio_perception/README.md)." >&2
    exit 1
fi

xhost +local:root >/dev/null 2>&1 || true
docker rm -f "$NAME" >/dev/null 2>&1 || true

# The image already carries custom_msgs (built into orio_docker's /home/ros_ws/devel)
# and the groundingdino package (installed at /opt/GroundingDINO). Only the perception
# source, weights, data and logging dirs come from the host via the devel_packages mount.
# TTY: -it is used interactively, but a merged single-terminal launcher pipes our stdout,
# where -it breaks capture; set ORIO_NO_TTY=1 there to drop it.
if [ -n "${ORIO_NO_TTY:-}" ]; then TTY_FLAGS=""; else TTY_FLAGS="-it"; fi
exec docker run --rm $TTY_FLAGS --name "$NAME" \
    --gpus all --ipc=host --net=host \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --env="XAUTHORITY=${XAUTH:-}" \
    --env="ROS_MASTER_URI=${ROS_MASTER_URI:-http://localhost:11311}" \
    --env="ORIO_REPO=/home/ros_ws" \
    --env="ORIO_PERCEPTION_ASSETS=/home/ros_ws/src/devel_packages/orio_perception" \
    --env="ORIO_GROUNDINGDINO_DIR=/opt/GroundingDINO" \
    --env="ORIO_LOGGING=${ORIO_LOGGING:-1}" \
    --env="ORIO_RUN_DIR=${ORIO_RUN_DIR:-}" \
    --env="ORIO_RUN_ID=${ORIO_RUN_ID:-}" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    ${XAUTH:+--volume="$XAUTH:$XAUTH"} \
    --volume="$REPO/src/devel_packages:/home/ros_ws/src/devel_packages" \
    --volume="$REPO/data:/home/ros_ws/data" \
    --volume="$REPO/logging:/home/ros_ws/logging" \
    ${BACKEND:+--env="ORIO_GRASP_BACKEND=$BACKEND"} \
    "$IMAGE" \
    bash -lc 'source /opt/ros/noetic/setup.bash \
        && source /home/ros_ws/devel/setup.bash \
        && if [ -n "${ORIO_GRASP_BACKEND:-}" ]; then \
               export PYTHONPATH=/home/ros_ws/src/devel_packages/orio_perception/scripts:$PYTHONPATH; \
               exec roslaunch orio_perception perception.launch grasp_backend:="$ORIO_GRASP_BACKEND"; \
           else \
               cd /home/ros_ws/src/devel_packages/orio_perception/scripts \
               && exec python3 perception_control_combined_pass_through.py; \
           fi'
