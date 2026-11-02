#!/usr/bin/env bash
# Start the labelling-test perception node (labelling_perception_node.py) in the
# orio/perception container, detached. The demo's run_perception.sh with two differences:
# this directory is mounted too (the node lives here, not in orio_perception), and the
# container is named orio_lbl_perception so it never replaces the demo's perception.
# A ROS master and the ZED (and, for ffs depth, the /ffs/compute node) must be up.
#
#   bash run_lbl_perception.sh [ffs|zed]      # depth source (default ffs)
#
# Env: ORIO_LBL_IMAGE_DIR  where the node saves its debug images, as a HOST path under the
#                          repo's logging/ (mounted at /home/ros_ws/logging). Optional.
#      ORIO_LBL_PROMPT, ORIO_LBL_BOX_THRESHOLD   GroundingDINO prompt / lowest score kept
#                          (default: the demo's "bright box on black background" / 0.25)
#      ORIO_LBL_PERCEPTION_CONTAINER, ORIO_PERCEPTION_IMAGE
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../../.." && pwd)"
NAME="${ORIO_LBL_PERCEPTION_CONTAINER:-orio_lbl_perception}"
IMAGE="${ORIO_PERCEPTION_IMAGE:-orio/perception:latest}"
DEPTH_SOURCE="${1:-ffs}"
case "$DEPTH_SOURCE" in ffs|zed) ;; *) echo "depth source must be ffs or zed, got '$DEPTH_SOURCE'" >&2; exit 2 ;; esac

PERC="$REPO/src/devel_packages/orio_perception"
if [ ! -f "$PERC/weights/groundingdino_swint_ogc.pth" ] || [ ! -f "$PERC/SAM_weights/sam_vit_b_01ec64.pth" ]; then
    echo "Perception weights missing under orio_perception/{weights,SAM_weights} (see orio_perception/README.md)." >&2
    exit 1
fi
if ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
    echo "Image $IMAGE missing: build it as for the demo (orio_bringup/docker/Dockerfile.perception)." >&2
    exit 1
fi

# The image dir is given as a host path; translate it to the container's view of logging/.
IMAGE_DIR_CONT=""
if [ -n "${ORIO_LBL_IMAGE_DIR:-}" ]; then
    case "$ORIO_LBL_IMAGE_DIR" in
        "$REPO/logging"/*) IMAGE_DIR_CONT="/home/ros_ws/logging/${ORIO_LBL_IMAGE_DIR#"$REPO/logging/"}" ;;
        *) echo "ORIO_LBL_IMAGE_DIR must be under $REPO/logging, got '$ORIO_LBL_IMAGE_DIR'" >&2; exit 2 ;;
    esac
fi

docker rm -f "$NAME" >/dev/null 2>&1 || true

# Detection overrides as private params; an unset one is left out so the node keeps the
# demo's value.
NODE_ARGS=()
[ -n "${ORIO_LBL_PROMPT:-}" ] && NODE_ARGS+=("_text_prompt:=$ORIO_LBL_PROMPT")
[ -n "${ORIO_LBL_BOX_THRESHOLD:-}" ] && NODE_ARGS+=("_box_threshold:=$ORIO_LBL_BOX_THRESHOLD")

exec docker run -d --name "$NAME" \
    --gpus all --ipc=host --net=host \
    --env="ROS_MASTER_URI=${ROS_MASTER_URI:-http://localhost:11311}" \
    --env="ORIO_REPO=/home/ros_ws" \
    --env="ORIO_PERCEPTION_ASSETS=/home/ros_ws/src/devel_packages/orio_perception" \
    --env="ORIO_PERCEPTION_SCRIPTS=/home/ros_ws/src/devel_packages/orio_perception/scripts" \
    --env="ORIO_GROUNDINGDINO_DIR=/opt/GroundingDINO" \
    --env="ORIO_LBL_IMAGE_DIR=$IMAGE_DIR_CONT" \
    --env="ORIO_LOGGING=0" \
    --env="PYTHONUNBUFFERED=1" \
    --volume="$REPO/src/devel_packages:/home/ros_ws/src/devel_packages" \
    --volume="$REPO/research:/home/ros_ws/research" \
    --volume="$REPO/logging:/home/ros_ws/logging" \
    "$IMAGE" \
    bash -lc 'source /opt/ros/noetic/setup.bash \
        && source /home/ros_ws/devel/setup.bash \
        && cd /home/ros_ws/src/devel_packages/orio_perception/scripts \
        && exec python3 /home/ros_ws/research/labelling_arm_test/perception/labelling_perception_node.py "$@"' \
    _ "_depth_source:=$DEPTH_SOURCE" "${NODE_ARGS[@]}"
