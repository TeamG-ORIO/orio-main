#!/usr/bin/env bash
# Start the Fast-FoundationStereo depth node (ffs_depth_node.py) in its own container.
# Long-running, like orio_bringup/docker/run_suction.sh: start it once at bring-up, it
# serves /ffs/compute and publishes /ffs/depth for the ZED-M. A ROS master must be up.
#
# Env:
#   ORIO_FFS_DIR         fast-foundationstereo checkout with the team's weights/ folder
#                        (default /home/orio/newest/zed/fast-foundationstereo)
#   ORIO_FFS_CHECKPOINT  .pth to load (default: $ORIO_FFS_DIR/weights/23-36-37/
#                        model_best_bp2_serialize.pth, the HF c-fast-foundationstereo release)
#   ORIO_FFS_SCALE       image scale before inference (default 0.5, see the node)
#   ORIO_FFS_ITERS       refinement iterations (default 8)
#   ORIO_FFS_RATE        continuous computation Hz, 0 = only when /ffs/compute is called (default 0)
#   ORIO_FFS_IMAGE, ORIO_FFS_CONTAINER
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TEST_DIR="$(dirname "$HERE")"                 # research/labelling_arm_test
NAME="${ORIO_FFS_CONTAINER:-orio_ffs}"
IMAGE="${ORIO_FFS_IMAGE:-orio/ffs-ros:latest}"
FFS_DIR="${ORIO_FFS_DIR:-/home/orio/newest/zed/fast-foundationstereo}"
CKPT="${ORIO_FFS_CHECKPOINT:-$FFS_DIR/weights/23-36-37/model_best_bp2_serialize.pth}"

if [ ! -f "$FFS_DIR/core/utils/utils.py" ]; then
    echo "fast-foundationstereo not found at $FFS_DIR (set ORIO_FFS_DIR)." >&2
    echo "  git clone https://github.com/nvlabs/fast-foundationstereo.git $FFS_DIR" >&2
    exit 1
fi
if [ ! -f "$CKPT" ]; then
    echo "No checkpoint at $CKPT (set ORIO_FFS_CHECKPOINT)." >&2
    echo "  See reference/ORIO_QUICKSTART.md for where the weights come from." >&2
    exit 1
fi
if ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
    echo "Image $IMAGE missing. Build it (needs the \`ffs\` image first, see README.md):" >&2
    echo "  docker build -t $IMAGE - < $HERE/Dockerfile" >&2
    exit 1
fi
CKPT="$(readlink -f "$CKPT")"
echo "Fast-FoundationStereo checkpoint: $CKPT"

docker rm -f "$NAME" >/dev/null 2>&1 || true

exec docker run -d --name "$NAME" --restart unless-stopped \
    --gpus all --ipc=host --net=host \
    -e NVIDIA_DISABLE_REQUIRE=1 \
    -e ROS_MASTER_URI="${ROS_MASTER_URI:-http://localhost:11311}" \
    -e ORIO_FFS_DIR=/opt/fast-foundationstereo \
    -v "$FFS_DIR:/opt/fast-foundationstereo:ro" \
    -v "$CKPT:/opt/ffs_checkpoint.pth:ro" \
    -v "$TEST_DIR:/opt/labelling_arm_test:ro" \
    "$IMAGE" \
    /opt/conda/envs/my/bin/python /opt/labelling_arm_test/ffs/ffs_depth_node.py \
        _model:=/opt/ffs_checkpoint.pth \
        _scale:="${ORIO_FFS_SCALE:-0.5}" \
        _valid_iters:="${ORIO_FFS_ITERS:-8}" \
        _rate:="${ORIO_FFS_RATE:-0}"
