#!/usr/bin/env bash
# Start the label-zone segmentation node (orio_perception/scripts/label_seg_node.py) in its
# own container. Long-running, like run_ffs.sh: start it once at bring-up; it answers
# /label_seg/request on /label_seg/result, which perception's /compute_grasps_labelling
# and /scan_label_zones use to find the items in the label zones. It runs the grasp planner's RGB-D segmentation
# model in the suction planner's image (torch + rfdetr), whichever planner the run picks
# with. A ROS master must be up.
#
# Env:
#   ORIO_LABEL_SEG_CHECKPOINT  RGB-D segmentation weights (default: orio_grasping/runs/seg/
#                              rgbd_v3_deformables/checkpoint_best_total.pth, the one the
#                              label zones were tested with)
#   ORIO_LABEL_SEG_MODEL       RF-DETR size it was trained as (default RFDETRSegLarge)
#   ORIO_GRASPING              orio_grasping checkout (default: the repo's)
#   ORIO_LABEL_SEG_IMAGE (default orio/suction:latest), ORIO_LABEL_SEG_CONTAINER
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../../../.." && pwd)"
SCRIPTS="$REPO/src/devel_packages/orio_perception/scripts"
NAME="${ORIO_LABEL_SEG_CONTAINER:-orio_label_seg}"
IMAGE="${ORIO_LABEL_SEG_IMAGE:-orio/suction:latest}"
GRASPING="${ORIO_GRASPING:-$REPO/src/devel_packages/orio_grasping}"
CKPT="${ORIO_LABEL_SEG_CHECKPOINT:-$GRASPING/runs/seg/rgbd_v3_deformables/checkpoint_best_total.pth}"

if [ ! -f "$GRASPING/orio_grasping/rgbd_seg.py" ]; then
    echo "orio_grasping not found at $GRASPING (set ORIO_GRASPING)." >&2
    exit 1
fi
if [ ! -f "$CKPT" ]; then
    echo "No segmentation checkpoint at $CKPT (set ORIO_LABEL_SEG_CHECKPOINT)." >&2
    exit 1
fi
if ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
    echo "Image $IMAGE missing. Build it:" >&2
    echo "  docker build -t $IMAGE - < $HERE/Dockerfile.suction" >&2
    exit 1
fi
CKPT="$(readlink -f "$CKPT")"
echo "label-zone segmentation checkpoint: $CKPT"

docker rm -f "$NAME" >/dev/null 2>&1 || true

exec docker run -d --name "$NAME" --restart unless-stopped \
    --gpus all --ipc=host --net=host \
    -e ROS_MASTER_URI="${ROS_MASTER_URI:-http://localhost:11311}" \
    -e PYTHONUNBUFFERED=1 \
    -v "$GRASPING:/opt/orio-grasping:ro" \
    -v "$CKPT:/opt/seg_checkpoint.pth:ro" \
    -v "$SCRIPTS:/opt/orio_perception_scripts:ro" \
    "$IMAGE" \
    python /opt/orio_perception_scripts/label_seg_node.py \
        _checkpoint:=/opt/seg_checkpoint.pth \
        _model:="${ORIO_LABEL_SEG_MODEL:-RFDETRSegLarge}" \
        _grasping_repo:=/opt/orio-grasping
