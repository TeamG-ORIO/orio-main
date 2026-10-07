#!/usr/bin/env bash
# Start the suction grasp planner container (the orio-grasping network, in place of DexNet).
# Long-running like run_dexnet.sh: start it once at bringup. Serves
# /suction_grasp_planner/plan_grasp with the same custom_msgs/PlanDexnetGrasp contract.
#
# Env:
#   ORIO_GRASPING        orio-grasping repo (default ~/orio-grasping): code, config.toml, calibration
#   SUCTION_CHECKPOINT   model checkpoint (default: the newest $ORIO_GRASPING/runs/*/best.pt)
#   SUCTION_MIN_SCORE    lowest seal x hold x access the planner accepts (default 0.30)
#   SUCTION_ROT180       1 (default): driver frames are upside down relative to the calibration
#   SUCTION_IMAGE, SUCTION_CONTAINER_NAME
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)"
NAME="${SUCTION_CONTAINER_NAME:-orio_suction}"
IMAGE="${SUCTION_IMAGE:-orio/suction:latest}"
GRASPING="${ORIO_GRASPING:-$HOME/orio-grasping}"
MIN_SCORE="${SUCTION_MIN_SCORE:-0.30}"
ROT180="${SUCTION_ROT180:-1}"

if [ ! -d "$REPO/devel/lib/python3/dist-packages/custom_msgs" ]; then
    echo "custom_msgs not built. Run catkin_make first." >&2
    exit 1
fi
if [ ! -f "$GRASPING/orio_grasping/model.py" ]; then
    echo "orio-grasping repo not found at $GRASPING (set ORIO_GRASPING)." >&2
    exit 1
fi
CKPT="${SUCTION_CHECKPOINT:-$(ls -t "$GRASPING"/runs/*/best.pt 2>/dev/null | head -1)}"
if [ -z "$CKPT" ] || [ ! -f "$CKPT" ]; then
    echo "No checkpoint: set SUCTION_CHECKPOINT or train one into $GRASPING/runs/." >&2
    exit 1
fi
CKPT="$(readlink -f "$CKPT")"
if ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
    echo "Image $IMAGE missing. Build it:" >&2
    echo "  docker build -t $IMAGE - < $REPO/src/devel_packages/orio_bringup/docker/Dockerfile.suction" >&2
    exit 1
fi
echo "suction planner checkpoint: $CKPT ($(date -r "$CKPT" '+saved %Y-%m-%d %H:%M'))"
[ "$ROT180" = "1" ] && ROT180_PARAM=true || ROT180_PARAM=false

docker rm -f "$NAME" >/dev/null 2>&1 || true

# orio_core (run events) goes where the catkin devel shim for it points.
exec docker run -d --name "$NAME" --restart unless-stopped \
    --gpus all --ipc=host --net=host \
    -e ROS_MASTER_URI="${ROS_MASTER_URI:-http://localhost:11311}" \
    -e ORIO_LOGGING="${ORIO_LOGGING:-1}" \
    -e ORIO_AFFORDANCE_DIR="${ORIO_AFFORDANCE_DIR:-}" \
    -v "$REPO/devel/lib/python3/dist-packages:/catkin_ws/devel/lib/python3/dist-packages:ro" \
    -v "$REPO/src/devel_packages/orio_core:/home/ros_ws/src/devel_packages/orio_core:ro" \
    -v "$GRASPING:/opt/orio-grasping:ro" \
    -v "$CKPT:/opt/checkpoint.pt:ro" \
    -v "$REPO/logging:/home/ros_ws/logging" \
    -v "$REPO/src/devel_packages/orio_bringup/docker/suction_grasp_planner.py:/opt/suction_grasp_planner.py:ro" \
    "$IMAGE" \
    python /opt/suction_grasp_planner.py _checkpoint:=/opt/checkpoint.pt \
        _min_score:="$MIN_SCORE" _rot180:="$ROT180_PARAM"
