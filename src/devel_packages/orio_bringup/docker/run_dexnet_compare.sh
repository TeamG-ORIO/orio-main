#!/usr/bin/env bash
# Compare the old and new DexNet methods on saved Xtion captures or live frames — no arm.
#
# Runs orio_perception/test/compare_dexnet_methods.py in a throwaway perception
# container (it needs GroundingDINO, SAM and rospy). roscore and the DexNet planner are
# started if they are not already up, and the ones this script started are stopped again
# on exit. The perception node must NOT be running: the script creates it itself.
#
# Each scene is a directory written by orio_perception/test/capture_xtion.py
# (color.npy, depth.npy, K.npy).
#
# With --live the Xtion driver is started too (unless it is already publishing) and each
# Enter in this terminal captures a frame, saves it under <out>/scenes/ and compares it.
#
# Usage:
#   bash run_dexnet_compare.sh <scene_dir> [<scene_dir> ...]
#   bash run_dexnet_compare.sh --live
#   bash run_dexnet_compare.sh --out logging/dexnet_compare/report1 <scene_dir> ...
#   bash run_dexnet_compare.sh --live --attempts 5
# Output: <out>/<scene>_compare.png, the six panels as separate PNGs, and <out>/summary.json.
#         Default <out> is logging/dexnet_compare/<timestamp>.
#
# The planner is started with no score threshold (DEXNET_MIN_Q_VALUE=0 unless set), so
# low-scoring grasps are still drawn. A planner that is already running keeps its own.
set -euo pipefail

DOCKER_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$DOCKER_DIR/../../../.." && pwd)"
NAME="${DEXNET_COMPARE_CONTAINER:-orio_dexnet_compare}"
IMAGE="${ORIO_PERCEPTION_IMAGE:-orio/perception:latest}"
ROSCORE="${ORIO_ROS1_CONTAINER:-orio_roscore}"
DEXNET="${DEXNET_CONTAINER_NAME:-orio_dexnet}"
SCRIPT=/home/ros_ws/src/devel_packages/orio_perception/test/compare_dexnet_methods.py

OUT="$REPO/logging/dexnet_compare/$(date +%Y%m%d_%H%M%S)"
SCENES=()
LIVE=0
EXTRA=()
CAMERA="${DEXNET_COMPARE_CAMERA_CONTAINER:-orio_compare_camera}"
CAMERA_IMAGE="${ORIO_ROS1_IMAGE:-orio_docker}"
while [ $# -gt 0 ]; do
    case "$1" in
        --out) OUT="$(realpath -m "$2")"; shift 2 ;;
        --live) LIVE=1; shift ;;
        --attempts) EXTRA+=(--attempts "$2"); shift 2 ;;
        -h|--help) sed -n '2,25p' "${BASH_SOURCE[0]}"; exit 0 ;;
        *) [ -d "$1" ] || { echo "Not a directory: $1" >&2; exit 1; }
           SCENES+=("$(realpath "$1")"); shift ;;
    esac
done
[ ${#SCENES[@]} -gt 0 ] || [ "$LIVE" -eq 1 ] \
    || { echo "Give at least one scene directory, or --live (see --help)." >&2; exit 1; }
mkdir -p "$OUT"

running() { docker ps --format '{{.Names}}' | grep -qx "$1"; }
ros() { docker exec "$ROSCORE" bash -c "source /opt/ros/noetic/setup.bash && $1"; }

if running "${ORIO_PERCEPTION_CONTAINER:-orio_perception}"; then
    echo "The perception container is running. Stop it first (this script replaces its node)." >&2
    exit 1
fi

STARTED=()
cleanup() {
    docker rm -f "$NAME" >/dev/null 2>&1 || true
    [ ${#STARTED[@]} -eq 0 ] || docker rm -f "${STARTED[@]}" >/dev/null 2>&1 || true
}
trap cleanup EXIT

if ! running "$ROSCORE"; then
    echo "starting roscore…"
    bash "$DOCKER_DIR/run_roscore.sh" >/dev/null 2>&1 &
    STARTED+=("$ROSCORE")
    for _ in $(seq 30); do ros "rostopic list" >/dev/null 2>&1 && break; sleep 1; done
    ros "rostopic list" >/dev/null 2>&1 || { echo "roscore did not start." >&2; exit 1; }
fi

if ! running "$DEXNET"; then
    echo "starting the DexNet planner (about 30 s)…"
    DEXNET_MIN_Q_VALUE="${DEXNET_MIN_Q_VALUE:-0}" ORIO_LOGGING=0 bash "$DOCKER_DIR/run_dexnet.sh" >/dev/null
    STARTED+=("$DEXNET")
else
    echo "DexNet planner already running; it keeps the score threshold it was started with."
fi
# Xtion driver for --live, in its own container so the arm container is not needed. The
# Xtion often fails to open its IR stream on the first try, hence the retries.
camera_up() { ros "timeout 4 rostopic echo -n 1 /camera/depth_registered/image_raw" >/dev/null 2>&1; }
if [ "$LIVE" -eq 1 ] && ! camera_up; then
    STARTED+=("$CAMERA")
    cam_ok=0
    for attempt in 1 2 3 4 5; do
        echo "starting the Xtion (attempt $attempt/5)…"
        docker rm -f "$CAMERA" >/dev/null 2>&1 || true
        docker run -d --rm --privileged --name "$CAMERA" --net host -v /dev:/dev \
            "$CAMERA_IMAGE" bash -c "source /opt/ros/noetic/setup.bash \
                && exec roslaunch openni2_launch openni2.launch depth_registration:=true" >/dev/null
        for _ in $(seq 5); do
            camera_up && { cam_ok=1; break; }
            running "$CAMERA" || break
        done
        [ "$cam_ok" -eq 1 ] && break
    done
    [ "$cam_ok" -eq 1 ] || { echo "Xtion is not publishing; check the cable (docs/TROUBLESHOOTING.md)." >&2; exit 1; }
fi

for _ in $(seq 120); do
    ros "rosservice list" 2>/dev/null | grep -q "/dexnet_grasp_planner/plan_grasp" && break
    sleep 1
done
ros "rosservice list" 2>/dev/null | grep -q "/dexnet_grasp_planner/plan_grasp" \
    || { echo "DexNet planner service did not come up; see: docker logs $DEXNET" >&2; exit 1; }

# Scene and output dirs are mounted at their host paths so arguments pass through as is.
MOUNTS=(--volume="$OUT:$OUT:rw")
for s in ${SCENES[@]+"${SCENES[@]}"}; do MOUNTS+=(--volume="$s:$s:ro"); done
ARGS=(--out "$OUT" ${EXTRA[@]+"${EXTRA[@]}"})
[ "$LIVE" -eq 1 ] && ARGS+=(--live)
# -i keeps stdin open for the Enter prompt; -t only when there is a terminal.
if [ -t 0 ]; then TTY_FLAGS="-it"; else TTY_FLAGS="-i"; fi

docker rm -f "$NAME" >/dev/null 2>&1 || true
docker run --rm $TTY_FLAGS --name "$NAME" \
    --gpus all --ipc=host --net=host \
    --env="ROS_MASTER_URI=${ROS_MASTER_URI:-http://localhost:11311}" \
    --env="ORIO_REPO=/home/ros_ws" \
    --env="ORIO_PERCEPTION_ASSETS=/home/ros_ws/src/devel_packages/orio_perception" \
    --env="ORIO_GROUNDINGDINO_DIR=/opt/GroundingDINO" \
    --env="ORIO_LOGGING=0" \
    --volume="$REPO/src/devel_packages:/home/ros_ws/src/devel_packages" \
    "${MOUNTS[@]}" \
    "$IMAGE" \
    bash -lc 'source /opt/ros/noetic/setup.bash && source /home/ros_ws/devel/setup.bash \
        && export PYTHONPATH=/home/ros_ws/src/devel_packages/orio_perception/scripts:$PYTHONPATH \
        && exec python3 "$@"' _ "$SCRIPT" "${ARGS[@]}" ${SCENES[@]+"${SCENES[@]}"}

echo "Figures: $OUT"
