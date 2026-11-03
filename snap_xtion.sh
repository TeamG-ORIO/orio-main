#!/usr/bin/env bash
# Save one RGB snapshot (plus depth and intrinsics) of what the Xtion sees.
#
# Reuses a running camera stack if there is one (e.g. a run_dexnet_pnp_single.sh or
# run_full_demo.sh session). Otherwise it brings up just what the picture needs - the ROS1
# master container, orio_docker_container and the Xtion driver - takes the picture, and
# tears down what THIS script started. Anything already running is left alone.
#
# The grab itself runs inside the perception container (the host has no rospy), as your
# user, so the files are yours rather than root's.
#
#   bash snap_xtion.sh [outdir]         default outdir: logging/xtion/<timestamp>/
#   bash snap_xtion.sh --keep           leave roscore/container/camera up afterwards
#
# Writes color.png, preview.png (RGB and depth side by side), color.npy, depth.npy and
# K.npy into outdir.
# Env: ORIO_CONTAINER, ORIO_ROS1_CONTAINER, ORIO_PERCEPTION_IMAGE, ORIO_CAM_ATTEMPTS.
set -uo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_TAG=snap
STACK_DIR="$REPO/logging/xtion/stack"
# shellcheck disable=SC1091
source "$REPO/src/devel_packages/orio_bringup/stack.sh"
IMAGE="${ORIO_PERCEPTION_IMAGE:-orio/perception:latest}"
SNAP_CONTAINER="orio_xtion_snap"

# ── Args ────────────────────────────────────────────────────────────────────
KEEP=0
OUT=""
while [ $# -gt 0 ]; do
    case "$1" in
        --keep) KEEP=1 ;;
        -h|--help) sed -n '2,17p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
        -*) echo "unknown option: $1" >&2; exit 2 ;;
        *) OUT="$1" ;;
    esac
    shift
done
OUT="${OUT:-$REPO/logging/xtion/$(date +%Y%m%d_%H%M%S)}"
mkdir -p "$OUT" || exit 1
OUT="$(cd "$OUT" && pwd)"

# ── Teardown: only what this script started ──────────────────────────────────
cleanup() {
    docker rm -f "$SNAP_CONTAINER" >/dev/null 2>&1 || true
    if [ "$KEEP" -eq 1 ]; then
        log "--keep: leaving things running. To stop them later:"
        [ "$XTION_STARTED" -eq 1 ]     && log "  camera:     docker exec $CONTAINER pkill -f xtion_only.launch"
        [ "$CONTAINER_STARTED" -eq 1 ] && log "  container:  docker rm -f $CONTAINER"
        [ "$STACK_FRESH" -eq 1 ]       && log "  roscore:    docker rm -f $ROS1_CONTAINER"
    else
        if [ "$XTION_STARTED" -eq 1 ] && [ "$CONTAINER_STARTED" -eq 0 ]; then
            docker exec "$CONTAINER" bash -c "pkill -f xtion_only.launch" >/dev/null 2>&1 || true
            stop_detached xtion
        fi
        [ "$CONTAINER_STARTED" -eq 1 ] && docker rm -f "$CONTAINER" >/dev/null 2>&1
        if [ "$STACK_FRESH" -eq 1 ]; then
            stop_detached roscore
            docker rm -f "$ROS1_CONTAINER" >/dev/null 2>&1
        fi
    fi
    # No `wait` here: with --keep the detached roscore is still a child of this shell.
    kill_bg
}
install_cleanup_traps

follow roscore roscore
follow cameras xtion

# ── 1. ROS1 master, 2. orio container, 3. Xtion driver (unless already streaming) ──
STACK_SERVICES=(roscore xtion)
stack_bringup_roscore || exit 1
main_container_bringup || exit 1
xtion_bringup || exit 1

# ── 4. Capture (foreground, inside the perception container) ─────────────────
docker rm -f "$SNAP_CONTAINER" >/dev/null 2>&1 || true
docker run --rm --net=host --name "$SNAP_CONTAINER" \
    --user "$(id -u):$(id -g)" \
    --env="HOME=/tmp" \
    --env="ROS_HOME=/tmp/ros" \
    --env="ROS_MASTER_URI=${ROS_MASTER_URI:-http://localhost:11311}" \
    --volume="$REPO/src/devel_packages/orio_perception/test:/capture:ro" \
    --volume="$OUT:/out" \
    "$IMAGE" \
    bash -c 'mkdir -p /tmp/ros/log && source /opt/ros/noetic/setup.bash && python3 /capture/capture_xtion.py /out'
rc=$?
if [ "$rc" -eq 0 ]; then
    log "saved: $OUT/color.png and $OUT/preview.png"
else
    log "capture FAILED (exit $rc)."
fi
exit "$rc"
