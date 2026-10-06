#!/usr/bin/env bash
# Save one RGB snapshot (plus depth and intrinsics) of what the Xtion sees.
#
# Reuses a running camera stack if there is one (e.g. a run_dexnet_pnp_single.sh or
# launch_demo.sh session). Otherwise it brings up just what the picture needs - the ROS1
# master container, orio_docker_container and the Xtion driver - takes the picture, and
# tears down what THIS script started. Anything already running is left alone.
#
# The grab itself runs inside the perception container (the host has no rospy), as your
# user, so the files are yours rather than root's.
#
#   bash snap_xtion.sh [outdir]         default outdir: logging/xtion/<timestamp>/
#   bash snap_xtion.sh --keep           leave roscore/container/camera up afterwards
#
# Writes color.png, color.npy, depth.npy and K.npy into outdir.
# Env: ORIO_CONTAINER, ORIO_ROS1_CONTAINER, ORIO_PERCEPTION_IMAGE, ORIO_CAM_ATTEMPTS.
set -uo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export ORIO_REPO="$REPO"
CONTAINER="${ORIO_CONTAINER:-orio_docker_container}"
ROS1_CONTAINER="${ORIO_ROS1_CONTAINER:-orio_roscore}"
IMAGE="${ORIO_PERCEPTION_IMAGE:-orio/perception:latest}"
SNAP_CONTAINER="orio_xtion_snap"
RGB_TOPIC="/camera/rgb/image_raw"
# shellcheck disable=SC1091
source "$REPO/src/devel_packages/orio_bringup/tmux/wait_for.sh"

# ── Args ────────────────────────────────────────────────────────────────────
KEEP=0
OUT=""
while [ $# -gt 0 ]; do
    case "$1" in
        --keep) KEEP=1 ;;
        -h|--help) sed -n '2,16p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
        -*) echo "unknown option: $1" >&2; exit 2 ;;
        *) OUT="$1" ;;
    esac
    shift
done
OUT="${OUT:-$REPO/logging/xtion/$(date +%Y%m%d_%H%M%S)}"
mkdir -p "$OUT" || exit 1
OUT="$(cd "$OUT" && pwd)"

log() { echo "[snap] $*"; }
prefix() { local label="$1"; sed -u "s/^/[$label] /" 2>/dev/null || sed "s/^/[$label] /"; }
PIDS=()
run_bg() { local label="$1"; shift; ( "$@" 2>&1 | prefix "$label" ) & PIDS+=($!); }
container_running() { docker ps --format '{{.Names}}' 2>/dev/null | grep -qx "$1"; }
xtion_publishing() {  # true once the RGB topic actually streams a frame
    timeout 6 docker exec "$ROS1_CONTAINER" bash -c \
        "$_ROS1_SETUP; timeout 4 rostopic echo -n 1 $RGB_TOPIC" >/dev/null 2>&1
}

# ── Teardown: only what this script started ──────────────────────────────────
STARTED_ROSCORE=0; STARTED_CONTAINER=0; STARTED_XTION=0
cleanup() {
    trap '' INT TERM HUP
    docker rm -f "$SNAP_CONTAINER" >/dev/null 2>&1 || true
    if [ "$KEEP" -eq 1 ]; then
        log "--keep: leaving things running. To stop them later:"
        [ "$STARTED_XTION" -eq 1 ]     && log "  camera:     docker exec $CONTAINER pkill -f xtion_only.launch"
        [ "$STARTED_CONTAINER" -eq 1 ] && log "  container:  docker rm -f $CONTAINER"
        [ "$STARTED_ROSCORE" -eq 1 ]   && log "  roscore:    docker rm -f $ROS1_CONTAINER"
    else
        if [ "$STARTED_XTION" -eq 1 ] && [ "$STARTED_CONTAINER" -eq 0 ]; then
            docker exec "$CONTAINER" bash -c "pkill -f xtion_only.launch" >/dev/null 2>&1 || true
        fi
        if [ "$STARTED_CONTAINER" -eq 1 ]; then docker rm -f "$CONTAINER" >/dev/null 2>&1 || true; fi
        if [ "$STARTED_ROSCORE" -eq 1 ]; then docker rm -f "$ROS1_CONTAINER" >/dev/null 2>&1 || true; fi
    fi
    for p in "${PIDS[@]:-}"; do [ -n "$p" ] && kill "$p" 2>/dev/null; done
    wait 2>/dev/null
}
trap 'cleanup; exit 130' INT
trap 'cleanup; exit 143' TERM
trap 'cleanup; exit 129' HUP
trap cleanup EXIT

# ── 1. ROS1 master (container; this host is 22.04/ROS2 with no roscore) ───────
if container_running "$ROS1_CONTAINER"; then
    log "roscore container '$ROS1_CONTAINER' already running - reusing."
else
    log "no ROS master running - starting roscore…"
    STARTED_ROSCORE=1
    docker rm -f "$ROS1_CONTAINER" >/dev/null 2>&1 || true
    docker run -d --rm --name "$ROS1_CONTAINER" --net host "${ORIO_ROS1_IMAGE:-orio_docker}" \
        bash -c 'source /opt/ros/noetic/setup.bash && exec roscore' >/dev/null \
        || { log "could not start the roscore container. Aborting."; exit 1; }
fi
wait_for_roscore 30 || { log "roscore did not come up in 30s. Aborting."; exit 1; }

# ── 2. Xtion driver, unless it is already streaming ──────────────────────────
if xtion_publishing; then
    log "Xtion already publishing $RGB_TOPIC - reusing."
else
    if container_running "$CONTAINER"; then
        log "container '$CONTAINER' already running - reusing."
    else
        log "starting $CONTAINER…"
        STARTED_CONTAINER=1
        run_bg docker bash -lc "cd '$REPO' && ORIO_NO_TTY=1 bash orio_run_docker.sh"
        wait_for_container "$CONTAINER" 60 || { log "'$CONTAINER' did not start in 60s. Aborting."; exit 1; }
    fi

    # The Xtion intermittently fails to open its IR stream on start-up, so retry, as
    # run_dexnet_pnp_single.sh does. A healthy start publishes in about 6 s.
    CAM_ATTEMPTS="${ORIO_CAM_ATTEMPTS:-5}"
    cam_ok=0
    for attempt in $(seq 1 "$CAM_ATTEMPTS"); do
        log "starting Xtion (attempt $attempt/$CAM_ATTEMPTS)…"
        docker exec "$CONTAINER" bash -c "pkill -f xtion_only.launch" >/dev/null 2>&1 || true
        STARTED_XTION=1
        run_bg cameras docker exec "$CONTAINER" bash -c \
            "source /home/ros_ws/devel/setup.bash && roslaunch manipulation xtion_only.launch"
        deadline=$(( SECONDS + ${ORIO_XTION_WAIT:-12} ))
        while [ "$SECONDS" -lt "$deadline" ]; do
            if xtion_publishing; then cam_ok=1; break; fi
            sleep 0.5
        done
        [ "$cam_ok" -eq 1 ] && break
        log "Xtion did not publish within ${ORIO_XTION_WAIT:-12}s on attempt $attempt."
    done
    if [ "$cam_ok" -ne 1 ]; then
        log "CAMERA NOT PUBLISHING: $RGB_TOPIC after $CAM_ATTEMPTS attempts. Check the Xtion"
        log "cable and the [cameras] lines above (see docs/TROUBLESHOOTING.md). Aborting."
        exit 1
    fi
    log "Xtion is publishing."
fi

# ── 3. Capture (foreground, inside the perception container) ─────────────────
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
    log "saved: $OUT/color.png"
else
    log "capture FAILED (exit $rc)."
fi
exit "$rc"
