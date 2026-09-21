#!/usr/bin/env bash
# Single-terminal DexNet pick-and-place bring-up (no tmux).
#
# Runs the same stack as `launch_demo.sh --session orio_dexnet_pnp`, but instead of
# tmux panes it starts every service in the background and merges their output into THIS
# terminal, each line prefixed with a [module] label. The pick loop (dexnet_pnp.py) runs
# in the FOREGROUND so its --confirm "Press Enter to pick" prompt owns the keyboard.
#
# Ctrl-C stops everything: background services are killed and the containers removed.
#
#   bash run_dexnet_pnp_single.sh                 # dry-run (no vacuum), confirm each pick
#   bash run_dexnet_pnp_single.sh --vacuum        # start pneumatics (real suction)
#   bash run_dexnet_pnp_single.sh --auto          # no per-pick Enter (auto-pick)
#   bash run_dexnet_pnp_single.sh --straight-down # ignore grasp tilt, approach vertically
#
# The iam-doc control PC (robot 1) is started over ssh; it opens its own gnome-terminal
# windows separately (they run for the robot and cannot be merged here). Logging defaults
# off (the rerun lib is broken on the perception image's py38); set ORIO_LOGGING=1 to try.
set -uo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export ORIO_REPO="$REPO"
FRANKAPY="${ORIO_FRANKAPY:-$REPO/src/git_packages/frankapy}"
CONTAINER="${ORIO_CONTAINER:-orio_docker_container}"
DEXNET_CONTAINER="${DEXNET_CONTAINER_NAME:-orio_dexnet}"
DOCKER_DIR="$REPO/src/devel_packages/orio_bringup/docker"
WF="$REPO/src/devel_packages/orio_bringup/tmux/wait_for.sh"
# shellcheck disable=SC1090
source "$WF"

export ORIO_LOGGING="${ORIO_LOGGING:-0}"   # rerun broken on perception py38; default off

# ── Args ────────────────────────────────────────────────────────────────────
USE_VACUUM=0
CONFIRM=1
PNP_EXTRA=()
for arg in "$@"; do
    case "$arg" in
        --vacuum)        USE_VACUUM=1 ;;
        --no-vacuum)     USE_VACUUM=0 ;;
        --auto)          CONFIRM=0 ;;
        --confirm)       CONFIRM=1 ;;
        --straight-down) PNP_EXTRA+=(--straight-down) ;;
        -h|--help) sed -n '2,20p' "$0"; exit 0 ;;
        *) echo "unknown option: $arg" >&2; exit 2 ;;
    esac
done
[ "$USE_VACUUM" -eq 0 ] && PNP_EXTRA+=(--no-vacuum)
[ "$CONFIRM" -eq 1 ]    && PNP_EXTRA+=(--confirm)

# ── Save a copy of everything to a timestamped log file ──────────────────────
# tee the whole script's output (launcher lines + every [module] stream + the pick
# loop) to logging/dexnet_pnp/<timestamp>.log while still showing it live. The pick
# loop's interactive `docker exec -it` reads stdin from the terminal (the TTY), which
# teeing stdout does not affect, so the --confirm Enter prompt still works.
LOG_DIR="$REPO/logging/dexnet_pnp"
mkdir -p "$LOG_DIR"
LOG_FILE="$LOG_DIR/$(date +%Y%m%d_%H%M%S).log"
# Redirect the rest of the script through tee (append so the exec below keeps writing).
exec > >(tee -a "$LOG_FILE") 2>&1
echo "[launcher] logging this run to $LOG_FILE"

# ── Log merging ──────────────────────────────────────────────────────────────
# Each service's stdout+stderr is piped through a prefixer and merged into this
# terminal. PIDs are tracked so the trap can kill them on exit.
PIDS=()

prefix() {  # $1 = label; reads stdin, writes "[label] line"
    local label="$1"
    sed -u "s/^/[$label] /" 2>/dev/null || sed "s/^/[$label] /"
}

run_bg() {  # $1 = label, rest = command; run in background, prefixed
    local label="$1"; shift
    ( "$@" 2>&1 | prefix "$label" ) &
    PIDS+=($!)
}

log() { echo "[launcher] $*"; }

# ── Teardown ─────────────────────────────────────────────────────────────────
cleanup() {
    echo
    log "shutting down…"
    # Stop the vacuum first if it was running (best effort).
    if [ "$USE_VACUUM" -eq 1 ]; then
        docker exec "$DEXNET_CONTAINER" bash -lc \
          "source /opt/ros/noetic/setup.bash 2>/dev/null; \
           rosservice call /orio/pnp_cup/off 2>/dev/null" >/dev/null 2>&1 || true
    fi
    for pid in "${PIDS[@]:-}"; do kill "$pid" 2>/dev/null || true; done
    log "removing containers…"
    docker rm -f orio_roscore "$CONTAINER" "$DEXNET_CONTAINER" orio_perception orio_cameras \
        >/dev/null 2>&1 || true
    log "done. Full log saved to: $LOG_FILE"
    # Give the tee (process substitution) a moment to flush the final lines to the file.
    sync; sleep 0.2
}
trap cleanup EXIT INT TERM

# ── Pre-flight cleanup of any stale containers ───────────────────────────────
log "clearing any stale orio containers…"
docker rm -f orio_roscore "$CONTAINER" "$DEXNET_CONTAINER" orio_perception orio_cameras \
    >/dev/null 2>&1 || true

# ── Bring-up (readiness-gated, background, merged logs) ───────────────────────
log "starting roscore…"
run_bg roscore bash "$DOCKER_DIR/run_roscore.sh"
wait_for_roscore 30 || { log "roscore did not start in 30s — check the [roscore] logs. Aborting."; exit 1; }

log "starting robot-1 control PC (iam-doc) — opens its own windows separately…"
( cd "$FRANKAPY" && bash ./bash_scripts/start_control_pc.sh -u student -i iam-doc ) 2>&1 \
    | prefix doc || true

log "starting main docker container…"
# ORIO_NO_TTY=1: run detached, no -it (we background it through a pipe = no TTY).
run_bg docker bash -lc "cd '$REPO' && ORIO_NO_TTY=1 ORIO_LOGGING=$ORIO_LOGGING bash orio_run_docker.sh"
wait_for_container "$CONTAINER" 60 || { log "main container '$CONTAINER' did not start in 60s — check the [docker] logs. Aborting."; exit 1; }

# ── Pre-flight: is the robot ready? ──────────────────────────────────────────
# Check BEFORE the heavy services (dexnet ~25s TF load, cameras, perception) so a
# locked / not-activated robot fails fast instead of after the full bring-up. Uses the
# same readiness path (frankapy) the real run uses. No motion.
log "pre-flight: checking robot 1 is ready…"
docker exec "$CONTAINER" bash -c \
        "source /home/ros_ws/devel/setup.bash && \
         cd /home/ros_ws/src/devel_packages/orio && \
         python3 dexnet_pnp.py --check-only" 2>&1 | prefix check
# ${PIPESTATUS[0]} is the docker exec's status; the sed prefixer always exits 0.
if [ "${PIPESTATUS[0]}" -ne 0 ]; then
    echo
    log "ROBOT NOT READY — aborting before starting cameras/dexnet/perception."
    log "Unlock/activate robot 1, e.g.:"
    log "    python3 src/devel_packages/orio_bringup/lock_arms.py --unlock 1"
    log "then re-run this script."
    exit 1
fi
log "robot is ready."

log "starting DexNet planner…"
run_bg dexnet bash -lc "ORIO_LOGGING=$ORIO_LOGGING bash '$DOCKER_DIR/run_dexnet.sh' && docker logs -f $DEXNET_CONTAINER"
if ! wait_for_service /dexnet_grasp_planner/plan_grasp 60; then
    log "DexNet planner did not come up in 60s — check the [dexnet] logs above. Aborting."
    exit 1
fi

# Cameras, with a self-healing retry. The Xtion can flap at startup if USB autosuspend
# suspends it mid-launch (see orio_bringup/udev/90-xtion-no-autosuspend.rules for the
# permanent fix). Each attempt (re)starts cameras.launch and waits for ACTUAL frames on
# /camera/rgb/image_raw; a fresh camera launch re-triggers device enumeration, which is
# exactly what "just re-running the script" did by hand. We automate that here.
CAM_ATTEMPTS="${ORIO_CAM_ATTEMPTS:-3}"
cam_ok=0
for attempt in $(seq 1 "$CAM_ATTEMPTS"); do
    log "starting cameras (attempt $attempt/$CAM_ATTEMPTS)…"
    # Kill any prior cameras.launch in the container so a retry re-enumerates the device.
    docker exec "$CONTAINER" bash -c "pkill -f cameras.launch" >/dev/null 2>&1 || true
    run_bg cameras docker exec "$CONTAINER" bash -c \
        "source /home/ros_ws/devel/setup.bash && roslaunch manipulation cameras.launch"
    if wait_for_topic_publishing /camera/rgb/image_raw 25; then
        cam_ok=1
        break
    fi
    log "Xtion not publishing on attempt $attempt — restarting cameras…"
done
if [ "$cam_ok" -ne 1 ]; then
    echo
    log "CAMERA NOT PUBLISHING: /camera/rgb/image_raw (Xtion) after $CAM_ATTEMPTS attempts."
    log "Likely causes:"
    log "  - USB autosuspend flapping — install the permanent fix once:"
    log "      bash src/devel_packages/orio_bringup/udev/install_xtion_udev.sh"
    log "  - Xtion unplugged / claimed by a stale container (docker ps | grep camera)"
    log "  - USB needs a physical replug (see the [cameras] logs above)"
    log "Aborting before perception/pick (they need camera frames)."
    exit 1
fi
if ! wait_for_topic_publishing /zedm/zed_node/rgb/image_rect_color 30; then
    echo
    log "CAMERA NOT PUBLISHING: /zedm/zed_node/rgb/image_rect_color (ZED). Check the ZED"
    log "is connected and the [cameras] logs above. Aborting."
    exit 1
fi

log "starting perception (dexnet backend)…"
run_bg perception bash -lc "ORIO_NO_TTY=1 ORIO_LOGGING=$ORIO_LOGGING bash '$DOCKER_DIR/run_perception.sh' dexnet"

if [ "$USE_VACUUM" -eq 1 ]; then
    log "starting pneumatics…"
    run_bg pneumatics bash -lc "cd '$REPO/src/devel_packages/orio' && ORIO_LOGGING=$ORIO_LOGGING python3 pneumatic_control_recovery.py"
else
    log "pneumatics DISABLED (dry-run): move the cup by hand."
fi

if ! wait_for_service /compute_grasps 90; then
    log "/compute_grasps did not come up in 90s — check the [perception] logs above"
    log "(model load, backend=dexnet, camera frames). Aborting."
    exit 1
fi

# ── Pick loop in the FOREGROUND so --confirm can read the keyboard ────────────
log "starting pick-and-place loop (Ctrl-C to stop everything)…"
echo "[launcher] pnp flags: ${PNP_EXTRA[*]}"
docker exec -e ORIO_LOGGING="$ORIO_LOGGING" -it "$CONTAINER" bash -c \
    "source /home/ros_ws/devel/setup.bash && cd /home/ros_ws/src/devel_packages/orio && \
     python3 dexnet_pnp.py ${PNP_EXTRA[*]}"

log "pick loop exited."
