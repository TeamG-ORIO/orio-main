#!/usr/bin/env bash
# Test the labelling arm (robot 2, iam-luisa) on its own, from ONE terminal, with the label
# height measured by Fast-FoundationStereo on the ZED instead of carried over from robot 1.
#
# Brings up: roscore, robot 2's control PC, orio_docker_container, the ZED, the
# Fast-FoundationStereo depth node (ffs/), the labelling perception node (perception/)
# and, with --vacuum, the pneumatics node. Then runs arm/labelling_arm_test.py in the
# foreground: fetch a label from the dispenser, find the label spot and its depth, put the
# label on the item, repeat.
#
#   bash research/labelling_arm_test/run_labelling_test.sh                 # dry run: no vacuum, Enter before each placement
#   bash research/labelling_arm_test/run_labelling_test.sh --vacuum        # real suction on the labelling cup
#   bash research/labelling_arm_test/run_labelling_test.sh --auto          # no prompt
#   bash research/labelling_arm_test/run_labelling_test.sh --zone 1        # only label zone 1 (default: auto)
#   bash research/labelling_arm_test/run_labelling_test.sh --skip-dispenser  # place with what is on the cup already
#   bash research/labelling_arm_test/run_labelling_test.sh --cycles 3      # stop after 3 labels
#   bash research/labelling_arm_test/run_labelling_test.sh --z-offset -0.005  # metres added to the measured z
#   bash research/labelling_arm_test/run_labelling_test.sh --depth zed     # the ZED's own depth, for comparison
#   bash research/labelling_arm_test/run_labelling_test.sh --prompt packet # GroundingDINO prompt (default: the demo's)
#   bash research/labelling_arm_test/run_labelling_test.sh --box-threshold 0.15  # lowest detection score kept (default 0.25)
#   bash research/labelling_arm_test/run_labelling_test.sh --no-motion     # perception + IK only: no control PC, no arm
#   bash research/labelling_arm_test/run_labelling_test.sh --unlock        # open robot 2's brakes first
#   bash research/labelling_arm_test/run_labelling_test.sh --fresh         # stop the running stack first
#
# Like run_pickup-only.sh: the services run in the background with their output
# merged here under [module] prefixes; the arm script runs in the FOREGROUND so its prompt
# owns the keyboard. Ctrl-C stops THIS RUN (arm script, pneumatics). roscore, the main
# container, the control PC, the ZED, Fast-FoundationStereo and perception stay up and the
# next run reuses them. To stop everything: bash research/labelling_arm_test/stop_labelling_test.sh
#
# Own perception container (orio_lbl_perception), ROS names (/labelling_test/*) and logs
# (logging/labelling_test/). roscore, orio_docker_container, the ZED launch and the
# Fast-FoundationStereo container (orio_ffs, /ffs/*) are the same ones run_full_demo.sh uses.
#
# Env: ORIO_LUISA_HOST (snaak@iam-luisa), ORIO_CONTAINER, ORIO_FFS_DIR / ORIO_FFS_CHECKPOINT /
#      ORIO_FFS_SCALE (see orio_bringup/docker/run_ffs.sh), ORIO_VACUUM_PORT (/dev/ttyACM0).
set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"
LOG_TAG=launcher
STACK_DIR="$REPO/logging/labelling_test/stack"
# No run logging here: the rerun recorder is not started and the nodes do not publish events.
export ORIO_LOGGING=0
# The labelling node reads the left image; the ZED-M's rgb topic is the same camera.
ZED_TOPIC=/zedm/zed_node/left/image_rect_color
# shellcheck disable=SC1091
source "$REPO/src/devel_packages/orio_bringup/stack.sh"
LBL_PERC_CONTAINER="${ORIO_LBL_PERCEPTION_CONTAINER:-orio_lbl_perception}"
# Inside the containers this directory is /home/ros_ws/research/labelling_arm_test.
ARM_SCRIPT_CONT=/home/ros_ws/research/labelling_arm_test/arm/labelling_arm_test.py

# ── Args ────────────────────────────────────────────────────────────────────
USE_VACUUM=0
DEPTH=ffs
NO_MOTION=0
UNLOCK=0
FRESH=0
ARM_ARGS=()
while [ $# -gt 0 ]; do
    arg="$1"; shift
    case "$arg" in
        --vacuum)         USE_VACUUM=1 ;;
        --no-vacuum)      USE_VACUUM=0 ;;
        --auto)           ARM_ARGS+=(--auto) ;;
        --confirm)        ARM_ARGS+=(--confirm) ;;
        --zone)           ARM_ARGS+=(--zone "${1:?--zone needs 1, 2 or auto}"); shift ;;
        --zone=*)         ARM_ARGS+=(--zone "${arg#*=}") ;;
        --skip-dispenser) ARM_ARGS+=(--skip-dispenser) ;;
        --cycles)         ARM_ARGS+=(--cycles "${1:?--cycles needs a number}"); shift ;;
        --cycles=*)       ARM_ARGS+=(--cycles "${arg#*=}") ;;
        --z-offset)       ARM_ARGS+=(--z-offset "${1:?--z-offset needs a value in metres}"); shift ;;
        --z-offset=*)     ARM_ARGS+=(--z-offset "${arg#*=}") ;;
        --depth)          DEPTH="${1:?--depth needs ffs or zed}"; shift ;;
        --depth=*)        DEPTH="${arg#*=}" ;;
        --no-ffs)         DEPTH=zed ;;
        --prompt)         export ORIO_LBL_PROMPT="${1:?--prompt needs text}"; shift ;;
        --prompt=*)       export ORIO_LBL_PROMPT="${arg#*=}" ;;
        --box-threshold)  export ORIO_LBL_BOX_THRESHOLD="${1:?--box-threshold needs a value in 0..1}"; shift ;;
        --box-threshold=*) export ORIO_LBL_BOX_THRESHOLD="${arg#*=}" ;;
        --no-motion)      NO_MOTION=1; ARM_ARGS+=(--no-motion) ;;
        --unlock)         UNLOCK=1 ;;
        --fresh)          FRESH=1 ;;
        -h|--help)        sed -n '2,36p' "$0"; exit 0 ;;
        *) echo "unknown option: $arg" >&2; exit 2 ;;
    esac
done
case "$DEPTH" in ffs|zed) ;; *) echo "--depth must be ffs or zed, got '$DEPTH'" >&2; exit 2 ;; esac
[ "$USE_VACUUM" -eq 0 ] && ARM_ARGS+=(--no-vacuum)

# ── Logging ──────────────────────────────────────────────────────────────────
LOG_DIR="$REPO/logging/labelling_test"
mkdir -p "$LOG_DIR"
RUN_STAMP="$(date +%Y%m%d_%H%M%S)"
LOG_FILE="$LOG_DIR/$RUN_STAMP.log"
# Perception's debug images. Perception outlives a run and reads its directory once, so it
# is given the fixed link current.images, which each run points at its own directory. The
# link is relative, so it resolves the same inside the container.
IMAGE_DIR="$LOG_DIR/$RUN_STAMP.images"
IMAGE_LINK="$LOG_DIR/current.images"
# Everything shown and saved, known noise dropped.
log_filter_start '' 1

STACK_SERVICES=(roscore zed ffs perception)
STACK_CONTAINERS=("$CONTAINER" "$FFS_CONTAINER" "$LBL_PERC_CONTAINER")

# ── Teardown (this run only) ─────────────────────────────────────────────────
cleanup() {
    echo
    log "stopping this run… (Ctrl-C again won't interrupt this)"
    local cups=()
    [ "$USE_VACUUM" -eq 1 ] && cups=(lbl_cup)
    stop_run_in_container 'labelling_arm_test\.py|pneumatic_control_recovery\.py' "${cups[@]}"
    kill_bg
    log "done. Full log: $LOG_FILE"
    if [ -d "$IMAGE_DIR" ]; then
        _n=$(find "$IMAGE_DIR" -name 'label_*.png' -not -name '*_color.png' 2>/dev/null | wc -l)
        [ "$_n" -gt 0 ] && log "perception images ($_n): $IMAGE_DIR"
    fi
    log "the stack (control PC, ZED, Fast-FoundationStereo, perception) is still running; the"
    log "next run reuses it. To stop everything: bash research/labelling_arm_test/stop_labelling_test.sh"
    sync; sleep 0.2
}
install_cleanup_traps

follow roscore roscore
follow zed zed
follow ffs ffs
follow perception perception

if [ "$FRESH" -eq 1 ]; then
    log "--fresh: stopping the running stack first…"
    bash "$HERE/stop_labelling_test.sh"
fi

# ── roscore, robot 2's control PC, the main container ────────────────────────
stack_bringup_roscore || exit 1
if [ "$NO_MOTION" -eq 0 ]; then
    if [ "$UNLOCK" -eq 1 ]; then
        unlock_robot 2 || exit 1
    fi
    luisa_bringup || { log "Is iam-luisa up (ping iam-luisa)?"; exit 1; }
fi
main_container_bringup 'labelling_arm_test\.py|pneumatic_control_recovery\.py' || exit 1

# ── Pre-flight: is robot 2 ready? (before the heavy services) ────────────────
if [ "$NO_MOTION" -eq 0 ]; then
    wait_for_topic "/robot_state_publisher_node_2/robot_state" 60 \
        || { log "robot 2 never published its state — its control PC did not come up. Aborting."; exit 1; }
    log "pre-flight: checking robot 2 is ready…"
    if ! preflight_check "python3 $ARM_SCRIPT_CONT --check-only"; then
        echo
        log "ROBOT 2 NOT READY — aborting before the cameras and perception start."
        log "  - brakes locked / not activated: python3 src/devel_packages/orio_bringup/lock_arms.py --unlock 2"
        log "    (or re-run with --unlock); a red light needs the error acknowledged in Desk first."
        log "  - franka-interface window on the control PC:"
        log "      ssh -t $LUISA_PC 'cd ~/franka-interface-docker && ./run.sh tmux attach -t franka'"
        log "    If it crashed, re-running this script restarts it. If it is stuck, add --fresh."
        exit 1
    fi
    log "robot 2 is ready."
fi

# ── ZED: without its own depth (NONE) unless --depth zed needs it (NEURAL) ───
ZED_DEPTH_MODE=$([ "$DEPTH" = zed ] && echo NEURAL || echo NONE)
zed_bringup "$ZED_DEPTH_MODE" || exit 1

# ── Fast-FoundationStereo depth ──────────────────────────────────────────────
if [ "$DEPTH" = ffs ]; then
    ffs_bringup
    ffs_wait || exit 1
fi

# ── Labelling perception ─────────────────────────────────────────────────────
mkdir -p "$IMAGE_DIR"
ln -sfn "${IMAGE_DIR##*/}" "$IMAGE_LINK"
PERC_CONF="depth=$DEPTH prompt=${ORIO_LBL_PROMPT:-default} box_threshold=${ORIO_LBL_BOX_THRESHOLD:-default}"
stack_service perception "$LBL_PERC_CONTAINER" "$PERC_CONF" \
    bash -lc "ORIO_LBL_IMAGE_DIR='$IMAGE_LINK' bash '$HERE/perception/run_lbl_perception.sh' $DEPTH && docker logs -f $LBL_PERC_CONTAINER"
if [ "$SERVICE_REUSED" -eq 1 ]; then
    log "labelling perception already running ($PERC_CONF)."
else
    log "starting labelling perception ($PERC_CONF; models take a minute or two)…"
fi
if ! stack_wait_service perception /labelling_test/compute_label_pose 240; then
    log "/labelling_test/compute_label_pose did not come up in 240s — check the [perception]"
    log "lines above (weights, GPU memory, camera frames). Aborting."
    exit 1
fi
log "perception images → $IMAGE_DIR"

# ── Pneumatics (this run only) ───────────────────────────────────────────────
if [ "$USE_VACUUM" -eq 1 ]; then
    log "starting pneumatics…"
    pneumatics_start
    pneumatics_wait lbl_cup || exit 1
else
    log "pneumatics DISABLED (dry run): the cup holds nothing."
fi

# ── The arm, in the FOREGROUND so its prompt can read the keyboard ───────────
log "arm flags: ${ARM_ARGS[*]}"
log "starting the labelling arm test (Ctrl-C stops this run; the stack stays up)…"
run_in_container "python3 -u $ARM_SCRIPT_CONT ${ARM_ARGS[*]}"
log "arm test exited."
