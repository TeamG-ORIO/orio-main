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
# Like run_dexnet_pnp_single.sh: the services run in the background with their output
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
export ORIO_REPO="$REPO"
CONTAINER="${ORIO_CONTAINER:-orio_docker_container}"
FFS_CONTAINER="${ORIO_FFS_CONTAINER:-orio_ffs}"
LBL_PERC_CONTAINER="${ORIO_LBL_PERCEPTION_CONTAINER:-orio_lbl_perception}"
BRINGUP="$REPO/src/devel_packages/orio_bringup"
DOCKER_DIR="$BRINGUP/docker"
LUISA="${ORIO_LUISA_HOST:-snaak@iam-luisa}"
# shellcheck disable=SC1091
source "$BRINGUP/tmux/wait_for.sh"
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
        --zone)           [ $# -gt 0 ] || { echo "--zone needs 1, 2 or auto" >&2; exit 2; }
                          ARM_ARGS+=(--zone "$1"); shift ;;
        --zone=*)         ARM_ARGS+=(--zone "${arg#*=}") ;;
        --skip-dispenser) ARM_ARGS+=(--skip-dispenser) ;;
        --cycles)         [ $# -gt 0 ] || { echo "--cycles needs a number" >&2; exit 2; }
                          ARM_ARGS+=(--cycles "$1"); shift ;;
        --cycles=*)       ARM_ARGS+=(--cycles "${arg#*=}") ;;
        --z-offset)       [ $# -gt 0 ] || { echo "--z-offset needs a value in metres" >&2; exit 2; }
                          ARM_ARGS+=(--z-offset "$1"); shift ;;
        --z-offset=*)     ARM_ARGS+=(--z-offset "${arg#*=}") ;;
        --depth)          [ $# -gt 0 ] || { echo "--depth needs ffs or zed" >&2; exit 2; }
                          DEPTH="$1"; shift ;;
        --depth=*)        DEPTH="${arg#*=}" ;;
        --no-ffs)         DEPTH=zed ;;
        --prompt)         [ $# -gt 0 ] || { echo "--prompt needs text" >&2; exit 2; }
                          export ORIO_LBL_PROMPT="$1"; shift ;;
        --prompt=*)       export ORIO_LBL_PROMPT="${arg#*=}" ;;
        --box-threshold)  [ $# -gt 0 ] || { echo "--box-threshold needs a value in 0..1" >&2; exit 2; }
                          export ORIO_LBL_BOX_THRESHOLD="$1"; shift ;;
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
STACK_DIR="$LOG_DIR/stack"        # the stack's services outlive a run: one log each here
mkdir -p "$LOG_DIR" "$STACK_DIR"
RUN_STAMP="$(date +%Y%m%d_%H%M%S)"
LOG_FILE="$LOG_DIR/$RUN_STAMP.log"
# Perception's debug images. Perception outlives a run and reads its directory once, so it
# is given the fixed link current.images, which each run points at its own directory. The
# link is relative, so it resolves the same inside the container.
IMAGE_DIR="$LOG_DIR/$RUN_STAMP.images"
IMAGE_LINK="$LOG_DIR/current.images"

# Everything this script prints flows through one filter: ANSI colours stripped, known
# noise dropped (ZED per-topic advertises, ikpy warnings, CUDA chatter), the rest shown
# AND saved. The arm script runs under `docker exec -it`, so the terminal is in raw-ish
# output mode for the whole merged stream and a bare \n would staircase lines; CRLF to the
# terminal, LF to the file (see run_dexnet_pnp_single.sh for the full story).
exec > >(
    LOG_FILE="$LOG_FILE" awk '
    function strip(s){ gsub(/\033\[[0-9;]*m/, "", s); gsub(/\r/, "", s); return s }
    {
        line = strip($0)
        if (!length(line)) next
        if (line ~ /Advertised on topic|ikpy|is of type .fixed.|warnings\.warn|NUMA node|TF_CPP|cuda_gpu|autolab_core not installed/) next
        print line >> ENVIRON["LOG_FILE"]; fflush(ENVIRON["LOG_FILE"])
        printf "%s\r\n", line; fflush()
    }'
) 2>&1
echo "[launcher] logging this run to $LOG_FILE"

PIDS=()
prefix() { local label="$1"; sed -u "s/^/[$label] /" 2>/dev/null || sed "s/^/[$label] /"; }
run_bg() { local label="$1"; shift; ( "$@" 2>&1 | prefix "$label" ) & PIDS+=($!); }
log() { echo "[launcher] $*"; }

# The stack: services that stay up between runs, each in its own session (setsid) so the
# terminal's Ctrl-C never reaches it, logging to $STACK_DIR/<name>.log, which every run
# follows into its merged output. <name>.pid holds the process group for the stop script.
start_detached() {  # $1 = name, rest = command
    local name="$1"; shift
    : >"$STACK_DIR/$name.log"
    setsid bash -c '"$@"' _ "$@" >>"$STACK_DIR/$name.log" 2>&1 </dev/null &
    echo $! >"$STACK_DIR/$name.pid"
}
stop_detached() { local f="$STACK_DIR/$1.pid"; [ -f "$f" ] && kill -- "-$(cat "$f")" 2>/dev/null; rm -f "$f"; }
follow() {  # $1 = label, $2 = service name; ends with this launcher (--pid)
    ( tail -n0 -F --pid=$$ "$STACK_DIR/$2.log" 2>/dev/null | prefix "$1" ) &
    PIDS+=($!)
}
container_up() { docker ps -q -f "name=^$1\$" 2>/dev/null | grep -q .; }
roscore_up()   { _ros_list rostopic >/dev/null 2>&1; }
topic_live()   {
    timeout 6 docker exec "$ROS1_CONTAINER" bash -c \
        "$_ROS1_SETUP; timeout 4 rostopic echo -n 1 $1" >/dev/null 2>&1
}
conf_is() { [ "$(cat "$STACK_DIR/$1.conf" 2>/dev/null)" = "$2" ]; }

# ── Teardown (this run only) ─────────────────────────────────────────────────
_CLEANED=0
cleanup() {
    [ "$_CLEANED" -eq 1 ] && return
    _CLEANED=1
    trap '' INT TERM HUP
    echo
    log "stopping this run… (Ctrl-C again won't interrupt this)"
    if [ "$USE_VACUUM" -eq 1 ]; then
        docker exec "$CONTAINER" bash -lc \
          "source /opt/ros/noetic/setup.bash 2>/dev/null; rosservice call /orio/lbl_cup/off 2>/dev/null" \
          >/dev/null 2>&1 || true
    fi
    # Killing a `docker exec` client does not stop the process it started: stop them inside.
    docker exec "$CONTAINER" pkill -f 'labelling_arm_test\.py|pneumatic_control_recovery\.py' \
        >/dev/null 2>&1 || true
    for pid in "${PIDS[@]:-}"; do kill "$pid" 2>/dev/null || true; done
    log "done. Full log: $LOG_FILE"
    if [ -d "$IMAGE_DIR" ]; then
        _n=$(find "$IMAGE_DIR" -name 'label_*.png' -not -name '*_color.png' 2>/dev/null | wc -l)
        [ "$_n" -gt 0 ] && log "perception images ($_n): $IMAGE_DIR"
    fi
    log "the stack (control PC, ZED, Fast-FoundationStereo, perception) is still running; the"
    log "next run reuses it. To stop everything: bash research/labelling_arm_test/stop_labelling_test.sh"
    sync; sleep 0.2
}
trap 'cleanup; exit 130' INT
trap 'cleanup; exit 143' TERM
trap 'cleanup; exit 129' HUP
trap cleanup EXIT

follow roscore roscore
follow zed zed
follow ffs ffs
follow perception perception

if [ "$FRESH" -eq 1 ]; then
    log "--fresh: stopping the running stack first…"
    bash "$HERE/stop_labelling_test.sh"
fi

# ── roscore ──────────────────────────────────────────────────────────────────
# Every node registers with the master: if it is gone nothing else is reusable.
STACK_FRESH=0
if roscore_up; then
    log "roscore already running — reusing the stack where it is healthy."
else
    STACK_FRESH=1
    log "no roscore — starting the stack from scratch…"
    for s in roscore zed ffs perception; do stop_detached "$s"; done
    docker rm -f orio_roscore "$CONTAINER" "$FFS_CONTAINER" "$LBL_PERC_CONTAINER" >/dev/null 2>&1 || true
    rm -f "$STACK_DIR"/*.conf
    log "starting roscore…"
    start_detached roscore bash "$DOCKER_DIR/run_roscore.sh"
    wait_for_roscore 30 || { log "roscore did not start in 30s — check the [roscore] lines. Aborting."; exit 1; }
fi

# ── Robot 2's control PC (iam-luisa) ─────────────────────────────────────────
# Its stack lives in a tmux session inside a container on the control PC and survives on
# its own; start_control_pc_luisa.sh is idempotent. Skipped with --no-motion.
if [ "$NO_MOTION" -eq 0 ]; then
    if [ "$UNLOCK" -eq 1 ]; then
        log "unlocking robot 2's brakes (the arm moves slightly)…"
        python3 "$BRINGUP/lock_arms.py" --unlock 2 2>&1 | prefix desk
        [ "${PIPESTATUS[0]}" -ne 0 ] && { log "unlock failed — see the [desk] lines. Aborting."; exit 1; }
    fi
    # Stop the stack, then clear franka_interface's shared memory ONLY once nothing runs:
    # a crash leaves its mutex locked there and the next start hangs forever at "Will try
    # to acquire lock while setting franka_interface status" (as on iam-doc, see
    # stop_dexnet_pnp.sh), while deleting it under a live process corrupts the next start.
    # The remote block has no `#` comments: ssh flattens it to one line.
    luisa_kill_stack() {
        timeout 40 ssh -o BatchMode=yes -o ConnectTimeout=8 "$LUISA" \
            "docker exec -i franka-interface bash -lc '
                tmux kill-session -t franka 2>/dev/null
                pkill -TERM -f \"[f]ranka_interface --robot_ip\" 2>/dev/null
                for _ in 1 2 3 4 5 6; do pgrep -f \"[f]ranka_interface --robot_ip\" >/dev/null || break; sleep 0.5; done
                pkill -KILL -f \"[f]ranka_interface --robot_ip\" 2>/dev/null
                pkill -KILL -f \"[r]oslaunch franka_ros_interface\" 2>/dev/null
                sleep 1
                if ! pgrep -f \"[f]ranka_interface --robot_ip\" >/dev/null; then
                    rm -f /dev/shm/run_loop_* /dev/shm/current_robot_state* /dev/shm/franka_interface_state_info_mutex
                fi
                true'" >/dev/null 2>&1 || true
    }
    # The stack's tmux session outlives franka_interface itself: after a user stop or an
    # unrecoverable fault it aborts ("terminate called ... Aborted (core dumped)") while
    # the ROS action server beside it keeps publishing robot_state. A session without the
    # process is that case, and the only way back is to start the stack again.
    luisa_fi_dead() {
        local out
        out="$(timeout 20 ssh -o BatchMode=yes -o ConnectTimeout=8 "$LUISA" \
            "docker exec franka-interface bash -lc 'tmux has-session -t franka 2>/dev/null && echo SESSION; pgrep -f \"[f]ranka_interface --robot_ip\" >/dev/null && echo PROCESS'" 2>/dev/null)"
        [[ "$out" == *SESSION* && "$out" != *PROCESS* ]]
    }
    if [ "$STACK_FRESH" -eq 1 ]; then
        # Fresh master: a leftover robot-2 stack is registered with a dead master. Restart it.
        luisa_kill_stack
    elif luisa_fi_dead; then
        log "franka-interface on $LUISA has exited (user stop / fault) — restarting robot 2's stack…"
        luisa_kill_stack
        sleep 2
    fi
    log "robot 2 control PC ($LUISA)…"
    luisa_out="$(bash "$BRINGUP/start_control_pc_luisa.sh" 2>&1)"; luisa_rc=$?
    echo "$luisa_out" | prefix luisa
    [ "$luisa_rc" -ne 0 ] && { log "robot 2 bring-up failed. Is iam-luisa up (ping iam-luisa)? Aborting."; exit 1; }
fi

# ── Main container (frankapy, the ZED driver, pneumatics live here) ───────────
CAMS_FRESH="$STACK_FRESH"
if container_up "$CONTAINER"; then
    log "main container already running."
    # A run killed hard can leave its arm script or pneumatics in there; two would fight.
    docker exec "$CONTAINER" pkill -f 'labelling_arm_test\.py|pneumatic_control_recovery\.py' >/dev/null 2>&1 || true
else
    CAMS_FRESH=1
    stop_detached zed
    log "starting main docker container…"
    run_bg docker bash -lc "cd '$REPO' && ORIO_NO_TTY=1 ORIO_LOGGING=0 bash orio_run_docker.sh"
    wait_for_container "$CONTAINER" 60 || { log "'$CONTAINER' did not start in 60s — check the [docker] lines. Aborting."; exit 1; }
fi

# ── Pre-flight: is robot 2 ready? (before the heavy services) ────────────────
if [ "$NO_MOTION" -eq 0 ]; then
    wait_for_topic "/robot_state_publisher_node_2/robot_state" 60 \
        || { log "robot 2 never published its state — its control PC did not come up. Aborting."; exit 1; }
    log "pre-flight: checking robot 2 is ready…"
    docker exec "$CONTAINER" bash -c \
        "source /home/ros_ws/devel/setup.bash && python3 $ARM_SCRIPT_CONT --check-only" 2>&1 | prefix check
    if [ "${PIPESTATUS[0]}" -ne 0 ]; then
        echo
        log "ROBOT 2 NOT READY — aborting before the cameras and perception start."
        log "  - brakes locked / not activated: python3 src/devel_packages/orio_bringup/lock_arms.py --unlock 2"
        log "    (or re-run with --unlock); a red light needs the error acknowledged in Desk first."
        log "  - franka-interface window on the control PC:"
        log "      ssh -t $LUISA 'cd ~/franka-interface-docker && ./run.sh tmux attach -t franka'"
        log "    If it crashed, re-running this script restarts it. If it is stuck, add --fresh."
        exit 1
    fi
    log "robot 2 is ready."
fi

# ── ZED ──────────────────────────────────────────────────────────────────────
# The driver publishes a topic only while something subscribes, so the check itself
# (one rostopic echo) is what makes an idle driver publish. zed_only.launch starts it
# without its own depth (no neural model); --depth zed needs that depth, so NEURAL then.
# A running driver is reused only if it was started with the depth mode this run needs.
ZED_DEPTH_MODE=$([ "$DEPTH" = zed ] && echo NEURAL || echo NONE)
zed_depth_mode() {
    timeout 6 docker exec "$ROS1_CONTAINER" bash -c \
        "$_ROS1_SETUP; rosparam get /zedm/zed_node/depth/depth_mode" 2>/dev/null
}
if [ "$CAMS_FRESH" -eq 0 ] && topic_live /zedm/zed_node/left/image_rect_color \
        && [ "$(zed_depth_mode)" = "$ZED_DEPTH_MODE" ]; then
    log "ZED already publishing (depth mode $ZED_DEPTH_MODE)."
else
    log "starting ZED camera (depth mode $ZED_DEPTH_MODE)…"
    stop_detached zed
    docker exec "$CONTAINER" bash -c "pkill -f 'zed_only.launch|cameras.launch'" >/dev/null 2>&1 || true
    start_detached zed docker exec "$CONTAINER" bash -c \
        "source /home/ros_ws/devel/setup.bash && roslaunch manipulation zed_only.launch depth_mode:=$ZED_DEPTH_MODE"
    if ! wait_for_topic_publishing /zedm/zed_node/left/image_rect_color 40; then
        echo
        log "CAMERA NOT PUBLISHING: /zedm/zed_node/left/image_rect_color. Check the ZED is"
        log "connected and the [zed] lines above (docs/TROUBLESHOOTING.md). Aborting."
        exit 1
    fi
fi

# ── Fast-FoundationStereo depth ──────────────────────────────────────────────
if [ "$DEPTH" = ffs ]; then
    FFS_CONF="scale=${ORIO_FFS_SCALE:-0.5} iters=${ORIO_FFS_ITERS:-8} ckpt=${ORIO_FFS_CHECKPOINT:-default} rate=${ORIO_FFS_RATE:-0}"
    if [ "$STACK_FRESH" -eq 0 ] && container_up "$FFS_CONTAINER" && conf_is ffs "$FFS_CONF"; then
        log "Fast-FoundationStereo already running."
    else
        log "starting Fast-FoundationStereo ($FFS_CONF)…"
        stop_detached ffs
        start_detached ffs bash -lc "bash '$REPO/src/devel_packages/orio_bringup/docker/run_ffs.sh' && docker logs -f $FFS_CONTAINER"
        echo "$FFS_CONF" >"$STACK_DIR/ffs.conf"
    fi
    if ! wait_for_service /ffs/compute 180; then
        log "/ffs/compute did not come up in 180s — check the [ffs] lines above (image built?"
        log "checkpoint found? see orio_bringup/docker/run_ffs.sh). Aborting."
        rm -f "$STACK_DIR/ffs.conf"
        exit 1
    fi
fi

# ── Labelling perception ─────────────────────────────────────────────────────
mkdir -p "$IMAGE_DIR"
ln -sfn "${IMAGE_DIR##*/}" "$IMAGE_LINK"
PERC_CONF="depth=$DEPTH prompt=${ORIO_LBL_PROMPT:-default} box_threshold=${ORIO_LBL_BOX_THRESHOLD:-default}"
if [ "$STACK_FRESH" -eq 0 ] && container_up "$LBL_PERC_CONTAINER" && conf_is perception "$PERC_CONF"; then
    log "labelling perception already running ($PERC_CONF)."
else
    log "starting labelling perception ($PERC_CONF; models take a minute or two)…"
    stop_detached perception
    start_detached perception bash -lc "ORIO_LBL_IMAGE_DIR='$IMAGE_LINK' bash '$HERE/perception/run_lbl_perception.sh' $DEPTH && docker logs -f $LBL_PERC_CONTAINER"
    echo "$PERC_CONF" >"$STACK_DIR/perception.conf"
fi
if ! wait_for_service /labelling_test/compute_label_pose 240; then
    log "/labelling_test/compute_label_pose did not come up in 240s — check the [perception]"
    log "lines above (weights, GPU memory, camera frames). Aborting."
    rm -f "$STACK_DIR/perception.conf"
    exit 1
fi
log "perception images → $IMAGE_DIR"

# ── Pneumatics (this run only) ───────────────────────────────────────────────
if [ "$USE_VACUUM" -eq 1 ]; then
    log "starting pneumatics…"
    # In the main container (the host python has no rospy); the ClearCore's serial port is
    # the same device node there (--privileged, -v /dev:/dev).
    run_bg pneumatics docker exec -e ORIO_LOGGING=0 \
        -e ORIO_VACUUM_PORT="${ORIO_VACUUM_PORT:-/dev/ttyACM0}" "$CONTAINER" bash -c \
        "source /home/ros_ws/devel/setup.bash && cd /home/ros_ws/src/devel_packages/orio && \
         python3 -u pneumatic_control_recovery.py"
    # The cup services must exist BEFORE the arm moves, or every cycle runs its whole
    # motion with the cup off and times out on them half way.
    if ! wait_for_service /orio/lbl_cup/on 30; then
        echo
        log "VACUUM NOT AVAILABLE: /orio/lbl_cup/on never appeared. Check the [pneumatics]"
        log "lines above: ClearCore plugged in (ls -l /dev/ttyACM*; ORIO_VACUUM_PORT)?"
        log "Re-run without --vacuum for a dry run. Aborting."
        exit 1
    fi
else
    log "pneumatics DISABLED (dry run): the cup holds nothing."
fi

# ── The arm, in the FOREGROUND so its prompt can read the keyboard ───────────
log "arm flags: ${ARM_ARGS[*]}"
log "starting the labelling arm test (Ctrl-C stops this run; the stack stays up)…"
docker exec -e ORIO_LOGGING=0 -it "$CONTAINER" bash -c \
    "source /home/ros_ws/devel/setup.bash && python3 -u $ARM_SCRIPT_CONT ${ARM_ARGS[*]}"
log "arm test exited."
