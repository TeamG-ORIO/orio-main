#!/usr/bin/env bash
# Reset a Franka arm to its home joints from ONE terminal (no tmux).
#
# Brings up only what the reset needs — the ROS1 master container, the robot's control PC
# (franka-interface + ROS action server), the orio container — runs reset_joints.py inside
# the container in the foreground, then tears down what THIS script started. Anything that
# was already running (roscore, orio_docker_container, a live control PC) is reused and
# left alone.
#
#   bash reset_robot.sh                 # robot 1 (iam-doc), reset_joints()
#   bash reset_robot.sh --robot 2       # robot 2 (iam-luisa)
#   bash reset_robot.sh --unlock        # open the brakes first (lock_arms.py --unlock N)
#   bash reset_robot.sh --pose          # reset_pose() instead of reset_joints()
#   bash reset_robot.sh --keep          # leave roscore/container/control PC up afterwards
#   bash reset_robot.sh --open-gripper  # also open the Franka Hand (robot 1 with hand only)
#   bash reset_robot.sh --close-gripper # also close it
#
# Env: ORIO_FRANKAPY, ORIO_CONTAINER, ORIO_CTRL_PC_USER/HOST/FI_PATH/ROBOT_IP (robot 1),
#      ORIO_LUISA_HOST (robot 2). Ctrl-C at any point runs the teardown.
set -uo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export ORIO_REPO="$REPO"
CONTAINER="${ORIO_CONTAINER:-orio_docker_container}"
ROS1_CONTAINER="${ORIO_ROS1_CONTAINER:-orio_roscore}"
BRINGUP="$REPO/src/devel_packages/orio_bringup"
# Robot 1 (iam-doc): franka-interface + action server started over ssh, as in
# run_dexnet_pnp_single.sh (start_control_pc.sh's gnome-terminals can't be torn down here).
CTRL_PC_USER="${ORIO_CTRL_PC_USER:-student}"
CTRL_PC_HOST="${ORIO_CTRL_PC_HOST:-iam-doc}"
CTRL_PC_FI_PATH="${ORIO_CTRL_PC_FI_PATH:-Documents/franka-interface}"
CTRL_PC_ROBOT_IP="${ORIO_CTRL_PC_ROBOT_IP:-172.16.0.2}"
WORKSTATION_HOST="$(hostname)"   # the control PC points ROS_MASTER_URI at this name
# Robot 2 (iam-luisa): its stack lives in a tmux session inside a container on the control PC.
LUISA="${ORIO_LUISA_HOST:-snaak@iam-luisa}"
# shellcheck disable=SC1091
source "$BRINGUP/tmux/wait_for.sh"

# ── Args ────────────────────────────────────────────────────────────────────
ROBOT=1
UNLOCK=0
KEEP=0
GRIPPER=none
RESET_ARGS=()
while [ $# -gt 0 ]; do
    case "$1" in
        --robot) shift; ROBOT="$1" ;;
        --robot=*) ROBOT="${1#*=}" ;;
        -2) ROBOT=2 ;;
        --unlock) UNLOCK=1 ;;
        --pose) RESET_ARGS+=(--use_pose) ;;
        --keep) KEEP=1 ;;
        --open-gripper)  GRIPPER=open ;;
        --close-gripper) GRIPPER=close ;;
        -h|--help) sed -n '2,19p' "$0"; exit 0 ;;
        *) echo "unknown option: $1" >&2; exit 2 ;;
    esac
    shift
done
case "$ROBOT" in 1|2) ;; *) echo "--robot must be 1 or 2" >&2; exit 2 ;; esac
RESET_ARGS+=(--robot_num "$ROBOT")
case "$GRIPPER" in
    none)  RESET_ARGS+=(--no_gripper) ;;
    close) RESET_ARGS+=(--close_grippers) ;;
    open)  ;;   # reset_joints.py opens the hand by default
esac

LOG_DIR="$REPO/logging/reset"
mkdir -p "$LOG_DIR"
CONTROL_PC_LOG="$LOG_DIR/$(date +%Y%m%d_%H%M%S).robot${ROBOT}.control_pc.log"

log() { echo "[reset] $*"; }
prefix() { local label="$1"; sed -u "s/^/[$label] /" 2>/dev/null || sed "s/^/[$label] /"; }
ts_prefix() { while IFS= read -r line; do printf '%s %s\n' "$(date +%H:%M:%S)" "$line"; done; }
PIDS=()
run_bg() { local label="$1"; shift; ( "$@" 2>&1 | prefix "$label" ) & PIDS+=($!); }
container_running() { docker ps --format '{{.Names}}' 2>/dev/null | grep -qx "$1"; }

# ── Teardown: only what this script started ──────────────────────────────────
STARTED_ROSCORE=0; STARTED_CONTAINER=0; STARTED_DOC=0; STARTED_LUISA=0
_CLEANED=0
cleanup() {
    [ "$_CLEANED" -eq 1 ] && return
    _CLEANED=1
    trap '' INT TERM HUP
    echo
    if [ "$KEEP" -eq 1 ]; then
        log "--keep: leaving everything up."
        [ "$STARTED_DOC" -eq 1 ]   && log "  robot 1 control PC: stop with  bash stop_demo.sh  (or ssh $CTRL_PC_USER@$CTRL_PC_HOST and kill franka_interface)"
        [ "$STARTED_LUISA" -eq 1 ] && log "  robot 2 control PC: ssh -t $LUISA 'cd ~/franka-interface-docker && ./run.sh tmux kill-session -t franka'"
        log "  containers: docker rm -f $ROS1_CONTAINER $CONTAINER"
        for pid in "${PIDS[@]:-}"; do kill "$pid" 2>/dev/null || true; done
        return
    fi
    log "shutting down what this script started… (Ctrl-C again won't interrupt this)"
    for pid in "${PIDS[@]:-}"; do kill "$pid" 2>/dev/null || true; done
    if [ "$STARTED_DOC" -eq 1 ]; then
        # Killing the local ssh clients does not stop the remote processes. Stop them
        # there, gracefully: SIGKILL leaves franka-interface's shared-memory mutex locked
        # and the next start hangs (see run_dexnet_pnp_single.sh for the full story).
        log "stopping control-PC processes on $CTRL_PC_HOST…"
        timeout 35 ssh -o BatchMode=yes -o ConnectTimeout=8 "$CTRL_PC_USER@$CTRL_PC_HOST" \
            'bash -s' <<'REMOTE_TEARDOWN' 2>/dev/null | prefix doc || true
fi_count() { pgrep -f '[f]ranka_interface --robot_ip' | wc -l; }
pkill -TERM -f '[r]oslaunch franka_ros_interface' 2>/dev/null
pkill -TERM -f '[f]ranka_interface --robot_ip' 2>/dev/null
for _ in $(seq 1 20); do [ "$(fi_count)" -eq 0 ] && break; sleep 0.5; done
if [ "$(fi_count)" -ne 0 ]; then
    echo "franka_interface did not exit on SIGTERM - forcing."
    pkill -KILL -f '[f]ranka_interface --robot_ip' 2>/dev/null; sleep 1
fi
pkill -KILL -f '[r]oslaunch franka_ros_interface' 2>/dev/null
pkill -KILL -f '[f]ranka_ros_interface' 2>/dev/null
if [ "$(fi_count)" -eq 0 ]; then
    rm -f /dev/shm/run_loop_* /dev/shm/current_robot_state* /dev/shm/franka_interface_state_info_mutex 2>/dev/null
else
    echo "WARNING: franka_interface STILL running after SIGKILL - left /dev/shm intact."
fi
REMOTE_TEARDOWN
    fi
    if [ "$STARTED_LUISA" -eq 1 ]; then
        log "stopping robot 2 stack on $LUISA…"
        timeout 30 ssh -o BatchMode=yes -o ConnectTimeout=8 "$LUISA" \
            "docker exec -i franka-interface bash -lc 'tmux kill-session -t franka 2>/dev/null; true'" >/dev/null 2>&1 || true
    fi
    if [ "$STARTED_CONTAINER" -eq 1 ]; then docker rm -f "$CONTAINER" >/dev/null 2>&1 || true; fi
    if [ "$STARTED_ROSCORE" -eq 1 ]; then docker rm -f "$ROS1_CONTAINER" >/dev/null 2>&1 || true; fi
    log "done."
    [ -s "$CONTROL_PC_LOG" ] && log "control PC log: $CONTROL_PC_LOG"
    sync; sleep 0.2
}
trap 'cleanup; exit 130' INT
trap 'cleanup; exit 143' TERM
trap 'cleanup; exit 129' HUP
trap cleanup EXIT

# ── 1. ROS1 master (container; this host is 22.04/ROS2 with no roscore) ───────
if container_running "$ROS1_CONTAINER"; then
    log "roscore container '$ROS1_CONTAINER' already running - reusing."
    ROSCORE_WAS_UP=1
else
    log "starting roscore…"
    ROSCORE_WAS_UP=0; STARTED_ROSCORE=1
    # Same as docker/run_roscore.sh but detached, so --keep can leave it running after we
    # exit (a foreground `docker run` stops the container when its client is killed).
    docker rm -f "$ROS1_CONTAINER" >/dev/null 2>&1 || true
    docker run -d --rm --name "$ROS1_CONTAINER" --net host "${ORIO_ROS1_IMAGE:-orio_docker}" \
        bash -c 'source /opt/ros/noetic/setup.bash && exec roscore' >/dev/null \
        || { log "could not start the roscore container. Aborting."; exit 1; }
    run_bg roscore docker logs -f "$ROS1_CONTAINER"
fi
wait_for_roscore 30 || { log "roscore did not come up in 30s. Aborting."; exit 1; }

# ── 2. Brakes ────────────────────────────────────────────────────────────────
if [ "$UNLOCK" -eq 1 ]; then
    log "unlocking robot $ROBOT brakes (arm moves slightly)…"
    python3 "$BRINGUP/lock_arms.py" --unlock "$ROBOT" 2>&1 | prefix desk
    [ "${PIPESTATUS[0]}" -ne 0 ] && { log "unlock failed - see [desk] lines above. Aborting."; exit 1; }
fi

# ── 3. Control PC ────────────────────────────────────────────────────────────
if [ "$ROBOT" -eq 1 ]; then
    log "checking ssh to $CTRL_PC_HOST…"
    if ! timeout 15 ssh -o BatchMode=yes -o ConnectTimeout=10 "$CTRL_PC_USER@$CTRL_PC_HOST" true >/dev/null 2>&1; then
        log "CANNOT SSH to $CTRL_PC_USER@$CTRL_PC_HOST. Is it up? (ping $CTRL_PC_HOST; nc -vz $CTRL_PC_HOST 22). Aborting."
        exit 1
    fi
    fi_running="$(timeout 15 ssh -o BatchMode=yes "$CTRL_PC_USER@$CTRL_PC_HOST" \
        "pgrep -f '[f]ranka_interface --robot_ip' | wc -l" 2>/dev/null | tr -dc 0-9)"
    if [ "${fi_running:-0}" -gt 0 ] && [ "$ROSCORE_WAS_UP" -eq 1 ]; then
        # A live controller registered with the master that is still running: reuse it.
        # (If we just started a NEW master, the old nodes are orphaned and must restart.)
        log "franka-interface already running on $CTRL_PC_HOST - reusing."
    else
        STARTED_DOC=1
        log "clearing stale control-PC state on $CTRL_PC_HOST…"
        timeout 30 ssh -o BatchMode=yes -o ConnectTimeout=8 "$CTRL_PC_USER@$CTRL_PC_HOST" \
            'bash -s' <<'REMOTE_CLEANUP' >/dev/null 2>&1 || true
fi_count() { pgrep -f '[f]ranka_interface --robot_ip' | wc -l; }
pkill -TERM -f '[f]ranka_interface --robot_ip' 2>/dev/null
pkill -TERM -f '[r]oslaunch franka_ros_interface' 2>/dev/null
for _ in $(seq 1 20); do [ "$(fi_count)" -eq 0 ] && break; sleep 0.5; done
if [ "$(fi_count)" -ne 0 ]; then pkill -KILL -f '[f]ranka_interface --robot_ip' 2>/dev/null; sleep 1; fi
pkill -KILL -f '[r]oslaunch franka_ros_interface' 2>/dev/null
pkill -KILL -f '[f]ranka_ros_interface' 2>/dev/null
if [ "$(fi_count)" -eq 0 ]; then
    rm -f /dev/shm/run_loop_* /dev/shm/current_robot_state* /dev/shm/franka_interface_state_info_mutex 2>/dev/null
fi
REMOTE_CLEANUP
        log "starting robot 1 control PC over ssh - raw output in ${CONTROL_PC_LOG##*/}"
        echo "===== control PC bring-up @ $(date -Is) - $CTRL_PC_USER@$CTRL_PC_HOST =====" >>"$CONTROL_PC_LOG"
        ( ssh -tt -o BatchMode=yes -o ConnectTimeout=10 "$CTRL_PC_USER@$CTRL_PC_HOST" \
              "cd '$CTRL_PC_FI_PATH/build' && \
               stdbuf -oL -eL ./franka_interface --robot_ip '$CTRL_PC_ROBOT_IP' --with_gripper 0 --log 0 --stop_on_error 0" \
              2>&1 | ts_prefix >>"$CONTROL_PC_LOG" ) &
        PIDS+=($!)
        ( ssh -tt -o BatchMode=yes -o ConnectTimeout=10 "$CTRL_PC_USER@$CTRL_PC_HOST" \
              "cd '$CTRL_PC_FI_PATH' && source bash_scripts/set_rosmaster.sh '$CTRL_PC_HOST' '$WORKSTATION_HOST' && \
               source catkin_ws/devel/setup.bash && roslaunch franka_ros_interface franka_ros_interface.launch robot_num:=1" \
              2>&1 | ts_prefix >>"$CONTROL_PC_LOG" ) &
        PIDS+=($!)
        # Surface genuine controller failures live; everything is in the log file.
        ( tail -n0 -F "$CONTROL_PC_LOG" 2>/dev/null \
            | grep --line-buffered -E 'has died|control loop|libfranka|terminate called|core dumped|segmentation|refused|Connection reset|cannot connect|reflex|e-stop|FCI' \
            | prefix doc ) &
        PIDS+=($!)
    fi
else
    if [ "$ROSCORE_WAS_UP" -eq 0 ]; then
        # Fresh master: a leftover robot-2 stack is registered with a dead master. Restart it.
        timeout 30 ssh -o BatchMode=yes -o ConnectTimeout=8 "$LUISA" \
            "docker exec -i franka-interface bash -lc 'tmux kill-session -t franka 2>/dev/null; true'" >/dev/null 2>&1 || true
    fi
    log "starting robot 2 control PC ($LUISA)…"
    luisa_out="$(bash "$BRINGUP/start_control_pc_luisa.sh" 2>&1)"; luisa_rc=$?
    echo "$luisa_out"
    [ "$luisa_rc" -ne 0 ] && { log "robot 2 bring-up failed. Aborting."; exit 1; }
    echo "$luisa_out" | grep -q ALREADY_RUNNING || STARTED_LUISA=1
fi

# ── 4. orio container (frankapy lives on its PYTHONPATH) ─────────────────────
if container_running "$CONTAINER"; then
    log "container '$CONTAINER' already running - reusing."
else
    log "starting $CONTAINER…"
    STARTED_CONTAINER=1
    run_bg docker bash -lc "cd '$REPO' && ORIO_NO_TTY=1 bash orio_run_docker.sh"
    wait_for_container "$CONTAINER" 60 || { log "'$CONTAINER' did not start in 60s. Aborting."; exit 1; }
fi

# ── 5. Reset (foreground) ────────────────────────────────────────────────────
wait_for_topic "/robot_state_publisher_node_${ROBOT}/robot_state" 60 \
    || { log "robot $ROBOT never published its state - the control PC did not come up. See $CONTROL_PC_LOG. Aborting."; exit 1; }
log "running reset_joints.py ${RESET_ARGS[*]} on robot $ROBOT - ARM WILL MOVE"
docker exec -it "$CONTAINER" bash -c \
    "source /home/ros_ws/devel/setup.bash && cd /home/ros_ws/src/devel_packages/orio && \
     python3 reset_joints.py ${RESET_ARGS[*]}"
rc=$?
if [ "$rc" -eq 0 ]; then
    log "reset finished."
else
    log "reset FAILED (exit $rc). If FrankaArm timed out: the brakes are probably locked -"
    log "re-run with --unlock, or  python3 src/devel_packages/orio_bringup/lock_arms.py --unlock $ROBOT"
fi
exit "$rc"
