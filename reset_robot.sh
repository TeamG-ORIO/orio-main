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
# Env: ORIO_CONTAINER, ORIO_CTRL_PC_USER/HOST/FI_PATH/ROBOT_IP (robot 1), ORIO_LUISA_HOST
#      (robot 2). Ctrl-C at any point runs the teardown. Logs go to logging/reset/.
set -uo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_TAG=reset
LOG_DIR="$REPO/logging/reset"
STACK_DIR="$LOG_DIR/stack"
# shellcheck disable=SC1091
source "$REPO/src/devel_packages/orio_bringup/stack.sh"

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
CONTROL_PC_LOG="$LOG_DIR/$(date +%Y%m%d_%H%M%S).robot${ROBOT}.control_pc.log"

# ── Teardown: only what this script started ──────────────────────────────────
cleanup() {
    echo
    if [ "$KEEP" -eq 1 ]; then
        log "--keep: leaving everything up."
        [ "$CTRL_PC_STARTED" -eq 1 ] && log "  robot 1 control PC: stop with  bash stop_dexnet_pnp.sh  (or ssh $CTRL_PC_USER@$CTRL_PC_HOST and kill franka_interface)"
        [ "$LUISA_STARTED" -eq 1 ]   && log "  robot 2 control PC: ssh -t $LUISA_PC 'cd ~/franka-interface-docker && ./run.sh tmux kill-session -t franka'"
        log "  containers: docker rm -f $ROS1_CONTAINER $CONTAINER"
        kill_bg
        return
    fi
    log "shutting down what this script started… (Ctrl-C again won't interrupt this)"
    kill_bg
    if [ "$CTRL_PC_STARTED" -eq 1 ]; then
        stop_detached control_pc_fi
        stop_detached control_pc_ros
        control_pc_teardown
    fi
    if [ "$LUISA_STARTED" -eq 1 ]; then
        log "stopping robot 2 stack on $LUISA_PC…"
        luisa_kill_session || true
    fi
    [ "$CONTAINER_STARTED" -eq 1 ] && docker rm -f "$CONTAINER" >/dev/null 2>&1
    if [ "$STACK_FRESH" -eq 1 ]; then
        stop_detached roscore
        docker rm -f "$ROS1_CONTAINER" >/dev/null 2>&1
    fi
    log "done."
    [ -s "$CONTROL_PC_LOG" ] && log "control PC log: $CONTROL_PC_LOG"
    sync; sleep 0.2
}
install_cleanup_traps

follow roscore roscore

# ── 1. ROS1 master ───────────────────────────────────────────────────────────
STACK_SERVICES=(roscore control_pc_fi control_pc_ros)
stack_bringup_roscore || exit 1

# ── 2. Brakes ────────────────────────────────────────────────────────────────
if [ "$UNLOCK" -eq 1 ]; then
    unlock_robot "$ROBOT" || exit 1
fi

# ── 3. Control PC ────────────────────────────────────────────────────────────
if [ "$ROBOT" -eq 1 ]; then
    control_pc_bringup || exit 1
else
    luisa_bringup || exit 1
fi

# ── 4. orio container (frankapy lives on its PYTHONPATH) ─────────────────────
main_container_bringup || exit 1

# ── 5. Reset (foreground) ────────────────────────────────────────────────────
wait_for_topic "/robot_state_publisher_node_${ROBOT}/robot_state" 60 \
    || { log "robot $ROBOT never published its state - the control PC did not come up. See $CONTROL_PC_LOG. Aborting."; exit 1; }
log "running reset_joints.py ${RESET_ARGS[*]} on robot $ROBOT - ARM WILL MOVE"
run_in_container "python3 reset_joints.py ${RESET_ARGS[*]}"
rc=$?
if [ "$rc" -eq 0 ]; then
    log "reset finished."
else
    log "reset FAILED (exit $rc). If FrankaArm timed out: the brakes are probably locked -"
    log "re-run with --unlock, or  python3 src/devel_packages/orio_bringup/lock_arms.py --unlock $ROBOT"
fi
exit "$rc"
