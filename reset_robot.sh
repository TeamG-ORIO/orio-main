#!/usr/bin/env bash
# Reset the Franka arms to their home joints from ONE terminal (no tmux).
#
# Brings up only what the reset needs — the ROS1 master container, each robot's control PC
# (franka-interface + ROS action server), the orio container — runs reset_joints.py inside
# the container in the foreground (robot 1, then robot 2), then tears down what THIS script
# started. Anything that was already running (roscore, orio_docker_container, a live
# control PC) is reused and left alone.
#
#   bash reset_robot.sh                 # both arms: robot 1 (iam-doc), then robot 2 (iam-luisa)
#   bash reset_robot.sh --robot 1       # robot 1 only (also -1)
#   bash reset_robot.sh --robot 2       # robot 2 only (also -2)
#   bash reset_robot.sh --unlock        # open the brakes first (lock_arms.py --unlock N)
#   bash reset_robot.sh --pose          # reset_pose() instead of reset_joints()
#   bash reset_robot.sh --keep          # leave roscore/container/control PCs up afterwards
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
ROBOT=both
UNLOCK=0
KEEP=0
GRIPPER=none
RESET_ARGS=()
while [ $# -gt 0 ]; do
    case "$1" in
        --robot) shift; ROBOT="$1" ;;
        --robot=*) ROBOT="${1#*=}" ;;
        -1) ROBOT=1 ;;
        -2) ROBOT=2 ;;
        --unlock) UNLOCK=1 ;;
        --pose) RESET_ARGS+=(--use_pose) ;;
        --keep) KEEP=1 ;;
        --open-gripper)  GRIPPER=open ;;
        --close-gripper) GRIPPER=close ;;
        -h|--help) sed -n '2,20p' "$0"; exit 0 ;;
        *) echo "unknown option: $1" >&2; exit 2 ;;
    esac
    shift
done
case "$ROBOT" in
    1|2)  ROBOTS=("$ROBOT") ;;
    both) ROBOTS=(1 2) ;;
    *) echo "--robot must be 1, 2 or both" >&2; exit 2 ;;
esac
GRIPPER_ARGS=()
case "$GRIPPER" in
    none)  GRIPPER_ARGS=(--no_gripper) ;;
    close) GRIPPER_ARGS=(--close_grippers) ;;
    open)  ;;   # reset_joints.py opens the hand by default
esac
# Only robot 1 has a hand: with both arms, the gripper option applies to robot 1 alone.
reset_args() {  # $1 = robot number
    local a=("${RESET_ARGS[@]}" --robot_num "$1")
    if [ "$1" -eq 2 ] && [ "${#ROBOTS[@]}" -gt 1 ]; then
        a+=(--no_gripper)
    else
        a+=("${GRIPPER_ARGS[@]}")
    fi
    echo "${a[*]}"
}
CONTROL_PC_LOG="$LOG_DIR/$(date +%Y%m%d_%H%M%S).robot1.control_pc.log"

# ── Teardown: only what this script started ──────────────────────────────────
cleanup() {
    echo
    if [ "$KEEP" -eq 1 ]; then
        log "--keep: leaving everything up."
        [ "$CTRL_PC_STARTED" -eq 1 ] && log "  robot 1 control PC: stop with  bash stop_all.sh  (or ssh $CTRL_PC_USER@$CTRL_PC_HOST and kill franka_interface)"
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
    for r in "${ROBOTS[@]}"; do unlock_robot "$r" || exit 1; done
fi

# ── 3. Control PCs ───────────────────────────────────────────────────────────
for r in "${ROBOTS[@]}"; do
    if [ "$r" -eq 1 ]; then
        control_pc_bringup || exit 1    # robot 1, iam-doc
    else
        luisa_bringup || exit 1         # robot 2, iam-luisa
    fi
done

# ── 4. orio container (frankapy lives on its PYTHONPATH) ─────────────────────
main_container_bringup || exit 1

# ── 5. Reset (foreground, one arm at a time) ─────────────────────────────────
# Wait for every arm's state first, so a dead control PC aborts before anything moves.
# A freshly started luisa roslaunch takes longer to publish.
for r in "${ROBOTS[@]}"; do
    if [ "$r" -eq 1 ]; then
        wait_for_topic "/robot_state_publisher_node_1/robot_state" 60 \
            || { log "robot 1 never published its state - the control PC did not come up. See $CONTROL_PC_LOG. Aborting."; exit 1; }
    else
        wait_for_topic "/robot_state_publisher_node_2/robot_state" 90 \
            || { log "robot 2 never published its state - check the [luisa] lines above. Aborting."; exit 1; }
    fi
done
rc=0
FAILED=()
for r in "${ROBOTS[@]}"; do
    args="$(reset_args "$r")"
    log "running reset_joints.py $args on robot $r - ARM WILL MOVE"
    run_in_container "python3 reset_joints.py $args"
    r_rc=$?
    if [ "$r_rc" -eq 0 ]; then
        log "robot $r reset finished."
    else
        log "robot $r reset FAILED (exit $r_rc)."
        FAILED+=("$r"); rc="$r_rc"
    fi
done
if [ "${#FAILED[@]}" -gt 0 ]; then
    log "reset FAILED on robot ${FAILED[*]}. If FrankaArm timed out: the brakes are probably locked -"
    log "re-run with --unlock, or  python3 src/devel_packages/orio_bringup/lock_arms.py --unlock N"
fi
exit "$rc"
