#!/usr/bin/env bash
# Grasp test from ONE terminal: plan a suction grasp on the live Xtion frame, draw it, and send
# the arm there. No vacuum, no perception node, no output box, no ZED.
#
# Brings up only what that needs: the ROS1 master container, the robot-1 control PC, the orio
# container, the Xtion driver, and a planner container of its own (orio_grasp_test_planner,
# service /grasp_test_planner/plan_grasp) serving the network vendored in
# src/devel_packages/orio_grasping. Anything already running (e.g. a run_dexnet_pnp_single.sh
# stack) is reused and left alone. On exit (Ctrl-C too) the planner and a control PC this
# script started are stopped; the camera stays on, with the master and the orio container it
# runs in, so the next run reuses it instead of restarting it. bash stop_dexnet_pnp.sh stops it.
# Each round (grasp_test.py): reset the arm, plan, Enter to move there, Enter to reset.
#
#   bash grasp_test.sh                       # seg_v2_deformables, arm goes down to the grasp
#   bash grasp_test.sh --no-arm              # plan and draw only: no control PC, no motion
#   bash grasp_test.sh --hover               # stop 10 cm above the grasp instead
#   bash grasp_test.sh --straight-down       # ignore the grasp tilt, approach vertically
#   bash grasp_test.sh --offset-x 0.00 --offset-y -0.015
#                                            # world-frame XY correction (metres) added to each
#                                            #   grasp (defaults: $ORIO_PNP_OFFSET_X/Y, else 0)
#   bash grasp_test.sh --min-q 0.2           # lowest planner score sent to the arm (default 0.30)
#   bash grasp_test.sh --max-tilt-deg 30     # steepest grasp sent to the arm (default 35)
#   bash grasp_test.sh --checkpoint <tag|path>
#                                            # grasp network: a run tag under orio_grasping/runs
#                                            #   (newest .pt in runs/*_<tag>/; default
#                                            #   seg_v2_deformables, its epoch 59)
#   bash grasp_test.sh --seg-checkpoint <tag|path>
#                                            # segmentation model the network's masks come from:
#                                            #   runs/seg/<tag> (default rgbd_v3_deformables, which
#                                            #   seg_v2_deformables was trained on; seg_v1 goes
#                                            #   with rgbd_v2)
#   bash grasp_test.sh --seg-threshold 0.5   # lowest mask confidence kept (default 0.4)
#
# Everything a run produces goes to data/grasp_test/<stamp>/: per round the planning frame with
# the cups (round_NNN_plan.png), the frame with the arm at the grasp, showing the target and
# where the arm reached (round_NNN_arm.png), rounds.jsonl with the numbers; the planner's panel
# (masks, height map, score map) and inputs per call (grasp_NNNN_*); and session.log.
#
# Env: ORIO_CONTAINER, ORIO_ROS1_CONTAINER, ORIO_CTRL_PC_USER/HOST/FI_PATH/ROBOT_IP,
#      ORIO_CAM_ATTEMPTS, ORIO_PNP_OFFSET_X/Y.
set -uo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export ORIO_REPO="$REPO"
CONTAINER="${ORIO_CONTAINER:-orio_docker_container}"
ROS1_CONTAINER="${ORIO_ROS1_CONTAINER:-orio_roscore}"
PLANNER_CONTAINER=orio_grasp_test_planner
PLANNER_NODE=grasp_test_planner
GRASPING="$REPO/src/devel_packages/orio_grasping"
RGB_TOPIC="/camera/rgb/image_raw"
CTRL_PC_USER="${ORIO_CTRL_PC_USER:-student}"
CTRL_PC_HOST="${ORIO_CTRL_PC_HOST:-iam-doc}"
CTRL_PC_FI_PATH="${ORIO_CTRL_PC_FI_PATH:-Documents/franka-interface}"
CTRL_PC_ROBOT_IP="${ORIO_CTRL_PC_ROBOT_IP:-172.16.0.2}"
WORKSTATION_HOST="$(hostname)"   # the control PC points ROS_MASTER_URI at this name
# shellcheck disable=SC1091
source "$REPO/src/devel_packages/orio_bringup/tmux/wait_for.sh"

# ── Args ────────────────────────────────────────────────────────────────────
NO_ARM=0
CKPT=seg_v2_deformables
SEG_CKPT=rgbd_v3_deformables
SEG_THRESHOLD=0.4
MIN_Q=0.30
NODE_ARGS=()
OFFSET_X="${ORIO_PNP_OFFSET_X:-0.0}"
OFFSET_Y="${ORIO_PNP_OFFSET_Y:-0.0}"
while [ $# -gt 0 ]; do
    arg="$1"; shift
    case "$arg" in
        --no-arm)        NO_ARM=1; NODE_ARGS+=(--no-arm) ;;
        --hover|--straight-down) NODE_ARGS+=("$arg") ;;
        --offset-x)      OFFSET_X="${1:?--offset-x needs metres}"; shift ;;
        --offset-y)      OFFSET_Y="${1:?--offset-y needs metres}"; shift ;;
        --max-tilt-deg)  NODE_ARGS+=(--max-tilt-deg "${1:?--max-tilt-deg needs degrees}"); shift ;;
        --min-q)         MIN_Q="${1:?--min-q needs a value in 0..1}"; shift ;;
        --checkpoint)    CKPT="${1:?--checkpoint needs a run tag or a path}"; shift ;;
        --seg-checkpoint) SEG_CKPT="${1:?--seg-checkpoint needs a run tag or a path}"; shift ;;
        --seg-threshold) SEG_THRESHOLD="${1:?--seg-threshold needs a value in 0..1}"; shift ;;
        -h|--help) sed -n '2,39p' "$0"; exit 0 ;;
        *) echo "unknown option: $arg" >&2; exit 2 ;;
    esac
done
NODE_ARGS+=(--offset-x "$OFFSET_X" --offset-y "$OFFSET_Y")
if [ ! -f "$CKPT" ]; then
    tag="$CKPT"
    CKPT="$(ls -t "$GRASPING"/runs/*_"$tag"/*.pt 2>/dev/null | head -1)"
    [ -n "$CKPT" ] || { echo "--checkpoint: no $GRASPING/runs/*_$tag/*.pt" >&2; exit 2; }
fi
CKPT="$(readlink -f "$CKPT")"
if [ ! -f "$SEG_CKPT" ]; then
    tag="$SEG_CKPT"
    SEG_CKPT="$GRASPING/runs/seg/$tag/checkpoint_best_total.pth"
    [ -f "$SEG_CKPT" ] || { echo "--seg-checkpoint: no $SEG_CKPT" >&2; exit 2; }
fi
SEG_CKPT="$(readlink -f "$SEG_CKPT")"

STAMP="$(date +%Y%m%d_%H%M%S)"
OUT="$REPO/data/grasp_test/$STAMP"
OUT_CONT="/home/ros_ws/data/grasp_test/$STAMP"   # the same folder in both containers
# data/ is created by docker (root), so make data/grasp_test once through a container and
# hand it to this user; the containers write into it as root, this script as the user.
if ! mkdir -p "$OUT" 2>/dev/null; then
    docker run --rm -v "$REPO/data:/data" "${ORIO_ROS1_IMAGE:-orio_docker}" \
        bash -c "mkdir -p /data/grasp_test && chown $(id -u):$(id -g) /data/grasp_test" \
        && mkdir -p "$OUT" || { echo "cannot create $OUT" >&2; exit 1; }
fi
CONTROL_PC_LOG="$OUT/control_pc.log"
# The whole run, minus colour codes and carriage returns, also goes to session.log.
# The terminal gets CRLF line ends: `docker exec -it` puts it in raw mode while the test loop
# runs, and there the [planner] / [cameras] lines (plain LF) would each start where the last
# one ended. The test loop's own lines already end in CRLF, so trailing CRs are not doubled.
exec > >(tee >(sed -u 's/\x1b\[[0-9;]*m//g; s/\r$//' >>"$OUT/session.log") | sed -u 's/\r*$/\r/') 2>&1

log() { echo "[grasp_test] $*"; }
prefix() { local label="$1"; sed -u "s/^/[$label] /" 2>/dev/null || sed "s/^/[$label] /"; }
ts_prefix() { while IFS= read -r line; do printf '%s %s\n' "$(date +%H:%M:%S)" "$line"; done; }
PIDS=()
run_bg() { local label="$1"; shift; ( "$@" 2>&1 | prefix "$label" ) & PIDS+=($!); }
container_running() { docker ps --format '{{.Names}}' 2>/dev/null | grep -qx "$1"; }
xtion_publishing() {  # true once the RGB topic actually streams a frame
    timeout 6 docker exec "$ROS1_CONTAINER" bash -c \
        "$_ROS1_SETUP; timeout 4 rostopic echo -n 1 $RGB_TOPIC" >/dev/null 2>&1
}

# ── Teardown: only what this script started ──────────────────────────────────
# Not the camera, nor the master and container it needs: they stay up for the next run.
STARTED_DOC=0
_CLEANED=0
cleanup() {
    [ "$_CLEANED" -eq 1 ] && return
    _CLEANED=1
    trap '' INT TERM HUP
    echo
    log "shutting down what this script started… (Ctrl-C again won't interrupt this)"
    docker rm -f "$PLANNER_CONTAINER" >/dev/null 2>&1 || true
    for pid in "${PIDS[@]:-}"; do [ -n "$pid" ] && kill "$pid" 2>/dev/null || true; done
    if [ "$STARTED_DOC" -eq 1 ]; then
        # Killing the local ssh clients does not stop the remote processes. Stop them there,
        # gracefully: SIGKILL leaves franka-interface's shared-memory mutex locked and the
        # next start hangs (see run_dexnet_pnp_single.sh). Same as reset_robot.sh.
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
    log "done. The camera stays on (bash stop_dexnet_pnp.sh stops it). Results: ${OUT#"$REPO"/}/"
    sync; sleep 0.2
}
trap 'cleanup; exit 130' INT
trap 'cleanup; exit 143' TERM
trap 'cleanup; exit 129' HUP
trap cleanup EXIT

log "grasp network: $CKPT"
log "segmentation: $SEG_CKPT (masks above $SEG_THRESHOLD)"
log "results → ${OUT#"$REPO"/}/"

# ── 1. ROS1 master (container; this host is 22.04/ROS2 with no roscore) ───────
if container_running "$ROS1_CONTAINER"; then
    log "roscore container '$ROS1_CONTAINER' already running - reusing."
    ROSCORE_WAS_UP=1
else
    log "starting roscore…"
    ROSCORE_WAS_UP=0
    docker rm -f "$ROS1_CONTAINER" >/dev/null 2>&1 || true
    docker run -d --rm --name "$ROS1_CONTAINER" --net host "${ORIO_ROS1_IMAGE:-orio_docker}" \
        bash -c 'source /opt/ros/noetic/setup.bash && exec roscore' >/dev/null \
        || { log "could not start the roscore container. Aborting."; exit 1; }
fi
wait_for_roscore 30 || { log "roscore did not come up in 30s. Aborting."; exit 1; }

# ── 2. Control PC (robot 1), as in reset_robot.sh ────────────────────────────
if [ "$NO_ARM" -eq 0 ]; then
    log "checking ssh to $CTRL_PC_HOST…"
    if ! timeout 15 ssh -o BatchMode=yes -o ConnectTimeout=10 "$CTRL_PC_USER@$CTRL_PC_HOST" true >/dev/null 2>&1; then
        log "CANNOT SSH to $CTRL_PC_USER@$CTRL_PC_HOST. Is it up? (or use --no-arm). Aborting."
        exit 1
    fi
    fi_running="$(timeout 15 ssh -o BatchMode=yes "$CTRL_PC_USER@$CTRL_PC_HOST" \
        "pgrep -f '[f]ranka_interface --robot_ip' | wc -l" 2>/dev/null | tr -dc 0-9)"
    if [ "${fi_running:-0}" -gt 0 ] && [ "$ROSCORE_WAS_UP" -eq 1 ]; then
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
        log "starting robot 1 control PC over ssh - raw output in ${CONTROL_PC_LOG#"$REPO"/}"
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
        ( tail -n0 -F "$CONTROL_PC_LOG" 2>/dev/null \
            | grep --line-buffered -E 'has died|control loop|libfranka|terminate called|core dumped|segmentation|refused|Connection reset|cannot connect|reflex|e-stop|FCI' \
            | prefix doc ) &
        PIDS+=($!)
    fi
fi

# ── 3. orio container (frankapy, the Xtion driver, grasp_test.py) ────────────
if container_running "$CONTAINER"; then
    log "container '$CONTAINER' already running - reusing."
else
    log "starting $CONTAINER…"
    run_bg docker bash -lc "cd '$REPO' && ORIO_NO_TTY=1 bash orio_run_docker.sh"
    wait_for_container "$CONTAINER" 60 || { log "'$CONTAINER' did not start in 60s. Aborting."; exit 1; }
fi

# ── 4. Xtion, unless it is already streaming (retries as in snap_xtion.sh) ───
# Started in its own session (setsid), as run_dexnet_pnp_single.sh does, so it keeps running
# after this script exits; its output goes to data/grasp_test/xtion.log.
XTION_LOG="$REPO/data/grasp_test/xtion.log"
if xtion_publishing; then
    log "Xtion already publishing $RGB_TOPIC - reusing."
else
    cam_ok=0
    for attempt in $(seq 1 "${ORIO_CAM_ATTEMPTS:-5}"); do
        log "starting Xtion (attempt $attempt/${ORIO_CAM_ATTEMPTS:-5})…"
        docker exec "$CONTAINER" bash -c "pkill -f xtion_only.launch" >/dev/null 2>&1 || true
        : >"$XTION_LOG"
        setsid docker exec "$CONTAINER" bash -c \
            "source /home/ros_ws/devel/setup.bash && roslaunch manipulation xtion_only.launch" \
            >>"$XTION_LOG" 2>&1 </dev/null &
        ( tail -n0 -F --pid=$$ "$XTION_LOG" 2>/dev/null | prefix cameras ) &
        PIDS+=($!)
        deadline=$(( SECONDS + 12 ))
        while [ "$SECONDS" -lt "$deadline" ]; do
            if xtion_publishing; then cam_ok=1; break; fi
            sleep 0.5
        done
        [ "$cam_ok" -eq 1 ] && break
        log "Xtion did not publish within 12s on attempt $attempt."
    done
    [ "$cam_ok" -eq 1 ] || { log "CAMERA NOT PUBLISHING: $RGB_TOPIC. Check the Xtion cable. Aborting."; exit 1; }
    log "Xtion is publishing."
fi

# ── 5. Grasp planner (its own container and node, beside any pick-and-place one) ──
log "starting the grasp planner (min score $MIN_Q)…"
ORIO_GRASPING="$GRASPING" SUCTION_CHECKPOINT="$CKPT" SUCTION_MIN_SCORE="$MIN_Q" \
    SUCTION_SEG_CHECKPOINT="$SEG_CKPT" SUCTION_SEG_THRESHOLD="$SEG_THRESHOLD" \
    SUCTION_CONTAINER_NAME="$PLANNER_CONTAINER" SUCTION_NODE_NAME="$PLANNER_NODE" \
    ORIO_LOGGING=0 ORIO_AFFORDANCE_DIR="$OUT_CONT" \
    bash "$REPO/src/devel_packages/orio_bringup/docker/run_suction.sh" 2>&1 | prefix planner
[ "${PIPESTATUS[0]}" -eq 0 ] || { log "could not start the planner. Aborting."; exit 1; }
run_bg planner docker logs -f "$PLANNER_CONTAINER"
wait_for_service "/$PLANNER_NODE/plan_grasp" 120 \
    || { log "planner did not come up in 120s - see the [planner] lines above. Aborting."; exit 1; }

# ── 6. The test loop (foreground, owns the keyboard) ─────────────────────────
if [ "$NO_ARM" -eq 0 ]; then
    wait_for_topic "/robot_state_publisher_node_1/robot_state" 60 \
        || { log "robot 1 never published its state - see ${CONTROL_PC_LOG#"$REPO"/}. Aborting."; exit 1; }
    log "ARM WILL MOVE: each grasp waits for Enter first."
fi
docker exec -it "$CONTAINER" bash -c \
    "source /home/ros_ws/devel/setup.bash && cd /home/ros_ws/src/devel_packages/orio && \
     python3 -u grasp_test.py --out '$OUT_CONT' --service '/$PLANNER_NODE/plan_grasp' ${NODE_ARGS[*]}"
