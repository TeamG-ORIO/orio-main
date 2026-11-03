#!/usr/bin/env bash
# Grasp test from ONE terminal: plan a suction grasp on the live Xtion frame, draw it, and send
# the arm there. No vacuum, no perception node, no output box, no ZED.
#
# Brings up only what that needs: the ROS1 master container, the robot-1 control PC, the orio
# container, the Xtion driver, and a planner container of its own (orio_grasp_test_planner,
# service /grasp_test_planner/plan_grasp) serving the network vendored in
# src/devel_packages/orio_grasping. Anything already running (e.g. a run_pickup-only.sh
# stack) is reused and left alone. On exit (Ctrl-C too) the planner and a control PC this
# script started are stopped; the camera stays on, with the master and the orio container it
# runs in, so the next run reuses it instead of restarting it. bash stop_all.sh stops it.
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
# (masks, height map, score map) and inputs per call (grasp_NNNN_*); and session.log. The
# services it starts log to data/grasp_test/stack/.
#
# Env: ORIO_CONTAINER, ORIO_ROS1_CONTAINER, ORIO_CTRL_PC_USER/HOST/FI_PATH/ROBOT_IP,
#      ORIO_CAM_ATTEMPTS, ORIO_PNP_OFFSET_X/Y.
set -uo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
GRASPING="$REPO/src/devel_packages/orio_grasping"
GT_PLANNER_CONTAINER=orio_grasp_test_planner
GT_PLANNER_NODE=grasp_test_planner

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
        -h|--help) sed -n '2,40p' "$0"; exit 0 ;;
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

LOG_TAG=grasp_test
STACK_DIR="$REPO/data/grasp_test/stack"
CONTROL_PC_LOG="$OUT/control_pc.log"
# shellcheck disable=SC1091
source "$REPO/src/devel_packages/orio_bringup/stack.sh"

# The whole run, minus colour codes and carriage returns, also goes to session.log.
# The terminal gets CRLF line ends: `docker exec -it` puts it in raw mode while the test loop
# runs, and there the [planner] / [cameras] lines (plain LF) would each start where the last
# one ended. The test loop's own lines already end in CRLF, so trailing CRs are not doubled.
exec > >(tee >(sed -u 's/\x1b\[[0-9;]*m//g; s/\r$//' >>"$OUT/session.log") | sed -u 's/\r*$/\r/') 2>&1

# ── Teardown: only what this script started ──────────────────────────────────
# Not the camera, nor the master and container it needs: they stay up for the next run.
cleanup() {
    echo
    log "shutting down what this script started… (Ctrl-C again won't interrupt this)"
    docker rm -f "$GT_PLANNER_CONTAINER" >/dev/null 2>&1 || true
    kill_bg
    if [ "$CTRL_PC_STARTED" -eq 1 ]; then
        stop_detached control_pc_fi
        stop_detached control_pc_ros
        control_pc_teardown
    fi
    log "done. The camera stays on (bash stop_all.sh stops it). Results: ${OUT#"$REPO"/}/"
    sync; sleep 0.2
}
install_cleanup_traps

log "grasp network: $CKPT"
log "segmentation: $SEG_CKPT (masks above $SEG_THRESHOLD)"
log "results → ${OUT#"$REPO"/}/"

follow roscore roscore
follow cameras xtion

# ── 1. ROS1 master, 2. control PC (robot 1), 3. orio container, 4. Xtion ─────
STACK_SERVICES=(roscore control_pc_fi control_pc_ros xtion)
stack_bringup_roscore || exit 1
if [ "$NO_ARM" -eq 0 ]; then
    control_pc_bringup || exit 1
fi
main_container_bringup 'grasp_test\.py' || exit 1
xtion_bringup || exit 1

# ── 5. Grasp planner (its own container and node, beside any pick-and-place one) ──
log "starting the grasp planner (min score $MIN_Q)…"
ORIO_GRASPING="$GRASPING" SUCTION_CHECKPOINT="$CKPT" SUCTION_MIN_SCORE="$MIN_Q" \
    SUCTION_SEG_CHECKPOINT="$SEG_CKPT" SUCTION_SEG_THRESHOLD="$SEG_THRESHOLD" \
    SUCTION_CONTAINER_NAME="$GT_PLANNER_CONTAINER" SUCTION_NODE_NAME="$GT_PLANNER_NODE" \
    ORIO_LOGGING=0 ORIO_AFFORDANCE_DIR="$OUT_CONT" \
    bash "$DOCKER_DIR/run_suction.sh" 2>&1 | prefix planner
[ "${PIPESTATUS[0]}" -eq 0 ] || { log "could not start the planner. Aborting."; exit 1; }
run_bg planner docker logs -f "$GT_PLANNER_CONTAINER"
wait_for_service "/$GT_PLANNER_NODE/plan_grasp" 120 \
    || { log "planner did not come up in 120s - see the [planner] lines above. Aborting."; exit 1; }

# ── 6. The test loop (foreground, owns the keyboard) ─────────────────────────
if [ "$NO_ARM" -eq 0 ]; then
    wait_for_topic "/robot_state_publisher_node_1/robot_state" 60 \
        || { log "robot 1 never published its state - see ${CONTROL_PC_LOG#"$REPO"/}. Aborting."; exit 1; }
    log "ARM WILL MOVE: each grasp waits for Enter first."
fi
run_in_container \
    "python3 -u grasp_test.py --out '$OUT_CONT' --service '/$GT_PLANNER_NODE/plan_grasp' ${NODE_ARGS[*]}"
