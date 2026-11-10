#!/usr/bin/env bash
# Single-terminal suction pick-and-place bring-up (no tmux). The grasp planner is the
# orio-grasping suction network by default, or DexNet with --planner dexnet.
#
# Every service starts in the background and merges its output into THIS terminal, each
# line prefixed with a [module] label. The pick loop (dexnet_pnp.py) runs in the
# FOREGROUND so its --confirm "Press Enter to pick" prompt owns the keyboard.
#
# Ctrl-C stops THIS RUN only (pick loop, pneumatics, recorder). The slow-to-start stack —
# roscore, the main container, the control PC, the grasp planner, both cameras and
# perception — stays up, and the next run reuses whatever of it is still healthy and
# starts the rest. To tear the whole stack down:  bash stop_all.sh
#
#   bash run_pickup-only.sh --fresh         # stop the running stack first, start clean
#   bash run_pickup-only.sh                 # dry-run (no vacuum), confirm each pick
#                                                 #   (Enter = execute, r = regenerate grasp, Ctrl-C = abort)
#   bash run_pickup-only.sh --vacuum        # start pneumatics (real suction)
#   bash run_pickup-only.sh --auto          # no per-pick Enter (auto-pick)
#                                                 #   (an empty bin still waits for a refill)
#   bash run_pickup-only.sh --straight-down # ignore grasp tilt, approach vertically
#   bash run_pickup-only.sh --no-logging    # skip the rerun recorder for this run
#   bash run_pickup-only.sh --live          # also stream to a running `rerun` viewer
#   bash run_pickup-only.sh --no-affordance # skip the per-grasp affordance panels
#   bash run_pickup-only.sh --offset-x 0.00 --offset-y -0.015
#                                                 # world-frame XY correction (metres) added to
#                                                 #   each grasp AFTER the planner, in the pick loop
#                                                 #   (defaults: $ORIO_PNP_OFFSET_X/Y, else 0)
#   bash run_pickup-only.sh --min-q 0.2     # lowest planner score to accept a grasp (default:
#                                                 #   $SUCTION_MIN_SCORE / $DEXNET_MIN_Q_VALUE, else 0.10, DexNet 0.30)
#   bash run_pickup-only.sh --planner dexnet
#                                                 # grasp planner: suction (default: the orio-grasping
#                                                 #   network) or dexnet
#                                                 #   (default: $ORIO_PLANNER, else suction)
#   bash run_pickup-only.sh --checkpoint depth_v1
#                                                 # suction network: a run tag (the newest
#                                                 #   $ORIO_GRASPING/runs/*_<tag>/best.pt) or a path
#                                                 #   (default: $SUCTION_CHECKPOINT, else the
#                                                 #   newest $ORIO_GRASPING/runs/*/best.pt)
#
# The iam-doc control PC (robot 1) is started over ssh and its output captured in
# logging/dexnet_pnp/stack/control_pc.log.
#
# Run logging (docs/LOGGING.md) is ON by default: a sidecar recorder writes
# logging/rerun/<run_id>/{recorder.rrd,run.json} — arm state, every node's log lines, and
# the pick loop's own events (pick/cmd/ik/grasp/service). View with:
#     rerun logging/rerun/<run_id>/recorder.rrd
#
# It also writes a diagnostic panel per grasp to logging/dexnet_pnp/<stamp>.affordance/,
# rendered by the planner container. The suction planner draws the chosen cups on the
# camera image, on the height map the network saw and on its score map (a ~0.5 s render).
# Beside each panel it keeps the call's raw inputs, grasp_NNNN_hhmmss/{color,depth,K}.npy
# (~2 MB, the logging/xtion layout) and a meta.json with the planner settings and result,
# so the models can be rerun offline. DexNet draws its six-panel affordance figure, the
# same one orio_perception/test/live_affordance.py produces offline, at the cost of a
# second forward pass plus a ~0.3 s render. --no-affordance skips it; so does --no-logging.
#
# The bring-up steps live in src/devel_packages/orio_bringup/stack.sh (shared with
# run_full_demo.sh, which uses the same stack).
set -uo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_TAG=launcher
# shellcheck disable=SC1091
source "$REPO/src/devel_packages/orio_bringup/stack.sh"

# Run logging (docs/LOGGING.md). ON by default: the recorder is a host-side sidecar and
# nothing in the pipeline depends on it. ORIO_LOGGING is ALSO exported into the containers,
# where it gates the nodes' /orio/events publishing (cheap: one std_msgs/String per event).
export ORIO_LOGGING="${ORIO_LOGGING:-1}"
export ORIO_RUN_ID="${ORIO_RUN_ID:-$(date +%Y%m%d_%H%M%S)}"
export ORIO_RUN_DIR="${ORIO_RUN_DIR:-$REPO/logging/rerun/$ORIO_RUN_ID}"
export ORIO_RERUN_LIVE="${ORIO_RERUN_LIVE:-0}"
# What the OPERATOR asked for, captured before anything can downgrade it: the recorder's
# venv check sets ORIO_LOGGING=0 when the host venv lacks rospy/rerun, which is a statement
# about THIS HOST, and must not reach the grasp panels (drawn in the planner container).
LOGGING_REQUESTED="$ORIO_LOGGING"

# ── Args ────────────────────────────────────────────────────────────────────
USE_VACUUM=0
CONFIRM=1
AFFORDANCE=1
# World-frame XY correction (metres) the pick loop adds to every grasp position after the
# planner returns it. The planner and the perception node never see it; it is the
# calibration residual (camera TF, default Xtion intrinsics, cup mounting) the classical
# path carried in grasp.yaml's classical.offset_x/y. Keep grasp.yaml's dexnet.offset_x/y at 0.
PNP_OFFSET_X="${ORIO_PNP_OFFSET_X:-0.0}"
PNP_OFFSET_Y="${ORIO_PNP_OFFSET_Y:-0.0}"
PLANNER="${ORIO_PLANNER:-suction}"
SUCTION_CKPT="${SUCTION_CHECKPOINT:-}"
# Lowest score (0..1) the planner accepts: the suction network's seal x hold x access, or
# DexNet's q-value. Below it the planner answers "rejected" and the pick loop re-scans;
# enough rejections in a row count as an empty bin (see --max-declines in dexnet_pnp.py).
MIN_Q=""
FRESH=0
PNP_EXTRA=()
while [ $# -gt 0 ]; do
    arg="$1"; shift
    case "$arg" in
        --vacuum)        USE_VACUUM=1 ;;
        --no-vacuum)     USE_VACUUM=0 ;;
        --auto)          CONFIRM=0 ;;
        --confirm)       CONFIRM=1 ;;
        --straight-down) PNP_EXTRA+=(--straight-down) ;;
        --no-logging)    export ORIO_LOGGING=0; LOGGING_REQUESTED=0 ;;
        --logging)       export ORIO_LOGGING=1; LOGGING_REQUESTED=1 ;;
        --live)          export ORIO_RERUN_LIVE=1 ;;
        --affordance)    AFFORDANCE=1 ;;
        --no-affordance) AFFORDANCE=0 ;;
        --offset-x)      PNP_OFFSET_X="${1:?--offset-x needs a value in metres}"; shift ;;
        --offset-y)      PNP_OFFSET_Y="${1:?--offset-y needs a value in metres}"; shift ;;
        --offset-x=*)    PNP_OFFSET_X="${arg#*=}" ;;
        --offset-y=*)    PNP_OFFSET_Y="${arg#*=}" ;;
        --min-q)         MIN_Q="${1:?--min-q needs a value in 0..1}"; shift ;;
        --min-q=*)       MIN_Q="${arg#*=}" ;;
        --planner)       PLANNER="${1:?--planner needs suction or dexnet}"; shift ;;
        --planner=*)     PLANNER="${arg#*=}" ;;
        --checkpoint)    SUCTION_CKPT="${1:?--checkpoint needs a run tag or a path}"; shift ;;
        --checkpoint=*)  SUCTION_CKPT="${arg#*=}" ;;
        --fresh)         FRESH=1 ;;
        -h|--help) sed -n '2,55p' "$0"; exit 0 ;;
        *) echo "unknown option: $arg" >&2; exit 2 ;;
    esac
done
[ "$USE_VACUUM" -eq 0 ] && PNP_EXTRA+=(--no-vacuum)
[ "$CONFIRM" -eq 1 ]    && PNP_EXTRA+=(--confirm)
for v in "$PNP_OFFSET_X" "$PNP_OFFSET_Y"; do
    if ! [[ "$v" =~ ^[+-]?([0-9]+\.?[0-9]*|\.[0-9]+)([eE][+-]?[0-9]+)?$ ]]; then
        echo "offset must be a number in metres, got '$v'" >&2; exit 2
    fi
done
# Always passed, so the "[launcher] pnp flags:" line records the correction in the run log.
PNP_EXTRA+=(--offset-x "$PNP_OFFSET_X" --offset-y "$PNP_OFFSET_Y")
planner_resolve || exit 2
# Drop every item at the same height: reach into the output box by the fixed margin only,
# not by the grasp z as well.
PNP_EXTRA+=(--zero-item-depth)

# ── This run's files, and the output filter ──────────────────────────────────
run_log_paths dexnet_pnp
log_filter_start '\[Pick\]|\[Place\]'

# The stack shared with run_full_demo.sh (same stack dir); stop_all.sh stops it.
STACK_SERVICES=(roscore control_pc_fi control_pc_ros planner zed xtion perception)
STACK_CONTAINERS=("$CONTAINER" "$DEXNET_CONTAINER" "$SUCTION_CONTAINER" "$PERCEPTION_CONTAINER" orio_cameras)

# ── Teardown (this run only) ─────────────────────────────────────────────────
cleanup() {
    echo
    log "stopping this run… (Ctrl-C again won't interrupt this)"
    recorder_flush
    local cups=()
    [ "$USE_VACUUM" -eq 1 ] && cups=(pnp_cup)
    stop_run_in_container 'dexnet_pnp\.py|pneumatic_control_recovery\.py' "${cups[@]}"
    kill_bg
    log "done. Full log saved to: $LOG_FILE"
    [ "$ORIO_LOGGING" != "0" ] && [ -f "$ORIO_RUN_DIR/recorder.rrd" ] && \
        log "rerun recording: $ORIO_RUN_DIR (view: rerun $ORIO_RUN_DIR/recorder.rrd)"
    [ -s "$CONTROL_PC_LOG" ] && log "control PC (iam-doc) log: $CONTROL_PC_LOG"
    if [ -d "${AFFORDANCE_DIR:-}" ]; then
        _n_aff=$(find "$AFFORDANCE_DIR" -name '*.png' 2>/dev/null | wc -l)
        [ "$_n_aff" -gt 0 ] && log "affordance panels ($_n_aff): $AFFORDANCE_DIR"
    fi
    [ -s "${VACUUM_LOG:-}" ] && log "raw vacuum readings: $VACUUM_LOG"
    log "the stack (cameras, planner, perception, control PC) is still running; the next run"
    log "reuses it. To stop everything: bash stop_all.sh"
    # Give the output filter a moment to flush the final lines to the file.
    sync; sleep 0.2
}
install_cleanup_traps

# Follow the stack's logs from the start, so a service started below shows from its first
# line and one already running shows from now on.
follow roscore roscore
follow "$PLANNER" planner
[ "${ORIO_USE_ZED:-0}" -eq 1 ] && follow cameras zed
follow cameras xtion
follow perception perception

if [ "$FRESH" -eq 1 ]; then
    log "--fresh: stopping the running stack first…"
    bash "$REPO/stop_all.sh"
fi

# ── Bring-up ─────────────────────────────────────────────────────────────────
stack_bringup_roscore || exit 1
# Single arm here, so --robots 1: asking for arm 2 would log an empty arm and its 11 MB
# of static meshes, and place it via cell.yaml at a base that isn't in use.
recorder_start --robots 1
control_pc_bringup || exit 1
main_container_bringup 'dexnet_pnp\.py|pneumatic_control_recovery\.py' || exit 1

# Pre-flight BEFORE the heavy services (planner, cameras, perception), so a locked or
# not-activated robot fails fast. Same readiness path (frankapy) as the run. No motion.
log "pre-flight: checking robot 1 is ready…"
if ! preflight_check "python3 dexnet_pnp.py --check-only"; then
    echo
    log "ROBOT NOT READY — aborting before starting cameras/planner/perception."
    log "Unlock/activate robot 1, e.g.:"
    log "    python3 src/devel_packages/orio_bringup/lock_arms.py --unlock 1"
    log "then re-run this script. If the control PC itself is stuck, add --fresh."
    exit 1
fi
log "robot is ready."

affordance_panels_setup
planner_bringup || exit 1

# Cameras, one at a time (ZED, then Xtion) so their init doesn't interleave.
# The ZED is OPTIONAL here (off by default since 2026-11-02): the pick pipeline plans from
# the Xtion; the ZED only feeds the labelling service, which times out cleanly without it.
# Its USB3 link is failing at the hardware level (docs/OPEN_ISSUES.md). Set ORIO_USE_ZED=1
# to bring it back once the camera/cable is replaced.
if [ "${ORIO_USE_ZED:-0}" -ne 1 ]; then
    log "ZED camera DISABLED (set ORIO_USE_ZED=1 to re-enable): labelling has no images; the pick pipeline is unaffected."
    zed_stop
else
    zed_bringup NONE || exit 1
fi
xtion_bringup || exit 1

perception_bringup "backend=$PLANNER"

if [ "$USE_VACUUM" -eq 1 ]; then
    log "starting pneumatics…"
    pneumatics_start
else
    log "pneumatics DISABLED (dry-run): move the cup by hand."
fi

if ! stack_wait_service perception /compute_grasps 90; then
    log "/compute_grasps did not come up in 90s — check the [perception] logs above"
    log "(model load, backend=$PLANNER, camera frames). Aborting."
    exit 1
fi
if [ "$USE_VACUUM" -eq 1 ]; then
    pneumatics_wait pnp_cup || exit 1
fi

# ── Pick loop in the FOREGROUND so --confirm can read the keyboard ────────────
log "starting pick-and-place loop (Ctrl-C or closing this terminal stops this run; the stack stays up)…"
echo "[launcher] pnp flags: ${PNP_EXTRA[*]}"
run_in_container "python3 -u dexnet_pnp.py ${PNP_EXTRA[*]}"

log "pick loop exited."
