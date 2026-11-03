#!/usr/bin/env bash
# Single-terminal FULL-SEQUENCE bring-up (no tmux): both arms pick, zone, label and
# output items, with the grasp planned by the orio-grasping suction network by default
# (or DexNet with --planner dexnet).
#
# Built like run_dexnet_pnp_single.sh: every service starts in the background and merges
# its output into THIS terminal, each line prefixed with a [module] label, and the
# sequence (full_sequence.py) runs in the FOREGROUND so its prompts own the keyboard. Two
# Tkinter windows will open from the sequence: the item name/expiry setup at start, and
# the per-arm recovery dialog when a state fails.
#
# Ctrl-C stops THIS RUN only (sequence, pneumatics, recorder). The slow-to-start stack —
# roscore, the main container, BOTH control PCs (iam-doc for arm 1, iam-luisa for arm 2),
# the grasp planner, the ZED, the Xtion, perception and the RFID inventory node — stays
# up, and the next run (this launcher or run_dexnet_pnp_single.sh: they share the stack)
# reuses whatever of it is still healthy. To tear the whole stack down:
#   bash stop_dexnet_pnp.sh
#
#   bash run_full_demo.sh --fresh         # stop the running stack first, start clean
#   bash run_full_demo.sh                 # real suction (both cups), auto-pick
#   bash run_full_demo.sh --no-vacuum     # dry-run: no pneumatics, move the cups by hand
#   bash run_full_demo.sh --confirm       # confirm each bin pick
#                                           #   (Enter = execute, r = regenerate, q = abort)
#   bash run_full_demo.sh --no-label-vacuum-check # test without labels: skip the label
#                                           #   cup's vacuum check, carry on as if one was picked
#   bash run_full_demo.sh --no-pnp-vacuum-test # skip the pick cup's vacuum check, carry
#                                           #   on as if the item was picked (broken sensor)
#   bash run_full_demo.sh --skip-item-setup # skip the item name/expiry GUI, use defaults
#   bash run_full_demo.sh --straight-down # ignore grasp tilt, approach vertically
#   bash run_full_demo.sh --no-zed        # skip the ZED: labelling poses will time out
#                                           #   (arm 2 will sit in its recovery GUI)
#   bash run_full_demo.sh --no-rfid       # skip the RFID node and label registration
#   bash run_full_demo.sh --no-logging    # skip the rerun recorder for this run
#   bash run_full_demo.sh --live          # also stream to a running `rerun` viewer
#   bash run_full_demo.sh --no-affordance # skip the per-grasp affordance panels
#   bash run_full_demo.sh --offset-x 0.00 --offset-y -0.015
#                                           # world-frame XY correction (metres) added to
#                                           #   every grasp before the arm moves
#   bash run_full_demo.sh --min-q 0.2     # lowest planner score accepted (default 0.30)
#   bash run_full_demo.sh --max-tilt-deg 30
#   bash run_full_demo.sh --planner dexnet
#   bash run_full_demo.sh --checkpoint <tag|path>   # suction network checkpoint
#
# The ZED is ON here (the labelling service needs it), unlike run_dexnet_pnp_single.sh.
# It runs without its own depth (no ZED neural model): the label height is measured by
# Fast-FoundationStereo on its stereo pair (orio_ffs container, orio_bringup/docker/run_ffs.sh).
# Its USB3 link has been flaky (EPROTO drops — docs/OPEN_ISSUES.md); if it drops mid-run
# arm 2's labelling pose times out and its recovery GUI opens.
#
# Env: ORIO_CONTAINER, ORIO_ROS1_CONTAINER, ORIO_CTRL_PC_USER/HOST/FI_PATH/ROBOT_IP,
#      ORIO_LUISA_HOST/ROBOT_NUM/IP, ORIO_MASTER_IP, ORIO_CAM_ATTEMPTS,
#      ORIO_PNP_OFFSET_X/Y, ORIO_GRASPING, SUCTION_CHECKPOINT,
#      ORIO_FFS_DIR/CHECKPOINT/SCALE/ITERS (see run_ffs.sh).
#
# The bring-up steps live in src/devel_packages/orio_bringup/stack.sh.
set -uo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_TAG=launcher
# shellcheck disable=SC1091
source "$REPO/src/devel_packages/orio_bringup/stack.sh"

# Run logging (docs/LOGGING.md). ON by default; see run_dexnet_pnp_single.sh.
export ORIO_LOGGING="${ORIO_LOGGING:-1}"
export ORIO_RUN_ID="${ORIO_RUN_ID:-$(date +%Y%m%d_%H%M%S)}"
export ORIO_RUN_DIR="${ORIO_RUN_DIR:-$REPO/logging/rerun/$ORIO_RUN_ID}"
export ORIO_RERUN_LIVE="${ORIO_RERUN_LIVE:-0}"
LOGGING_REQUESTED="$ORIO_LOGGING"

# ── Args ────────────────────────────────────────────────────────────────────
USE_VACUUM=1
CONFIRM=0
AFFORDANCE=1
USE_ZED=1
USE_RFID=1
PNP_OFFSET_X="${ORIO_PNP_OFFSET_X:-0.0}"
PNP_OFFSET_Y="${ORIO_PNP_OFFSET_Y:-0.0}"
PLANNER="${ORIO_PLANNER:-suction}"
SUCTION_CKPT="${SUCTION_CHECKPOINT:-}"
MIN_Q=""
FRESH=0
SKIP_ITEM_SETUP=0
SM_EXTRA=()
while [ $# -gt 0 ]; do
    arg="$1"; shift
    case "$arg" in
        --no-vacuum)     USE_VACUUM=0 ;;
        --confirm)       CONFIRM=1 ;;
        --straight-down) SM_EXTRA+=(--straight-down) ;;
        --max-tilt-deg)  SM_EXTRA+=(--max-tilt-deg "${1:?--max-tilt-deg needs degrees}"); shift ;;
        --max-declines)  SM_EXTRA+=(--max-declines "${1:?--max-declines needs a count}"); shift ;;
        --zero-item-depth) SM_EXTRA+=(--zero-item-depth) ;;
        --no-label-vacuum-check) SM_EXTRA+=(--no-label-vacuum-check) ;;
        --no-pnp-vacuum-test) SM_EXTRA+=(--no-pnp-vacuum-test) ;;
        --skip-item-setup) SKIP_ITEM_SETUP=1; SM_EXTRA+=(--skip-item-setup) ;;
        --no-zed)        USE_ZED=0 ;;
        --no-rfid)       USE_RFID=0; SM_EXTRA+=(--no-rfid) ;;
        --no-logging)    export ORIO_LOGGING=0; LOGGING_REQUESTED=0 ;;
        --logging)       export ORIO_LOGGING=1; LOGGING_REQUESTED=1 ;;
        --live)          export ORIO_RERUN_LIVE=1 ;;
        --affordance)    AFFORDANCE=1 ;;
        --no-affordance) AFFORDANCE=0 ;;
        --offset-x)      PNP_OFFSET_X="${1:?--offset-x needs metres}"; shift ;;
        --offset-y)      PNP_OFFSET_Y="${1:?--offset-y needs metres}"; shift ;;
        --offset-x=*)    PNP_OFFSET_X="${arg#*=}" ;;
        --offset-y=*)    PNP_OFFSET_Y="${arg#*=}" ;;
        --min-q)         MIN_Q="${1:?--min-q needs a value in 0..1}"; shift ;;
        --min-q=*)       MIN_Q="${arg#*=}" ;;
        --planner)       PLANNER="${1:?--planner needs suction or dexnet}"; shift ;;
        --planner=*)     PLANNER="${arg#*=}" ;;
        --checkpoint)    SUCTION_CKPT="${1:?--checkpoint needs a run tag or a path}"; shift ;;
        --checkpoint=*)  SUCTION_CKPT="${arg#*=}" ;;
        --fresh)         FRESH=1 ;;
        -h|--help) sed -n '2,52p' "$0"; exit 0 ;;
        *) echo "unknown option: $arg" >&2; exit 2 ;;
    esac
done
[ "$USE_VACUUM" -eq 0 ] && SM_EXTRA+=(--no-vacuum)
[ "$CONFIRM" -eq 1 ]    && SM_EXTRA+=(--confirm)
for v in "$PNP_OFFSET_X" "$PNP_OFFSET_Y"; do
    if ! [[ "$v" =~ ^[+-]?([0-9]+\.?[0-9]*|\.[0-9]+)([eE][+-]?[0-9]+)?$ ]]; then
        echo "offset must be a number in metres, got '$v'" >&2; exit 2
    fi
done
# Always passed, so the "[launcher] sm flags:" line records the correction in the run log.
SM_EXTRA+=(--offset-x "$PNP_OFFSET_X" --offset-y "$PNP_OFFSET_Y")
planner_resolve || exit 2

# ── This run's files, and the output filter ──────────────────────────────────
# Run logs live in logging/demo/; the STACK is SHARED with run_dexnet_pnp_single.sh (same
# stack dir), so either launcher reuses it and stop_dexnet_pnp.sh stops it.
run_log_paths demo
log_filter_start '\[FetchInput\]|\[DropToZone\]|\[RetrieveFromZone\]|\[PlaceOutput\]|\[GetLabel\]|\[ApplyLabel\]|\[TaskSelector\]|\[ZoneManager\]|\[RFID\]|\[ORIO\]|Recovery\]'

STACK_SERVICES=(roscore control_pc_fi control_pc_ros planner zed ffs xtion perception rfid)
STACK_CONTAINERS=("$CONTAINER" "$DEXNET_CONTAINER" "$SUCTION_CONTAINER" "$FFS_CONTAINER" "$PERCEPTION_CONTAINER" orio_cameras)

# ── Teardown (this run only) ─────────────────────────────────────────────────
cleanup() {
    echo
    log "stopping this run… (Ctrl-C again won't interrupt this)"
    recorder_flush
    local cups=()
    [ "$USE_VACUUM" -eq 1 ] && cups=(pnp_cup lbl_cup)
    stop_run_in_container 'full_sequence\.py|pneumatic_control_recovery\.py' "${cups[@]}"
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
    log "the stack (cameras, Fast-FoundationStereo, planner, perception, both control PCs, RFID)"
    log "is still running; the next run reuses it. To stop everything: bash stop_dexnet_pnp.sh"
    sync; sleep 0.2
}
install_cleanup_traps

# Follow the stack's logs from the start.
follow roscore roscore
follow "$PLANNER" planner
[ "$USE_ZED" -eq 1 ] && follow cameras zed
[ "$USE_ZED" -eq 1 ] && follow ffs ffs
follow cameras xtion
follow perception perception
[ "$USE_RFID" -eq 1 ] && follow rfid rfid

if [ "$FRESH" -eq 1 ]; then
    log "--fresh: stopping the running stack first…"
    bash "$REPO/stop_dexnet_pnp.sh"
fi

# ── Bring-up ─────────────────────────────────────────────────────────────────
stack_bringup_roscore || exit 1
recorder_start                      # both arms
control_pc_bringup || exit 1        # robot 1, iam-doc
luisa_bringup || exit 1             # robot 2, iam-luisa
main_container_bringup 'full_sequence\.py|dexnet_pnp\.py|pneumatic_control_recovery\.py' || exit 1

# Pre-flight: are BOTH robots ready? Before the heavy services, so a locked robot fails
# fast. No motion. Both robot-state topics must be flowing first — a freshly started luisa
# roslaunch needs a moment, and FrankaArm's own readiness wait gives up after ~10s.
wait_for_topic "/robot_state_publisher_node_1/robot_state" 60 \
    || { log "robot 1 never published its state — see $CONTROL_PC_LOG. Aborting."; exit 1; }
wait_for_topic "/robot_state_publisher_node_2/robot_state" 90 \
    || { log "robot 2 never published its state — check the [luisa] logs above. Aborting."; exit 1; }
log "pre-flight: checking robots 1 and 2 are ready…"
if ! preflight_check "python3 full_sequence.py --check-only"; then
    echo
    log "A ROBOT IS NOT READY — aborting before starting cameras/planner/perception."
    log "Unlock/activate the arm named above, e.g.:"
    log "    python3 src/devel_packages/orio_bringup/lock_arms.py --unlock 1   (or 2)"
    log "then re-run this script. If a control PC itself is stuck, add --fresh."
    exit 1
fi
log "both robots are ready."

affordance_panels_setup
planner_bringup || exit 1

# Cameras: ZED first (labelling), then the Xtion (picking), one at a time. A ZED left
# running with its own depth (another launcher, or before this change) is restarted, so
# its neural depth model is not loaded alongside Fast-FoundationStereo.
if [ "$USE_ZED" -ne 1 ]; then
    log "ZED DISABLED (--no-zed): labelling poses will time out and arm 2 will need its"
    log "recovery GUI. The pick pipeline is unaffected."
    zed_stop
elif ! zed_bringup NONE; then
    log "Or run with --no-zed (labelling then times out). The labelling half of the sequence needs the ZED."
    exit 1
fi
xtion_bringup || exit 1

# Fast-FoundationStereo (label depth from the ZED pair: /ffs/compute). Started here and
# waited for after perception, so the two load their models together.
[ "$USE_ZED" -eq 1 ] && ffs_bringup

# Perception (serves /compute_grasps and /compute_grasps_labelling). The labelling service
# needs SAM/GroundingDINO, which a planner backend skips by default;
# ORIO_LOAD_LABELLING_MODELS=1 loads them alongside it. Part of the conf key, so a stack
# started by run_dexnet_pnp_single.sh (no labelling models) is restarted here rather than
# reused. lbl_depth=ffs restarts a perception from before the label depth moved to
# Fast-FoundationStereo.
LBL_MODELS=$([ "$USE_ZED" -eq 1 ] && echo 1 || echo 0)
perception_bringup "backend=$PLANNER labelling=$LBL_MODELS lbl_depth=ffs" "ORIO_LOAD_LABELLING_MODELS=$LBL_MODELS"

# RFID inventory node (/inventory/add_item). Best effort: the sequence degrades to a
# warning per label when it is absent.
if [ "$USE_RFID" -eq 1 ]; then
    if docker exec "$CONTAINER" python3 -c "import mercury, sllurp" >/dev/null 2>&1; then
        log "starting RFID inventory node (db_manager)…"
        stop_detached rfid
        start_detached rfid docker exec "$CONTAINER" bash -c \
            "source /home/ros_ws/devel/setup.bash && \
             cd /home/ros_ws/src/devel_packages/orio_rfid && python3 -u db_manager.py"
    else
        log "NOTE: RFID node NOT started — the container lacks the 'mercury' and/or"
        log "      'sllurp' python modules (see src/devel_packages/orio_rfid/README.md:"
        log "      pip install sllurp; build RFID/python-mercuryapi). Label registration"
        log "      will log a warning per label and continue. --no-rfid silences this."
    fi
else
    log "RFID node DISABLED (--no-rfid): labels are not registered."
fi

# Pneumatics (both cups), run-scoped.
if [ "$USE_VACUUM" -eq 1 ]; then
    log "starting pneumatics (pnp + lbl cups)…"
    pneumatics_start
else
    log "pneumatics DISABLED (dry-run): move the cups by hand."
fi

if ! stack_wait_service perception /compute_grasps 90; then
    log "/compute_grasps did not come up in 90s — check the [perception] logs above. Aborting."
    exit 1
fi
wait_for_service /compute_grasps_labelling 15 \
    || log "NOTE: /compute_grasps_labelling not up (yet) — labelling will fail until it is."
if [ "$USE_ZED" -eq 1 ]; then
    ffs_wait || exit 1
fi
if [ "$USE_VACUUM" -eq 1 ]; then
    pneumatics_wait pnp_cup lbl_cup || exit 1
fi

# ── The sequence in the FOREGROUND so its prompts can read the keyboard ───────
log "starting the full sequence (Ctrl-C or closing this terminal stops this run; the stack stays up)…"
[ "$SKIP_ITEM_SETUP" -eq 0 ] && log "a Tkinter window will open for the item name/expiry before any motion."
echo "[launcher] sm flags: ${SM_EXTRA[*]}"
run_in_container "python3 -u full_sequence.py ${SM_EXTRA[*]}"

log "sequence exited."
