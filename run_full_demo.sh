#!/usr/bin/env bash
# Single-terminal FULL-SEQUENCE bring-up (no tmux): both arms pick, zone, label and
# output items, with the grasp planned by the orio-grasping suction network by default
# (or DexNet with --planner dexnet).
#
# The full-demo successor to launch_demo.sh + the orio tmux layout, built like
# run_dexnet_pnp_single.sh: every service starts in the background and merges its output
# into THIS terminal, each line prefixed with a [module] label, and the sequence
# (full_sequence.py) runs in the FOREGROUND so its prompts own the keyboard. Two Tkinter
# windows will open from the sequence: the item name/expiry setup at start, and the
# per-arm recovery dialog when a state fails.
#
# Ctrl-C stops THIS RUN only (sequence, pneumatics, recorder). The slow-to-start stack —
# roscore, the main container, BOTH control PCs (iam-doc for arm 1, iam-luisa for arm 2),
# the grasp planner, the ZED, the Xtion, perception and the RFID inventory node — stays
# up, and the next run (this launcher or run_dexnet_pnp_single.sh: they share the stack)
# reuses whatever of it is still healthy. To tear the whole stack down:
#   bash stop_dexnet_pnp.sh
#
#   bash run_full_demo.sh --fresh         # stop the running stack first, start clean
#   bash run_full_demo.sh                 # dry-run (no vacuum), confirm each bin pick
#                                           #   (Enter = execute, r = regenerate, q = abort)
#   bash run_full_demo.sh --vacuum        # start pneumatics (real suction, both cups)
#   bash run_full_demo.sh --auto          # no per-pick Enter (auto-pick)
#   bash run_full_demo.sh --straight-down # ignore grasp tilt, approach vertically
#   bash run_full_demo.sh --no-zed        # skip the ZED: labelling poses will time out
#                                           #   (arm 2 will sit in its recovery GUI)
#   bash run_full_demo.sh --no-rfid       # skip the RFID inventory node
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
# Its USB3 link has been flaky (EPROTO drops — docs/OPEN_ISSUES.md); if it drops mid-run
# arm 2's labelling pose times out and its recovery GUI opens.
#
# Env: ORIO_CONTAINER, ORIO_ROS1_CONTAINER, ORIO_CTRL_PC_USER/HOST/FI_PATH/ROBOT_IP,
#      ORIO_LUISA_HOST/ROBOT_NUM/IP, ORIO_MASTER_IP, ORIO_CAM_ATTEMPTS,
#      ORIO_PNP_OFFSET_X/Y, ORIO_GRASPING, SUCTION_CHECKPOINT.
set -uo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export ORIO_REPO="$REPO"
CONTAINER="${ORIO_CONTAINER:-orio_docker_container}"
# iam-doc control PC (robot 1): franka-interface + ROS action server over ssh, output
# captured (see run_dexnet_pnp_single.sh for why not start_control_pc.sh).
CTRL_PC_USER="${ORIO_CTRL_PC_USER:-student}"
CTRL_PC_HOST="${ORIO_CTRL_PC_HOST:-iam-doc}"
CTRL_PC_FI_PATH="${ORIO_CTRL_PC_FI_PATH:-Documents/franka-interface}"
CTRL_PC_ROBOT_IP="${ORIO_CTRL_PC_ROBOT_IP:-172.16.0.2}"
CTRL_PC_ROBOT_NUM="${ORIO_CTRL_PC_ROBOT_NUM:-1}"
# iam-luisa control PC (robot 2, the label arm): its stack runs in tmux inside a
# container THERE, brought up by start_control_pc_luisa.sh (idempotent).
LUISA_PC="${ORIO_LUISA_HOST:-snaak@iam-luisa}"
WORKSTATION_HOST="$(hostname)"   # ROS master lives here; the control PCs point ROS at it
DEXNET_CONTAINER="${DEXNET_CONTAINER_NAME:-orio_dexnet}"
SUCTION_CONTAINER="${SUCTION_CONTAINER_NAME:-orio_suction}"
DOCKER_DIR="$REPO/src/devel_packages/orio_bringup/docker"
WF="$REPO/src/devel_packages/orio_bringup/tmux/wait_for.sh"
# shellcheck disable=SC1090
source "$WF"

# Run logging (docs/LOGGING.md). ON by default; see run_dexnet_pnp_single.sh.
export ORIO_LOGGING="${ORIO_LOGGING:-1}"
export ORIO_RUN_ID="${ORIO_RUN_ID:-$(date +%Y%m%d_%H%M%S)}"
export ORIO_RUN_DIR="${ORIO_RUN_DIR:-$REPO/logging/rerun/$ORIO_RUN_ID}"
export ORIO_RERUN_LIVE="${ORIO_RERUN_LIVE:-0}"
LOGGING_REQUESTED="$ORIO_LOGGING"
PERC_DIR="$REPO/src/devel_packages/orio_perception"
PERC_VENV="${ORIO_PERCEPTION_VENV:-${ORIO_PERCEPTION_ASSETS:-$PERC_DIR}/venv}"

# ── Args ────────────────────────────────────────────────────────────────────
USE_VACUUM=0
CONFIRM=1
AFFORDANCE=1
USE_ZED=1
USE_RFID=1
PNP_OFFSET_X="${ORIO_PNP_OFFSET_X:-0.0}"
PNP_OFFSET_Y="${ORIO_PNP_OFFSET_Y:-0.0}"
PLANNER="${ORIO_PLANNER:-suction}"
SUCTION_CKPT="${SUCTION_CHECKPOINT:-}"
ORIO_GRASPING="${ORIO_GRASPING:-$HOME/orio-grasping}"
MIN_Q=""
FRESH=0
SM_EXTRA=()
while [ $# -gt 0 ]; do
    arg="$1"; shift
    case "$arg" in
        --vacuum)        USE_VACUUM=1 ;;
        --no-vacuum)     USE_VACUUM=0 ;;
        --auto)          CONFIRM=0 ;;
        --confirm)       CONFIRM=1 ;;
        --straight-down) SM_EXTRA+=(--straight-down) ;;
        --max-tilt-deg)  SM_EXTRA+=(--max-tilt-deg "${1:?--max-tilt-deg needs degrees}"); shift ;;
        --max-declines)  SM_EXTRA+=(--max-declines "${1:?--max-declines needs a count}"); shift ;;
        --check-vacuum)  SM_EXTRA+=(--check-vacuum) ;;
        --zero-item-depth) SM_EXTRA+=(--zero-item-depth) ;;
        --no-zed)        USE_ZED=0 ;;
        --no-rfid)       USE_RFID=0 ;;
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
        -h|--help) sed -n '2,48p' "$0"; exit 0 ;;
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
case "$PLANNER" in
    suction)
        PLANNER_CONTAINER="$SUCTION_CONTAINER"
        PLANNER_SERVICE=/suction_grasp_planner/plan_grasp
        PLANNER_SCRIPT=run_suction.sh
        MIN_Q="${MIN_Q:-${SUCTION_MIN_SCORE:-0.30}}"
        if [ -z "$SUCTION_CKPT" ]; then
            SUCTION_CKPT="$(ls -t "$ORIO_GRASPING"/runs/*/best.pt 2>/dev/null | head -1)"
            [ -n "$SUCTION_CKPT" ] || { echo "no checkpoint: train one into $ORIO_GRASPING/runs/ or set SUCTION_CHECKPOINT" >&2; exit 2; }
        elif [ ! -f "$SUCTION_CKPT" ]; then
            tag="$SUCTION_CKPT"
            SUCTION_CKPT="$(ls -t "$ORIO_GRASPING"/runs/*_"$tag"/best.pt 2>/dev/null | head -1)"
            [ -n "$SUCTION_CKPT" ] || { echo "--checkpoint: no $ORIO_GRASPING/runs/*_$tag/best.pt" >&2; exit 2; }
        fi
        SUCTION_CKPT="$(readlink -f "$SUCTION_CKPT")"
        export SUCTION_CHECKPOINT="$SUCTION_CKPT" ;;
    dexnet)
        PLANNER_CONTAINER="$DEXNET_CONTAINER"
        PLANNER_SERVICE=/dexnet_grasp_planner/plan_grasp
        PLANNER_SCRIPT=run_dexnet.sh
        MIN_Q="${MIN_Q:-${DEXNET_MIN_Q_VALUE:-0.30}}" ;;
    *) echo "--planner must be suction or dexnet, got '$PLANNER'" >&2; exit 2 ;;
esac
if ! [[ "$MIN_Q" =~ ^(1(\.0+)?|0(\.[0-9]+)?|\.[0-9]+)$ ]]; then
    echo "--min-q must be a number between 0 and 1, got '$MIN_Q'" >&2; exit 2
fi
export DEXNET_MIN_Q_VALUE="$MIN_Q" SUCTION_MIN_SCORE="$MIN_Q"

# ── Log filtering + saving ───────────────────────────────────────────────────
# One classifier routes every line: DROP (noise), SAVE (file only), SHOW (terminal +
# file). Same mechanism as run_dexnet_pnp_single.sh; the SHOW prefixes cover the
# sequence's states. Run logs live in logging/demo/; the STACK is SHARED with
# run_dexnet_pnp_single.sh (same stack dir), so either launcher reuses it and
# stop_dexnet_pnp.sh stops it.
LOG_DIR="$REPO/logging/demo"
mkdir -p "$LOG_DIR"
RUN_STAMP="$(date +%Y%m%d_%H%M%S)"
LOG_FILE="$LOG_DIR/$RUN_STAMP.log"
STACK_DIR="$REPO/logging/dexnet_pnp/stack"
mkdir -p "$STACK_DIR"
CONTROL_PC_LOG="$STACK_DIR/control_pc.log"
AFFORDANCE_DIR="$LOG_DIR/$RUN_STAMP.affordance"
AFFORDANCE_LINK="$LOG_DIR/current.affordance"
AFFORDANCE_LINK_CONT="/home/ros_ws/logging/demo/current.affordance"

exec > >(
    LOG_FILE="$LOG_FILE" awk '
    function strip(s){ gsub(/\033\[[0-9;]*m/, "", s); gsub(/\r/, "", s); return s }
    {
        line = strip($0)
        show = 0; save = 0

        # DROP FIRST: import-time noise a SHOW rule below would otherwise rescue.
        if (line ~ /autolab_core not installed as catkin package/) { next }

        # SHOW: signal + any error, regardless of source.
        if (line ~ /\[launcher\]|\[check\]|\[doc\]|\[luisa\]|\[stop\]|\[Main\]|\[Hardware\]|\[Grasp\]|\[Loop\]|\[FetchInput\]|\[DropToZone\]|\[RetrieveFromZone\]|\[PlaceOutput\]|\[GetLabel\]|\[ApplyLabel\]|\[TaskSelector\]|\[ZoneManager\]|\[RFID\]|\[ORIO\]|Recovery\]|Press Enter/) { show=1 }
        if (line ~ /GAVE UP|ERROR|Error|error|Traceback|has died|disconnected|CAMERA NOT DETECTED|FAIL|Rejected|refus|not ready|NOT ready|Aborting/) { show=1 }
        if (line ~ /Planned suction grasp|grasp q=/) { show=1 }
        if (line ~ /Affordance panel|Grasp panel/) { show=1 }

        # DROP: pure noise — never shown, never saved.
        else if (line ~ /tensorflow|cuda_gpu|dso_loader|NUMA node|StreamExecutor|gpu_device|libcu[a-z]+\.so|XLA service|Building (convolutional|fully|Softmax)|Converting fc layer|Advertised on topic|ikpy|is of type .fixed.|Detection max range|warnings\.warn/) { next }

        # Everything else: SAVE to file only (debug context, not live).
        if (show) { save=1 }
        else { save=1; show=0 }

        if (save && length(line)) { print line >> ENVIRON["LOG_FILE"]; fflush(ENVIRON["LOG_FILE"]) }
        if (show && length(line)) { printf "%s\r\n", line; fflush() }
    }'
) 2>&1
echo "[launcher] logging this run to $LOG_FILE"

# ── Log merging ──────────────────────────────────────────────────────────────
PIDS=()

prefix() {  # $1 = label; reads stdin, writes "[label] line"
    local label="$1"
    sed -u "s/^/[$label] /" 2>/dev/null || sed "s/^/[$label] /"
}

ts_prefix() {  # reads stdin, prepends an ISO timestamp; for the raw control-PC log
    while IFS= read -r line; do printf '%s %s\n' "$(date +%H:%M:%S)" "$line"; done
}

run_bg() {  # $1 = label, rest = command; run in background, prefixed
    local label="$1"; shift
    ( "$@" 2>&1 | prefix "$label" ) &
    PIDS+=($!)
}

log() { echo "[launcher] $*"; }

# ── The stack: services that stay up between runs ───────────────────────────
# Same mechanism and stack dir as run_dexnet_pnp_single.sh, extended with the second
# control PC (luisa) and the RFID node. bash stop_dexnet_pnp.sh tears it all down.
STACK_SERVICES=(roscore control_pc_fi control_pc_ros luisa planner zed xtion perception rfid)

start_detached() {  # $1 = name, rest = command (may be an exported function)
    local name="$1"; shift
    : >"$STACK_DIR/$name.log"
    setsid bash -c '"$@"' _ "$@" >>"$STACK_DIR/$name.log" 2>&1 </dev/null &
    echo $! >"$STACK_DIR/$name.pid"
}

stop_detached() {  # $1 = name
    local f="$STACK_DIR/$1.pid"
    [ -f "$f" ] && kill -- "-$(cat "$f")" 2>/dev/null
    rm -f "$f"
}

follow() {  # $1 = label, $2 = service name; for this run only
    ( tail -n0 -F --pid=$$ "$STACK_DIR/$2.log" 2>/dev/null | prefix "$1" ) &
    PIDS+=($!)
}

# Quiet health checks.
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
    # Flush the rerun recorder FIRST (it writes recorder.rrd + run.json on SIGINT).
    if [ -n "${RECORDER_PID:-}" ] && kill -0 "$RECORDER_PID" 2>/dev/null; then
        log "flushing run recorder…"
        kill -INT "$RECORDER_PID" 2>/dev/null || true
        for _ in $(seq 1 25); do
            kill -0 "$RECORDER_PID" 2>/dev/null || break
            sleep 0.2
        done
        kill -0 "$RECORDER_PID" 2>/dev/null && log "recorder still running — killing it."
    fi
    # Both vacuums off first if they were running (best effort).
    if [ "$USE_VACUUM" -eq 1 ]; then
        docker exec "$CONTAINER" bash -lc \
          "source /opt/ros/noetic/setup.bash 2>/dev/null; \
           rosservice call /orio/pnp_cup/off 2>/dev/null; \
           rosservice call /orio/lbl_cup/off 2>/dev/null" >/dev/null 2>&1 || true
    fi
    # The sequence and pneumatics run inside the main container, which stays up.
    docker exec "$CONTAINER" pkill -f 'full_sequence\.py|pneumatic_control_recovery\.py' \
        >/dev/null 2>&1 || true
    for pid in "${PIDS[@]:-}"; do kill "$pid" 2>/dev/null || true; done
    log "done. Full log saved to: $LOG_FILE"
    [ "$ORIO_LOGGING" != "0" ] && [ -f "$ORIO_RUN_DIR/recorder.rrd" ] && \
        log "rerun recording: $ORIO_RUN_DIR (view: rerun $ORIO_RUN_DIR/recorder.rrd)"
    [ -s "$CONTROL_PC_LOG" ] && log "control PC (iam-doc) log: $CONTROL_PC_LOG"
    if [ -d "${AFFORDANCE_DIR:-}" ]; then
        _n_aff=$(find "$AFFORDANCE_DIR" -name '*.png' 2>/dev/null | wc -l)
        [ "$_n_aff" -gt 0 ] && log "affordance panels ($_n_aff): $AFFORDANCE_DIR"
    fi
    log "the stack (cameras, planner, perception, both control PCs, RFID) is still running;"
    log "the next run reuses it. To stop everything: bash stop_dexnet_pnp.sh"
    sync; sleep 0.2
}
trap 'cleanup; exit 130' INT
trap 'cleanup; exit 143' TERM
trap 'cleanup; exit 129' HUP
trap cleanup EXIT

# Follow the stack's logs from the start.
follow roscore roscore
follow "$PLANNER" planner
[ "$USE_ZED" -eq 1 ] && follow cameras zed
follow cameras xtion
follow perception perception
follow luisa luisa
[ "$USE_RFID" -eq 1 ] && follow rfid rfid

if [ "$FRESH" -eq 1 ]; then
    log "--fresh: stopping the running stack first…"
    bash "$REPO/stop_dexnet_pnp.sh"
fi

# ── roscore ──────────────────────────────────────────────────────────────────
STACK_FRESH=0
if roscore_up; then
    log "roscore already running — reusing the stack where it is healthy."
else
    STACK_FRESH=1
    log "no roscore — starting the stack from scratch…"
    for s in "${STACK_SERVICES[@]}"; do stop_detached "$s"; done
    docker rm -f orio_roscore "$CONTAINER" "$DEXNET_CONTAINER" "$SUCTION_CONTAINER" orio_perception orio_cameras \
        >/dev/null 2>&1 || true
    rm -f "$STACK_DIR"/*.conf
    log "starting roscore…"
    start_detached roscore bash "$DOCKER_DIR/run_roscore.sh"
    wait_for_roscore 30 || { log "roscore did not start in 30s — check the [roscore] logs. Aborting."; exit 1; }
fi

# ── Run recorder (docs/LOGGING.md) ───────────────────────────────────────────
# Sidecar; both robots this time (the tmux demo's default). See
# run_dexnet_pnp_single.sh for the venv requirements explained at length.
RECORDER_PID=""
if [ "$ORIO_LOGGING" != "0" ]; then
    if [ ! -x "$PERC_VENV/bin/python3" ]; then
        log "NOTE: no perception venv at $PERC_VENV — run logging is OFF for this run."
        ORIO_LOGGING=0
    elif ! "$PERC_VENV/bin/python3" -c "import rospy, rerun" >/dev/null 2>&1; then
        log "NOTE: $PERC_VENV lacks rospy and/or rerun — run logging is OFF for this run."
        log "      (see run_dexnet_pnp_single.sh for the pip command; --no-logging silences this)"
        ORIO_LOGGING=0
    else
        log "starting run recorder (both arms) → $ORIO_RUN_DIR"
        ( "$PERC_VENV/bin/python3" "$REPO/src/devel_packages/orio_logging/recorder.py" \
              2>&1 | prefix recorder ) &
        RECORDER_PID=$!
        PIDS+=("$RECORDER_PID")
    fi
fi

# ── Control PC 1 (iam-doc, the pick-and-place arm) ───────────────────────────
log "checking ssh to control PC 1 ($CTRL_PC_HOST)…"
if ! timeout 15 ssh -o BatchMode=yes -o ConnectTimeout=10 \
        "$CTRL_PC_USER@$CTRL_PC_HOST" 'true' >/dev/null 2>&1; then
    echo
    log "CANNOT SSH to $CTRL_PC_USER@$CTRL_PC_HOST — the control PC is unreachable."
    log "Checks: ping $CTRL_PC_HOST; nc -vz $CTRL_PC_HOST 22; restart sshd at its console."
    log "Aborting before bring-up (nothing can run without the control PC)."
    exit 1
fi
log "ssh to control PC 1 OK."

# Tripwire: forward only GENUINE control-PC failures into the merged [doc] stream.
# (Same classifier as run_dexnet_pnp_single.sh — see there for why each line is benign.)
( tail -n0 -F --pid=$$ "$CONTROL_PC_LOG" 2>/dev/null \
    | awk '
        /libprotobuf/                                        { next }
        /RobotStateMessage/                                  { next }
        /ParsingFromArray Exception/                          { next }
        /done_skill_id error/                                { next }
        /[Cc]aught Franka Exception/                         { next }
        /automatic error recovery|Error recovery finished/   { next }
        /Skill terminated|Will run the control loop/         { next }
        /Failed to get static transforms/                    { next }
        /Robot state save thread encountered Franka exception/ {
            if (seen_statesave) next; seen_statesave=1
            print "[note] control PC: robot-state save thread is throwing exceptions (repeating; suppressed after first — controller may be wedged)"
            fflush(); next
        }
        /has died|process has died|control loop|libfranka|terminate called|core dumped|Aborted|segmentation|refused|Connection reset|cannot connect|not connected|e-stop|estop|FCI|reflex/ { print; fflush(); next }
        /[Ee]rror|[Ee]xception|violation|[Ff]atal/           { print; fflush() }
      ' \
    | prefix doc ) &
PIDS+=($!)

control_pc_up() {
    timeout 15 ssh -o BatchMode=yes -o ConnectTimeout=8 "$CTRL_PC_USER@$CTRL_PC_HOST" \
        'pgrep -f "[f]ranka_interface --robot_ip" >/dev/null && pgrep -f "[r]oslaunch franka_ros_interface" >/dev/null' \
        >/dev/null 2>&1
}

control_pc_fi() {
    ssh -tt -o BatchMode=yes -o ConnectTimeout=10 "$CTRL_PC_USER@$CTRL_PC_HOST" \
        "mkdir -p ~/franka_logs && ls -t ~/franka_logs/*.log 2>/dev/null | tail -n +51 | xargs -r rm -f; \
         cd '$CTRL_PC_FI_PATH/build' && \
         stdbuf -oL -eL ./franka_interface --robot_ip '$CTRL_PC_ROBOT_IP' --with_gripper 0 --log 0 --stop_on_error 0 \
           2>&1 | tee ~/franka_logs/franka_interface_\$(date +%Y%m%d_%H%M%S).log" \
        2>&1 | ts_prefix >>"$CONTROL_PC_LOG"
}
control_pc_ros() {
    ssh -tt -o BatchMode=yes -o ConnectTimeout=10 "$CTRL_PC_USER@$CTRL_PC_HOST" \
        "cd '$CTRL_PC_FI_PATH' && source bash_scripts/set_rosmaster.sh '$CTRL_PC_HOST' '$WORKSTATION_HOST' && source catkin_ws/devel/setup.bash && roslaunch franka_ros_interface franka_ros_interface.launch robot_num:=$CTRL_PC_ROBOT_NUM" \
        2>&1 | ts_prefix >>"$CONTROL_PC_LOG"
}
export -f control_pc_fi control_pc_ros ts_prefix
export CTRL_PC_USER CTRL_PC_HOST CTRL_PC_FI_PATH CTRL_PC_ROBOT_IP CTRL_PC_ROBOT_NUM \
       WORKSTATION_HOST CONTROL_PC_LOG

control_pc_wedged() {
    awk '/===== control PC bring-up/ { bad = 0 } /bad_cast/ { bad = 1 } END { exit !bad }' \
        "$CONTROL_PC_LOG" 2>/dev/null
}

CTRL_PC_REUSE=0
if [ "$STACK_FRESH" -eq 0 ] && control_pc_up; then
    if control_pc_wedged; then
        log "control PC 1 hit std::bad_cast since it started (franka-interface is wedged) — restarting it."
    else
        CTRL_PC_REUSE=1
    fi
fi
if [ "$CTRL_PC_REUSE" -eq 1 ]; then
    log "control PC 1 already running on $CTRL_PC_HOST."
else
stop_detached control_pc_fi
stop_detached control_pc_ros
control_pc_cleanup() {
log "clearing stale control-PC state on $CTRL_PC_HOST…"
timeout 30 ssh -o BatchMode=yes -o ConnectTimeout=8 "$CTRL_PC_USER@$CTRL_PC_HOST" \
    'bash -s' <<'REMOTE_CLEANUP' >/dev/null 2>&1 || true
fi_count() { pgrep -f '[f]ranka_interface --robot_ip' | wc -l; }
pkill -TERM -f '[f]ranka_interface --robot_ip' 2>/dev/null
pkill -TERM -f '[r]oslaunch franka_ros_interface' 2>/dev/null
for _ in $(seq 1 20); do
    [ "$(fi_count)" -eq 0 ] && break
    sleep 0.5
done
if [ "$(fi_count)" -ne 0 ]; then
    pkill -KILL -f '[f]ranka_interface --robot_ip' 2>/dev/null
    sleep 1
fi
pkill -KILL -f '[r]oslaunch franka_ros_interface' 2>/dev/null
pkill -KILL -f '[f]ranka_ros_interface' 2>/dev/null
if [ "$(fi_count)" -eq 0 ]; then
    rm -f /dev/shm/run_loop_* /dev/shm/current_robot_state* \
          /dev/shm/franka_interface_state_info_mutex 2>/dev/null
    echo "CLEANUP_OK"
else
    echo "CLEANUP_FAILED: franka_interface still running; left /dev/shm alone"
fi
REMOTE_CLEANUP
}
control_pc_cleanup

log "starting robot-1 control PC ($CTRL_PC_HOST) over ssh — logging to $CONTROL_PC_LOG"
{
    echo "===== control PC bring-up @ $(date -Is) — $CTRL_PC_USER@$CTRL_PC_HOST ====="
    echo "===== franka-interface | robot_ip=$CTRL_PC_ROBOT_IP ====="
} >>"$CONTROL_PC_LOG"
FI_ATTEMPTS="${ORIO_FI_ATTEMPTS:-2}"
fi_mode_error=0
for fi_attempt in $(seq 1 "$FI_ATTEMPTS"); do
    fi_from=$(( $(wc -l <"$CONTROL_PC_LOG") + 1 ))
    start_detached control_pc_fi control_pc_fi
    sleep 5
    timeout 15 ssh -o BatchMode=yes -o ConnectTimeout=8 "$CTRL_PC_USER@$CTRL_PC_HOST" \
        'pgrep -f "[f]ranka_interface --robot_ip" >/dev/null' >/dev/null 2>&1 && { fi_mode_error=0; break; }
    tail -n +"$fi_from" "$CONTROL_PC_LOG" | grep -q "not possible in the current mode" || break
    fi_mode_error=1
    [ "$fi_attempt" -eq "$FI_ATTEMPTS" ] && break
    log "arm is not accepting commands yet (franka-interface attempt $fi_attempt/$FI_ATTEMPTS) — retrying in 10 s…"
    sleep 10
    control_pc_cleanup
done
if [ "$fi_mode_error" -eq 1 ]; then
    echo
    log "ROBOT 1 NOT ACCEPTING COMMANDS: franka-interface keeps exiting with \"command not"
    log "possible in the current mode\". The arm is locked, user-stopped or in an error state."
    log "  - Unlock and activate it:  python3 src/devel_packages/orio_bringup/lock_arms.py --unlock 1"
    log "  - In Desk, make sure FCI is activated."
    log "Then re-run this script. Aborting."
    exit 1
fi
start_detached control_pc_ros control_pc_ros
log "control PC 1: franka-interface + action server launched over ssh (detail in ${CONTROL_PC_LOG##*/})"
sleep 3
fi

# ── Control PC 2 (iam-luisa, the label arm) ──────────────────────────────────
# Its whole stack runs in tmux INSIDE the franka-interface container on iam-luisa;
# start_control_pc_luisa.sh is idempotent and leaves an already-running session alone.
luisa_up() {
    timeout 20 ssh -o BatchMode=yes -o ConnectTimeout=8 "$LUISA_PC" \
        "docker exec franka-interface bash -lc 'tmux has-session -t franka'" >/dev/null 2>&1
}
if [ "$STACK_FRESH" -eq 0 ] && luisa_up; then
    log "control PC 2 (luisa) already running."
else
    if [ "$STACK_FRESH" -eq 1 ] && luisa_up; then
        # New master: the old session points at a roscore that no longer exists.
        log "restarting control PC 2 (luisa) — its session points at the old master…"
        timeout 30 ssh -o BatchMode=yes -o ConnectTimeout=8 "$LUISA_PC" \
            "docker exec franka-interface bash -lc 'tmux kill-session -t franka'" >/dev/null 2>&1 || true
    fi
    log "starting robot-2 control PC ($LUISA_PC)…"
    : >"$STACK_DIR/luisa.log"
    if ! bash "$REPO/src/devel_packages/orio_bringup/start_control_pc_luisa.sh" \
            >>"$STACK_DIR/luisa.log" 2>&1; then
        echo
        log "CONTROL PC 2 (luisa) DID NOT COME UP — see the [luisa] lines above."
        log "Checks: ssh $LUISA_PC; is 'iam-luisa' in /etc/hosts (192.168.2.3)?"
        log "Aborting: the label arm cannot run without it."
        exit 1
    fi
fi

# The main container: cameras + the sequence run in it.
CAMS_FRESH="$STACK_FRESH"
if container_up "$CONTAINER"; then
    log "main container already running."
    # A run that was killed hard can leave its sequence or pneumatics running in there.
    docker exec "$CONTAINER" pkill -f 'full_sequence\.py|dexnet_pnp\.py|pneumatic_control_recovery\.py' \
        >/dev/null 2>&1 || true
else
    CAMS_FRESH=1
    stop_detached zed
    stop_detached xtion
    log "starting main docker container…"
    run_bg docker bash -lc "cd '$REPO' && ORIO_NO_TTY=1 ORIO_LOGGING=$ORIO_LOGGING bash orio_run_docker.sh"
    wait_for_container "$CONTAINER" 60 || { log "main container '$CONTAINER' did not start in 60s — check the [docker] logs. Aborting."; exit 1; }
fi

# ── Pre-flight: are BOTH robots ready? ───────────────────────────────────────
# Before the heavy services, so a locked robot fails fast. No motion. Both robot-state
# topics must be flowing first — a freshly started luisa roslaunch needs a moment, and
# FrankaArm's own readiness wait gives up after ~10s.
wait_for_topic "/robot_state_publisher_node_1/robot_state" 60 \
    || { log "robot 1 never published its state — see $CONTROL_PC_LOG. Aborting."; exit 1; }
wait_for_topic "/robot_state_publisher_node_2/robot_state" 90 \
    || { log "robot 2 never published its state — check the [luisa] logs above. Aborting."; exit 1; }
log "pre-flight: checking robots 1 and 2 are ready…"
docker exec "$CONTAINER" bash -c \
        "source /home/ros_ws/devel/setup.bash && \
         cd /home/ros_ws/src/devel_packages/orio && \
         python3 full_sequence.py --check-only" 2>&1 | prefix check
if [ "${PIPESTATUS[0]}" -ne 0 ]; then
    echo
    log "A ROBOT IS NOT READY — aborting before starting cameras/planner/perception."
    log "Unlock/activate the arm named above, e.g.:"
    log "    python3 src/devel_packages/orio_bringup/lock_arms.py --unlock 1   (or 2)"
    log "then re-run this script. If a control PC itself is stuck, add --fresh."
    exit 1
fi
log "both robots are ready."

# Affordance panels (rendered by the planner; see run_dexnet_pnp_single.sh).
PANELS=0
if [ "$AFFORDANCE" -eq 1 ] && [ "$LOGGING_REQUESTED" != "0" ]; then
    PANELS=1
    mkdir -p "$AFFORDANCE_DIR"
    ln -sfn "${AFFORDANCE_DIR##*/}" "$AFFORDANCE_LINK"
    export ORIO_AFFORDANCE_DIR="$AFFORDANCE_LINK_CONT"
    log "grasp panels → $AFFORDANCE_DIR"
else
    export ORIO_AFFORDANCE_DIR=""
fi

# ── Grasp planner (reused only if its start-up settings match this run) ───────
PLANNER_CONF="planner=$PLANNER min_q=$MIN_Q panels=$PANELS"
[ "$PLANNER" = suction ] && PLANNER_CONF+=" checkpoint=$SUCTION_CKPT"
if [ "$STACK_FRESH" -eq 0 ] && container_up "$PLANNER_CONTAINER" && conf_is planner "$PLANNER_CONF"; then
    log "$PLANNER grasp planner already running (min score $MIN_Q)."
else
    log "starting $PLANNER grasp planner (min score $MIN_Q)…"
    stop_detached planner
    docker rm -f "$DEXNET_CONTAINER" "$SUCTION_CONTAINER" >/dev/null 2>&1 || true
    start_detached planner bash -lc "ORIO_LOGGING=$ORIO_LOGGING ORIO_AFFORDANCE_DIR='$ORIO_AFFORDANCE_DIR' DEXNET_MIN_Q_VALUE='$MIN_Q' SUCTION_MIN_SCORE='$MIN_Q' bash '$DOCKER_DIR/$PLANNER_SCRIPT' && docker logs -f $PLANNER_CONTAINER"
    echo "$PLANNER_CONF" >"$STACK_DIR/planner.conf"
fi
if ! wait_for_service "$PLANNER_SERVICE" 60; then
    log "$PLANNER planner did not come up in 60s — check the [$PLANNER] logs above. Aborting."
    rm -f "$STACK_DIR/planner.conf"
    exit 1
fi

# ── Cameras: ZED first (labelling), then the Xtion (picking), one at a time ──
if [ "$USE_ZED" -ne 1 ]; then
    log "ZED DISABLED (--no-zed): labelling poses will time out and arm 2 will need its"
    log "recovery GUI. The pick pipeline is unaffected."
    stop_detached zed
    docker exec "$CONTAINER" bash -c "pkill -f 'zed_only.launch|cameras.launch'" >/dev/null 2>&1 || true
elif [ "$CAMS_FRESH" -eq 0 ] && topic_live /zedm/zed_node/rgb/image_rect_color; then
    log "ZED already publishing."
else
    log "starting ZED camera…"
    stop_detached zed
    docker exec "$CONTAINER" bash -c "pkill -f 'zed_only.launch|cameras.launch'" >/dev/null 2>&1 || true
    start_detached zed docker exec "$CONTAINER" bash -c \
        "source /home/ros_ws/devel/setup.bash && roslaunch manipulation zed_only.launch"
    if ! wait_for_topic_publishing /zedm/zed_node/rgb/image_rect_color 40; then
        echo
        log "CAMERA NOT PUBLISHING: /zedm/zed_node/rgb/image_rect_color (ZED)."
        log "Its USB3 link has a known hardware fault (EPROTO drops — docs/OPEN_ISSUES.md):"
        log "replug it on USB3 and re-run, or run with --no-zed (labelling then times out)."
        log "Aborting: the labelling half of the sequence needs the ZED."
        exit 1
    fi
    log "ZED up. Letting the USB bus settle before starting the Xtion…"
    sleep 2
fi

# Xtion with the fast-fail retry (see run_dexnet_pnp_single.sh for the timing note).
XTION_WAIT="${ORIO_XTION_WAIT:-12}"
xtion_failed() {  # $1 = log file for this attempt
    grep -qE 'disconnected|Can.t initialize stream|Couldn.t create .* stream|No devices connected' \
        "$1" 2>/dev/null
}
CAM_ATTEMPTS="${ORIO_CAM_ATTEMPTS:-5}"
cam_ok=0
if [ "$CAMS_FRESH" -eq 0 ] && topic_live /camera/rgb/image_raw; then
    log "Xtion already publishing."
    cam_ok=1
fi
attempt=0
while [ "$cam_ok" -eq 0 ] && [ "$attempt" -lt "$CAM_ATTEMPTS" ]; do
    attempt=$(( attempt + 1 ))
    log "starting Xtion (attempt $attempt/$CAM_ATTEMPTS)…"
    stop_detached xtion
    docker exec "$CONTAINER" bash -c "pkill -f xtion_only.launch" >/dev/null 2>&1 || true
    start_detached xtion docker exec "$CONTAINER" bash -c \
        "source /home/ros_ws/devel/setup.bash && roslaunch manipulation xtion_only.launch"
    xtion_deadline=$(( SECONDS + XTION_WAIT ))
    while :; do
        if topic_live /camera/rgb/image_raw; then
            cam_ok=1; break
        fi
        if xtion_failed "$STACK_DIR/xtion.log"; then
            log "Xtion reported a device error on attempt $attempt — retrying immediately…"
            break
        fi
        if [ "$SECONDS" -ge "$xtion_deadline" ]; then
            log "Xtion did not publish within ${XTION_WAIT}s on attempt $attempt — retrying…"
            break
        fi
        sleep 0.5
    done
    [ "$cam_ok" -eq 1 ] && echo "[wait] topic '/camera/rgb/image_raw' is publishing."
done
if [ "$cam_ok" -ne 1 ]; then
    echo
    log "CAMERA NOT PUBLISHING: /camera/rgb/image_raw (Xtion) after $CAM_ATTEMPTS attempts."
    log "Usually a marginal USB cable / power — try another cable or port."
    log "Aborting before perception/sequence (they need camera frames)."
    exit 1
fi

# ── Perception (serves /compute_grasps and /compute_grasps_labelling) ─────────
# The labelling service needs SAM/GroundingDINO, which a planner backend skips by
# default; ORIO_LOAD_LABELLING_MODELS=1 loads them alongside it. Part of the conf key,
# so a stack started by run_dexnet_pnp_single.sh (no labelling models) is restarted
# here rather than reused.
LBL_MODELS=$([ "$USE_ZED" -eq 1 ] && echo 1 || echo 0)
PERCEPTION_CONF="backend=$PLANNER labelling=$LBL_MODELS"
if [ "$STACK_FRESH" -eq 0 ] && container_up orio_perception && conf_is perception "$PERCEPTION_CONF"; then
    log "perception already running ($PLANNER backend, labelling models=$LBL_MODELS)."
else
    log "starting perception ($PLANNER backend, labelling models=$LBL_MODELS)…"
    stop_detached perception
    docker rm -f orio_perception >/dev/null 2>&1 || true
    start_detached perception bash -lc "ORIO_NO_TTY=1 ORIO_LOGGING=$ORIO_LOGGING ORIO_LOAD_LABELLING_MODELS=$LBL_MODELS bash '$DOCKER_DIR/run_perception.sh' $PLANNER"
    echo "$PERCEPTION_CONF" >"$STACK_DIR/perception.conf"
fi

# ── RFID inventory node (/inventory/add_item) ─────────────────────────────────
# Best effort: the sequence degrades to a warning per label when it is absent.
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
    log "RFID node DISABLED (--no-rfid): label registration will log a warning per label."
fi

# ── Pneumatics (both cups), run-scoped like the pick launcher ────────────────
if [ "$USE_VACUUM" -eq 1 ]; then
    log "starting pneumatics (pnp + lbl cups)…"
    run_bg pneumatics docker exec -e ORIO_LOGGING="$ORIO_LOGGING" \
        -e ORIO_VACUUM_PORT="${ORIO_VACUUM_PORT:-/dev/ttyACM0}" "$CONTAINER" bash -c \
        "source /home/ros_ws/devel/setup.bash && cd /home/ros_ws/src/devel_packages/orio && \
         python3 -u pneumatic_control_recovery.py"
else
    log "pneumatics DISABLED (dry-run): move the cups by hand."
fi

if ! wait_for_service /compute_grasps 90; then
    log "/compute_grasps did not come up in 90s — check the [perception] logs above. Aborting."
    rm -f "$STACK_DIR/perception.conf"
    exit 1
fi
wait_for_service /compute_grasps_labelling 15 \
    || log "NOTE: /compute_grasps_labelling not up (yet) — labelling will fail until it is."

# With --vacuum, both suction services must exist BEFORE any arm moves (see
# run_dexnet_pnp_single.sh for the failure this prevents).
if [ "$USE_VACUUM" -eq 1 ]; then
    if ! wait_for_service /orio/pnp_cup/on 30 || ! wait_for_service /orio/lbl_cup/on 10; then
        echo
        log "VACUUM NOT AVAILABLE: the cup services never appeared — the pneumatics node"
        log "is not running. Check the [pneumatics] logs above (ClearCore port/firmware)."
        log "Aborting before the sequence: running with no suction moves the arms for nothing."
        log "Re-run without --vacuum for a dry run."
        exit 1
    fi
fi

# ── The sequence in the FOREGROUND so its prompts can read the keyboard ───────
log "starting the full sequence (Ctrl-C or closing this terminal stops this run; the stack stays up)…"
log "a Tkinter window will open for the item name/expiry before any motion."
echo "[launcher] sm flags: ${SM_EXTRA[*]}"
docker exec -e ORIO_LOGGING="$ORIO_LOGGING" -it "$CONTAINER" bash -c \
    "source /home/ros_ws/devel/setup.bash && cd /home/ros_ws/src/devel_packages/orio && \
     python3 -u full_sequence.py ${SM_EXTRA[*]}"

log "sequence exited."
