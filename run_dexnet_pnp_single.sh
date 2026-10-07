#!/usr/bin/env bash
# Single-terminal suction pick-and-place bring-up (no tmux). The grasp planner is the
# orio-grasping suction network by default, or DexNet with --planner dexnet.
#
# Runs the same stack as `launch_demo.sh --session orio_dexnet_pnp`, but instead of
# tmux panes it starts every service in the background and merges their output into THIS
# terminal, each line prefixed with a [module] label. The pick loop (dexnet_pnp.py) runs
# in the FOREGROUND so its --confirm "Press Enter to pick" prompt owns the keyboard.
#
# Ctrl-C stops THIS RUN only (pick loop, pneumatics, recorder). The slow-to-start stack —
# roscore, the main container, the control PC, the grasp planner, both cameras and
# perception — stays up, and the next run reuses whatever of it is still healthy and
# starts the rest. To tear the whole stack down:  bash stop_dexnet_pnp.sh
#
#   bash run_dexnet_pnp_single.sh --fresh         # stop the running stack first, start clean
#   bash run_dexnet_pnp_single.sh                 # dry-run (no vacuum), confirm each pick
#                                                 #   (Enter = execute, r = regenerate grasp, Ctrl-C = abort)
#   bash run_dexnet_pnp_single.sh --vacuum        # start pneumatics (real suction)
#   bash run_dexnet_pnp_single.sh --auto          # no per-pick Enter (auto-pick)
#                                                 #   (an empty bin still waits for a refill)
#   bash run_dexnet_pnp_single.sh --straight-down # ignore grasp tilt, approach vertically
#   bash run_dexnet_pnp_single.sh --no-logging    # skip the rerun recorder for this run
#   bash run_dexnet_pnp_single.sh --live          # also stream to a running `rerun` viewer
#   bash run_dexnet_pnp_single.sh --no-affordance # skip the per-grasp affordance panels
#   bash run_dexnet_pnp_single.sh --offset-x 0.00 --offset-y -0.015
#                                                 # world-frame XY correction (metres) added to
#                                                 #   each grasp AFTER DexNet, in the pick loop
#                                                 #   (defaults: $ORIO_PNP_OFFSET_X/Y, else 0)
#   bash run_dexnet_pnp_single.sh --min-q 0.2     # lowest planner score to accept a grasp (default:
#                                                 #   $SUCTION_MIN_SCORE / $DEXNET_MIN_Q_VALUE, else 0.30)
#   bash run_dexnet_pnp_single.sh --planner dexnet
#                                                 # grasp planner: suction (default: the orio-grasping
#                                                 #   network, checkpoint from $SUCTION_CHECKPOINT, else
#                                                 #   the newest in $ORIO_GRASPING/runs) or dexnet
#                                                 #   (default: $ORIO_PLANNER, else suction)
#
# The iam-doc control PC (robot 1) is started over ssh; it opens its own gnome-terminal
# windows separately (they run for the robot and cannot be merged here).
#
# Run logging (docs/LOGGING.md) is ON by default: a sidecar recorder writes
# logging/rerun/<run_id>/{recorder.rrd,run.json} — arm state, every node's log lines, and
# the pick loop's own events (pick/cmd/ik/grasp/service). It runs on the HOST perception
# venv (py3.10, rerun 0.37.2), not in a container, so the py38 rerun breakage that disables
# perception's own image logging does not affect it. View with:
#     rerun logging/rerun/<run_id>/recorder.rrd
#
# It also writes a diagnostic panel per grasp to logging/dexnet_pnp/<stamp>.affordance/,
# the only per-grasp image this launcher writes, rendered by the planner container. The
# suction planner draws the chosen cups on the camera image, on the height map the network
# saw and on its score map (a ~0.5 s render). DexNet draws its six-panel affordance figure
# (workspace, depth, segmask, affordance map, affordance over scene, chosen grasp), the
# same one orio_perception/test/live_affordance.py produces offline, at the cost of a
# second forward pass plus a ~0.3 s render. --no-affordance skips it; so does --no-logging.
set -uo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export ORIO_REPO="$REPO"
FRANKAPY="${ORIO_FRANKAPY:-$REPO/src/git_packages/frankapy}"
CONTAINER="${ORIO_CONTAINER:-orio_docker_container}"
# iam-doc control PC (robot 1): its franka-interface + ROS action server run over ssh.
# We tap them directly (instead of start_control_pc.sh's gnome-terminals) so the crash
# output is captured. See the "control PC" bring-up block below.
CTRL_PC_USER="${ORIO_CTRL_PC_USER:-student}"
CTRL_PC_HOST="${ORIO_CTRL_PC_HOST:-iam-doc}"
CTRL_PC_FI_PATH="${ORIO_CTRL_PC_FI_PATH:-Documents/franka-interface}"
CTRL_PC_ROBOT_IP="${ORIO_CTRL_PC_ROBOT_IP:-172.16.0.2}"
CTRL_PC_ROBOT_NUM="${ORIO_CTRL_PC_ROBOT_NUM:-1}"
WORKSTATION_HOST="$(hostname)"   # ROS master lives here; the control PC points ROS at it
DEXNET_CONTAINER="${DEXNET_CONTAINER_NAME:-orio_dexnet}"
SUCTION_CONTAINER="${SUCTION_CONTAINER_NAME:-orio_suction}"
DOCKER_DIR="$REPO/src/devel_packages/orio_bringup/docker"
WF="$REPO/src/devel_packages/orio_bringup/tmux/wait_for.sh"
# shellcheck disable=SC1090
source "$WF"

# Run logging (docs/LOGGING.md). ON by default: the recorder is a host-side sidecar and
# nothing in the pipeline depends on it. ORIO_LOGGING is ALSO exported into the containers,
# where it gates the nodes' /orio/events publishing (cheap: one std_msgs/String per event).
export ORIO_LOGGING="${ORIO_LOGGING:-1}"
export ORIO_RUN_ID="${ORIO_RUN_ID:-$(date +%Y%m%d_%H%M%S)}"
export ORIO_RUN_DIR="${ORIO_RUN_DIR:-$REPO/logging/rerun/$ORIO_RUN_ID}"
export ORIO_RERUN_LIVE="${ORIO_RERUN_LIVE:-0}"
# What the OPERATOR asked for, captured before anything can downgrade it. The recorder's
# venv check below sets ORIO_LOGGING=0 when the host venv lacks rospy/rerun — that is a
# statement about THIS HOST, not about what was wanted, and it must not reach the grasp
# images, which are drawn by PIL inside the perception container and need neither.
LOGGING_REQUESTED="$ORIO_LOGGING"
# The recorder needs rerun + rospy, which live in the host perception venv (py3.10).
PERC_DIR="$REPO/src/devel_packages/orio_perception"
PERC_VENV="${ORIO_PERCEPTION_VENV:-${ORIO_PERCEPTION_ASSETS:-$PERC_DIR}/venv}"

# ── Args ────────────────────────────────────────────────────────────────────
USE_VACUUM=0
CONFIRM=1
AFFORDANCE=1
# World-frame XY correction (metres) the pick loop adds to every grasp position after
# DexNet returns it. DexNet and the perception node never see it; it is the calibration
# residual (camera TF, default Xtion intrinsics, cup mounting) the classical path carried
# in grasp.yaml's classical.offset_x/y. Measure on the robot and set it here, via the
# environment, or with --offset-x/--offset-y. Keep grasp.yaml's dexnet.offset_x/y at 0.
PNP_OFFSET_X="${ORIO_PNP_OFFSET_X:-0.0}"
PNP_OFFSET_Y="${ORIO_PNP_OFFSET_Y:-0.0}"
# Grasp planner: suction (the orio-grasping network) or dexnet.
PLANNER="${ORIO_PLANNER:-suction}"
# Lowest score (0..1) the planner accepts: the suction network's seal x hold x access, or
# DexNet's q-value. Below it the planner answers "rejected" and the pick loop re-scans;
# enough rejections in a row count as an empty bin (see --max-declines in dexnet_pnp.py).
# Lower it to try low-confidence grasps on hard piles, raise it to skip them. Passed to
# the planner container at start-up. Resolved after the args, once the planner is known.
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
        --offset-x)      [ $# -gt 0 ] || { echo "--offset-x needs a value in metres" >&2; exit 2; }
                         PNP_OFFSET_X="$1"; shift ;;
        --offset-y)      [ $# -gt 0 ] || { echo "--offset-y needs a value in metres" >&2; exit 2; }
                         PNP_OFFSET_Y="$1"; shift ;;
        --offset-x=*)    PNP_OFFSET_X="${arg#*=}" ;;
        --offset-y=*)    PNP_OFFSET_Y="${arg#*=}" ;;
        --min-q)         [ $# -gt 0 ] || { echo "--min-q needs a value in 0..1" >&2; exit 2; }
                         MIN_Q="$1"; shift ;;
        --min-q=*)       MIN_Q="${arg#*=}" ;;
        --planner)       [ $# -gt 0 ] || { echo "--planner needs suction or dexnet" >&2; exit 2; }
                         PLANNER="$1"; shift ;;
        --planner=*)     PLANNER="${arg#*=}" ;;
        --fresh)         FRESH=1 ;;
        -h|--help) sed -n '2,53p' "$0"; exit 0 ;;
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
case "$PLANNER" in
    suction)
        PLANNER_CONTAINER="$SUCTION_CONTAINER"
        PLANNER_SERVICE=/suction_grasp_planner/plan_grasp
        PLANNER_SCRIPT=run_suction.sh
        MIN_Q="${MIN_Q:-${SUCTION_MIN_SCORE:-0.30}}" ;;
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
# Drop every item at the same height: reach into the output box by the fixed margin only,
# not by the grasp z as well. This launcher only; the tmux layout keeps per-item depth.
PNP_EXTRA+=(--zero-item-depth)

# ── Log filtering + saving ───────────────────────────────────────────────────
# The whole script's output flows through one classifier that routes each line:
#   DROP   — pure noise (TF/CUDA spam, GQCNN layer chatter, ZED per-topic advertises,
#            ikpy warnings): neither shown nor saved.
#   SAVE   — useful for debugging but not live (init/startup detail, GPU/session info,
#            control-PC + container chatter): written to the log file only.
#   SHOW   — signal (stage progress, grasps, pick/place steps, prompts, ANY error):
#            shown on the terminal AND saved.
# Errors always win: anything matching the error patterns is SHOWN regardless of source.
# ANSI colour codes are stripped so both terminal and file stay clean.
LOG_DIR="$REPO/logging/dexnet_pnp"
mkdir -p "$LOG_DIR"
RUN_STAMP="$(date +%Y%m%d_%H%M%S)"
LOG_FILE="$LOG_DIR/$RUN_STAMP.log"
# The stack's services outlive a run, so their output goes to files here (one per service,
# see start_detached) that each run follows into its own output.
STACK_DIR="$LOG_DIR/stack"
mkdir -p "$STACK_DIR"
# iam-doc's franka-interface + ROS action server go to their OWN file, raw and unfiltered:
# it's a different machine over ssh, outlives this launcher, and when diagnosing a crash
# you want every line, not the merged stream's SHOW/SAVE/DROP triage. It lives as long as
# the control PC processes do, so it is appended to, with a banner per start.
CONTROL_PC_LOG="$STACK_DIR/control_pc.log"
# The per-grasp diagnostic panel (see the header) — one per grasp, ON by default,
# --no-affordance to skip. Written by the PLANNER, in its container, because the dense
# score maps only exist there; the service response carries just the chosen grasp. The
# planner outlives a run and reads its directory once, at start-up, so it is given the
# fixed link current.affordance, which each run points at its own directory. The link is
# relative, so it resolves the same in the container ($REPO/logging is mounted there at
# /home/ros_ws/logging).
AFFORDANCE_DIR="$LOG_DIR/$RUN_STAMP.affordance"
AFFORDANCE_LINK="$LOG_DIR/current.affordance"
AFFORDANCE_LINK_CONT="/home/ros_ws/logging/dexnet_pnp/current.affordance"

# Route the whole script through the classifier. awk writes SHOW lines to stdout (fd 1 =
# terminal, still live) and SHOW+SAVE lines to the log file (via >> in awk). The pick
# loop's interactive `docker exec -it` reads stdin from the terminal, unaffected by this
# stdout pipe, so the --confirm Enter prompt still works.
exec > >(
    LOG_FILE="$LOG_FILE" awk '
    function strip(s){ gsub(/\033\[[0-9;]*m/, "", s); gsub(/\r/, "", s); return s }
    {
        line = strip($0)
        show = 0; save = 0

        # DROP FIRST: import-time noise that would otherwise be rescued by a SHOW rule
        # below (the [check] prefix, or a bare "WARNING"). autolab_core is pip-installed,
        # not built as a catkin package, so its optional ROS *service* helpers
        # (publish_to_ros / delete_from_ros / rigid_transform_from_ros) are unavailable.
        # Nothing in this repo or the vendored packages calls them — we use RigidTransform
        # only as a pose container and do TF through tf2/frankapy. Harmless, one per run.
        if (line ~ /autolab_core not installed as catkin package/) { next }

        # SHOW: signal + any error, regardless of source.
        if (line ~ /\[launcher\]|\[check\]|\[doc\]|\[stop\]|\[Pick\]|\[Place\]|\[Loop\]|\[Grasp\]|\[Main\]|\[Hardware\]|Press Enter/) { show=1 }
        if (line ~ /GAVE UP|ERROR|Error|error|Traceback|has died|disconnected|CAMERA NOT DETECTED|FAIL|Rejected|refus|not ready|NOT ready|Aborting/) { show=1 }
        if (line ~ /Planned suction grasp|grasp q=/) { show=1 }
        if (line ~ /Affordance panel|Grasp panel/) { show=1 }

        # DROP: pure noise — never shown, never saved.
        else if (line ~ /tensorflow|cuda_gpu|dso_loader|NUMA node|StreamExecutor|gpu_device|libcu[a-z]+\.so|XLA service|Building (convolutional|fully|Softmax)|Converting fc layer|Advertised on topic|ikpy|is of type .fixed.|Detection max range|warnings\.warn/) { next }

        # Everything else: SAVE to file only (debug context, not live).
        if (show) { save=1 }
        else { save=1; show=0 }

        # Terminal gets CRLF, the log file plain LF. The pick loop runs under
        # `docker exec -it`, so its pty is in cooked mode and the terminal stays in raw-ish
        # output mode for the whole merged stream: there a bare \n is only a LINE FEED (down
        # one row, SAME column), so lines would staircase to the right. The \r resets the
        # column. We strip the CRs the pty adds in strip() and re-add exactly one here, so
        # pty-borne and pipe-borne lines are terminated identically. The file wants no CRs.
        if (save && length(line)) { print line >> ENVIRON["LOG_FILE"]; fflush(ENVIRON["LOG_FILE"]) }
        if (show && length(line)) { printf "%s\r\n", line; fflush() }
    }'
) 2>&1
echo "[launcher] logging this run to $LOG_FILE"

# ── Log merging ──────────────────────────────────────────────────────────────
# Each service's stdout+stderr is piped through a prefixer and merged into this
# terminal. PIDs are tracked so the trap can kill them on exit.
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
# roscore, the main container, the control PC, the grasp planner, both cameras and
# perception take most of the start-up time, so they outlive a run and the next run reuses
# them. Each runs in its own session (setsid), so the terminal's Ctrl-C and hang-up never
# reach it, and writes to $STACK_DIR/<name>.log, which every run follows (follow) into its
# merged output. Its process group id goes to <name>.pid so it can be killed as a whole.
# bash stop_dexnet_pnp.sh tears the stack down.
STACK_SERVICES=(roscore control_pc_fi control_pc_ros planner zed xtion perception)

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

# tail --pid=$$: killing the ( … ) subshell does not kill the tail inside it, and the log
# outlives the run, so without it every finished run left a follower re-printing new
# lines into its old log and terminal. With it, the tail ends when this launcher does.
follow() {  # $1 = label, $2 = service name; for this run only
    ( tail -n0 -F --pid=$$ "$STACK_DIR/$2.log" 2>/dev/null | prefix "$1" ) &
    PIDS+=($!)
}

# Quiet health checks (the wait_for_* gates print "GAVE UP" lines, which read as errors).
container_up() { docker ps -q -f "name=^$1\$" 2>/dev/null | grep -q .; }
roscore_up()   { _ros_list rostopic >/dev/null 2>&1; }
topic_live()   {
    timeout 6 docker exec "$ROS1_CONTAINER" bash -c \
        "$_ROS1_SETUP; timeout 4 rostopic echo -n 1 $1" >/dev/null 2>&1
}
conf_is() { [ "$(cat "$STACK_DIR/$1.conf" 2>/dev/null)" = "$2" ]; }  # $1 = name, $2 = config

# ── Teardown (this run only) ─────────────────────────────────────────────────
_CLEANED=0
cleanup() {
    [ "$_CLEANED" -eq 1 ] && return   # run once: INT then EXIT (or HUP) both fire
    _CLEANED=1
    # Ignore further stop signals so an impatient second Ctrl-C can't abort teardown.
    trap '' INT TERM HUP
    echo
    log "stopping this run… (Ctrl-C again won't interrupt this)"
    # Flush the rerun recorder FIRST, before anything it is recording goes away.
    # It writes recorder.rrd + run.json on SIGINT; a plain kill in the PIDS loop below
    # would drop rerun's batcher window (~200 ms) and, worse, skip run.json entirely.
    # (This mirrors what stop_demo.sh does to the recorder pane in the tmux layout.)
    if [ -n "${RECORDER_PID:-}" ] && kill -0 "$RECORDER_PID" 2>/dev/null; then
        log "flushing run recorder…"
        kill -INT "$RECORDER_PID" 2>/dev/null || true
        for _ in $(seq 1 25); do          # up to ~5 s, same budget as stop_demo.sh
            kill -0 "$RECORDER_PID" 2>/dev/null || break
            sleep 0.2
        done
        kill -0 "$RECORDER_PID" 2>/dev/null && log "recorder still running — killing it."
    fi
    # Stop the vacuum first if it was running (best effort).
    if [ "$USE_VACUUM" -eq 1 ]; then
        docker exec "$CONTAINER" bash -lc \
          "source /opt/ros/noetic/setup.bash 2>/dev/null; \
           rosservice call /orio/pnp_cup/off 2>/dev/null" >/dev/null 2>&1 || true
    fi
    # The pick loop and pneumatics run inside the main container, which stays up. Killing
    # a `docker exec` client does not stop the process it started, so stop them in there.
    docker exec "$CONTAINER" pkill -f 'dexnet_pnp\.py|pneumatic_control_recovery\.py' \
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
    log "the stack (cameras, planner, perception, control PC) is still running; the next run"
    log "reuses it. To stop everything: bash stop_dexnet_pnp.sh"
    # Give the tee (process substitution) a moment to flush the final lines to the file.
    sync; sleep 0.2
}
# HUP = terminal window closed (was previously untrapped → everything leaked).
# On INT/TERM/HUP we run cleanup then exit; the EXIT trap also runs cleanup for the
# normal-exit path (pick loop returns on its own), guarded by _CLEANED so it's once.
trap 'cleanup; exit 130' INT
trap 'cleanup; exit 143' TERM
trap 'cleanup; exit 129' HUP
trap cleanup EXIT

# Follow the stack's logs from the start, so a service started below shows from its first
# line and one already running shows from now on.
follow roscore roscore
follow "$PLANNER" planner
follow cameras zed
follow cameras xtion
follow perception perception

if [ "$FRESH" -eq 1 ]; then
    log "--fresh: stopping the running stack first…"
    bash "$REPO/stop_dexnet_pnp.sh"
fi

# ── roscore ──────────────────────────────────────────────────────────────────
# Every ROS node registers with the master, so if it is gone nothing else is reusable:
# clear the rest of the stack and start it all again (STACK_FRESH forces each step below).
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
# Sidecar: subscribes only, and nothing downstream depends on it — if it is absent, slow
# or crashed, the run is unaffected. Started right after roscore so it captures the whole
# bring-up (including a failure during it), and tracked in its OWN pid so teardown can
# SIGINT it FIRST and let rerun flush the .rrd before the containers go away.
RECORDER_PID=""
if [ "$ORIO_LOGGING" != "0" ]; then
    # The recorder needs BOTH rerun and rospy in one interpreter. Check before starting it:
    # on a host with no ROS1 (this workstation is 22.04 — noetic is not installable, which
    # is why the whole stack is containerised) the import fails and the recorder would die
    # a few seconds in, having already printed a traceback into the merged stream. Checking
    # here turns that into one clear, actionable line. See docs/LOGGING.md.
    if [ ! -x "$PERC_VENV/bin/python3" ]; then
        log "NOTE: no perception venv at $PERC_VENV — run logging is OFF for this run."
        log "      (set ORIO_PERCEPTION_VENV, or pass --no-logging to silence this)"
        ORIO_LOGGING=0
    elif ! "$PERC_VENV/bin/python3" -c "import rospy, rerun" >/dev/null 2>&1; then
        log "NOTE: $PERC_VENV lacks rospy and/or rerun — run logging is OFF for this run."
        log "      The recorder needs both in ONE interpreter. rospy normally comes from a"
        log "      sourced ROS1 env, which this host does not have (/opt/ros/noetic is empty)."
        log "      To enable it, add the pure-python ROS1 packages to that venv:"
        log "        $PERC_VENV/bin/pip install --extra-index-url https://rospypi.github.io/simple/ \\"
        log "            rospy rosgraph_msgs std_msgs geometry_msgs sensor_msgs actionlib_msgs"
        log "      (rerun 0.37.2 needs py>=3.9, so this cannot live in the py38 containers.)"
        log "      Pass --no-logging to silence this."
        ORIO_LOGGING=0
    else
        log "starting run recorder → $ORIO_RUN_DIR"
        # Single arm here, so --robots 1: asking for arm 2 would log an empty arm and its
        # 11 MB of static meshes, and place it via cell.yaml at a base that isn't in use.
        ( "$PERC_VENV/bin/python3" "$REPO/src/devel_packages/orio_logging/recorder.py" \
              --robots 1 2>&1 | prefix recorder ) &
        RECORDER_PID=$!
        PIDS+=("$RECORDER_PID")
    fi
fi

# Gate: is iam-doc reachable over ssh AT ALL? If sshd is down/wedged (TCP connects but no
# banner) or the host is unreachable, the control-PC taps below launch into the void and the
# run hangs later at the pre-flight FrankaArm construction (which blocks in frankapy's
# wait_for_franka_interface with no timeout we control). Fail fast here with a clear message.
log "checking ssh to control PC ($CTRL_PC_HOST)…"
if ! timeout 15 ssh -o BatchMode=yes -o ConnectTimeout=10 \
        "$CTRL_PC_USER@$CTRL_PC_HOST" 'true' >/dev/null 2>&1; then
    echo
    log "CANNOT SSH to $CTRL_PC_USER@$CTRL_PC_HOST — the control PC is unreachable."
    log "The host may be down, mid-boot, or its sshd is wedged (TCP connects but no banner)."
    log "Checks:"
    log "  - ping $CTRL_PC_HOST        (is the host up / on the network?)"
    log "  - nc -vz $CTRL_PC_HOST 22   (is sshd answering?)"
    log "  - if ping works but ssh times out on the banner: reboot iam-doc or, at its"
    log "    console, 'sudo systemctl restart ssh'. (Often follows a reboot / port move.)"
    log "Aborting before bring-up (nothing can run without the control PC)."
    exit 1
fi
log "ssh to control PC OK."

# Tripwire: forward only GENUINE control-PC failures into the merged [doc] stream, so the
# terminal stays clean but a real controller death shows up immediately. Everything (incl.
# the benign chatter below) is still saved in full to CONTROL_PC_LOG for debugging.
#
# franka-interface is noisy in NORMAL operation, so we must EXCLUDE its routine chatter or
# a healthy run looks like a wall of red (see the 20260921_120405 run):
#   - "libprotobuf ERROR ... RobotStateMessage ... missing required fields": a transient
#     per-frame shared-memory parse glitch; the next frame is fine. Fired 44x on a good run.
#   - "Caught Franka Exception" / "automatic error recovery" / "Error recovery finished":
#     the controller hit a recoverable reflex between skills and SELF-HEALED. The presence
#     of a recovery line means it's fine — not a crash.
#   - "Skill terminated" / "Will run the control loop": normal per-motion lifecycle.
# We KEEP: real deaths / unrecoverable faults (has died, control loop exit, libfranka
# fatal, connection refused, e-stop) — the things that actually stop the robot.
( tail -n0 -F --pid=$$ "$CONTROL_PC_LOG" 2>/dev/null \
    | awk '
        # benign, self-healing, or routine — never forward to the terminal (still in file):
        /libprotobuf/                                        { next }
        /RobotStateMessage/                                  { next }
        /ParsingFromArray Exception/                          { next }  # same per-frame parse glitch, wrapper msg
        /done_skill_id error/                                { next }  # normal skill-id bookkeeping WARN
        /[Cc]aught Franka Exception/                         { next }
        /automatic error recovery|Error recovery finished/   { next }
        /Skill terminated|Will run the control loop/         { next }
        /Failed to get static transforms/                    { next }
        # "Robot state save thread ... Franka exception": a distress line that repeats
        # ~120x/min when the controller is wedged (see the 115030 stall). Show it ONCE so
        # you know state-saving is unhappy, then suppress the flood (all copies in file).
        /Robot state save thread encountered Franka exception/ {
            if (seen_statesave) next; seen_statesave=1
            print "[note] control PC: robot-state save thread is throwing exceptions (repeating; suppressed after first — controller may be wedged)"
            fflush(); next
        }
        # genuine failure signatures — forward these:
        /has died|process has died|control loop|libfranka|terminate called|core dumped|Aborted|segmentation|refused|Connection reset|cannot connect|not connected|e-stop|estop|FCI|reflex/ { print; fflush(); next }
        # anything else with error/exception/violation that is NOT one of the benign cases above:
        /[Ee]rror|[Ee]xception|violation|[Ff]atal/           { print; fflush() }
      ' \
    | prefix doc ) &
PIDS+=($!)

# The control PC is reused when both of its processes are still running on iam-doc. If
# franka-interface is up but unhappy (reflex, wrong mode), the pre-flight check below
# fails and --fresh restarts it.
control_pc_up() {
    timeout 15 ssh -o BatchMode=yes -o ConnectTimeout=8 "$CTRL_PC_USER@$CTRL_PC_HOST" \
        'pgrep -f "[f]ranka_interface --robot_ip" >/dev/null && pgrep -f "[r]oslaunch franka_ros_interface" >/dev/null' \
        >/dev/null 2>&1
}

# The two ssh sessions that run them, started detached (they are part of the stack).
# `ssh -tt` gives a pty so franka-interface flushes; the remote commands mirror the
# reference start_franka_interface_on_control_pc.sh / start_franka_ros_interface scripts.
# (1) franka-interface — the realtime C++ controller. Its stderr carries the crash cause.
control_pc_fi() {
    ssh -tt -o BatchMode=yes -o ConnectTimeout=10 "$CTRL_PC_USER@$CTRL_PC_HOST" \
        "mkdir -p ~/franka_logs && ls -t ~/franka_logs/*.log 2>/dev/null | tail -n +51 | xargs -r rm -f; \
         cd '$CTRL_PC_FI_PATH/build' && \
         stdbuf -oL -eL ./franka_interface --robot_ip '$CTRL_PC_ROBOT_IP' --with_gripper 0 --log 0 --stop_on_error 0 \
           2>&1 | tee ~/franka_logs/franka_interface_\$(date +%Y%m%d_%H%M%S).log" \
        2>&1 | ts_prefix >>"$CONTROL_PC_LOG"
}
# (2) ROS action server — bridges franka-interface to ROS; publishes robot_state.
control_pc_ros() {
    ssh -tt -o BatchMode=yes -o ConnectTimeout=10 "$CTRL_PC_USER@$CTRL_PC_HOST" \
        "cd '$CTRL_PC_FI_PATH' && source bash_scripts/set_rosmaster.sh '$CTRL_PC_HOST' '$WORKSTATION_HOST' && source catkin_ws/devel/setup.bash && roslaunch franka_ros_interface franka_ros_interface.launch robot_num:=$CTRL_PC_ROBOT_NUM" \
        2>&1 | ts_prefix >>"$CONTROL_PC_LOG"
}
export -f control_pc_fi control_pc_ros ts_prefix
export CTRL_PC_USER CTRL_PC_HOST CTRL_PC_FI_PATH CTRL_PC_ROBOT_IP CTRL_PC_ROBOT_NUM \
       WORKSTATION_HOST CONTROL_PC_LOG

# franka-interface can also wedge while still running: a skill sent during its automatic
# error recovery is read back empty from shared memory, the control loop dies on
# std::bad_cast, and every later motion is ignored until the process restarts. So a
# bad_cast since its last start (the last bring-up banner in the log) means restart it.
control_pc_wedged() {
    awk '/===== control PC bring-up/ { bad = 0 } /bad_cast/ { bad = 1 } END { exit !bad }' \
        "$CONTROL_PC_LOG" 2>/dev/null
}

CTRL_PC_REUSE=0
if [ "$STACK_FRESH" -eq 0 ] && control_pc_up; then
    if control_pc_wedged; then
        log "control PC hit std::bad_cast since it started (franka-interface is wedged) — restarting it."
    else
        CTRL_PC_REUSE=1
    fi
fi
if [ "$CTRL_PC_REUSE" -eq 1 ]; then
    log "control PC already running on $CTRL_PC_HOST."
else
stop_detached control_pc_fi
stop_detached control_pc_ros
# Clear stale control-PC state BEFORE starting franka-interface. stop_dexnet_pnp.sh does
# this too, but a stack that crashed or was never stopped never ran it — so its shared
# memory may still be sitting there. franka-interface
# does not remove these on exit (verified: they survive even a clean SIGTERM), and a
# segment left with a LOCKED mutex makes the next start hang at
# "Will try to acquire lock while setting franka_interface status". Start from clean.
control_pc_cleanup() {
log "clearing stale control-PC state on $CTRL_PC_HOST…"
timeout 30 ssh -o BatchMode=yes -o ConnectTimeout=8 "$CTRL_PC_USER@$CTRL_PC_HOST" \
    'bash -s' <<'REMOTE_CLEANUP' >/dev/null 2>&1 || true
# CAREFUL: "franka_interface" is 16 chars and Linux truncates the process `comm` field to
# 15 ("franka_interfac"). So `pkill -x franka_interface` NEVER matches, and
# `ps -eo comm | grep -x franka_interface` ALWAYS returns nothing. An earlier version of
# this cleanup used both and therefore never killed anything, while still deleting the
# /dev/shm segments out from under the live process — which is worse than doing nothing
# and directly caused the "Will try to acquire lock" hang on the next run.
# Match on the full command line (pgrep -f) instead, and count with `pgrep -fc`.
fi_count() { pgrep -f '[f]ranka_interface --robot_ip' | wc -l; }

# 1) Ask nicely — franka-interface releases its lock and closes FCI on SIGTERM.
pkill -TERM -f '[f]ranka_interface --robot_ip' 2>/dev/null
pkill -TERM -f '[r]oslaunch franka_ros_interface' 2>/dev/null

# 2) Wait for a real exit (up to 10s), verifying with a match that actually works.
for _ in $(seq 1 20); do
    [ "$(fi_count)" -eq 0 ] && break
    sleep 0.5
done

# 3) Force anything that refused to die, then confirm it is really gone.
if [ "$(fi_count)" -ne 0 ]; then
    pkill -KILL -f '[f]ranka_interface --robot_ip' 2>/dev/null
    sleep 1
fi
pkill -KILL -f '[r]oslaunch franka_ros_interface' 2>/dev/null
pkill -KILL -f '[f]ranka_ros_interface' 2>/dev/null

# 4) ONLY once nothing is running may we remove the shared memory. Deleting it while a
#    process still holds it is what corrupts the next start.
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
# We do NOT use start_control_pc.sh here: it launches franka-interface + the ROS action
# server in gnome-terminals ON iam-doc, whose output we can't capture — so when the
# control PC crashes (libfranka reflex, comms violation, FCI drop) the reason is lost.
# Instead we ssh in ourselves and tee the raw combined stdout/stderr to CONTROL_PC_LOG.
{
    echo "===== control PC bring-up @ $(date -Is) — $CTRL_PC_USER@$CTRL_PC_HOST ====="
    echo "===== franka-interface | robot_ip=$CTRL_PC_ROBOT_IP ====="
} >>"$CONTROL_PC_LOG"
# franka-interface exits at once with "command not possible in the current mode" when the
# arm is not accepting commands. Right after the previous franka-interface let go of it
# that can clear by itself, so retry once; if it persists the arm is locked, user-stopped
# or in an error state, which only the operator can clear, so stop and say so.
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
    log "ROBOT NOT ACCEPTING COMMANDS: franka-interface keeps exiting with \"command not"
    log "possible in the current mode\". The arm is locked, user-stopped or in an error state."
    log "  - Check the arm's light: white = user stop pressed (release it); red = error"
    log "    (acknowledge it in Desk); yellow = brakes locked."
    log "  - Unlock and activate it:  python3 src/devel_packages/orio_bringup/lock_arms.py --unlock 1"
    log "  - In Desk, make sure FCI is activated."
    log "Then re-run this script. Aborting."
    exit 1
fi
start_detached control_pc_ros control_pc_ros
log "control PC: franka-interface + action server launched over ssh (detail in ${CONTROL_PC_LOG##*/})"
sleep 3
fi

# The cameras run inside the main container, so a new container means new cameras.
CAMS_FRESH="$STACK_FRESH"
if container_up "$CONTAINER"; then
    log "main container already running."
    # A run that was killed hard can leave its pick loop or pneumatics node running in
    # there; two of either would fight over the arm or the ClearCore's serial port.
    docker exec "$CONTAINER" pkill -f 'dexnet_pnp\.py|pneumatic_control_recovery\.py' \
        >/dev/null 2>&1 || true
else
    CAMS_FRESH=1
    stop_detached zed
    stop_detached xtion
    log "starting main docker container…"
    # ORIO_NO_TTY=1: run detached, no -it (we background it through a pipe = no TTY).
    run_bg docker bash -lc "cd '$REPO' && ORIO_NO_TTY=1 ORIO_LOGGING=$ORIO_LOGGING bash orio_run_docker.sh"
    wait_for_container "$CONTAINER" 60 || { log "main container '$CONTAINER' did not start in 60s — check the [docker] logs. Aborting."; exit 1; }
fi

# ── Pre-flight: is the robot ready? ──────────────────────────────────────────
# Check BEFORE the heavy services (grasp planner, cameras, perception) so a
# locked / not-activated robot fails fast instead of after the full bring-up. Uses the
# same readiness path (frankapy) the real run uses. No motion.
log "pre-flight: checking robot 1 is ready…"
docker exec "$CONTAINER" bash -c \
        "source /home/ros_ws/devel/setup.bash && \
         cd /home/ros_ws/src/devel_packages/orio && \
         python3 dexnet_pnp.py --check-only" 2>&1 | prefix check
# ${PIPESTATUS[0]} is the docker exec's status; the sed prefixer always exits 0.
if [ "${PIPESTATUS[0]}" -ne 0 ]; then
    echo
    log "ROBOT NOT READY — aborting before starting cameras/planner/perception."
    log "Unlock/activate robot 1, e.g.:"
    log "    python3 src/devel_packages/orio_bringup/lock_arms.py --unlock 1"
    log "then re-run this script. If the control PC itself is stuck, add --fresh."
    exit 1
fi
log "robot is ready."

# Gated on LOGGING_REQUESTED as well as the flag: the recorder's venv check may have
# zeroed ORIO_LOGGING, and these panels are rendered in the planner container and need
# nothing from the host venv. --no-logging and --no-affordance both turn them off.
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
# The planner reads its settings once, at start-up: reuse it only if they match this run.
PLANNER_CONF="planner=$PLANNER min_q=$MIN_Q panels=$PANELS"
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
    rm -f "$STACK_DIR/planner.conf"   # so the next run restarts it rather than reusing it
    exit 1
fi

# Cameras — started one at a time (ZED, then Xtion) so their init doesn't interleave, with
# a retry around the Xtion, which intermittently fails to open its IR stream and drops off
# the bus. NOTE: USB autosuspend and bus-bandwidth contention were both investigated and
# ruled out (the Xtion shows suspended_time=0 and is the only high-speed device on its bus);
# the remaining suspicion is the cable/power/camera itself. The retry is the mitigation.
# (See docs/TROUBLESHOOTING.md.)

# (1) ZED first.
if [ "$CAMS_FRESH" -eq 0 ] && topic_live /zedm/zed_node/rgb/image_rect_color; then
    log "ZED already publishing."
else
    log "starting ZED camera…"
    stop_detached zed
    docker exec "$CONTAINER" bash -c "pkill -f 'zed_only.launch|cameras.launch'" >/dev/null 2>&1 || true
    start_detached zed docker exec "$CONTAINER" bash -c \
        "source /home/ros_ws/devel/setup.bash && roslaunch manipulation zed_only.launch"
    if ! wait_for_topic_publishing /zedm/zed_node/rgb/image_rect_color 40; then
        echo
        log "CAMERA NOT PUBLISHING: /zedm/zed_node/rgb/image_rect_color (ZED). Check the ZED"
        log "is connected and the [cameras] logs above. Aborting."
        exit 1
    fi
    log "ZED up. Letting the USB bus settle before starting the Xtion…"
    sleep 2   # let ZED init traffic on Bus 001 quiesce before the Xtion claims bandwidth
fi

# (2) Xtion second, with a self-healing retry.
#
# Timing note: when the Xtion flaps it dies FAST — it logs `Device "1d27/..." disconnected`
# about 2s in — while a healthy start publishes in ~6s. So rather than waiting out a long
# topic timeout (which burned ~23s of dead time per failed attempt), we race two signals:
#   success → /camera/rgb/image_raw actually publishing
#   failure → the driver printing "disconnected" / "Can't initialize stream"
# and retry the moment either lands. A failed attempt now costs ~2-3s instead of 25s.
# XTION_WAIT is a backstop for the case where it neither publishes nor says anything.
XTION_WAIT="${ORIO_XTION_WAIT:-12}"   # backstop; success normally lands in ~6s

# Watch an attempt's log for the driver's own failure signatures.
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

    # Each attempt starts a fresh xtion.log, which we watch for failure lines while the
    # run's follow merges it into the terminal/log as [cameras].
    start_detached xtion docker exec "$CONTAINER" bash -c \
        "source /home/ros_ws/devel/setup.bash && roslaunch manipulation xtion_only.launch"

    # Race: publishing (success) vs. a driver failure line (fail fast) vs. backstop timeout.
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
    log "The driver usually fails with \"Can't initialize stream of type 1\" (the IR stream)"
    log "and then \"disconnected\". Ruled out already: USB autosuspend (suspended_time=0) and"
    log "bus bandwidth (the Xtion is the only high-speed device on its bus). Likely causes:"
    log "  - Marginal USB cable or insufficient power — try another cable / a rear-panel"
    log "    port / a powered hub. (Most common cause of this exact symptom.)"
    log "  - The Xtion itself failing at init (old hardware; IR draws the most power)."
    log "  - Xtion unplugged / claimed by a stale container (docker ps | grep camera)"
    log "  See docs/OPEN_ISSUES.md if this becomes persistent."
    log "Aborting before perception/pick (they need camera frames)."
    exit 1
fi

PERCEPTION_CONF="backend=$PLANNER"
if [ "$STACK_FRESH" -eq 0 ] && container_up orio_perception && conf_is perception "$PERCEPTION_CONF"; then
    log "perception already running ($PLANNER backend)."
else
    log "starting perception ($PLANNER backend)…"
    stop_detached perception
    docker rm -f orio_perception >/dev/null 2>&1 || true
    start_detached perception bash -lc "ORIO_NO_TTY=1 ORIO_LOGGING=$ORIO_LOGGING bash '$DOCKER_DIR/run_perception.sh' $PLANNER"
    echo "$PERCEPTION_CONF" >"$STACK_DIR/perception.conf"
fi

if [ "$USE_VACUUM" -eq 1 ]; then
    log "starting pneumatics…"
    # Runs INSIDE the main container, not on the host: the host python3 has no rospy, so
    # the node died on `import rospy` the moment it started, /orio/pnp_cup/{on,off} were
    # never advertised, and every pick timed out on them while the arm ran the full motion
    # with no suction (see the 20260922_150030 run). The ClearCore is reachable from in
    # there — the container is --privileged with -v /dev:/dev, so /dev/ttyACM0 is the same
    # device node. ORIO_VACUUM_PORT still overrides it.
    run_bg pneumatics docker exec -e ORIO_LOGGING="$ORIO_LOGGING" \
        -e ORIO_VACUUM_PORT="${ORIO_VACUUM_PORT:-/dev/ttyACM0}" "$CONTAINER" bash -c \
        "source /home/ros_ws/devel/setup.bash && cd /home/ros_ws/src/devel_packages/orio && \
         python3 -u pneumatic_control_recovery.py"
else
    log "pneumatics DISABLED (dry-run): move the cup by hand."
fi

if ! wait_for_service /compute_grasps 90; then
    log "/compute_grasps did not come up in 90s — check the [perception] logs above"
    log "(model load, backend=$PLANNER, camera frames). Aborting."
    rm -f "$STACK_DIR/perception.conf"   # so the next run restarts it rather than reusing it
    exit 1
fi

# Gate: with --vacuum, the suction services must actually exist BEFORE the arm moves.
# Without this the pick loop runs the whole motion — descend, grip, retract, transit,
# drop — with the cup off, timing out ~8s on /orio/pnp_cup/on mid-pick and again on
# /orio/pnp_cup/off at the place. That looks like a hardware/suction problem but is just
# a node that never started, and the failure scrolls past ~50s before the first pick.
# Fail here instead, while nothing is holding anything.
if [ "$USE_VACUUM" -eq 1 ]; then
    if ! wait_for_service /orio/pnp_cup/on 30; then
        echo
        log "VACUUM NOT AVAILABLE: /orio/pnp_cup/on never appeared — the pneumatics node"
        log "is not running. Check the [pneumatics] logs above. Common causes:"
        log "  - ClearCore not plugged in / on another port: ls -l /dev/ttyACM* and set"
        log "    ORIO_VACUUM_PORT (default /dev/ttyACM0)."
        log "  - The ClearCore lost its firmware (pneumatic_control/pneumatic_control.ino)"
        log "    so it never enumerates as a serial device."
        log "  - An import error in pneumatic_control_recovery.py (it needs the container's"
        log "    ROS python, not the host's — the host python3 has no rospy)."
        log "Aborting before the pick loop: picking with no suction moves the arm for nothing."
        log "Re-run without --vacuum for a dry run."
        exit 1
    fi
fi

# ── Pick loop in the FOREGROUND so --confirm can read the keyboard ────────────
log "starting pick-and-place loop (Ctrl-C or closing this terminal stops this run; the stack stays up)…"
echo "[launcher] pnp flags: ${PNP_EXTRA[*]}"
docker exec -e ORIO_LOGGING="$ORIO_LOGGING" -it "$CONTAINER" bash -c \
    "source /home/ros_ws/devel/setup.bash && cd /home/ros_ws/src/devel_packages/orio && \
     python3 -u dexnet_pnp.py ${PNP_EXTRA[*]}"

log "pick loop exited."
