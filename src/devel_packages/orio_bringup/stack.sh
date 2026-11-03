#!/usr/bin/env bash
# Shared bring-up library for the single-terminal launchers (run_full_demo.sh,
# run_pickup-only.sh, grasp_test.sh, reset_robot.sh, snap_xtion.sh,
# research/labelling_arm_test/*) and the stop scripts. Source it after setting REPO:
#
#   REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
#   LOG_TAG=launcher                      # prefix of log lines (default: launcher)
#   STACK_DIR="$REPO/logging/dexnet_pnp/stack"   # where detached services log (default)
#   source "$REPO/src/devel_packages/orio_bringup/stack.sh"
#
# What it provides, in the order a launcher uses it:
#   output      prefix ts_prefix run_bg log kill_bg log_filter_start
#   stack       start_detached stop_detached follow conf_is stack_service stack_wait_service
#               stack_bringup_roscore stack_kill_pids
#   health      container_up roscore_up topic_live zed_depth_mode
#   control PC  control_pc_bringup control_pc_teardown (robot 1, iam-doc)
#               luisa_bringup luisa_kill_session luisa_kill_stack (robot 2, iam-luisa)
#   services    main_container_bringup xtion_bringup zed_bringup zed_stop
#               planner_resolve planner_bringup perception_bringup ffs_bringup ffs_wait
#               recorder_start recorder_flush pneumatics_start pneumatics_wait vacuum_off
#   run         affordance_panels_setup run_log_paths unlock_robot preflight_check
#               run_in_container stop_run_in_container install_cleanup_traps
#
# Every *_bringup returns non-zero after printing what went wrong, so a launcher writes
#   control_pc_bringup || exit 1
# Readiness waits come from tmux/wait_for.sh, sourced here.
#
# The stack is the set of slow-to-start services that outlive a run: roscore, the main
# container, the control PCs, the grasp planner, the cameras, Fast-FoundationStereo and
# perception. Each runs in its own session (setsid) so the terminal's Ctrl-C never reaches
# it, logs to $STACK_DIR/<name>.log, which a run follows into its merged output, and
# records its process group in <name>.pid so a stop script can kill it as a whole. A
# service that reads its settings at start-up also writes <name>.conf; it is reused only
# when the conf matches the run's.

[ -n "${REPO:-}" ] || { echo "stack.sh: set REPO before sourcing" >&2; return 2 2>/dev/null || exit 2; }

export ORIO_REPO="$REPO"
BRINGUP_DIR="$REPO/src/devel_packages/orio_bringup"
DOCKER_DIR="$BRINGUP_DIR/docker"
LOG_TAG="${LOG_TAG:-launcher}"
STACK_DIR="${STACK_DIR:-$REPO/logging/dexnet_pnp/stack}"
mkdir -p "$STACK_DIR"

# Containers. The ROS1 master, the main orio container (frankapy, cameras, the arm
# scripts), the planners, perception and Fast-FoundationStereo.
CONTAINER="${ORIO_CONTAINER:-orio_docker_container}"
ROS1_CONTAINER="${ORIO_ROS1_CONTAINER:-orio_roscore}"
DEXNET_CONTAINER="${DEXNET_CONTAINER_NAME:-orio_dexnet}"
SUCTION_CONTAINER="${SUCTION_CONTAINER_NAME:-orio_suction}"
PERCEPTION_CONTAINER="${ORIO_PERCEPTION_CONTAINER:-orio_perception}"
FFS_CONTAINER="${ORIO_FFS_CONTAINER:-orio_ffs}"

# iam-doc control PC (robot 1, the pick-and-place arm): franka-interface + the ROS action
# server, started over ssh so their output is captured (frankapy's start_control_pc.sh
# opens gnome-terminals on the control PC, whose output is lost when it crashes).
CTRL_PC_USER="${ORIO_CTRL_PC_USER:-student}"
CTRL_PC_HOST="${ORIO_CTRL_PC_HOST:-iam-doc}"
CTRL_PC_FI_PATH="${ORIO_CTRL_PC_FI_PATH:-Documents/franka-interface}"
CTRL_PC_ROBOT_IP="${ORIO_CTRL_PC_ROBOT_IP:-172.16.0.2}"
CTRL_PC_ROBOT_NUM="${ORIO_CTRL_PC_ROBOT_NUM:-1}"
WORKSTATION_HOST="$(hostname)"   # the ROS master lives here; the control PCs point ROS at it
# Raw, unfiltered output of both control-PC processes: a different machine, outlives a run,
# and when diagnosing a crash you want every line. Appended to, with a banner per start.
CONTROL_PC_LOG="${CONTROL_PC_LOG:-$STACK_DIR/control_pc.log}"
# iam-luisa control PC (robot 2, the label arm): its stack runs in tmux inside a container
# THERE, brought up by start_control_pc_luisa.sh (idempotent).
LUISA_PC="${ORIO_LUISA_HOST:-snaak@iam-luisa}"

# Camera topics the health checks look at.
XTION_TOPIC="${XTION_TOPIC:-/camera/rgb/image_raw}"
ZED_TOPIC="${ZED_TOPIC:-/zedm/zed_node/rgb/image_rect_color}"

# The host perception venv (py3.10): the only interpreter with both rospy and rerun, which
# the run recorder needs.
PERC_DIR="$REPO/src/devel_packages/orio_perception"
PERC_VENV="${ORIO_PERCEPTION_VENV:-${ORIO_PERCEPTION_ASSETS:-$PERC_DIR}/venv}"

# Which detached services and containers belong to this launcher's stack. A launcher
# overrides these before stack_bringup_roscore; a fresh master clears them all.
STACK_SERVICES=(roscore)
STACK_CONTAINERS=()

# Set by the *_bringup functions so a teardown can undo only what this run started.
STACK_FRESH=1          # 1 until stack_bringup_roscore finds a live master
CAMS_FRESH=1           # 1 when the main container (and so the cameras) is new
CTRL_PC_STARTED=0
LUISA_STARTED=0
CONTAINER_STARTED=0
XTION_STARTED=0
RECORDER_PID=""
PIDS=()

# shellcheck disable=SC1091
source "$BRINGUP_DIR/tmux/wait_for.sh"

# ── Output ───────────────────────────────────────────────────────────────────
prefix() {  # $1 = label; reads stdin, writes "[label] line"
    local label="$1"
    sed -u "s/^/[$label] /" 2>/dev/null || sed "s/^/[$label] /"
}

ts_prefix() {  # reads stdin, prepends a timestamp; for the raw control-PC log
    while IFS= read -r line; do printf '%s %s\n' "$(date +%H:%M:%S)" "$line"; done
}

run_bg() {  # $1 = label, rest = command; run in the background, prefixed, for this run only
    local label="$1"; shift
    ( "$@" 2>&1 | prefix "$label" ) &
    PIDS+=($!)
}

log() { echo "[$LOG_TAG] $*"; }

kill_bg() {  # stop this run's background processes (followers, recorder, pneumatics, ...)
    local pid
    for pid in "${PIDS[@]:-}"; do [ -n "$pid" ] && kill "$pid" 2>/dev/null || true; done
}

# Route the whole launcher's output through one classifier, which sends each line to:
#   DROP  pure noise (TF/CUDA spam, GQCNN layer chatter, ZED per-topic advertises, ikpy
#         warnings): neither shown nor saved;
#   SAVE  useful for debugging but not live (start-up detail, container chatter): the log
#         file only;
#   SHOW  signal (stage progress, grasps, prompts, ANY error): the terminal AND the file.
# $1 = extra SHOW regex for this launcher's own stage prefixes (ERE; may be empty).
# $2 = 1 to show every line that is not dropped (a small launcher with little chatter).
# Errors always win: a line matching the error patterns is shown whatever else it matches.
# ANSI colours are stripped. The terminal gets CRLF: the foreground script runs under
# `docker exec -it`, whose pty leaves the terminal in raw-ish output mode for the whole
# merged stream, where a bare \n only moves down a row and lines would staircase. The
# file gets plain LF. Needs LOG_FILE.
log_filter_start() {
    exec > >(
        LOG_FILE="$LOG_FILE" SHOW_RE="${1:-}" SHOW_ALL="${2:-0}" awk '
        function strip(s){ gsub(/\033\[[0-9;]*m/, "", s); gsub(/\r/, "", s); return s }
        {
            line = strip($0)
            if (!length(line)) next
            # autolab_core is pip-installed, not a catkin package, so it reports its
            # optional ROS helpers missing once per import. Nothing here uses them.
            if (line ~ /autolab_core not installed as catkin package/) next

            show = (ENVIRON["SHOW_ALL"] == "1")
            if (ENVIRON["SHOW_RE"] != "" && line ~ ENVIRON["SHOW_RE"]) show = 1
            if (line ~ /\[launcher\]|\[check\]|\[doc\]|\[luisa\]|\[stop\]|\[Main\]|\[Hardware\]|\[Grasp\]|\[Loop\]|Press Enter/) show = 1
            if (line ~ /GAVE UP|ERROR|Error|error|Traceback|has died|disconnected|CAMERA NOT DETECTED|FAIL|Rejected|refus|not ready|NOT ready|Aborting/) show = 1
            if (line ~ /Planned suction grasp|grasp q=|Affordance panel|Grasp panel/) show = 1

            if (!show && line ~ /tensorflow|cuda_gpu|dso_loader|NUMA node|StreamExecutor|gpu_device|libcu[a-z]+\.so|XLA service|Building (convolutional|fully|Softmax)|Converting fc layer|Advertised on topic|ikpy|is of type .fixed.|Detection max range|warnings\.warn|TF_CPP/) next

            print line >> ENVIRON["LOG_FILE"]; fflush(ENVIRON["LOG_FILE"])
            if (show) { printf "%s\r\n", line; fflush() }
        }'
    ) 2>&1
    echo "[$LOG_TAG] logging this run to $LOG_FILE"
}

# ── Detached services (the stack) ────────────────────────────────────────────
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
# outlives the run, so the tail must end when this launcher does.
follow() {  # $1 = label, $2 = service name; merges the service's log into this run
    ( tail -n0 -F --pid=$$ "$STACK_DIR/$2.log" 2>/dev/null | prefix "$1" ) &
    PIDS+=($!)
}

conf_is() { [ "$(cat "$STACK_DIR/$1.conf" 2>/dev/null)" = "$2" ]; }  # $1 = name, $2 = conf

# Start a long-running containerised service unless the same one is already up: the
# container is running, the master was not restarted, and its recorded conf matches.
# $1 = name, $2 = container, $3 = conf, rest = command. Sets SERVICE_REUSED to 1 or 0.
stack_service() {
    local name="$1" container="$2" conf="$3"; shift 3
    if [ "$STACK_FRESH" -eq 0 ] && container_up "$container" && conf_is "$name" "$conf"; then
        SERVICE_REUSED=1
        return 0
    fi
    SERVICE_REUSED=0
    stop_detached "$name"
    docker rm -f "$container" >/dev/null 2>&1 || true
    start_detached "$name" "$@"
    echo "$conf" >"$STACK_DIR/$name.conf"
}

# Wait for a service's ROS service; on a timeout drop its conf so the next run restarts it
# instead of reusing a broken one. $1 = name, $2 = ROS service, $3 = timeout s.
stack_wait_service() {
    wait_for_service "$2" "$3" && return 0
    rm -f "$STACK_DIR/$1.conf"
    return 1
}

stack_kill_pids() {  # kill every detached service in STACK_DIR, except the names given
    local f name keep
    for f in "$STACK_DIR"/*.pid; do
        [ -f "$f" ] || continue
        name="$(basename "$f" .pid)"
        for keep in "$@"; do [ "$name" = "$keep" ] && continue 2; done
        kill -- "-$(cat "$f")" 2>/dev/null
        rm -f "$f"
    done
}

# ── Health checks (quiet; the wait_for_* gates print "GAVE UP" lines) ─────────
container_up() { docker ps -q -f "name=^$1\$" 2>/dev/null | grep -q .; }
roscore_up()   { _ros_list rostopic >/dev/null 2>&1; }
topic_live()   {  # true once the topic actually streams a message
    timeout 6 docker exec "$ROS1_CONTAINER" bash -c \
        "$_ROS1_SETUP; timeout 4 rostopic echo -n 1 $1" >/dev/null 2>&1
}
zed_depth_mode() {  # the depth mode the running ZED driver was started with
    timeout 6 docker exec "$ROS1_CONTAINER" bash -c \
        "$_ROS1_SETUP; rosparam get /zedm/zed_node/depth/depth_mode" 2>/dev/null
}

# ── roscore ──────────────────────────────────────────────────────────────────
# Every ROS node registers with the master, so if it is gone nothing else is reusable:
# the stack's services and containers are cleared and started again (STACK_FRESH=1 makes
# every later step start rather than reuse).
stack_bringup_roscore() {
    STACK_FRESH=0
    if roscore_up; then
        log "roscore already running — reusing the stack where it is healthy."
        return 0
    fi
    STACK_FRESH=1
    log "no roscore — starting the stack from scratch…"
    local s
    for s in "${STACK_SERVICES[@]}"; do stop_detached "$s"; done
    docker rm -f "$ROS1_CONTAINER" "${STACK_CONTAINERS[@]}" >/dev/null 2>&1 || true
    rm -f "$STACK_DIR"/*.conf
    log "starting roscore…"
    start_detached roscore bash "$DOCKER_DIR/run_roscore.sh"
    wait_for_roscore 30 || { log "roscore did not start in 30s — check the [roscore] logs. Aborting."; return 1; }
}

# ── Control PC 1: iam-doc (robot 1) ──────────────────────────────────────────
ssh_ctrl_pc() {  # $1 = timeout s, rest = ssh arguments
    local t="$1"; shift
    timeout "$t" ssh -o BatchMode=yes -o ConnectTimeout=8 "$CTRL_PC_USER@$CTRL_PC_HOST" "$@"
}

# Both control-PC processes running there.
control_pc_up() {
    ssh_ctrl_pc 15 'pgrep -f "[f]ranka_interface --robot_ip" >/dev/null && pgrep -f "[r]oslaunch franka_ros_interface" >/dev/null' \
        >/dev/null 2>&1
}

# The two ssh sessions that run the processes, started detached (they are part of the
# stack). `ssh -tt` gives a pty so franka-interface flushes; the remote commands mirror
# franka-interface's start_franka_interface_on_control_pc.sh / start_franka_ros_interface.
# (1) franka-interface, the realtime C++ controller. Its stderr carries the crash cause.
control_pc_fi() {
    ssh -tt -o BatchMode=yes -o ConnectTimeout=10 "$CTRL_PC_USER@$CTRL_PC_HOST" \
        "mkdir -p ~/franka_logs && ls -t ~/franka_logs/*.log 2>/dev/null | tail -n +51 | xargs -r rm -f; \
         cd '$CTRL_PC_FI_PATH/build' && \
         stdbuf -oL -eL ./franka_interface --robot_ip '$CTRL_PC_ROBOT_IP' --with_gripper 0 --log 0 --stop_on_error 0 \
           2>&1 | tee ~/franka_logs/franka_interface_\$(date +%Y%m%d_%H%M%S).log" \
        2>&1 | ts_prefix >>"$CONTROL_PC_LOG"
}
# (2) the ROS action server: bridges franka-interface to ROS, publishes robot_state.
control_pc_ros() {
    ssh -tt -o BatchMode=yes -o ConnectTimeout=10 "$CTRL_PC_USER@$CTRL_PC_HOST" \
        "cd '$CTRL_PC_FI_PATH' && source bash_scripts/set_rosmaster.sh '$CTRL_PC_HOST' '$WORKSTATION_HOST' && source catkin_ws/devel/setup.bash && roslaunch franka_ros_interface franka_ros_interface.launch robot_num:=$CTRL_PC_ROBOT_NUM" \
        2>&1 | ts_prefix >>"$CONTROL_PC_LOG"
}

# Stop both processes ON the control PC, gracefully, and clear what they leave behind.
# Killing the local ssh sessions does not stop them, and they must not be SIGKILLed first:
# franka-interface keeps the robot state in shared memory (/dev/shm) behind a mutex and
# holds the FCI connection, and a kill -9 leaves a locked mutex (the next start hangs at
# "Will try to acquire lock while setting franka_interface status"), a half-written buffer
# (the next run dies on std::bad_cast) or the arm in a mode that rejects commands. So:
# SIGTERM, wait, SIGKILL only what is left, and remove the shared memory ONLY once nothing
# holds it: deleting it under a live process is what corrupts the next start.
# "franka_interface" is 16 characters and Linux truncates a process's comm to 15, so
# `pkill -x franka_interface` never matches; match the full command line instead.
_CTRL_PC_REMOTE_STOP='
fi_count() { pgrep -f "[f]ranka_interface --robot_ip" | wc -l; }
pkill -TERM -f "[r]oslaunch franka_ros_interface" 2>/dev/null
pkill -TERM -f "[f]ranka_interface --robot_ip" 2>/dev/null
for _ in $(seq 1 20); do [ "$(fi_count)" -eq 0 ] && break; sleep 0.5; done
if [ "$(fi_count)" -ne 0 ]; then
    echo "franka_interface did not exit on SIGTERM - forcing."
    pkill -KILL -f "[f]ranka_interface --robot_ip" 2>/dev/null
    sleep 1
fi
pkill -KILL -f "[r]oslaunch franka_ros_interface" 2>/dev/null
pkill -KILL -f "[f]ranka_ros_interface" 2>/dev/null
if [ "$(fi_count)" -eq 0 ]; then
    rm -f /dev/shm/run_loop_* /dev/shm/current_robot_state* /dev/shm/franka_interface_state_info_mutex 2>/dev/null
    echo "control PC stopped"
else
    echo "WARNING: franka_interface STILL running after SIGKILL - left /dev/shm intact."
fi
'
_control_pc_stop_remote() { ssh_ctrl_pc 35 'bash -s' <<<"$_CTRL_PC_REMOTE_STOP"; }

# Before a start: clear stale state quietly. A stack that crashed or was never stopped
# never ran the teardown, so its shared memory may still be sitting there.
control_pc_cleanup() {
    log "clearing stale control-PC state on $CTRL_PC_HOST…"
    _control_pc_stop_remote >/dev/null 2>&1 || true
}

# At a stop: the same, reporting what happened under a [doc] prefix.
control_pc_teardown() {
    log "stopping control-PC processes on $CTRL_PC_HOST…"
    local out rc
    out="$(_control_pc_stop_remote 2>/dev/null)"; rc=$?
    [ -n "$out" ] && printf '%s\n' "$out" | prefix doc
    [ "$rc" -eq 0 ] || log "WARNING: could not reach $CTRL_PC_HOST; stop franka-interface there by hand"
}

# franka-interface can wedge while still running: a skill sent during its automatic error
# recovery is read back empty from shared memory, the control loop dies on std::bad_cast,
# and every later motion is ignored until it restarts. A bad_cast since its last start
# (the last bring-up banner in the log) means restart it.
control_pc_wedged() {
    awk '/===== control PC bring-up/ { bad = 0 } /bad_cast/ { bad = 1 } END { exit !bad }' \
        "$CONTROL_PC_LOG" 2>/dev/null
}

# Tripwire: forward only GENUINE control-PC failures into the merged [doc] stream, so the
# terminal stays clean but a real controller death shows up at once. Everything, including
# the benign chatter dropped here, is still in CONTROL_PC_LOG.
# franka-interface is noisy in NORMAL operation: "libprotobuf ERROR ... missing required
# fields" is a per-frame shared-memory parse glitch (44x on a good run); "Caught Franka
# Exception" / "automatic error recovery" is a recoverable reflex that SELF-HEALED; "Skill
# terminated" / "Will run the control loop" is the per-motion lifecycle. Kept: real deaths
# and unrecoverable faults, the things that actually stop the robot.
_control_pc_tripwire() {
    ( tail -n0 -F --pid=$$ "$CONTROL_PC_LOG" 2>/dev/null \
        | awk '
            /libprotobuf/                                        { next }
            /RobotStateMessage/                                  { next }
            # the tail of a parse-glitch line that arrived split in two (its field list)
            /(current|last_motion)_errors_/                      { next }
            /ParsingFromArray Exception/                         { next }
            /done_skill_id error/                                { next }
            /[Cc]aught Franka Exception/                         { next }
            /automatic error recovery|Error recovery finished/   { next }
            /Skill terminated|Will run the control loop/         { next }
            /Failed to get static transforms/                    { next }
            # a distress line that repeats ~120x/min when the controller is wedged: once.
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
}

# Bring up robot 1's control PC, reusing a healthy one. Sets CTRL_PC_STARTED=1 when it
# started the processes. Returns 1 when the control PC is unreachable or the arm refuses
# commands, after saying what to do.
control_pc_bringup() {
    # Gate: reachable over ssh AT ALL? If sshd is down or wedged (TCP connects but no
    # banner) the taps below launch into the void and the run hangs later in frankapy's
    # wait_for_franka_interface, which has no timeout. Fail fast here instead.
    log "checking ssh to control PC ($CTRL_PC_HOST)…"
    if ! timeout 15 ssh -o BatchMode=yes -o ConnectTimeout=10 "$CTRL_PC_USER@$CTRL_PC_HOST" true >/dev/null 2>&1; then
        echo
        log "CANNOT SSH to $CTRL_PC_USER@$CTRL_PC_HOST — the control PC is unreachable."
        log "The host may be down, mid-boot, or its sshd is wedged (TCP connects but no banner)."
        log "Checks:"
        log "  - ping $CTRL_PC_HOST        (is the host up / on the network?)"
        log "  - nc -vz $CTRL_PC_HOST 22   (is sshd answering?)"
        log "  - if ping works but ssh times out on the banner: reboot $CTRL_PC_HOST or, at its"
        log "    console, 'sudo systemctl restart ssh'. (Often follows a reboot / port move.)"
        log "Aborting before bring-up (nothing can run without the control PC)."
        return 1
    fi
    log "ssh to control PC OK."
    _control_pc_tripwire

    local reuse=0
    if [ "$STACK_FRESH" -eq 0 ] && control_pc_up; then
        if control_pc_wedged; then
            log "control PC hit std::bad_cast since it started (franka-interface is wedged) — restarting it."
        else
            reuse=1
        fi
    fi
    if [ "$reuse" -eq 1 ]; then
        log "control PC already running on $CTRL_PC_HOST."
        return 0
    fi

    CTRL_PC_STARTED=1
    stop_detached control_pc_fi
    stop_detached control_pc_ros
    control_pc_cleanup
    export -f control_pc_fi control_pc_ros ts_prefix
    export CTRL_PC_USER CTRL_PC_HOST CTRL_PC_FI_PATH CTRL_PC_ROBOT_IP CTRL_PC_ROBOT_NUM \
           WORKSTATION_HOST CONTROL_PC_LOG
    log "starting robot-1 control PC ($CTRL_PC_HOST) over ssh — logging to $CONTROL_PC_LOG"
    {
        echo "===== control PC bring-up @ $(date -Is) — $CTRL_PC_USER@$CTRL_PC_HOST ====="
        echo "===== franka-interface | robot_ip=$CTRL_PC_ROBOT_IP ====="
    } >>"$CONTROL_PC_LOG"
    # franka-interface exits at once with "command not possible in the current mode" when
    # the arm is not accepting commands. Right after the previous franka-interface let go
    # of it that can clear by itself, so retry once; if it persists the arm is locked,
    # user-stopped or in an error state, which only the operator can clear.
    local attempts="${ORIO_FI_ATTEMPTS:-2}" attempt from mode_error=0
    for attempt in $(seq 1 "$attempts"); do
        from=$(( $(wc -l <"$CONTROL_PC_LOG") + 1 ))
        start_detached control_pc_fi control_pc_fi
        sleep 5
        ssh_ctrl_pc 15 'pgrep -f "[f]ranka_interface --robot_ip" >/dev/null' >/dev/null 2>&1 && { mode_error=0; break; }
        tail -n +"$from" "$CONTROL_PC_LOG" | grep -q "not possible in the current mode" || break
        mode_error=1
        [ "$attempt" -eq "$attempts" ] && break
        log "arm is not accepting commands yet (franka-interface attempt $attempt/$attempts) — retrying in 10 s…"
        sleep 10
        control_pc_cleanup
    done
    if [ "$mode_error" -eq 1 ]; then
        echo
        log "ROBOT NOT ACCEPTING COMMANDS: franka-interface keeps exiting with \"command not"
        log "possible in the current mode\". The arm is locked, user-stopped or in an error state."
        log "  - Check the arm's light: white = user stop pressed (release it); red = error"
        log "    (acknowledge it in Desk); yellow = brakes locked."
        log "  - Unlock and activate it:  python3 src/devel_packages/orio_bringup/lock_arms.py --unlock $CTRL_PC_ROBOT_NUM"
        log "  - In Desk, make sure FCI is activated."
        log "Then re-run this script. Aborting."
        return 1
    fi
    start_detached control_pc_ros control_pc_ros
    log "control PC: franka-interface + action server launched over ssh (detail in ${CONTROL_PC_LOG##*/})"
    sleep 3
}

# ── Control PC 2: iam-luisa (robot 2) ────────────────────────────────────────
ssh_luisa() {  # $1 = timeout s, rest = ssh arguments
    local t="$1"; shift
    timeout "$t" ssh -o BatchMode=yes -o ConnectTimeout=8 "$LUISA_PC" "$@"
}

luisa_up() {  # the franka tmux session exists in the franka-interface container there
    ssh_luisa 20 "docker exec franka-interface bash -lc 'tmux has-session -t franka'" >/dev/null 2>&1
}

# Same wedge as control PC 1's: after a std::bad_cast its franka-interface still accepts
# skills but never runs them. Its output lives only in the tmux window, whose scrollback
# starts with the session, so this means "since it started".
luisa_wedged() {
    ssh_luisa 20 "docker exec franka-interface bash -lc 'tmux capture-pane -p -J -t franka:franka_interface -S -'" \
        2>/dev/null | grep -q bad_cast
}

# The session outlives franka-interface itself: after a user stop or an unrecoverable
# fault it aborts while the ROS action server beside it keeps publishing robot_state. A
# session without the process is that case; the only way back is to start the stack again.
luisa_fi_dead() {
    local out
    out="$(ssh_luisa 20 "docker exec franka-interface bash -lc 'tmux has-session -t franka 2>/dev/null && echo SESSION; pgrep -f \"[f]ranka_interface --robot_ip\" >/dev/null && echo PROCESS'" 2>/dev/null)"
    [[ "$out" == *SESSION* && "$out" != *PROCESS* ]]
}

# End the stack's tmux session there; the franka-interface container itself stays (it is
# managed by docker compose on that machine and cheap to keep). No TTY: this may run from
# a background process, where -t would suspend it.
luisa_kill_session() {
    ssh_luisa 30 "docker exec -i franka-interface bash -lc 'tmux kill-session -t franka 2>/dev/null; true'" >/dev/null 2>&1
}

# Stop the stack there, then clear franka-interface's shared memory ONLY once nothing runs
# (see control_pc_teardown for why). No `#` comments inside: ssh flattens it to one line.
luisa_kill_stack() {
    ssh_luisa 40 "docker exec -i franka-interface bash -lc '
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

# Bring up robot 2's control PC, reusing a healthy one. Its output goes through [luisa]
# and to $STACK_DIR/luisa.log. Sets LUISA_STARTED=1 when it (re)started the stack.
luisa_bringup() {
    local reuse=0
    if [ "$STACK_FRESH" -eq 0 ] && luisa_up; then
        if luisa_wedged; then
            log "control PC 2 hit std::bad_cast since it started (franka-interface is wedged) — restarting it."
        elif luisa_fi_dead; then
            log "franka-interface on $LUISA_PC has exited (user stop / fault) — restarting robot 2's stack…"
        else
            reuse=1
        fi
    fi
    if [ "$reuse" -eq 1 ]; then
        log "control PC 2 (luisa) already running."
        return 0
    fi
    if luisa_up; then
        # A fresh master: the old session points at a roscore that no longer exists.
        [ "$STACK_FRESH" -eq 1 ] && log "restarting control PC 2 (luisa) — its session points at the old master…"
        luisa_kill_stack
        sleep 2
    fi
    log "starting robot-2 control PC ($LUISA_PC)…"
    local out rc
    out="$(bash "$BRINGUP_DIR/start_control_pc_luisa.sh" 2>&1)"; rc=$?
    if [ -n "$out" ]; then
        printf '%s\n' "$out" >>"$STACK_DIR/luisa.log"
        printf '%s\n' "$out" | prefix luisa
    fi
    if [ "$rc" -ne 0 ]; then
        echo
        log "CONTROL PC 2 (luisa) DID NOT COME UP — see the [luisa] lines above."
        log "Checks: ssh $LUISA_PC; is 'iam-luisa' in /etc/hosts (192.168.2.3)?"
        return 1
    fi
    [[ "$out" == *ALREADY_RUNNING* ]] || LUISA_STARTED=1
}

# ── Main container (frankapy, the cameras and the arm scripts run in it) ──────
# $1 = pkill pattern of run processes a hard-killed run may have left in a reused
# container (two pick loops would fight over the arm, two pneumatics over the serial
# port). Sets CAMS_FRESH (a new container means new cameras) and CONTAINER_STARTED.
main_container_bringup() {
    CAMS_FRESH="$STACK_FRESH"
    if container_up "$CONTAINER"; then
        log "main container already running."
        if [ -n "${1:-}" ]; then
            docker exec "$CONTAINER" pkill -f "$1" >/dev/null 2>&1 || true
        fi
        return 0
    fi
    CAMS_FRESH=1
    CONTAINER_STARTED=1
    stop_detached zed
    stop_detached xtion
    log "starting main docker container…"
    # ORIO_NO_TTY=1: detached, no -it (it runs backgrounded through a pipe, with no TTY).
    run_bg docker bash -lc "ORIO_NO_TTY=1 ORIO_LOGGING=${ORIO_LOGGING:-1} bash '$DOCKER_DIR/run_orio.sh'"
    wait_for_container "$CONTAINER" 60 \
        || { log "main container '$CONTAINER' did not start in 60s — check the [docker] logs. Aborting."; return 1; }
}

# ── Cameras ──────────────────────────────────────────────────────────────────
# The ZED's driver publishes a topic only while something subscribes, so the health check
# itself (one rostopic echo) is what makes an idle driver publish. zed_only.launch starts
# it with the depth mode given (NONE = no depth model; the label depth is
# Fast-FoundationStereo's). A running driver is reused only with the mode this run needs.
# $1 = depth mode (default NONE).
zed_bringup() {
    local mode="${1:-NONE}"
    if [ "$CAMS_FRESH" -eq 0 ] && topic_live "$ZED_TOPIC" && [ "$(zed_depth_mode)" = "$mode" ]; then
        log "ZED already publishing (depth mode $mode)."
        return 0
    fi
    log "starting ZED camera (depth mode $mode)…"
    zed_stop
    start_detached zed docker exec "$CONTAINER" bash -c \
        "source /home/ros_ws/devel/setup.bash && roslaunch manipulation zed_only.launch depth_mode:=$mode"
    if ! wait_for_topic_publishing "$ZED_TOPIC" 40; then
        echo
        log "CAMERA NOT PUBLISHING: $ZED_TOPIC (ZED). Check it is connected and the [cameras]"
        log "lines above. Its USB3 link has a known fault (EPROTO drops — docs/OPEN_ISSUES.md):"
        log "replug it on USB3 and re-run. Aborting."
        return 1
    fi
    log "ZED up. Letting the USB bus settle before starting the Xtion…"
    sleep 2
}

zed_stop() {
    stop_detached zed
    docker exec "$CONTAINER" bash -c "pkill -f 'zed_only.launch|cameras.launch'" >/dev/null 2>&1 || true
}

# The Xtion intermittently fails to open its IR stream and drops off the bus (USB
# autosuspend and bus bandwidth were ruled out; the remaining suspicion is the cable,
# power or the camera itself — docs/TROUBLESHOOTING.md). The retry is the mitigation.
# When it flaps it dies FAST (`Device ... disconnected` ~2 s in) while a healthy start
# publishes in ~6 s, so each attempt races the topic publishing against the driver's own
# failure lines and retries the moment either lands; ORIO_XTION_WAIT is the backstop.
# Sets XTION_STARTED=1 when it started the driver.
_xtion_failed() {  # $1 = this attempt's log
    grep -qE 'disconnected|Can.t initialize stream|Couldn.t create .* stream|No devices connected' "$1" 2>/dev/null
}
xtion_bringup() {
    local wait="${ORIO_XTION_WAIT:-12}" attempts="${ORIO_CAM_ATTEMPTS:-5}" attempt=0 ok=0 deadline
    if [ "$CAMS_FRESH" -eq 0 ] && topic_live "$XTION_TOPIC"; then
        log "Xtion already publishing."
        return 0
    fi
    XTION_STARTED=1
    while [ "$ok" -eq 0 ] && [ "$attempt" -lt "$attempts" ]; do
        attempt=$(( attempt + 1 ))
        log "starting Xtion (attempt $attempt/$attempts)…"
        stop_detached xtion
        docker exec "$CONTAINER" bash -c "pkill -f xtion_only.launch" >/dev/null 2>&1 || true
        start_detached xtion docker exec "$CONTAINER" bash -c \
            "source /home/ros_ws/devel/setup.bash && roslaunch manipulation xtion_only.launch"
        deadline=$(( SECONDS + wait ))
        while :; do
            if topic_live "$XTION_TOPIC"; then ok=1; break; fi
            if _xtion_failed "$STACK_DIR/xtion.log"; then
                log "Xtion reported a device error on attempt $attempt — retrying immediately…"
                break
            fi
            if [ "$SECONDS" -ge "$deadline" ]; then
                log "Xtion did not publish within ${wait}s on attempt $attempt — retrying…"
                break
            fi
            sleep 0.5
        done
        [ "$ok" -eq 1 ] && echo "[wait] topic '$XTION_TOPIC' is publishing."
    done
    if [ "$ok" -ne 1 ]; then
        echo
        log "CAMERA NOT PUBLISHING: $XTION_TOPIC (Xtion) after $attempts attempts."
        log "The driver usually fails with \"Can't initialize stream of type 1\" (the IR stream)"
        log "and then \"disconnected\". Likely causes:"
        log "  - Marginal USB cable or insufficient power — try another cable / a rear-panel"
        log "    port / a powered hub. (Most common cause of this exact symptom.)"
        log "  - The Xtion itself failing at init (old hardware; IR draws the most power)."
        log "  - Xtion unplugged / claimed by a stale container (docker ps | grep camera)"
        log "  See docs/TROUBLESHOOTING.md. Aborting."
        return 1
    fi
}

# ── Grasp planner ────────────────────────────────────────────────────────────
# Turn PLANNER (suction|dexnet), SUCTION_CKPT (a run tag, a path or empty) and MIN_Q
# (empty = default) into PLANNER_CONTAINER, PLANNER_SERVICE, PLANNER_SCRIPT, a resolved
# SUCTION_CKPT and a validated MIN_Q, exported for the planner scripts. Returns 2 on a bad
# value, after saying which.
planner_resolve() {
    local grasping="${ORIO_GRASPING:-$HOME/orio-grasping}" tag
    case "$PLANNER" in
        suction)
            PLANNER_CONTAINER="$SUCTION_CONTAINER"
            PLANNER_SERVICE=/suction_grasp_planner/plan_grasp
            PLANNER_SCRIPT=run_suction.sh
            MIN_Q="${MIN_Q:-${SUCTION_MIN_SCORE:-0.30}}"
            # Checkpoint: a path, a run tag (the newest $grasping/runs/*_<tag>/best.pt) or,
            # empty, the newest $grasping/runs/*/best.pt (run_suction.sh's own default).
            if [ -z "$SUCTION_CKPT" ]; then
                SUCTION_CKPT="$(ls -t "$grasping"/runs/*/best.pt 2>/dev/null | head -1)"
                [ -n "$SUCTION_CKPT" ] || { echo "no checkpoint: train one into $grasping/runs/ or set SUCTION_CHECKPOINT" >&2; return 2; }
            elif [ ! -f "$SUCTION_CKPT" ]; then
                tag="$SUCTION_CKPT"
                SUCTION_CKPT="$(ls -t "$grasping"/runs/*_"$tag"/best.pt 2>/dev/null | head -1)"
                [ -n "$SUCTION_CKPT" ] || { echo "--checkpoint: no $grasping/runs/*_$tag/best.pt" >&2; return 2; }
            fi
            SUCTION_CKPT="$(readlink -f "$SUCTION_CKPT")"
            export SUCTION_CHECKPOINT="$SUCTION_CKPT" ;;
        dexnet)
            PLANNER_CONTAINER="$DEXNET_CONTAINER"
            PLANNER_SERVICE=/dexnet_grasp_planner/plan_grasp
            PLANNER_SCRIPT=run_dexnet.sh
            MIN_Q="${MIN_Q:-${DEXNET_MIN_Q_VALUE:-0.30}}" ;;
        *) echo "--planner must be suction or dexnet, got '$PLANNER'" >&2; return 2 ;;
    esac
    if ! [[ "$MIN_Q" =~ ^(1(\.0+)?|0(\.[0-9]+)?|\.[0-9]+)$ ]]; then
        echo "--min-q must be a number between 0 and 1, got '$MIN_Q'" >&2; return 2
    fi
    export DEXNET_MIN_Q_VALUE="$MIN_Q" SUCTION_MIN_SCORE="$MIN_Q"
}

# The planner reads its settings once, at start-up, so it is reused only when they match
# this run (planner, score threshold, panels, checkpoint). Needs planner_resolve and
# affordance_panels_setup first. Both planner containers are removed on a start so a
# switch of planner does not leave the other one holding the GPU.
planner_bringup() {
    local conf="planner=$PLANNER min_q=$MIN_Q panels=$PANELS"
    [ "$PLANNER" = suction ] && conf+=" checkpoint=$SUCTION_CKPT"
    if [ "$STACK_FRESH" -eq 0 ] && container_up "$PLANNER_CONTAINER" && conf_is planner "$conf"; then
        log "$PLANNER grasp planner already running (min score $MIN_Q)."
    else
        log "starting $PLANNER grasp planner (min score $MIN_Q)…"
        stop_detached planner
        docker rm -f "$DEXNET_CONTAINER" "$SUCTION_CONTAINER" >/dev/null 2>&1 || true
        start_detached planner bash -lc "ORIO_LOGGING=${ORIO_LOGGING:-1} ORIO_AFFORDANCE_DIR='${ORIO_AFFORDANCE_DIR:-}' DEXNET_MIN_Q_VALUE='$MIN_Q' SUCTION_MIN_SCORE='$MIN_Q' bash '$DOCKER_DIR/$PLANNER_SCRIPT' && docker logs -f $PLANNER_CONTAINER"
        echo "$conf" >"$STACK_DIR/planner.conf"
    fi
    stack_wait_service planner "$PLANNER_SERVICE" 60 \
        || { log "$PLANNER planner did not come up in 60s — check the [$PLANNER] logs above. Aborting."; return 1; }
}

# ── Perception ───────────────────────────────────────────────────────────────
# $1 = conf key (what this run needs it started with), $2 = extra env assignments for
# run_perception.sh (e.g. ORIO_LOAD_LABELLING_MODELS=1). The caller waits for its
# services with stack_wait_service perception …
perception_bringup() {
    stack_service perception "$PERCEPTION_CONTAINER" "$1" \
        bash -lc "ORIO_NO_TTY=1 ORIO_LOGGING=${ORIO_LOGGING:-1} ${2:-} bash '$DOCKER_DIR/run_perception.sh' $PLANNER"
    if [ "$SERVICE_REUSED" -eq 1 ]; then
        log "perception already running ($1)."
    else
        log "starting perception ($1)…"
    fi
}

# ── Fast-FoundationStereo (label depth from the ZED pair: /ffs/compute) ──────
ffs_bringup() {
    local conf="scale=${ORIO_FFS_SCALE:-0.5} iters=${ORIO_FFS_ITERS:-8} ckpt=${ORIO_FFS_CHECKPOINT:-default} rate=${ORIO_FFS_RATE:-0}"
    stack_service ffs "$FFS_CONTAINER" "$conf" \
        bash -lc "bash '$DOCKER_DIR/run_ffs.sh' && docker logs -f $FFS_CONTAINER"
    if [ "$SERVICE_REUSED" -eq 1 ]; then
        log "Fast-FoundationStereo already running ($conf)."
    else
        log "starting Fast-FoundationStereo ($conf)…"
    fi
}

ffs_wait() {
    stack_wait_service ffs /ffs/compute 180 && return 0
    echo
    log "/ffs/compute did not come up in 180s — check the [ffs] lines above (image built?"
    log "checkpoint found? see src/devel_packages/orio_bringup/docker/run_ffs.sh)."
    log "Aborting: without it no label pose has a depth."
    return 1
}

# ── Run recorder (docs/LOGGING.md) ───────────────────────────────────────────
# A sidecar: subscribes only, and nothing downstream depends on it. Started right after
# roscore so it captures the whole bring-up, and tracked in its OWN pid so a teardown can
# SIGINT it FIRST and let rerun flush the .rrd before the containers go away. Needs both
# rerun and rospy in one interpreter, the host perception venv; when that venv lacks them
# this says so once and turns run logging off for the run (ORIO_LOGGING=0) rather than
# letting the recorder die a few seconds in with a traceback. Arguments go to recorder.py.
recorder_start() {
    RECORDER_PID=""
    [ "${ORIO_LOGGING:-1}" != "0" ] || return 0
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
        ( "$PERC_VENV/bin/python3" "$REPO/src/devel_packages/orio_logging/recorder.py" "$@" \
              2>&1 | prefix recorder ) &
        RECORDER_PID=$!
        PIDS+=("$RECORDER_PID")
    fi
}

# Flush the recorder FIRST at a teardown, before anything it records goes away: it writes
# recorder.rrd + run.json on SIGINT, and a plain kill would drop rerun's batcher window
# and skip run.json entirely.
recorder_flush() {
    [ -n "$RECORDER_PID" ] && kill -0 "$RECORDER_PID" 2>/dev/null || return 0
    log "flushing run recorder…"
    kill -INT "$RECORDER_PID" 2>/dev/null || true
    local i
    for i in $(seq 1 25); do          # up to ~5 s
        kill -0 "$RECORDER_PID" 2>/dev/null || break
        sleep 0.2
    done
    kill -0 "$RECORDER_PID" 2>/dev/null && log "recorder still running — killing it."
}

# ── Pneumatics (run-scoped) ──────────────────────────────────────────────────
# Runs INSIDE the main container: the host python3 has no rospy, so on the host the node
# died on `import rospy`, the cup services were never advertised, and every pick timed out
# on them while the arm ran the full motion with no suction. The ClearCore is reachable
# from in there (--privileged, -v /dev:/dev); ORIO_VACUUM_PORT overrides /dev/ttyACM0.
# Raw sensor readings go to VACUUM_LOG_CONT when a run sets it (see run_log_paths).
pneumatics_start() {
    local -a envs=(-e ORIO_LOGGING="${ORIO_LOGGING:-1}" -e ORIO_VACUUM_PORT="${ORIO_VACUUM_PORT:-/dev/ttyACM0}")
    [ -n "${VACUUM_LOG_CONT:-}" ] && envs+=(-e ORIO_VACUUM_LOG="$VACUUM_LOG_CONT")
    run_bg pneumatics docker exec "${envs[@]}" "$CONTAINER" bash -c \
        "source /home/ros_ws/devel/setup.bash && cd /home/ros_ws/src/devel_packages/orio && \
         python3 -u pneumatic_control_recovery.py"
}

# With the vacuum on, the cup services must exist BEFORE any arm moves. Without this the
# arm runs the whole motion with the cup off, timing out on /orio/<cup>/on mid-pick; it
# looks like a hardware problem but is a node that never started. Arguments: cup names.
pneumatics_wait() {
    local cup
    for cup in "$@"; do
        wait_for_service "/orio/$cup/on" 30 && continue
        echo
        log "VACUUM NOT AVAILABLE: /orio/$cup/on never appeared — the pneumatics node is not"
        log "running. Check the [pneumatics] logs above. Common causes:"
        log "  - ClearCore not plugged in / on another port: ls -l /dev/ttyACM* and set"
        log "    ORIO_VACUUM_PORT (default /dev/ttyACM0)."
        log "  - The ClearCore lost its firmware (pneumatic_control/pneumatic_control.ino)"
        log "    so it never enumerates as a serial device."
        log "  - An import error in pneumatic_control_recovery.py (it needs the container's"
        log "    ROS python, not the host's)."
        log "Aborting before any motion: moving with no suction moves the arm for nothing."
        log "Re-run without the vacuum for a dry run."
        return 1
    done
}

vacuum_off() {  # best effort; arguments: cup names
    local cup calls=""
    for cup in "$@"; do calls+="rosservice call /orio/$cup/off 2>/dev/null; "; done
    docker exec "$CONTAINER" bash -lc "source /opt/ros/noetic/setup.bash 2>/dev/null; $calls" >/dev/null 2>&1 || true
}

# ── Per-run paths and setup ──────────────────────────────────────────────────
# One run's files under logging/<name>/: the merged log, the grasp panels, the raw vacuum
# readings. $REPO/logging is /home/ros_ws/logging in the containers, hence the *_CONT
# paths. The panel directory is reached through the fixed link current.affordance: the
# planner outlives a run and reads its directory once, at start-up, so each run points
# the link at its own directory. The link is relative, so it resolves in the container too.
run_log_paths() {  # $1 = directory name under logging/
    LOG_DIR="$REPO/logging/$1"
    mkdir -p "$LOG_DIR"
    RUN_STAMP="$(date +%Y%m%d_%H%M%S)"
    LOG_FILE="$LOG_DIR/$RUN_STAMP.log"
    AFFORDANCE_DIR="$LOG_DIR/$RUN_STAMP.affordance"
    AFFORDANCE_LINK="$LOG_DIR/current.affordance"
    AFFORDANCE_LINK_CONT="/home/ros_ws/logging/$1/current.affordance"
    VACUUM_LOG="$LOG_DIR/$RUN_STAMP.vacuum.csv"
    VACUUM_LOG_CONT="/home/ros_ws/logging/$1/$RUN_STAMP.vacuum.csv"
}

# The per-grasp diagnostic panel, drawn by the planner in its container (the dense score
# maps only exist there). ON when AFFORDANCE=1 and the operator asked for logging
# (LOGGING_REQUESTED, captured before the recorder's venv check could zero ORIO_LOGGING:
# the panels need nothing from the host venv). Sets PANELS and ORIO_AFFORDANCE_DIR.
affordance_panels_setup() {
    PANELS=0
    if [ "${AFFORDANCE:-1}" -eq 1 ] && [ "${LOGGING_REQUESTED:-1}" != "0" ]; then
        PANELS=1
        mkdir -p "$AFFORDANCE_DIR"
        ln -sfn "${AFFORDANCE_DIR##*/}" "$AFFORDANCE_LINK"
        export ORIO_AFFORDANCE_DIR="$AFFORDANCE_LINK_CONT"
        log "grasp panels → $AFFORDANCE_DIR"
    else
        export ORIO_AFFORDANCE_DIR=""
    fi
}

unlock_robot() {  # $1 = robot number; open the brakes via Desk (the arm moves slightly)
    log "unlocking robot $1 brakes (arm moves slightly)…"
    python3 "$BRINGUP_DIR/lock_arms.py" --unlock "$1" 2>&1 | prefix desk
    [ "${PIPESTATUS[0]}" -eq 0 ] || { log "unlock failed — see the [desk] lines above. Aborting."; return 1; }
}

# Run a command in the main container with the workspace sourced, from the orio package
# directory. preflight_check prefixes its output with [check] and returns its status;
# run_in_container is the FOREGROUND run with a TTY, so the script's prompts own the
# keyboard.
preflight_check() {  # $1 = command, e.g. "python3 full_sequence.py --check-only"
    docker exec "$CONTAINER" bash -c \
        "source /home/ros_ws/devel/setup.bash && cd /home/ros_ws/src/devel_packages/orio && $1" 2>&1 | prefix check
    return "${PIPESTATUS[0]}"
}
run_in_container() {  # $1 = command
    docker exec -e ORIO_LOGGING="${ORIO_LOGGING:-1}" -it "$CONTAINER" bash -c \
        "source /home/ros_ws/devel/setup.bash && cd /home/ros_ws/src/devel_packages/orio && $1"
}

# Stop a run's processes inside the main container, which stays up: killing a `docker
# exec` client does not stop what it started. $1 = pkill pattern, rest = cups to switch
# off first. Returns pkill's status (0 = something was stopped).
stop_run_in_container() {
    local pat="$1"; shift
    container_up "$CONTAINER" || return 1
    [ $# -gt 0 ] && vacuum_off "$@"
    docker exec "$CONTAINER" pkill -f "$pat" >/dev/null 2>&1
}

# ── Teardown plumbing ────────────────────────────────────────────────────────
# A launcher defines cleanup() and calls install_cleanup_traps. cleanup runs once: INT
# then EXIT (or HUP, the terminal window closed) both fire, and an impatient second
# Ctrl-C must not abort a teardown half way.
_CLEANED=0
_cleanup_once() {
    [ "$_CLEANED" -eq 1 ] && return
    _CLEANED=1
    trap '' INT TERM HUP
    cleanup
}
install_cleanup_traps() {
    trap '_cleanup_once; exit 130' INT
    trap '_cleanup_once; exit 143' TERM
    trap '_cleanup_once; exit 129' HUP
    trap _cleanup_once EXIT
}
