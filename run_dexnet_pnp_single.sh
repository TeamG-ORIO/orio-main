#!/usr/bin/env bash
# Single-terminal DexNet pick-and-place bring-up (no tmux).
#
# Runs the same stack as `launch_demo.sh --session orio_dexnet_pnp`, but instead of
# tmux panes it starts every service in the background and merges their output into THIS
# terminal, each line prefixed with a [module] label. The pick loop (dexnet_pnp.py) runs
# in the FOREGROUND so its --confirm "Press Enter to pick" prompt owns the keyboard.
#
# Ctrl-C stops everything: background services are killed and the containers removed.
#
#   bash run_dexnet_pnp_single.sh                 # dry-run (no vacuum), confirm each pick
#                                                 #   (Enter = execute, r = regenerate grasp, Ctrl-C = abort)
#   bash run_dexnet_pnp_single.sh --vacuum        # start pneumatics (real suction)
#   bash run_dexnet_pnp_single.sh --auto          # no per-pick Enter (auto-pick)
#   bash run_dexnet_pnp_single.sh --straight-down # ignore grasp tilt, approach vertically
#
# The iam-doc control PC (robot 1) is started over ssh; it opens its own gnome-terminal
# windows separately (they run for the robot and cannot be merged here). Logging defaults
# off (the rerun lib is broken on the perception image's py38); set ORIO_LOGGING=1 to try.
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
DOCKER_DIR="$REPO/src/devel_packages/orio_bringup/docker"
WF="$REPO/src/devel_packages/orio_bringup/tmux/wait_for.sh"
# shellcheck disable=SC1090
source "$WF"

export ORIO_LOGGING="${ORIO_LOGGING:-0}"   # rerun broken on perception py38; default off

# ── Args ────────────────────────────────────────────────────────────────────
USE_VACUUM=0
CONFIRM=1
PNP_EXTRA=()
for arg in "$@"; do
    case "$arg" in
        --vacuum)        USE_VACUUM=1 ;;
        --no-vacuum)     USE_VACUUM=0 ;;
        --auto)          CONFIRM=0 ;;
        --confirm)       CONFIRM=1 ;;
        --straight-down) PNP_EXTRA+=(--straight-down) ;;
        -h|--help) sed -n '2,20p' "$0"; exit 0 ;;
        *) echo "unknown option: $arg" >&2; exit 2 ;;
    esac
done
[ "$USE_VACUUM" -eq 0 ] && PNP_EXTRA+=(--no-vacuum)
[ "$CONFIRM" -eq 1 ]    && PNP_EXTRA+=(--confirm)

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
# iam-doc's franka-interface + ROS action server go to their OWN file, raw and unfiltered:
# it's a different machine over ssh, outlives this launcher, and when diagnosing a crash
# you want every line, not the merged stream's SHOW/SAVE/DROP triage. Paired by timestamp.
CONTROL_PC_LOG="$LOG_DIR/$RUN_STAMP.control_pc.log"

# Route the whole script through the classifier. awk writes SHOW lines to stdout (fd 1 =
# terminal, still live) and SHOW+SAVE lines to the log file (via >> in awk). The pick
# loop's interactive `docker exec -it` reads stdin from the terminal, unaffected by this
# stdout pipe, so the --confirm Enter prompt still works.
exec > >(
    LOG_FILE="$LOG_FILE" awk '
    function strip(s){ gsub(/\033\[[0-9;]*m/, "", s); return s }
    {
        line = strip($0)
        show = 0; save = 0

        # SHOW: signal + any error, regardless of source.
        if (line ~ /\[launcher\]|\[check\]|\[doc\]|\[Pick\]|\[Place\]|\[Loop\]|\[Grasp\]|\[Main\]|\[Hardware\]|Press Enter/) { show=1 }
        if (line ~ /GAVE UP|ERROR|Error|error|Traceback|has died|disconnected|CAMERA NOT DETECTED|FAIL|Rejected|refus|not ready|NOT ready|Aborting/) { show=1 }
        if (line ~ /Planned suction grasp|grasp q=/) { show=1 }

        # DROP: pure noise — never shown, never saved.
        else if (line ~ /tensorflow|cuda_gpu|dso_loader|NUMA node|StreamExecutor|gpu_device|libcu[a-z]+\.so|XLA service|Building (convolutional|fully|Softmax)|Converting fc layer|Advertised on topic|ikpy|is of type .fixed.|Detection max range|warnings\.warn/) { next }

        # Everything else: SAVE to file only (debug context, not live).
        if (show) { save=1 }
        else { save=1; show=0 }

        if (save && length(line)) { print line >> ENVIRON["LOG_FILE"]; fflush(ENVIRON["LOG_FILE"]) }
        if (show && length(line)) { print line; fflush() }
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

# ── Teardown ─────────────────────────────────────────────────────────────────
_CLEANED=0
cleanup() {
    [ "$_CLEANED" -eq 1 ] && return   # run once: INT then EXIT (or HUP) both fire
    _CLEANED=1
    # Ignore further stop signals so an impatient second Ctrl-C can't abort teardown
    # (the remote ssh kill can take a few seconds).
    trap '' INT TERM HUP
    echo
    log "shutting down… (Ctrl-C again won't interrupt this)"
    # Stop the vacuum first if it was running (best effort).
    if [ "$USE_VACUUM" -eq 1 ]; then
        docker exec "$DEXNET_CONTAINER" bash -lc \
          "source /opt/ros/noetic/setup.bash 2>/dev/null; \
           rosservice call /orio/pnp_cup/off 2>/dev/null" >/dev/null 2>&1 || true
    fi
    for pid in "${PIDS[@]:-}"; do kill "$pid" 2>/dev/null || true; done
    # Stop the control-PC processes ON iam-doc. Killing the local ssh client (the PIDs
    # above) does NOT stop the remote roslaunch/franka_interface — they keep running on
    # iam-doc and orphan. So reach in over a fresh, short ssh and stop them there.
    #
    # GRACEFULLY. franka-interface keeps its robot state in boost interprocess shared
    # memory (/dev/shm) guarded by a mutex, and holds an FCI connection to the robot.
    # SIGKILL (-9) cannot be caught, so it can leave: a LOCKED, owner-less mutex (the next
    # franka-interface then hangs forever at "Will try to acquire lock while setting
    # franka_interface status"), a half-written shared buffer (next run dies on a
    # std::bad_cast), and the arm stuck in a mode that rejects commands ("Set Joint
    # Impedance command rejected: command not possible in the current mode"). All three
    # were observed. So: SIGTERM, give it time to unwind, and only then SIGKILL stragglers,
    # then clear any shared-memory segment left behind.
    log "stopping control-PC processes on $CTRL_PC_HOST…"
    timeout 35 ssh -o BatchMode=yes -o ConnectTimeout=8 "$CTRL_PC_USER@$CTRL_PC_HOST" \
        'bash -s' <<'REMOTE_TEARDOWN' 2>/dev/null | prefix doc || true
# See the matching note in the pre-run cleanup: "franka_interface" is 16 chars and the
# kernel truncates `comm` to 15, so `pkill -x franka_interface` and
# `ps -eo comm | grep -x franka_interface` NEVER match. Match the full command line.
fi_count() { pgrep -f '[f]ranka_interface --robot_ip' | wc -l; }

# 1) SIGTERM — franka-interface releases its shared-memory lock and closes the FCI
#    connection on a catchable signal. SIGKILL cannot be caught, and leaves a LOCKED,
#    owner-less mutex (next start hangs at "Will try to acquire lock while setting
#    franka_interface status"), a half-written buffer (std::bad_cast), and the arm in a
#    mode that rejects commands. All three were observed.
pkill -TERM -f '[r]oslaunch franka_ros_interface' 2>/dev/null
pkill -TERM -f '[f]ranka_interface --robot_ip' 2>/dev/null

# 2) Give it a real chance to unwind (up to 10s).
for _ in $(seq 1 20); do
    [ "$(fi_count)" -eq 0 ] && break
    sleep 0.5
done

# 3) Force only what refused to exit.
if [ "$(fi_count)" -ne 0 ]; then
    echo "franka_interface did not exit on SIGTERM — forcing."
    pkill -KILL -f '[f]ranka_interface --robot_ip' 2>/dev/null
    sleep 1
fi
pkill -KILL -f '[r]oslaunch franka_ros_interface' 2>/dev/null
pkill -KILL -f '[f]ranka_ros_interface' 2>/dev/null

# 4) Remove the shared memory ONLY if nothing is still running. franka-interface never
#    removes these itself (verified: they survive even a clean SIGTERM), but deleting them
#    while a process still holds them corrupts the next start rather than helping.
if [ "$(fi_count)" -eq 0 ]; then
    rm -f /dev/shm/run_loop_* /dev/shm/current_robot_state* \
          /dev/shm/franka_interface_state_info_mutex 2>/dev/null
else
    echo "WARNING: franka_interface STILL running after SIGKILL — left /dev/shm intact."
fi
REMOTE_TEARDOWN
    log "removing containers…"
    docker rm -f orio_roscore "$CONTAINER" "$DEXNET_CONTAINER" orio_perception orio_cameras \
        >/dev/null 2>&1 || true
    log "done. Full log saved to: $LOG_FILE"
    [ -s "$CONTROL_PC_LOG" ] && log "control PC (iam-doc) log: $CONTROL_PC_LOG"
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

# ── Pre-flight cleanup of any stale containers ───────────────────────────────
log "clearing any stale orio containers…"
docker rm -f orio_roscore "$CONTAINER" "$DEXNET_CONTAINER" orio_perception orio_cameras \
    >/dev/null 2>&1 || true

# ── Bring-up (readiness-gated, background, merged logs) ───────────────────────
log "starting roscore…"
run_bg roscore bash "$DOCKER_DIR/run_roscore.sh"
wait_for_roscore 30 || { log "roscore did not start in 30s — check the [roscore] logs. Aborting."; exit 1; }

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

# Clear stale control-PC state BEFORE starting franka-interface. The teardown below does
# this too, but a previous run that crashed, was SIGKILLed, or had its terminal closed
# never ran its teardown — so its shared memory is still sitting there. franka-interface
# does not remove these on exit (verified: they survive even a clean SIGTERM), and a
# segment left with a LOCKED mutex makes the next start hang at
# "Will try to acquire lock while setting franka_interface status". Start from clean.
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

log "starting robot-1 control PC ($CTRL_PC_HOST) over ssh — logging to $CONTROL_PC_LOG"
# We do NOT use start_control_pc.sh here: it launches franka-interface + the ROS action
# server in gnome-terminals ON iam-doc, whose output we can't capture — so when the
# control PC crashes (libfranka reflex, comms violation, FCI drop) the reason is lost.
# Instead we ssh in ourselves and tee the raw combined stdout/stderr to CONTROL_PC_LOG.
# `ssh -tt` gives a pty so franka-interface flushes; the remote heredoc mirrors the
# reference start_franka_interface_on_control_pc.sh / start_franka_ros_interface scripts.
{
    echo "===== control PC bring-up @ $(date -Is) — $CTRL_PC_USER@$CTRL_PC_HOST ====="
    echo "===== franka-interface | robot_ip=$CTRL_PC_ROBOT_IP ====="
} >>"$CONTROL_PC_LOG"

# (1) franka-interface — the realtime C++ controller. Its stderr carries the crash cause.
( ssh -tt -o BatchMode=yes -o ConnectTimeout=10 "$CTRL_PC_USER@$CTRL_PC_HOST" \
      "mkdir -p ~/franka_logs && ls -t ~/franka_logs/*.log 2>/dev/null | tail -n +51 | xargs -r rm -f; \
       cd '$CTRL_PC_FI_PATH/build' && \
       stdbuf -oL -eL ./franka_interface --robot_ip '$CTRL_PC_ROBOT_IP' --with_gripper 0 --log 0 --stop_on_error 0 \
         2>&1 | tee ~/franka_logs/franka_interface_\$(date +%Y%m%d_%H%M%S).log" \
      2>&1 | ts_prefix >>"$CONTROL_PC_LOG" ) &
PIDS+=($!)

# (2) ROS action server — bridges franka-interface to ROS; publishes robot_state.
( ssh -tt -o BatchMode=yes -o ConnectTimeout=10 "$CTRL_PC_USER@$CTRL_PC_HOST" \
      "cd '$CTRL_PC_FI_PATH' && source bash_scripts/set_rosmaster.sh '$CTRL_PC_HOST' '$WORKSTATION_HOST' && source catkin_ws/devel/setup.bash && roslaunch franka_ros_interface franka_ros_interface.launch robot_num:=$CTRL_PC_ROBOT_NUM" \
      2>&1 | ts_prefix >>"$CONTROL_PC_LOG" ) &
PIDS+=($!)

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
( tail -n0 -F "$CONTROL_PC_LOG" 2>/dev/null \
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

log "control PC: franka-interface + action server launched over ssh (detail in ${CONTROL_PC_LOG##*/})"
sleep 3

log "starting main docker container…"
# ORIO_NO_TTY=1: run detached, no -it (we background it through a pipe = no TTY).
run_bg docker bash -lc "cd '$REPO' && ORIO_NO_TTY=1 ORIO_LOGGING=$ORIO_LOGGING bash orio_run_docker.sh"
wait_for_container "$CONTAINER" 60 || { log "main container '$CONTAINER' did not start in 60s — check the [docker] logs. Aborting."; exit 1; }

# ── Pre-flight: is the robot ready? ──────────────────────────────────────────
# Check BEFORE the heavy services (dexnet ~25s TF load, cameras, perception) so a
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
    log "ROBOT NOT READY — aborting before starting cameras/dexnet/perception."
    log "Unlock/activate robot 1, e.g.:"
    log "    python3 src/devel_packages/orio_bringup/lock_arms.py --unlock 1"
    log "then re-run this script."
    exit 1
fi
log "robot is ready."

log "starting DexNet planner…"
run_bg dexnet bash -lc "ORIO_LOGGING=$ORIO_LOGGING bash '$DOCKER_DIR/run_dexnet.sh' && docker logs -f $DEXNET_CONTAINER"
if ! wait_for_service /dexnet_grasp_planner/plan_grasp 60; then
    log "DexNet planner did not come up in 60s — check the [dexnet] logs above. Aborting."
    exit 1
fi

# Cameras — started one at a time (ZED, then Xtion) so their init doesn't interleave, with
# a retry around the Xtion, which intermittently fails to open its IR stream and drops off
# the bus. NOTE: USB autosuspend and bus-bandwidth contention were both investigated and
# ruled out (the Xtion shows suspended_time=0 and is the only high-speed device on its bus);
# the remaining suspicion is the cable/power/camera itself. The retry is the mitigation.
# (See docs/TROUBLESHOOTING.md.)

# (1) ZED first.
log "starting ZED camera…"
docker exec "$CONTAINER" bash -c "pkill -f 'zed_only.launch|cameras.launch'" >/dev/null 2>&1 || true
run_bg cameras docker exec "$CONTAINER" bash -c \
    "source /home/ros_ws/devel/setup.bash && roslaunch manipulation zed_only.launch"
if ! wait_for_topic_publishing /zedm/zed_node/rgb/image_rect_color 40; then
    echo
    log "CAMERA NOT PUBLISHING: /zedm/zed_node/rgb/image_rect_color (ZED). Check the ZED"
    log "is connected and the [cameras] logs above. Aborting."
    exit 1
fi
log "ZED up. Letting the USB bus settle before starting the Xtion…"
sleep 2   # let ZED init traffic on Bus 001 quiesce before the Xtion claims bandwidth

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

CAM_ATTEMPTS="${ORIO_CAM_ATTEMPTS:-3}"
cam_ok=0
for attempt in $(seq 1 "$CAM_ATTEMPTS"); do
    log "starting Xtion (attempt $attempt/$CAM_ATTEMPTS)…"
    docker exec "$CONTAINER" bash -c "pkill -f xtion_only.launch" >/dev/null 2>&1 || true

    # Tee this attempt's output to a scratch file so we can watch it for failure lines,
    # while still merging it into the terminal/log as [cameras] like before.
    xtion_out="$(mktemp -t orio_xtion.XXXXXX)"
    ( docker exec "$CONTAINER" bash -c \
        "source /home/ros_ws/devel/setup.bash && roslaunch manipulation xtion_only.launch" 2>&1 \
        | tee "$xtion_out" | prefix cameras ) &
    xtion_job=$!
    PIDS+=($xtion_job)

    # Race: publishing (success) vs. a driver failure line (fail fast) vs. backstop timeout.
    xtion_deadline=$(( SECONDS + XTION_WAIT ))
    while :; do
        if timeout 5 docker exec "$CONTAINER" bash -c \
             "source /opt/ros/noetic/setup.bash 2>/dev/null; source /home/ros_ws/devel/setup.bash 2>/dev/null; \
              timeout 3 rostopic echo -n 1 /camera/rgb/image_raw" \
             >/dev/null 2>&1; then
            cam_ok=1; break
        fi
        if xtion_failed "$xtion_out"; then
            log "Xtion reported a device error on attempt $attempt — retrying immediately…"
            break
        fi
        if [ "$SECONDS" -ge "$xtion_deadline" ]; then
            log "Xtion did not publish within ${XTION_WAIT}s on attempt $attempt — retrying…"
            break
        fi
        sleep 0.5
    done
    rm -f "$xtion_out"
    [ "$cam_ok" -eq 1 ] && { echo "[wait] topic '/camera/rgb/image_raw' is publishing."; break; }
    # Failed attempt: tear down this attempt's pipeline so retries don't stack up.
    kill "$xtion_job" 2>/dev/null || true
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

log "starting perception (dexnet backend)…"
run_bg perception bash -lc "ORIO_NO_TTY=1 ORIO_LOGGING=$ORIO_LOGGING bash '$DOCKER_DIR/run_perception.sh' dexnet"

if [ "$USE_VACUUM" -eq 1 ]; then
    log "starting pneumatics…"
    run_bg pneumatics bash -lc "cd '$REPO/src/devel_packages/orio' && ORIO_LOGGING=$ORIO_LOGGING python3 pneumatic_control_recovery.py"
else
    log "pneumatics DISABLED (dry-run): move the cup by hand."
fi

if ! wait_for_service /compute_grasps 90; then
    log "/compute_grasps did not come up in 90s — check the [perception] logs above"
    log "(model load, backend=dexnet, camera frames). Aborting."
    exit 1
fi

# ── Pick loop in the FOREGROUND so --confirm can read the keyboard ────────────
log "starting pick-and-place loop (Ctrl-C OR closing this terminal stops everything)…"
echo "[launcher] pnp flags: ${PNP_EXTRA[*]}"
docker exec -e ORIO_LOGGING="$ORIO_LOGGING" -it "$CONTAINER" bash -c \
    "source /home/ros_ws/devel/setup.bash && cd /home/ros_ws/src/devel_packages/orio && \
     python3 dexnet_pnp.py ${PNP_EXTRA[*]}"

log "pick loop exited."
