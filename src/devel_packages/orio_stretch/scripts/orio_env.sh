#!/usr/bin/env bash
# orio_env.sh - common environment + checks for the orio_stretch scripts (Stretch RE1: recording, ORB-SLAM3
# mapping, camera-only navigation, RFID passes).
# Source it:  source "$(dirname "$0")/orio_env.sh"      (or from an interactive shell, to use roslaunch/rosrun directly)
#
# CODE lives in this package; DATA (recordings, maps, logs, per-site and per-boot settings) lives outside the repo:
#   $ORIO_DIR      this package (scripts/, launch/, config/nav/, rviz/)
#   $ORIO_DATA     data root, default ~/orio (override by exporting ORIO_DATA before the call)
#   $DATASETS_DIR  $ORIO_DATA/datasets   one folder per recording / run
#   $MAPS_DIR      $ORIO_DATA/maps       orbslam/<map>.osa + <map>.live.yaml
#   $LOG_DIR       $ORIO_DATA/maps/logs  one log per launch
#   $SITE_DIR      $ORIO_DATA/config     waypoints_<map>.yaml, truth_<site>.txt, payload.env, data_pose.env
#   $RFID_DIR      ../orio_rfid/RFID     reader tools (rfid_log.py, rfid_pose_join.py, tag_registry.csv)
#   $ORBSLAM_WS    ORB-SLAM3 catkin workspace, default ~/orbslam_ws
# Also provides: log/warn/die, preflight_hardware, require_no_master, require_master, wait_for_node,
#                wait_for_topic, wait_for_action, check_runstop, wait_on_launch, orio_pkg_path

ORIO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export ORIO_DATA="${ORIO_DATA:-$HOME/orio}"
DATASETS_DIR="$ORIO_DATA/datasets"
MAPS_DIR="$ORIO_DATA/maps"
LOG_DIR="$ORIO_DATA/maps/logs"
SITE_DIR="$ORIO_DATA/config"
RFID_DIR="$(cd "$ORIO_DIR/../orio_rfid/RFID" 2>/dev/null && pwd)"
ORBSLAM_WS="${ORBSLAM_WS:-$HOME/orbslam_ws}"
mkdir -p "$DATASETS_DIR" "$MAPS_DIR/orbslam" "$LOG_DIR" "$SITE_DIR"

# The setup files read the positional parameters. Sourced from a script they would get the SCRIPT's own
# arguments (`--help` made them print their usage instead of setting the environment): hide them for a moment.
# They also read variables that are not set yet in a fresh shell (ROS_DISTRO, ...). A terminal has them from
# ~/.bashrc; a script started over ssh or by demo_agent.py does not, and under the callers' `set -u` the first such
# read ended the script ("ROS_DISTRO: unbound variable", 2026-11-13): switch -u off while they are sourced.
_orio_args=("$@"); set --
case $- in *u*) _orio_u=true; set +u;; *) _orio_u=false;; esac
# shellcheck disable=SC1091
source /opt/ros/noetic/setup.bash
# shellcheck disable=SC1091
source "$HOME/catkin_ws/devel/setup.bash"
$_orio_u && set -u; unset _orio_u
set -- "${_orio_args[@]}"; unset _orio_args

# stretch_body needs HELLO_FLEET_PATH and HELLO_FLEET_ID. ~/.bashrc exports them, but only for interactive shells
# (it returns early otherwise): take the two export lines from there when they are missing.
if [ -z "${HELLO_FLEET_PATH:-}" ] || [ -z "${HELLO_FLEET_ID:-}" ]; then
  eval "$(grep -E '^[[:space:]]*export HELLO_FLEET_(PATH|ID)=' "$HOME/.bashrc" 2>/dev/null)"
  [ -n "${HELLO_FLEET_PATH:-}" ] && [ -n "${HELLO_FLEET_ID:-}" ] || echo "[orio] WARNING: HELLO_FLEET_PATH / HELLO_FLEET_ID are not set and not in ~/.bashrc: stretch_body will not start" >&2
fi

# Make `roslaunch orio_stretch ...` / `rosrun orio_stretch ...` resolve to THIS checkout without a symlink in a
# catkin workspace. Only this package is added: orio-main also holds an unrelated package called "orio".
# Call it again after sourcing another workspace (catkin's setup files rebuild ROS_PACKAGE_PATH from scratch).
orio_pkg_path() {
  case ":${ROS_PACKAGE_PATH:-}:" in *":$ORIO_DIR:"*) ;; *) export ROS_PACKAGE_PATH="$ORIO_DIR:${ROS_PACKAGE_PATH:-}";; esac
}
orio_pkg_path
export DISPLAY="${DISPLAY:-:0}"

log()  { echo "[$(basename "${0:-orio}")] $*"; }
warn() { echo "[$(basename "${0:-orio}")] WARNING: $*" >&2; }
die()  { echo "[$(basename "${0:-orio}")] ERROR: $*" >&2; exit 1; }

# Any process holding the robot's serial ports or the camera makes stretch_driver exit silently
# and the RealSense fail with "Device or resource busy" (typically ~/teleop_server's autostart).
preflight_hardware() {
  local busy="" pid cmd devs
  for p in /proc/[0-9]*; do
    pid="${p#/proc/}"
    [ "$pid" = "$$" ] && continue
    cmd="$(tr '\0' ' ' < "$p/cmdline" 2>/dev/null | cut -c1-80)"
    [ -z "$cmd" ] && continue
    devs="$(ls -l "$p/fd" 2>/dev/null | grep -oE '/dev/(ttyACM[0-9]+|video[0-9]+|hello-[a-z-]+)' | sort -u | tr '\n' ' ')"
    [ -n "$devs" ] && busy="$busy
   pid $pid: $cmd -> $devs"
  done
  if [ -n "$busy" ]; then
    echo "[orio] ERROR: other processes own the robot hardware, the driver/camera cannot start:$busy" >&2
    echo "[orio]   If it is a stale ROS stack: scripts/ros_clean.sh" >&2
    echo "[orio]   If it is ~/teleop_server (remote teleop tunnel): pkill -f robot_server.py; pkill -f camera_server.py" >&2
    echo "[orio]   (set ORIO_SKIP_PREFLIGHT=true to ignore)" >&2
    [ "${ORIO_SKIP_PREFLIGHT:-false}" = "true" ] || exit 1
  fi
}

require_no_master() {
  if rostopic list >/dev/null 2>&1; then
    die "a ROS master is already running (pgrep -af 'rosmaster|roslaunch'). Stop it first: scripts/ros_clean.sh"
  fi
}
require_master() { rostopic list >/dev/null 2>&1 || die "no ROS master running"; }

# wait_for_node /name [timeout_s]
wait_for_node() {
  local n="$1" t="${2:-60}" i=0 nodes
  while :; do
    nodes="$(rosnode list 2>/dev/null)"
    grep -qx "$n" <<< "$nodes" && return 0
    sleep 1; i=$((i+1)); [ "$i" -ge "$t" ] && return 1
  done
}
# wait_for_topic /topic [timeout_s]   (waits for a publisher AND one message)
wait_for_topic() {
  local tp="$1" t="${2:-60}"
  timeout "$t" rostopic echo -n1 "$tp" >/dev/null 2>&1
}
# wait_for_action /ns [timeout_s]
wait_for_action() {
  local a="$1" t="${2:-60}" i=0 info
  while :; do
    info="$(rostopic info "$a/status" 2>/dev/null)"
    # "Publishers:" followed by " * /node ..." means the action server is up
    grep -A1 "^Publishers:" <<< "$info" | grep -q "^ \* /" && return 0
    sleep 1; i=$((i+1)); [ "$i" -ge "$t" ] && return 1
  done
}

check_runstop() {
  local v
  v="$(timeout 5 rostopic echo -n1 /is_runstopped 2>/dev/null | grep -oE 'True|False' | head -1)"
  if [ "$v" = "True" ]; then
    warn "RUNSTOP IS ENGAGED - the base will not move. Release it (glowing button) or: rosservice call /runstop \"data: false\""
    return 1
  fi
  return 0
}

# forward Ctrl-C to a background roslaunch and wait for it to finish cleanly
wait_on_launch() {
  local pid="$1"
  trap 'echo; log "stopping (Ctrl-C) ..."; kill -INT '"$pid"' 2>/dev/null' INT TERM
  wait "$pid"
  trap - INT TERM
}
