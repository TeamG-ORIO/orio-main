#!/usr/bin/env bash
#
# record_wrap.sh - launch-prefix wrapper around `rosbag record` for record.launch.
#
# roslaunch runs:  record_wrap.sh /opt/ros/noetic/lib/rosbag/record <rosbag args...>
# Before starting rosbag it:
#   1. creates the run folder  $ORIO_RUN_DIR
#   2. copies the URDF, head calibration and camera preset next to the bag
#   3. dumps the parameter server (again 20 s later, once the camera has loaded)
#   4. writes meta.txt (robot, date, package versions, launch args)
#   5. moves the head to the fixed recording pose (ORIO_HEAD_PAN / ORIO_HEAD_TILT)
# then runs rosbag record as a child, forwards SIGINT/SIGTERM to it, and after it
# closes the bag writes `rosbag info` + a final param dump into the folder.
#
# Environment (set by record.launch via <env>):
#   ORIO_DATASETS_DIR parent folder (required)
#   ORIO_RUN_NAME     run folder name; empty -> run_YYYYMMDD_HHMMSS
#   ORIO_HEAD_PAN     rad, default 0.0
#   ORIO_HEAD_TILT    rad, default -0.30
#   ORIO_SET_HEAD     "true"/"false" - skip the head move if false
#   ORIO_LAUNCH_ARGS  free text, recorded in meta.txt
set -u
DATASETS_DIR="${ORIO_DATASETS_DIR:?ORIO_DATASETS_DIR not set}"
RUN_NAME="${ORIO_RUN_NAME:-}"
[ -z "$RUN_NAME" ] && RUN_NAME="run_$(date +%Y%m%d_%H%M%S)"
RUN_DIR="${DATASETS_DIR%/}/$RUN_NAME"
if [ -e "$RUN_DIR" ]; then
  echo "[record_wrap] ERROR: $RUN_DIR already exists - refusing to record into an existing run folder (pass a new run_name:=)." >&2
  exit 1
fi
# the launch file passes -o @RUN_DIR@/run ; substitute now that the folder is known
set -- "${@//@RUN_DIR@/$RUN_DIR}"
HEAD_PAN="${ORIO_HEAD_PAN:-0.0}"
HEAD_TILT="${ORIO_HEAD_TILT:--0.30}"
SET_HEAD="${ORIO_SET_HEAD:-true}"
ORIO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
STRETCH_ROS="$HOME/catkin_ws/src/stretch_ros"

log() { echo "[record_wrap] $*"; }

# ---- 0. preflight: nothing else may own the D435i or the robot's serial ports --------
# (a stray stretch_body process, or the ~/teleop_server autostart service, silently kills
#  stretch_driver and makes the RealSense fail with "Device or resource busy")
if [ "${ORIO_SKIP_PREFLIGHT:-false}" != "true" ]; then
  BUSY=""
  for p in /proc/[0-9]*; do
    pid="${p#/proc/}"
    [ "$pid" = "$$" ] && continue
    # match on the FULL command line: cut to 70 chars, ".../stretch_core/nodes/stretch_driver" loses its
    # last word and this launch's own driver gets reported as a foreign owner of /dev/ttyACM*
    full="$( { tr '\0' ' ' < "$p/cmdline"; } 2>/dev/null)"
    cmd="${full:0:70}"
    case "$full" in *stretch_driver*|*realsense2_camera*|*nodelet*|*record_wrap*) continue;; esac
    devs="$(ls -l "$p/fd" 2>/dev/null | grep -o -E '/dev/(ttyACM[0-9]+|video[0-9]+)' | sort -u | tr '\n' ' ')"
    [ -n "$devs" ] && BUSY="$BUSY
  pid $pid: $cmd -> $devs"
  done
  if [ -n "$BUSY" ]; then
    echo "[record_wrap] ERROR: other processes own the robot hardware; stretch_driver and the camera cannot start:$BUSY" >&2
    echo "[record_wrap]        Stop them first (e.g. the ~/teleop_server service: pkill -f robot_server.py; pkill -f camera_server.py)," >&2
    echo "[record_wrap]        or set ORIO_SKIP_PREFLIGHT=true to record anyway. See docs/RECORDING.md." >&2
    exit 1
  fi
fi

mkdir -p "$RUN_DIR"
log "run folder: $RUN_DIR"

# ---- 2. static files that are NOT topics ----------------------------------
cp "$STRETCH_ROS/stretch_description/urdf/stretch.urdf"               "$RUN_DIR/stretch.urdf"               2>/dev/null || log "WARN: urdf copy failed"
cp "$STRETCH_ROS/stretch_core/config/controller_calibration_head.yaml" "$RUN_DIR/controller_calibration_head.yaml" 2>/dev/null || log "WARN: head calibration copy failed"
cp "$STRETCH_ROS/stretch_core/config/HighAccuracyPreset.json"          "$RUN_DIR/d435i_preset.json"          2>/dev/null || true
cp "$HOME/stretch_user/$(hostname)/stretch_user_params.yaml"           "$RUN_DIR/stretch_user_params.yaml"           2>/dev/null || true
cp "$HOME/stretch_user/$(hostname)/stretch_configuration_params.yaml"  "$RUN_DIR/stretch_configuration_params.yaml"  2>/dev/null || true
cp "$ORIO_DIR/launch/record.launch"                                    "$RUN_DIR/record.launch.used"          2>/dev/null || true

# ---- 4. meta.txt ---------------------------------------------------------------
{
  echo "robot:            $(hostname)"
  echo "date:             $(date -Is)"
  echo "ros_distro:       ${ROS_DISTRO:-?}"
  echo "stretch_ros git:  $(git -C "$STRETCH_ROS" log -1 --format='%h %cd' 2>/dev/null)"
  echo "stretch_body:     $(pip3 show hello-robot-stretch-body 2>/dev/null | awk '/^Version/{print $2}')"
  echo "realsense2_camera:$(rosversion realsense2_camera 2>/dev/null)"
  echo "librealsense:     $(dpkg-query -W -f='${Version}' librealsense2 2>/dev/null)"
  echo "rtabmap_ros:      $(rosversion rtabmap_slam 2>/dev/null)"
  echo "head_pose_cmd:    pan=$HEAD_PAN tilt=$HEAD_TILT (set_head=$SET_HEAD)"
  echo "launch_args:      ${ORIO_LAUNCH_ARGS:-}"
  echo "rosbag cmd:       $*"
  echo
  echo "# Fill in by hand after the run:"
  echo "start_mark:       (taped floor mark id / description)"
  echo "end_mark:         (did the robot end on the start mark?  yes/no, approx error)"
  echo "notes:            (lighting, people walking, doors opened, anything odd)"
} > "$RUN_DIR/meta.txt"

# ---- 3. param dumps -------------------------------------------------------------
rosparam dump "$RUN_DIR/params_at_start.yaml" 2>/dev/null || log "WARN: rosparam dump failed"
( sleep 20; rosparam dump "$RUN_DIR/params_20s.yaml" 2>/dev/null ) &

# ---- 5. head pose ----------------------------------------------------------------
if [ "$SET_HEAD" = "true" ]; then
  log "moving head to pan=$HEAD_PAN tilt=$HEAD_TILT ..."
  python3 "$ORIO_DIR/scripts/set_head_pose.py" --pan "$HEAD_PAN" --tilt "$HEAD_TILT" --timeout 40 | tee -a "$RUN_DIR/meta.txt"
  if [ "${PIPESTATUS[0]}" -eq 0 ]; then
    log "head set."
  else
    log "WARN: set_head_pose failed (driver not up? runstop engaged?). Recording anyway - note it in meta.txt."
    echo "head_pose_WARNING: set_head_pose FAILED, head pose unknown at start" >> "$RUN_DIR/meta.txt"
  fi
fi

# ---- run rosbag record as a child so we get a post-hook ----------------------
log "starting: $*"
"$@" &
CHILD=$!
forward() { log "signal -> rosbag record ($CHILD)"; kill -INT "$CHILD" 2>/dev/null; }
trap forward INT TERM
wait "$CHILD"; RC=$?
trap - INT TERM
# rosbag may still be flushing the last chunk when wait returns on a trapped signal
while kill -0 "$CHILD" 2>/dev/null; do sleep 0.5; done
log "rosbag record exited rc=$RC"

# ---- post-hook -----------------------------------------------------------------
rosparam dump "$RUN_DIR/params_at_end.yaml" 2>/dev/null || true
shopt -s nullglob
for b in "$RUN_DIR"/*.bag; do
  rosbag info "$b" > "${b%.bag}.info.txt" 2>&1 || true
done
for b in "$RUN_DIR"/*.bag.active; do
  log "WARN: $b was not closed cleanly. Run:  rosbag reindex '$b'"
done
log "done. $(ls "$RUN_DIR" | tr '\n' ' ')"
exit $RC
