#!/usr/bin/env bash
# nav_orbslam_start.sh - navigation with the IR camera (ORB-SLAM3) as the localiser.
#
#   scripts/nav_orbslam_start.sh <map> [--run-dir DIR] [--no-rviz] [--waypoints FILE [--pause]]
#
#   <map>        saved atlas: maps/orbslam/<map>.osa (orbslam_run.sh --save-map) + <map>.live.yaml
#   --run-dir    the orbslam run folder the map came from (default datasets/basement_map/orbslam/ir-rgbd_lap1):
#                its frame_upright.json and dense_2d_orbslam.yaml define the map frame and the 2D map
#   --waypoints  after localisation, drive this waypoint file with scripts/nav_waypoints.py (--pause = Enter before each)
#   --start-at N the robot is parked on waypoint N of that file, facing as when N was marked: navigation starts at once
#                on wheel odometry from there, ORB-SLAM3 takes over when it recognises the map (no teleop needed)
#   --no-drive   bring everything up and localise/seed, but do not drive the waypoints (rfid_nav_run.sh drives them)
#   --lift M     lift height for the run (default: the saved data-collection pose). Use the height the map was
#                recorded with, e.g. 0.60 for a map recorded by ~/orio/scripts/rfid_pass.sh, 0.90 for one by slam_record.sh
#
# preflight -> nav_orbslam.launch (driver, lidar, camera, ORB-SLAM3 + bridge, map, move_base, RViz) -> camera to the
# recording settings (projector alternating) -> pose (saved data-collection pose) -> wait until ORB-SLAM3 has
# recognised the saved map (drive a little or turn the robot if it takes long) -> waypoints, or RViz "2D Nav Goal".
# Ctrl-C stops everything.
set -uo pipefail
source "$(dirname "$0")/orio_env.sh"
# shellcheck disable=SC1091
source "$ORBSLAM_WS/devel/setup.bash" --extend; orio_pkg_path
MAP="${1:-}"; case "$MAP" in ""|-h|--help) sed -n '2,15p' "$0"; exit 1;; esac; shift
RUN_DIR="$DATASETS_DIR/basement_map/orbslam/ir-rgbd_lap1"; RVIZ=true; WP=""; PAUSE=""; START_AT=""; LIFT_OVERRIDE=""; NO_DRIVE=false
while [ $# -gt 0 ]; do
  case "$1" in
    --run-dir) RUN_DIR="$2"; shift;;
    --no-rviz) RVIZ=false;;
    --waypoints) WP="$2"; shift;;
    --pause) PAUSE="--pause";;
    --start-at) START_AT="$2"; shift;;
    --lift) LIFT_OVERRIDE="$2"; shift;;
    --no-drive) NO_DRIVE=true;;
    *) die "unknown option $1";;
  esac; shift
done
RUN_DIR="$(readlink -f "$RUN_DIR")"      # nodes start from ~/.ros: the path must be absolute
INITIAL=""
if [ -n "$START_AT" ]; then
  [ -n "$WP" ] || die "--start-at needs --waypoints (the point's position comes from that file)"
  INITIAL="$(python3 -c '
import sys, yaml
for w in yaml.safe_load(open(sys.argv[1]))["waypoints"]:
    if str(w["name"]) == sys.argv[2]: print("%.3f %.3f %.3f" % (w["x"], w["y"], w.get("yaw", 0.0)))' "$WP" "$START_AT")"
  [ -n "$INITIAL" ] || die "no waypoint named '$START_AT' in $WP"
fi
for f in "$MAPS_DIR/orbslam/$MAP.osa" "$MAPS_DIR/orbslam/$MAP.live.yaml" "$RUN_DIR/frame_upright.json" "$RUN_DIR/dense_2d_orbslam.yaml"; do
  [ -f "$f" ] || die "missing $f"
done
require_no_master
preflight_hardware
TS="$(date +%Y%m%d_%H%M%S)"; LOG="$LOG_DIR/nav_orbslam_$TS.log"
log "launching (log: $LOG)"
roslaunch orio_stretch nav_orbslam.launch map:="$MAP" maps_dir:="$MAPS_DIR/orbslam" run_dir:="$RUN_DIR" rviz:="$RVIZ" initial_pose:="$INITIAL" > "$LOG" 2>&1 &
LPID=$!
stop_stack() { kill -INT "$LPID" 2>/dev/null; wait "$LPID" 2>/dev/null; }
trap 'echo; log "stopping ..."; stop_stack; exit 130' INT TERM
wait_for_action /stretch_controller/follow_joint_trajectory 90 || { stop_stack; die "stretch_driver did not come up (see $LOG)"; }
wait_for_topic /camera/infra1/image_rect_raw 60 || { stop_stack; die "no IR images (see $LOG)"; }
wait_for_topic /odom 30 || { stop_stack; die "no /odom"; }
log "camera: projector alternating (same as the recording the map was built from)"
python3 "$ORIO_DIR/scripts/cam_setup.py" --emitter alternate 2>>"$LOG" | grep -q '"ok": true' || warn "camera settings not confirmed (see $LOG)"
POSE_FILE="$SITE_DIR/data_pose.env"
if [ -s "$POSE_FILE" ]; then
  # shellcheck disable=SC1090
  source "$POSE_FILE"
  [ -n "$LIFT_OVERRIDE" ] && POSE_LIFT="$LIFT_OVERRIDE"
  log "moving to the data-collection pose (lift $POSE_LIFT, head tilt $POSE_HEAD_TILT) ..."
  python3 "$ORIO_DIR/scripts/prepare_pose.py" --split-lift --no-gripper --lift-effort "${POSE_LIFT_EFFORT:-0}" --lift "$POSE_LIFT" --arm "$POSE_ARM" \
    --wrist-yaw "$POSE_WRIST_YAW" --pan "$POSE_HEAD_PAN" --tilt "$POSE_HEAD_TILT" 2>&1 | grep '^POSE' || warn "pose not confirmed"
fi
if [ -n "$INITIAL" ]; then
  log "start pose given (waypoint $START_AT: $INITIAL): driving on wheel odometry from there until ORB-SLAM3 recognises the map"
  i=0
  until timeout 3 rostopic echo -n1 /orbslam/seeded 2>/dev/null | grep -q True; do
    kill -0 "$LPID" 2>/dev/null || die "the launch died (see $LOG)"
    sleep 1; i=$((i+1)); [ "$i" -ge 60 ] && die "the bridge never seeded the pose (see $LOG)"
  done
  log "SEEDED. Watch for 'localised in the saved map (visual fix)' in $LOG as it drives."
else
log "waiting for ORB-SLAM3 to recognise the saved map (up to 3 min; it helps to drive slowly along a mapped path) ..."
i=0
until timeout 3 rostopic echo -n1 /orbslam/localized 2>/dev/null | grep -q True; do
  kill -0 "$LPID" 2>/dev/null || die "the launch died (see $LOG)"
  sleep 2; i=$((i+2)); [ "$i" -ge 180 ] && { warn "not localised after 3 min - drive around the mapped area; waiting on"; i=0; }
done
log "LOCALISED. map -> odom is published by orbslam_tf_bridge. Use RViz '2D Nav Goal' or the waypoint runner."
fi
if [ -n "$WP" ] && ! $NO_DRIVE; then
  python3 "$ORIO_DIR/scripts/nav_waypoints.py" "$WP" $PAUSE || true
fi
log "stack running; Ctrl-C to stop"
wait "$LPID"
