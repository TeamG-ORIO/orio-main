#!/usr/bin/env bash
# orbslam_run.sh - run ORB-SLAM3 OFFLINE on a recorded run and print how well it tracked. No robot needed.
#
#   scripts/orbslam_run.sh <run> [mode|all] [options]          (mode defaults to ir-rgbd)
#
#   <run>   run name under ~/orio/datasets (from scripts/slam_record.sh), a run folder, or a .bag file
#   <mode>  rgbd                colour + depth aligned to colour     (what the robot uses today; rolling shutter)
#           rgbd-inertial       the same + camera IMU
#           ir-stereo           left + right IR camera               (global shutter)
#           ir-stereo-inertial  the same + camera IMU
#           ir-rgbd             left IR + depth                      (global shutter + depth)  <- DEFAULT (best so far)
#           all                 every mode the bag supports, one after the other, then a comparison table
#
#   --rate R          playback speed (default 0.5 = half speed so this NUC keeps up; "frames dropped" in the
#                     summary tells you if it did not)
#   --tag T           results go to <run>/orbslam/<mode>_T instead of <run>/orbslam/<mode> (keep several variants)
#   --set Key=Value   override an ORB-SLAM3 setting, repeatable. e.g. --set ORBextractor.iniThFAST=12
#                     --set ORBextractor.minThFAST=4 --set ORBextractor.nFeatures=2000   (more features in blurry frames)
#   --start S         skip the first S seconds of the bag (default: up to where the camera settings were applied)
#   --duration D      only play D seconds
#   --save-map NAME   save ORB-SLAM3's map to ~/orio/maps/orbslam/NAME.osa at the end
#   --load-map NAME   start from that saved map (same mode as it was saved with). ORB-SLAM3 starts a new map and
#                     MERGES it into the loaded one when it recognises a place; orbslam.log then says "Merge detected"
#   --viewer          open ORB-SLAM3's own viewer window (needs the robot's screen or X forwarding)
#   --video           also write tracking.webm: the camera image with the features ORB-SLAM3 is tracking drawn on it
#   --port P          ROS master port for this run (default 11511); give another one to run two at the same time
#
# Output in <run>/orbslam/<mode>/: summary.txt (the numbers), trajectory.png (top-down path against wheel
# odometry), frames.csv (state of every frame), final.tum (trajectory), settings.yaml (generated from the bag), orbslam.log, feed.log, play.log.
# It uses its own ROS master on port 11511, so it does not disturb (and is not disturbed by) a running robot stack.
set -uo pipefail
case "${1:-}" in ""|-h|--help) sed -n '2,30p' "$0"; exit 1;; esac     # before sourcing ROS: setup.sh would eat "--help"
source "$(dirname "$0")/orio_env.sh"
OWS="$ORBSLAM_WS"
[ -f "$OWS/devel/setup.bash" ] || die "ORB-SLAM3 is not built: $OWS/devel/setup.bash is missing (see docs/SLAM_TESTS.md)"
# shellcheck disable=SC1091
source "$OWS/devel/setup.bash" --extend; orio_pkg_path
export LD_LIBRARY_PATH="$OWS/deps/install/lib:${LD_LIBRARY_PATH:-}"
PORT=11511

RUN="${1:-}"; MODE="${2:-ir-rgbd}"
case "$RUN" in ""|-h|--help) sed -n '2,30p' "$0"; exit 1;; esac
if [[ "$MODE" == --* ]]; then MODE=ir-rgbd; shift 1; else [ $# -ge 2 ] && shift 2 || shift 1; fi   # "orbslam_run.sh <run> --rate 1" = ir-rgbd
ALL_ARGS=("$@")
RATE=0.5; TAG=""; SETS=(); START=""; DURATION=""; SAVE_MAP=""; LOAD_MAP=""; VIEWER=false; VIDEO=false
while [ $# -gt 0 ]; do
  case "$1" in
    --rate) RATE="$2"; shift;;
    --tag) TAG="$2"; shift;;
    --set) SETS+=(--set "$2"); shift;;
    --start) START="$2"; shift;;
    --duration) DURATION="$2"; shift;;
    --save-map) SAVE_MAP="$2"; shift;;
    --load-map) LOAD_MAP="$2"; shift;;
    --viewer) VIEWER=true;;
    --video) VIDEO=true;;
    --port) PORT="$2"; shift;;
    *) die "unknown option $1";;
  esac; shift
done
[[ "$PORT" =~ ^[0-9]+$ ]] || die "--port must be a number"
export ROS_MASTER_URI="http://localhost:$PORT"
S="$ORIO_DIR/scripts"

RUN_DIR="$(python3 -c 'import sys; sys.path.insert(0, sys.argv[1]); import slam_bag; print(slam_bag.resolve_run(sys.argv[2])[0])' "$S" "$RUN")" || exit 1

if [ "$MODE" = all ]; then
  [ -z "$SAVE_MAP$LOAD_MAP" ] || die "--save-map / --load-map belong to one mode, not to 'all'"
  for m in rgbd rgbd-inertial ir-stereo ir-stereo-inertial ir-rgbd; do
    echo; echo "================ $m ================"
    "$0" "$RUN" "$m" "${ALL_ARGS[@]}" || warn "mode $m did not complete"
  done
  echo; echo "================ comparison: $RUN ================"
  grep -h '^RESULT' "$RUN_DIR"/orbslam/*/summary.txt 2>/dev/null | sed 's/^RESULT //'
  exit 0
fi

[[ "${TAG:-x}" =~ ^[A-Za-z0-9_.-]+$ ]] || die "--tag must be a simple word"
OUT="$RUN_DIR/orbslam/$MODE${TAG:+_$TAG}"
ATLAS_DIR="$MAPS_DIR/orbslam"
mkdir -p "$ATLAS_DIR"
for n in "$SAVE_MAP" "$LOAD_MAP"; do [[ "${n:-x}" =~ ^[A-Za-z0-9_.-]+$ ]] || die "map names must be simple words"; done
[ -z "$LOAD_MAP" ] || [ -f "$ATLAS_DIR/$LOAD_MAP.osa" ] || die "no such map: $ATLAS_DIR/$LOAD_MAP.osa"
[ -n "$SAVE_MAP" ] && [ -e "$ATLAS_DIR/$SAVE_MAP.osa" ] && die "$ATLAS_DIR/$SAVE_MAP.osa already exists - pick another name"
if [ -d "$OUT" ]; then case "$OUT" in */orbslam/?*) rm -rf "$OUT";; esac; fi
mkdir -p "$OUT"

# ---- settings from the bag ------------------------------------------------------------------
ATLAS_ARGS=(); [ -n "$SAVE_MAP" ] && ATLAS_ARGS+=(--save-atlas "$SAVE_MAP"); [ -n "$LOAD_MAP" ] && ATLAS_ARGS+=(--load-atlas "$LOAD_MAP")
INFO="$(python3 "$S/orbslam_settings.py" "$RUN" "$MODE" --out "$OUT/settings.yaml" "${SETS[@]}" "${ATLAS_ARGS[@]}")" || exit 1
USER_START="$START"
eval "$INFO"      # NODE IMAGE SECOND ROLE LASER EMITTER INERTIAL START FPS CAM_FRAME BODY_FRAME BAG
[ -n "$USER_START" ] && START="$USER_START"
log "mode $MODE on $BAG"
log "projector during the run: $EMITTER; feeding frames with laser: $LASER; ${FPS} Hz; start at ${START} s; playback rate $RATE"

# ---- private ROS master ------------------------------------------------------------------------
rostopic list >/dev/null 2>&1 && die "something is already using the ROS master on port $PORT (another orbslam_run.sh?)"
PIDS=()
cleanup() {
  trap - INT TERM EXIT
  # the node, the logger and the video writer are not in PIDS (they are stopped one by one on a normal
  # run); on an interrupt they must go too, or they linger and hold the master port
  for p in "${ORB_PID:-}" "${LOG_PID:-}" "${VIDEO_PID:-}"; do [ -n "$p" ] && kill -INT "$p" 2>/dev/null; done
  for p in "${PIDS[@]}"; do kill -INT "$p" 2>/dev/null; done
  for _ in $(seq 1 20); do
    alive=false; for p in "${PIDS[@]}"; do kill -0 "$p" 2>/dev/null && alive=true; done
    $alive || break; sleep 0.5
  done
  for p in "${PIDS[@]}" "${ORB_PID:-}" "${LOG_PID:-}" "${VIDEO_PID:-}"; do [ -n "$p" ] && kill -KILL "$p" 2>/dev/null; done
}
trap 'echo; log "interrupted"; cleanup; exit 130' INT TERM
trap cleanup EXIT
roscore -p "$PORT" > "$OUT/roscore.log" 2>&1 &
PIDS+=($!)
i=0; until rostopic list >/dev/null 2>&1; do sleep 0.5; i=$((i+1)); [ "$i" -ge 40 ] && die "roscore did not start (see $OUT/roscore.log)"; done
rosparam set use_sim_time true

python3 "$S/orb_feed.py" __name:=orb_feed _laser:="$LASER" _image_in:="$IMAGE" _second_in:="$SECOND" > "$OUT/feed.log" 2>&1 &
PIDS+=($!)
python3 "$S/orbslam_log.py" __name:=orbslam_log _out:="$OUT/frames.csv" > "$OUT/log.log" 2>&1 &
LOG_PID=$!
VIDEO_PID=""
if $VIDEO; then
  python3 "$S/orbslam_video.py" __name:=orbslam_video _out:="$OUT/tracking.webm" _fps:="$FPS" > "$OUT/video.log" 2>&1 &
  VIDEO_PID=$!
fi

# ---- the ORB-SLAM3 node (cwd = atlas folder: ORB-SLAM3 saves/loads ./NAME.osa) ----------------
REMAPS=(/camera/rgb/image_raw:=/orb_feed/image /camera/left/image_raw:=/orb_feed/image /imu:=/camera/imu)
if [ "$ROLE" = right ]; then REMAPS+=(/camera/right/image_raw:=/orb_feed/second)
else REMAPS+=(/camera/depth_registered/image_raw:=/orb_feed/second); fi
VOC="$(rospack find orb_slam3_ros)/orb_slam3/Vocabulary/ORBvoc.txt.bin"
( cd "$ATLAS_DIR" && exec rosrun orb_slam3_ros "$NODE" __name:=orb_slam3 \
    _voc_file:="$VOC" _settings_file:="$OUT/settings.yaml" _enable_pangolin:="$VIEWER" \
    _world_frame_id:=orb_world _cam_frame_id:=orb_camera _imu_frame_id:=orb_imu _base_link_frame_id:=orb_none \
    _working_path:="$OUT/" "${REMAPS[@]}" ) > "$OUT/orbslam.log" 2>&1 &
ORB_PID=$!
disown "$ORB_PID"      # the node aborts in a ROS plugin destructor AFTER its clean shutdown; keep bash quiet about it
log "loading the ORB vocabulary ..."
i=0
until rostopic list 2>/dev/null | grep -qx /orb_slam3/tracking_state; do
  kill -0 "$ORB_PID" 2>/dev/null || { tail -5 "$OUT/orbslam.log"; die "the ORB-SLAM3 node died at start (see $OUT/orbslam.log)"; }
  sleep 1; i=$((i+1)); [ "$i" -ge 180 ] && die "the ORB-SLAM3 node never became ready (see $OUT/orbslam.log)"
done
sleep 2

# ---- play only the topics this mode needs ------------------------------------------------------
TOPICS=("$IMAGE" "$SECOND" /camera/infra1/metadata /camera/infra2/metadata /camera/depth/metadata)
[ "$INERTIAL" = 1 ] && TOPICS+=(/camera/imu)
PLAY_ARGS=(--clock -r "$RATE" -s "$START"); [ -n "$DURATION" ] && PLAY_ARGS+=(-u "$DURATION")
log "playing the bag (Ctrl-C aborts) ..."
rosbag play "${PLAY_ARGS[@]}" "$BAG" --topics "${TOPICS[@]}" < /dev/null > "$OUT/play.log" 2>&1 &
PLAY_PID=$!
PIDS+=("$PLAY_PID")
while kill -0 "$PLAY_PID" 2>/dev/null; do
  kill -0 "$ORB_PID" 2>/dev/null || { warn "the ORB-SLAM3 node crashed during the run (see $OUT/orbslam.log)"; break; }
  sleep 2
done
sleep 3      # let the node finish the frames still in its queue

# ---- final trajectory, shutdown (writes the map if asked), numbers ------------------------------
if kill -0 "$ORB_PID" 2>/dev/null; then
  timeout 120 rosservice call /orb_slam3/save_traj "name: '$OUT/final'" > "$OUT/save_traj.log" 2>&1 || warn "could not save the final trajectory (see $OUT/save_traj.log)"
  [ -n "$SAVE_MAP" ] && log "saving the map (can take a minute) ..."
  kill -INT "$ORB_PID" 2>/dev/null
  i=0; while kill -0 "$ORB_PID" 2>/dev/null; do sleep 1; i=$((i+1)); [ "$i" -ge 180 ] && { warn "ORB-SLAM3 did not shut down, killing it"; kill -KILL "$ORB_PID"; }; done
fi
kill -INT "$LOG_PID" 2>/dev/null; wait "$LOG_PID" 2>/dev/null
if [ -n "$VIDEO_PID" ]; then
  kill -INT "$VIDEO_PID" 2>/dev/null; wait "$VIDEO_PID" 2>/dev/null
  [ -s "$OUT/tracking.webm" ] && log "video: $OUT/tracking.webm" || warn "no tracking video was written (see $OUT/video.log)"
fi
[ -n "$SAVE_MAP" ] && { [ -f "$ATLAS_DIR/$SAVE_MAP.osa" ] && log "map saved: $ATLAS_DIR/$SAVE_MAP.osa" || warn "the map was NOT saved (see $OUT/orbslam.log)"; }
[ -n "$LOAD_MAP" ] && log "merges with the loaded map: $(grep -c 'Merge detected' "$OUT/orbslam.log")"
cleanup
grep -h '^NOTE' "$OUT"/*.log 2>/dev/null
echo
python3 "$S/orbslam_eval.py" "$OUT" --bag "$BAG" --mode "$MODE${TAG:+_$TAG}" --frame "$BODY_FRAME"
log "details: $OUT"
