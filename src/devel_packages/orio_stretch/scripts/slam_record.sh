#!/usr/bin/env bash
# slam_record.sh - record ONE dataset that every offline SLAM test can run on (one terminal, one command).
#
#   scripts/slam_record.sh <run_name> [options]
#
# You drive with the orio keyboard teleop in THIS terminal (head keys locked). The bag holds everything
# record.launch records (colour, depth, IMU, lidar, wheel odometry, TF) PLUS the two global-shutter IR
# cameras and the per-frame camera metadata (real exposure, gain, projector state).
#
#   --emitter alternate|on|off   IR projector (default alternate):
#                  alternate = toggles every frame: half the frames are clean IR (for IR stereo), the other
#                              half have projector depth (for RGB-D / RTAB-Map). ONE run serves all tests.
#                  off       = clean IR in every frame, depth is passive stereo only (weak on blank walls)
#                  on        = normal depth in every frame, IR images are full of dots (bad for IR stereo)
#   --fps N        camera frame rate, 30 (default) or 15. With "alternate" each kind of frame comes at N/2.
#   --color-exposure auto|N   colour sensor: auto (default) or manual, units of 100 us (50 = 5 ms)
#   --color-gain G            0..128, only with a manual colour exposure
#   --ir-exposure auto|N      IR sensor: auto (default) or manual, MICROSECONDS (4000 = 4 ms)
#   --ir-gain G               16..248, only with a manual IR exposure
#   --pose saved|keep|stow    saved = the pose saved by scripts/drive.sh (default when one is saved);
#                             keep = only the head is set (pan 0, tilt -0.35), lift/arm/wrist are not moved;
#                             stow = prepare_pose.py defaults (lift 0.2 m, arm in, wrist yaw 3.4)
#
# Reference points: while driving, the number keys 1..9 stop the base and record "the robot is standing on
# taped floor point N" in the bag (/slam_test/mark). scripts/marks_report.py <run> then lists where every
# method thinks those points are, and compares against tape-measured distances. See docs/SLAM_TESTS.md.
#
# One run = one camera setting. To compare exposures, record the same route once per setting, e.g.
#   scripts/slam_record.sh lab_auto
#   scripts/slam_record.sh lab_short --color-exposure 50 --color-gain 128 --ir-exposure 3000 --ir-gain 100
# then run ORB-SLAM3 offline on each and compare the two summaries:
#   scripts/orbslam_run.sh lab_auto all ; scripts/orbslam_run.sh lab_short all
#
# What it does: preflight -> record.launch (drivers + rosbag) -> wait until the bag is open -> pose ->
# set + log the camera settings -> teleop -> stop the stack so the bag closes -> check the frame rates.
# If the runstop is engaged it waits for you to release it (hold the head button ~2 s).
# Ctrl-C ONCE ends the teleop, then the recording shuts down cleanly.  Output: ~/orio/datasets/<run_name>/
set -uo pipefail
source "$(dirname "$0")/orio_env.sh"

RUN="${1:-}"
case "$RUN" in ""|-h|--help) sed -n '2,36p' "$0"; exit 1;; esac
shift
POSE_FILE="$SITE_DIR/data_pose.env"
EMITTER=alternate; FPS=30; CEXP=auto; CGAIN=""; IEXP=auto; IGAIN=""; POSE_MODE=""
while [ $# -gt 0 ]; do
  case "$1" in
    --emitter) EMITTER="$2"; shift;;
    --fps) FPS="$2"; shift;;
    --color-exposure) CEXP="$2"; shift;;
    --color-gain) CGAIN="$2"; shift;;
    --ir-exposure) IEXP="$2"; shift;;
    --ir-gain) IGAIN="$2"; shift;;
    --pose) POSE_MODE="$2"; shift;;
    *) die "unknown option $1";;
  esac; shift
done
case "$EMITTER" in alternate|on|off) ;; *) die "--emitter must be alternate, on or off";; esac
case "$FPS" in 15|30) ;; *) die "--fps must be 15 or 30";; esac
if [ -z "$POSE_MODE" ]; then [ -s "$POSE_FILE" ] && POSE_MODE=saved || POSE_MODE=keep; fi
case "$POSE_MODE" in keep|stow) ;; saved) [ -s "$POSE_FILE" ] || die "no saved pose yet - set one with scripts/drive.sh";; *) die "--pose must be saved, keep or stow";; esac
HEAD_ARGS=(head_tilt:=-0.35)
if [ "$POSE_MODE" = saved ]; then
  # shellcheck disable=SC1090
  source "$POSE_FILE"
  HEAD_ARGS=(head_pan:="$POSE_HEAD_PAN" head_tilt:="$POSE_HEAD_TILT")
fi
CAM_ARGS=(--emitter "$EMITTER" --color-exposure "$CEXP" --ir-exposure "$IEXP")
[ -n "$CGAIN" ] && CAM_ARGS+=(--color-gain "$CGAIN")
[ -n "$IGAIN" ] && CAM_ARGS+=(--ir-gain "$IGAIN")

RUN_DIR="$DATASETS_DIR/$RUN"
[ -e "$RUN_DIR" ] && die "$RUN_DIR already exists - pick a new run name"
require_no_master
preflight_hardware

TS="$(date +%Y%m%d_%H%M%S)"; LOG="$LOG_DIR/slam_record_${RUN}_$TS.log"
log "launching drivers + rosbag (log: $LOG)"
roslaunch orio_stretch record.launch datasets_dir:="$DATASETS_DIR" teleop:=false infra:=true fps:="$FPS" "${HEAD_ARGS[@]}" run_name:="$RUN" \
  extra_topics:="/slam_test/config /slam_test/mark" > "$LOG" 2>&1 &
LPID=$!
stop_stack() { kill -INT "$LPID" 2>/dev/null; wait "$LPID" 2>/dev/null; }
# nothing useful was recorded yet: stop, remove the run folder (so the name can be reused) and exit
abort_run() {
  stop_stack
  case "$RUN_DIR" in "$DATASETS_DIR"/?*) rm -rf "$RUN_DIR";; esac
  die "$1  (nothing recorded; see $LOG)"
}

log "waiting for the bag to open (camera reset + head move, up to 2 min) ..."
i=0
until ls "$RUN_DIR"/run_*.bag.active >/dev/null 2>&1; do
  kill -0 "$LPID" 2>/dev/null || die "record.launch exited early - see $LOG"
  sleep 1; i=$((i+1)); [ "$i" -ge 120 ] && { stop_stack; die "the bag never opened - see $LOG"; }
done
wait_for_topic /camera/color/image_raw 30 || abort_run "no colour images - camera busy?"
wait_for_topic /camera/infra1/image_rect_raw 30 || abort_run "no IR images"
wait_for_topic /odom 30 || abort_run "no /odom - stretch_driver did not come up"
# The runstop latches (it survives a restart of the stack): with it engaged the base ignores the
# teleop and the pose goals are rejected. Wait for the release instead of failing.
runstop_engaged() { timeout 5 rostopic echo -n1 /is_runstopped 2>/dev/null | grep -q True; }
if runstop_engaged; then
  warn "RUNSTOP IS ENGAGED. Hold the button on the robot's head for ~2 s until it beeps. Waiting (up to 3 min) ..."
  i=0
  while runstop_engaged; do
    sleep 1; i=$((i+1)); [ "$i" -ge 90 ] && abort_run "the runstop was never released"
  done
  log "runstop released."
  sleep 2      # the driver needs a moment to switch back to navigation mode
fi

# Body pose (three modes: saved / keep / stow).
if [ "$POSE_MODE" = saved ]; then
  log "moving to your saved data-collection pose (lift $POSE_LIFT, arm $POSE_ARM, wrist yaw $POSE_WRIST_YAW, head $POSE_HEAD_PAN / $POSE_HEAD_TILT) ..."
  POSE="$(python3 "$ORIO_DIR/scripts/prepare_pose.py" --split-lift --no-gripper --lift-effort "${POSE_LIFT_EFFORT:-0}" --lift "$POSE_LIFT" --arm "$POSE_ARM" --wrist-yaw "$POSE_WRIST_YAW" \
            --pan "$POSE_HEAD_PAN" --tilt "$POSE_HEAD_TILT" 2>&1 | grep '^POSE')"
  awk -v L="$POSE_LIFT" -v A="$POSE_ARM" -v W="$POSE_WRIST_YAW" -v P="$POSE_HEAD_PAN" -v T="$POSE_HEAD_TILT" '
       function near(a, b, tol) { return (a - b < tol && b - a < tol) }
       /^POSE/ { for (i = 2; i <= NF; i++) { split($i, kv, "="); v[kv[1]] = kv[2] }
                 ok = (near(v["lift"], L, 0.03) && near(v["arm"], A, 0.03) && near(v["wrist_yaw"], W, 0.12) &&
                       near(v["head_pan"], P, 0.06) && near(v["head_tilt"], T, 0.06)) }
       END { exit ok ? 0 : 1 }' <<< "$POSE" || abort_run "the robot did not reach your saved pose (${POSE:-unknown}). Runstop? Something in the way?"
elif [ "$POSE_MODE" = stow ]; then
  log "stowing the arm (lift 0.2 m, arm retracted, wrist yaw 3.4) and pointing the head ..."
  POSE="$(python3 "$ORIO_DIR/scripts/prepare_pose.py" 2>&1 | grep '^POSE')"
  [ -n "$POSE" ] || abort_run "could not stow the arm / read the joint states"
else
  # head only (again, in case the runstop blocked the head move at launch); the arm stays where it is
  POSE="$(python3 "$ORIO_DIR/scripts/prepare_pose.py" --no-arm 2>&1 | grep '^POSE')"
  [ -n "$POSE" ] || abort_run "could not set the head / read the joint states"
fi
log "pose: $POSE"
echo "body_pose ($POSE_MODE):  $POSE" >> "$RUN_DIR/meta.txt"

log "camera: projector $EMITTER, colour exposure $CEXP ${CGAIN:+gain $CGAIN}, IR exposure $IEXP ${IGAIN:+gain $IGAIN}"
CAMCFG="$(python3 "$ORIO_DIR/scripts/cam_setup.py" "${CAM_ARGS[@]}" 2>>"$LOG" | grep '^CAMCFG ')" || true
case "$CAMCFG" in
  *'"ok": true'*) echo "camera_config:    ${CAMCFG#CAMCFG }" >> "$RUN_DIR/meta.txt";;
  *) abort_run "the camera did not accept the requested settings (${CAMCFG:-no answer from cam_setup.py})";;
esac

cat <<MSG

  RECORDING $RUN  (${FPS} fps, projector $EMITTER). Drive with the keys below.
  For a useful SLAM test: start on a floor mark, drive a loop, come back to the mark.
  Taped reference points: stop ON the point, wait a second, press its number (1..9). Mark the start point
  when you leave it and again when you are back on it.
  Press Ctrl-C ONCE (or capital Q) when you are done: the teleop stops the base, then the bag is closed.
MSG
sleep 1
trap 'true' INT      # Ctrl-C goes to the teleop only; this script still closes the bag
python3 "$ORIO_DIR/scripts/stretch_twist_arm_teleop.py" _lock_head:=true _mark_points:=true _cmd_vel_topic:=/stretch/cmd_vel \
  _start_linear:=0.10 _start_angular:=0.30 _max_linear:=0.30 _max_angular:=0.80 || true
trap - INT
echo
log "stopping the stack (closing the bag) ..."
stop_stack

INFO="$(ls "$RUN_DIR"/run_*.info.txt 2>/dev/null | head -1)"
if [ -z "$INFO" ]; then
  warn "no .info.txt in $RUN_DIR - the bag may not have closed cleanly (rosbag reindex)"
  exit 1
fi
grep -E "^duration|^size" "$INFO"
# achieved frame rates: a loaded NUC drops frames silently, and a dark room makes the colour sensor slow down
DUR="$(grep -oE '\(([0-9]+)s\)|^duration: +[0-9.]+s' "$INFO" | grep -oE '[0-9.]+' | head -1)"
for tp in /camera/color/image_raw /camera/aligned_depth_to_color/image_raw /camera/infra1/image_rect_raw /camera/infra2/image_rect_raw /camera/imu /odom; do
  n="$(awk -v t="$tp" '{ for (i = 1; i < NF; i++) if ($i == t) { print $(i+1); exit } }' "$INFO")"
  awk -v t="$tp" -v n="${n:-0}" -v d="${DUR:-0}" -v fps="$FPS" 'BEGIN {
        hz = (d > 0) ? n / d : 0
        note = ""
        if (t ~ /image/ && hz < 0.9 * fps) note = "   <-- below the requested " fps " fps (frames dropped, or the colour sensor slowed down for a dark scene)"
        printf "  %-45s %7d msgs  %6.1f Hz%s\n", t, n, hz, note }'
done
log "done: $RUN_DIR"
grep -q '/slam_test/mark' "$INFO" && log "reference points marked:  scripts/marks_report.py $RUN"
log "next:  scripts/site_build.sh $RUN <map_name>      or      scripts/orbslam_run.sh $RUN all"
