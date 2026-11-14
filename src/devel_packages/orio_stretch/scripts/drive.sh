#!/usr/bin/env bash
# drive.sh - drive the robot with the orio keyboard teleop. No recording, no camera, no lidar.
# At start it puts the robot into the stow pose of scripts/prepare_pose.py:
# arm retracted, wrist yaw 3.4 (stowed), head pan 0 / tilt -0.35 - but with the lift UP at 0.9 m (payload
# on the gripper), and the lift moves first. The gripper is not commanded.
#
#   scripts/drive.sh [--no-pose] [--lift M] [--lift-effort N] [--hold]
#
#   --no-pose         do not move the arm / lift / head at start (use next to furniture)
#   --lift M          lift height of the pose in metres (default 0.9)
#   --lift-effort N   contact threshold (effort %, default 75; 0 = factory) for moving the lift UP. Needed with a payload on
#                     the gripper: the factory threshold (~47 %) stops the lift at once ("contact detected").
#                     Only upward lift steps use it. Make sure nothing is above the arm.
#   --hold            no keyboard teleop: bring the driver up, print MANUAL READY and wait until stopped
#                     (SIGINT / SIGTERM). demo_agent.py uses this for the GUI's manual driving; the pose is not saved.
#
# Use the arm keys (i/k lift, l/j arm, o/u wrist yaw, ' / ; head tilt, . / , head pan) to put the robot
# into your data-collection pose. When you quit, the pose the robot is in is saved to
# $ORIO_DATA/config/data_pose.env; scripts/slam_record.sh and nav_orbslam_start.sh return to exactly that pose.
#
# What it does: preflight -> stretch_driver only -> waits for you to release the runstop if it is
# engaged -> navigation mode -> mapping pose -> keyboard teleop in THIS terminal (w/s/a/d, SPACE = stop).
# Ctrl-C ONCE ends the teleop and stops the driver.
set -uo pipefail
source "$(dirname "$0")/orio_env.sh"

LIFT_EFFORT=75; SET_POSE=true; LIFT=0.9; HOLD=false
# upward lift contact threshold measured with the payload by scripts/home_payload.py (overrides the 75 above)
# shellcheck disable=SC1091
[ -s "$SITE_DIR/payload.env" ] && source "$SITE_DIR/payload.env"
while [ $# -gt 0 ]; do
  case "$1" in
    --lift-effort) LIFT_EFFORT="$2"; shift;;
    --no-pose) SET_POSE=false;;
    --lift) LIFT="$2"; shift;;
    --hold) HOLD=true;;
    -h|--help) sed -n '2,23p' "$0"; exit 0;;
    *) die "unknown option $1";;
  esac; shift
done

require_no_master
preflight_hardware

TS="$(date +%Y%m%d_%H%M%S)"; LOG="$LOG_DIR/drive_$TS.log"
log "starting stretch_driver (log: $LOG)"
setsid roslaunch stretch_core stretch_driver.launch > "$LOG" 2>&1 &
LPID=$!
stop_stack() { kill -INT "$LPID" 2>/dev/null; wait "$LPID" 2>/dev/null; }
# the driver runs in its own session (so Ctrl-C in the teleop does not kill it first): stop it ourselves
trap 'echo; log "cancelled - stopping the driver ..."; stop_stack; exit 130' INT
# ... also when this script is killed or its terminal is closed, so no orphan driver keeps the motor ports
trap 'stop_stack' EXIT
trap 'exit 143' TERM HUP

wait_for_action /stretch_controller/follow_joint_trajectory 90 || { stop_stack; die "stretch_driver did not come up (see $LOG)"; }

# The runstop latches (it survives a restart): with it engaged the base ignores the teleop.
runstop_engaged() { timeout 5 rostopic echo -n1 /is_runstopped 2>/dev/null | grep -q True; }
if runstop_engaged; then
  warn "RUNSTOP IS ENGAGED. Hold the button on the robot's head for ~2 s until it beeps. Waiting (up to 3 min) ..."
  i=0
  while runstop_engaged; do
    sleep 1; i=$((i+1)); [ "$i" -ge 90 ] && { stop_stack; die "the runstop was never released"; }
  done
  log "runstop released."
  sleep 2
fi

timeout 20 rosservice call /switch_to_navigation_mode >/dev/null 2>&1 || { stop_stack; die "could not switch the driver to navigation mode (see $LOG)"; }
MODE="$(timeout 5 rostopic echo -n1 /mode 2>/dev/null | grep -oE 'navigation|position|runstopped|manipulation' | head -1)"
[ "$MODE" = navigation ] || { stop_stack; die "driver mode is '${MODE:-unknown}', not navigation (see $LOG)"; }
log "driver is in navigation mode."
if $SET_POSE; then
  log "moving to the mapping pose: lift up to $LIFT m FIRST, then arm in + wrist stowed, then head tilt -0.35 ..."
  POSE="$(python3 "$ORIO_DIR/scripts/prepare_pose.py" --split-lift --no-gripper --lift "$LIFT" --lift-effort "$LIFT_EFFORT" 2>&1 | grep '^POSE')"
  [ -n "$POSE" ] && log "$POSE" || warn "prepare_pose failed - check the arm/head manually"
  awk -v L="$LIFT" '/^POSE/ { for (i = 2; i <= NF; i++) { split($i, kv, "="); v[kv[1]] = kv[2] }
                 if (v["wrist_yaw"] < 3.2) print "wrist yaw " v["wrist_yaw"] " (want 3.4)"
                 if (v["arm"] > 0.08) print "arm " v["arm"] " (want 0)"
                 if (v["lift"] < L - 0.05 || v["lift"] > L + 0.05) print "lift " v["lift"] " (want " L ")"
                 if (v["head_tilt"] < -0.41 || v["head_tilt"] > -0.29) print "head tilt " v["head_tilt"] " (want -0.35)" }' <<< "$POSE" |
    while read -r l; do warn "NOT in the mapping pose: $l"; done
fi
log "lift-up contact threshold: $LIFT_EFFORT %"

if $HOLD; then
  log "MANUAL READY: driver up in navigation mode, no keyboard teleop. Stop with: pkill -INT -f 'drive.sh --hold'"
  sleep infinity & wait $!       # the INT trap above stops the driver and exits
  stop_stack; exit 0
fi

trap 'true' INT      # Ctrl-C ends the teleop; this script then stops the driver
python3 "$ORIO_DIR/scripts/stretch_twist_arm_teleop.py" _cmd_vel_topic:=/stretch/cmd_vel \
  _lift_up_effort:="$LIFT_EFFORT" _start_linear:=0.05 _start_angular:=0.20 _max_linear:=0.30 _max_angular:=0.80 || true
trap - INT
echo
POSE_FILE="$SITE_DIR/data_pose.env"
python3 - "$POSE_FILE" "$LIFT_EFFORT" <<'PY' && log "saved the current pose as the data-collection pose: $(tr '\n' ' ' < "$POSE_FILE")" || warn "could not read the joint states - pose NOT saved"
import sys
import rospy
from sensor_msgs.msg import JointState
rospy.init_node('save_pose', anonymous=True)
js = rospy.wait_for_message('/stretch/joint_states', JointState, timeout=5.0)
keys = (('POSE_LIFT', 'joint_lift'), ('POSE_ARM', 'wrist_extension'), ('POSE_WRIST_YAW', 'joint_wrist_yaw'),
        ('POSE_HEAD_PAN', 'joint_head_pan'), ('POSE_HEAD_TILT', 'joint_head_tilt'))
with open(sys.argv[1], 'w') as f:
    for var, joint in keys:
        f.write('%s=%.3f\n' % (var, js.position[js.name.index(joint)]))
    f.write('POSE_LIFT_EFFORT=%s\n' % sys.argv[2])
PY
log "teleop closed; stopping the driver ..."
stop_stack
log "stopped."
