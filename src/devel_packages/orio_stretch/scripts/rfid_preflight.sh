#!/usr/bin/env bash
# rfid_preflight.sh - checks before the first RFID pass of a session (about 1 min). Nothing is recorded.
#
#   scripts/rfid_preflight.sh [--lidar-only] [--no-reader] [--no-lidar] [--auto-run]
#
#   1. teleop server stopped, no ROS master, nothing holding the motors / camera
#   2. disk space for the session (c1: 2.8 GB per minute at 30 fps)
#   3. robot: battery voltage, homed, runstop, payload lift threshold, arm retracted
#   4. RFID reader: ping, LLRP port, a 10 s read-rate check (point the panel at a shelf)
#   5. lidar: started alone for ~10 s; how many ranges are finite (c1 was 100% inf)
#   6. files the passes copy: tag registry, antenna transform, phase calibration
# Ends with a list of what to fix. --lidar-only reruns just step 5 (e.g. after clearing things off the turret).
# --no-lidar skips step 5: the camera-only navigation (nav_orbslam.launch, lidar:=false) and rfid_nav_run.sh do not
# use the lidar; only the teleop-driven recording session logs /scan.
# --auto-run checks for the AUTONOMOUS run (rfid_nav_run.sh) instead of the teleop-driven recording session:
# no lidar (as --no-lidar); 5 GB of disk is enough (no camera bag is recorded); an extended arm is a NOTE, because the
# run itself moves to the saved data-collection pose first (homing always leaves the arm 0.10 m out). demo_agent.py passes it.
set -uo pipefail
source "$(dirname "$0")/orio_env.sh"
[ -n "$RFID_DIR" ] || die "the reader tools are missing: $ORIO_DIR/../orio_rfid/RFID (same repo, package orio_rfid)"
SCR="$(mktemp -d /tmp/rfid_preflight.XXXX)"
LIDAR_ONLY=false; READER=true; LIDAR=true; AUTO=false
for a in "$@"; do case "$a" in --lidar-only) LIDAR_ONLY=true;; --no-reader) READER=false;; --no-lidar) LIDAR=false;; --auto-run) AUTO=true; LIDAR=false;; -h|--help) sed -n '2,17p' "$0"; exit 0;; *) die "unknown option $a";; esac; done
PROBLEMS=()
ok()  { echo "  OK    $*"; }
bad() { echo "  FIX   $*"; PROBLEMS+=("$*"); }
note(){ echo "  NOTE  $*"; }

lidar_test() {
  echo "== lidar (alone, ~12 s)"
  if pgrep -f rplidarNode >/dev/null; then note "an rplidarNode is already running - skipped"; return; fi
  local port=11399 out="$SCR/lidar.log"
  ROS_MASTER_URI=http://localhost:$port roscore -p $port > "$SCR/core.log" 2>&1 & local C=$!
  sleep 4
  ROS_MASTER_URI=http://localhost:$port rosrun rplidar_ros rplidarNode _serial_port:=/dev/hello-lrf _serial_baudrate:=115200 \
      _frame_id:=laser _angle_compensate:=true _scan_mode:=Boost > "$out" 2>&1 & local L=$!
  sleep 6
  local res
  res="$(ROS_MASTER_URI=http://localhost:$port timeout 8 python3 - <<'PY' 2>/dev/null
import math, rospy
from sensor_msgs.msg import LaserScan
rospy.init_node('lidar_check', anonymous=True)
fr = []; n = 0; near = 0
for _ in range(5):
    m = rospy.wait_for_message('/scan', LaserScan, timeout=3)
    f = [r for r in m.ranges if math.isfinite(r)]; fr.append(len(f) / max(len(m.ranges), 1)); n = len(m.ranges)
    near += sum(1 for r in f if r < 0.4)
print('%d %.3f %d' % (n, sum(fr) / len(fr), near // 5))
PY
)"
  kill -INT $L 2>/dev/null; wait $L 2>/dev/null; kill -INT $C 2>/dev/null; wait $C 2>/dev/null
  grep -q "Can not start scan" "$out" && note "rplidar: $(grep -o 'Can not start scan[^!]*' "$out" | head -1) (SDK timeout: the head did not reach scanning speed, or it did not answer)"
  if [ -z "$res" ]; then
    bad "lidar: no /scan at all. Turret spinning freely? Anything resting on or rubbing the turret? Weight on the robot's back pressing on it? USB cable at /dev/hello-lrf?"
  else
    read -r n frac near <<< "$res"
    pct=$(awk -v f="$frac" 'BEGIN { printf "%d", 100 * f }')
    if [ "$pct" -ge 30 ]; then ok "lidar: $n ranges per scan, $pct% finite, ~$near returns closer than 0.4 m"
    else bad "lidar: only $pct% of ranges finite ($n per scan). Clear the ring around the turret (min range 0.15 m) and rerun: scripts/rfid_preflight.sh --lidar-only"; fi
  fi
}

if $LIDAR_ONLY; then
  require_no_master; lidar_test; rm -rf "$SCR"
  [ ${#PROBLEMS[@]} -eq 0 ] && echo "lidar OK" || exit 1
  exit 0
fi

echo "== processes"
if pgrep -f "robot_server.py|camera_server.py" >/dev/null; then
  bad "teleop server is running (owns the motors and camera): pkill -f 'bash start_teleop.sh'; pkill -f robot_server.py; pkill -f camera_server.py"
else ok "teleop server stopped"; fi
if rostopic list >/dev/null 2>&1; then bad "a ROS master is running: scripts/ros_clean.sh"; else ok "no ROS master"; fi

echo "== disk"
FREE_GB=$(df -BG --output=avail "$DATASETS_DIR/" | tail -1 | tr -dc 0-9)
# P0..P6 ~ 2 + 10 + 10 + 10 + 12 + 10 + 2 min = 56 min at 2.8 GB/min (30 fps) = 160 GB
if $AUTO; then
  if [ "$FREE_GB" -ge 5 ]; then ok "$FREE_GB GB free (an autonomous run writes well under 1 GB: reads and poses, no camera bag)"
  else bad "$FREE_GB GB free: free some space in $DATASETS_DIR"; fi
elif [ "$FREE_GB" -ge 170 ]; then ok "$FREE_GB GB free (the session needs ~160 GB at 30 fps, ~85 GB at 15 fps)"
elif [ "$FREE_GB" -ge 90 ]; then bad "$FREE_GB GB free: enough only at --fps 15, or copy c1 off and delete it (it is 47 GB)"
else bad "$FREE_GB GB free: not enough for the session"; fi
ls -d "$DATASETS_DIR"/c2_* >/dev/null 2>&1 && note "c2 folders already exist: $(ls -d "$DATASETS_DIR"/c2_* | xargs -n1 basename | tr '\n' ' ')"

echo "== robot"
if pgrep -f "robot_server.py" >/dev/null; then
  note "robot checks skipped (teleop server holds the hardware)"
else
  # with --auto-run an extended arm is fine IF there is a saved data-collection pose for the run to go to
  ARM_OK_POSE=""; $AUTO && ARM_OK_POSE="$(grep -oP '^POSE_ARM=\K.*' "$SITE_DIR/data_pose.env" 2>/dev/null || true)"
  cat > "$SCR/robot.py" <<'PY'
import stretch_body.robot as rb
r = rb.Robot()
if not r.startup():
    print('FIX stretch_body could not start (another process on the serial ports?)'); raise SystemExit
try:
    r.pimu.pull_status(); v = r.pimu.status['voltage']
    print(('OK' if v > 11.8 else 'FIX') + ' battery %.1f V%s' % (v, '' if v > 11.8 else ' - charge before the passes (plug in while measuring / moving items)'))
    print(('OK' if not r.pimu.status['runstop_event'] else 'FIX') + ' runstop ' + ('released' if not r.pimu.status['runstop_event'] else 'ENGAGED - hold the head button ~2 s'))
    homed = r.is_calibrated()
    print(('OK' if homed else 'FIX') + ' robot ' + ('homed' if homed else 'NOT homed: python3 scripts/home_payload.py (NOT stretch_robot_home.py: the gripper payload spoils its lift zero)'))
    r.lift.pull_status(); r.arm.pull_status()
    print('OK lift %.3f m, arm %.3f m' % (r.lift.status['pos'], r.arm.status['pos']))
    if r.arm.status['pos'] > 0.05:
        import os
        if os.environ.get('ARM_OK_POSE'):     # --auto-run: the run retracts the arm itself before it drives
            print('NOTE arm extended %.3f m (homing leaves it there): the run moves to the saved pose first (arm %s m, wrist stowed)' % (r.arm.status['pos'], os.environ['ARM_OK_POSE']))
        else:
            print('FIX arm extended %.3f m: retract it before the passes' % r.arm.status['pos'])
finally:
    r.stop()
PY
  while read -r lvl msg; do [ "$lvl" = OK ] && ok "$msg" || { [ "$lvl" = NOTE ] && note "$msg" || bad "$msg"; }; done \
    < <(ARM_OK_POSE="$ARM_OK_POSE" timeout 40 python3 "$SCR/robot.py" 2>/dev/null | grep -E '^(OK|FIX|NOTE) ' || echo 'FIX stretch_body check failed or timed out')
fi
PENV="$SITE_DIR/payload.env"
if [ -s "$PENV" ] && grep -q '^LIFT_EFFORT=' "$PENV"; then
  BOOT=$(date -d "$(uptime -s)" +%s); PT=$(stat -c %Y "$PENV")
  if [ "$PT" -ge "$BOOT" ]; then ok "payload homing done this boot ($(date -d @"$PT" '+%F %H:%M')), $(grep '^LIFT_EFFORT=' "$PENV")"
  else bad "payload homing is from before the last boot ($(date -d @"$PT" '+%F %H:%M')): python3 scripts/home_payload.py (re-measures the lift effort, re-homes)"; fi
else bad "no LIFT_EFFORT in config/payload.env: python3 scripts/home_payload.py"; fi

if $READER; then
  echo "== RFID reader"
  if ping -c1 -W2 192.168.0.20 >/dev/null 2>&1; then ok "reader answers ping"
    if timeout 3 bash -c '</dev/tcp/192.168.0.20/5084' 2>/dev/null; then ok "LLRP port 5084 open"
      echo "  ... 10 s read check (panel facing a shelf)"
      python3 "$RFID_DIR/rfid_log.py" scan preflight --duration 10 --out-dir "$SCR" > "$SCR/rfid.out" 2>&1
      reads=$(grep -oE '[0-9]+ reads of [0-9]+ tags' "$SCR/rfid.out" | tail -1)
      n=$(awk '{print $1}' <<< "$reads")
      if [ -n "$n" ] && [ "$n" -gt 1500 ]; then ok "RFID: $reads in 10 s"
      elif [ -n "$n" ]; then bad "RFID: only $reads in 10 s (expect ~4000; --report-every 20? panel cable on port 4? panel facing the shelves?)"
      else bad "RFID: the read check failed: $(tail -2 "$SCR/rfid.out" | tr '\n' ' ')"; fi
      grep -q "hop table: 50 channels" "$SCR/rfid.out" && ok "hop table saved (50 channels)" || note "hop table line: $(grep 'hop table' "$SCR/rfid.out" | head -1)"
    else bad "reader pings but LLRP port 5084 is closed (still booting? another client connected, e.g. the web UI?)"; fi
  else bad "reader 192.168.0.20 does not answer: power, ~60 s boot, Ethernet to the robot"; fi
fi

if ! $LIDAR; then note "lidar not checked (--no-lidar: the autonomous run navigates with the camera only)"
elif pgrep -f robot_server.py >/dev/null; then note "lidar test skipped (stop the teleop server first)"; else lidar_test; fi

echo "== files"
[ -s "$RFID_DIR/tag_registry.csv" ] && ok "tag registry: $(($(wc -l < "$RFID_DIR/tag_registry.csv") - 1)) tags" || bad "no tag_registry.csv in $RFID_DIR"
[ -s "$RFID_DIR/antenna_transform.yaml" ] && ok "antenna_transform.yaml present" || bad "no antenna_transform.yaml"
python3 -c "import sllurp" 2>/dev/null && ok "sllurp importable" || bad "python3 cannot import sllurp"
note "clock: $(date '+%F %T %Z'), NTP synced: $(timedatectl show -p NTPSynchronized --value 2>/dev/null || echo ?) (RFID and bag share this clock; only photo times depend on it)"

rm -rf "$SCR"
echo
if [ ${#PROBLEMS[@]} -eq 0 ]; then echo "PREFLIGHT OK - next: scripts/rfid_nav_run.sh <map> --run-dir DIR --waypoints FILE"
else echo "PREFLIGHT: ${#PROBLEMS[@]} thing(s) to fix:"; for p in "${PROBLEMS[@]}"; do echo "  - $p"; done; exit 1; fi
