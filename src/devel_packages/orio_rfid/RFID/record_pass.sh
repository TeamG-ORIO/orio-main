#!/usr/bin/env bash
# record_pass.sh - one corridor pass: rosbag (poses, lift, lidar) + RFID read log, same clock, same folder.
#
#   record_pass.sh <label> [rfid_log.py scan options...]      e.g.  record_pass.sh s1_lift0          (defaults: 33 dBm, mode 0, Dual Target, session 0, reports every 20 reads)
#
# Needs the navigation stack running (scripts/nav_start.sh <map>) so /tf has map->base_link from AMCL; the teleop
# server must be stopped for that (it owns the hardware). Drive with the keyboard/teleop of your choice; Ctrl-C ends
# the pass and prints the read-rate report.
set -u
LABEL="${1:?usage: record_pass.sh <label> [rfid_log.py scan options]}"; shift
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TS="$(date +%Y%m%d_%H%M%S)"; DIR="$HOME/orio/datasets/rfid_${LABEL}_$TS"; mkdir -p "$DIR"
if ! rostopic list >/dev/null 2>&1; then echo "[record_pass] no ROS master: start the navigation stack first (scripts/nav_start.sh <map>)"; exit 1; fi
if rostopic list | grep -q "^/orbslam/pose$"; then echo "[record_pass] localiser: ORB-SLAM3 (orbslam_tf_bridge)"; elif rostopic list | grep -q "^/amcl_pose$"; then echo "[record_pass] localiser: AMCL"; else echo "[record_pass] WARNING: neither /orbslam/pose nor /amcl_pose is published - poses will be odometry only"; fi
cp "$HERE/antenna_transform.yaml" "$DIR/" 2>/dev/null; cp "$HERE/tag_registry.csv" "$DIR/" 2>/dev/null
echo "label=$LABEL date=$TS args=$*" > "$DIR/meta.txt"; rosparam dump "$DIR/params.yaml" 2>/dev/null
echo "[record_pass] recording to $DIR"
rosbag record -O "$DIR/pass.bag" /tf /tf_static /joint_states /stretch/joint_states /amcl_pose /odom /scan /particlecloud /map_metadata /orbslam/pose /orbslam/localized /orb_slam3/camera_pose /orb_slam3/tracking_state > "$DIR/rosbag.log" 2>&1 &
BAG=$!
sleep 2
python3 "$HERE/rfid_log.py" scan "$LABEL" --out-dir "$DIR" "$@"
echo "[record_pass] stopping the bag ..."; kill -INT $BAG; wait $BAG 2>/dev/null
rosbag info "$DIR/pass.bag" > "$DIR/rosbag_info.txt" 2>&1; echo "[record_pass] done: $DIR"
