#!/usr/bin/env bash
# ros_clean.sh - stop every ROS process of a previous mapping/navigation run (NOT ~/teleop_server).
# Run this when a launch says "auto-starting new master" or a script says a master is already running.
PATTERNS="roslaunch|rosmaster|rosout|stretch_driver|realsense2_camera|rplidarNode|laser_filter|scan_to_scan|laser_scan_matcher|rtabmap|rtabmap_viz|move_base|amcl|map_server|nodelet|rviz|teleop_twist|stretch_twist_arm_teleop|prepare_pose|relocalize|d435i_|topic_tools|robot_state_publisher|joint_state_publisher"
echo "[ros_clean] sending SIGINT to: "; pgrep -af "$PATTERNS" | grep -v ros_clean || echo "   (nothing running)"
pkill -INT -f "$PATTERNS" 2>/dev/null
for _ in $(seq 1 15); do pgrep -f "$PATTERNS" | grep -qv "$$" || break; sleep 1; done
if pgrep -f "$PATTERNS" | grep -qv "$$"; then
  echo "[ros_clean] still alive after 15 s, killing:"; pgrep -af "$PATTERNS" | grep -v ros_clean
  pkill -KILL -f "$PATTERNS" 2>/dev/null; sleep 1
fi
pgrep -af "$PATTERNS" | grep -v ros_clean || echo "[ros_clean] clean"
