#!/usr/bin/env bash
# nav_orbslam_selftest.sh - test the live ORB-SLAM3 localisation chain WITHOUT the robot, on a recorded run.
#
#   scripts/nav_orbslam_selftest.sh [run] [start_s] [map] [run_dir] [waypoints] [start_at]
#     run      recorded run to replay (default basement_map), start_s = seconds into it (default 296 = lap 2)
#     map      saved atlas (default basement_lap1), run_dir = its orbslam run folder (frame + 2D map)
#     waypoints  waypoint file of that map (default config/waypoints_basement.yaml)
#     start_at   waypoint name the replay starts on: the bridge is SEEDED there, as nav_orbslam_start.sh --start-at
#
# Also checks the live obstacle layer: at every marked point, how many cells the LOCAL costmap marks lethal on
# floor that is free in the saved 2D map (= false obstacles on open floor; should be ~0).
#
# Replays the bag into exactly the nodes nav_orbslam.launch starts live (orb_feed, ORB-SLAM3 with the saved
# map, orbslam_tf_bridge, map_server, move_base - no hardware), and checks that map -> base_link at every taped
# point matches the waypoint file within 25 cm. Uses its own ROS master (port 11511).
set -uo pipefail
source "$(dirname "$0")/orio_env.sh"
# shellcheck disable=SC1091
source "$ORBSLAM_WS/devel/setup.bash" --extend; orio_pkg_path
RUN="${1:-basement_map}"; START="${2:-296}"; MAP="${3:-basement_lap1}"
RUN_DIR="${4:-$DATASETS_DIR/basement_map/orbslam/ir-rgbd_lap1}"
RUN_DIR="$(readlink -f "$RUN_DIR")"
WP="$(readlink -f "${5:-$SITE_DIR/waypoints_basement.yaml}")"; START_AT="${6:-}"
INITIAL=""
if [ -n "$START_AT" ]; then
  INITIAL="$(python3 -c '
import sys, yaml
for w in yaml.safe_load(open(sys.argv[1]))["waypoints"]:
    if str(w["name"]) == sys.argv[2]: print("%.3f %.3f %.3f" % (w["x"], w["y"], w.get("yaw", 0.0)))' "$WP" "$START_AT")"
  [ -n "$INITIAL" ] || die "no waypoint $START_AT in $WP"
fi
BAG="$(python3 -c 'import sys; sys.path.insert(0, sys.argv[1]); import slam_bag; print(slam_bag.resolve_run(sys.argv[2])[1])' "$ORIO_DIR/scripts" "$RUN")" || exit 1
for f in "$MAPS_DIR/orbslam/$MAP.osa" "$MAPS_DIR/orbslam/$MAP.live.yaml" "$RUN_DIR/frame_upright.json" "$RUN_DIR/dense_2d_orbslam.yaml" "$WP"; do [ -f "$f" ] || die "missing $f"; done
export ROS_MASTER_URI=http://localhost:11511
rostopic list >/dev/null 2>&1 && die "port 11511 is in use (another orbslam_run.sh?)"
OUT="$LOG_DIR/nav_selftest_$(date +%Y%m%d_%H%M%S)"; mkdir -p "$OUT"
PIDS=()
cleanup() { trap - INT TERM EXIT; for p in "${PIDS[@]}"; do kill -INT "$p" 2>/dev/null; done; sleep 3; for p in "${PIDS[@]}"; do kill -KILL "$p" 2>/dev/null; done; }
trap 'cleanup; exit 130' INT TERM
trap cleanup EXIT
roscore -p 11511 > "$OUT/roscore.log" 2>&1 & PIDS+=($!)
until rostopic list >/dev/null 2>&1; do sleep 0.5; done
rosparam set use_sim_time true
roslaunch orio_stretch nav_orbslam.launch map:="$MAP" maps_dir:="$MAPS_DIR/orbslam" run_dir:="$RUN_DIR" hardware:=false rviz:=false initial_pose:="$INITIAL" > "$OUT/launch.log" 2>&1 & PIDS+=($!)
log "waiting for ORB-SLAM3 ..."
until rostopic list 2>/dev/null | grep -qx /orb_slam3/tracking_state; do sleep 1; done; sleep 2
python3 - "$WP" "$OUT/checker.txt" > "$OUT/checker.log" 2>&1 <<'PY' &
import sys, json, math, yaml, rospy, tf2_ros
import numpy as np
from std_msgs.msg import String, Bool
from nav_msgs.msg import OccupancyGrid
rospy.init_node('selftest_checker')
grids = {}
def on_grid(m, key): grids[key] = m
rospy.Subscriber('/map', OccupancyGrid, on_grid, 'static')
rospy.Subscriber('/move_base/local_costmap/costmap', OccupancyGrid, on_grid, 'local')
def robot_xy():
    try:
        t = buf.lookup_transform('map', 'base_link', rospy.Time(0), rospy.Duration(0.5)).transform.translation
        return t.x, t.y
    except Exception:
        return None, None
def false_obstacles():
    """lethal LOCAL-costmap cells (cost 100) that lie on FREE cells of the saved 2D map"""
    if 'static' not in grids or 'local' not in grids:
        return 'no costmap yet'
    st, lo = grids['static'], grids['local']
    sa = np.array(st.data, np.int16).reshape(st.info.height, st.info.width)
    la = np.array(lo.data, np.int16).reshape(lo.info.height, lo.info.width)
    if lo.header.frame_id.strip('/') != 'map':
        try:
            tr = buf.lookup_transform('map', lo.header.frame_id, rospy.Time(0), rospy.Duration(0.5)); dx, dy = tr.transform.translation.x, tr.transform.translation.y
            q = tr.transform.rotation; yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        except Exception:
            return 'no odom->map'
    else:
        dx = dy = yaw = 0.0
    iy, ix = np.nonzero(la == 100)
    if not len(ix):
        return '0 lethal cells'
    xs = lo.info.origin.position.x + (ix + 0.5) * lo.info.resolution; ys = lo.info.origin.position.y + (iy + 0.5) * lo.info.resolution
    mx = dx + math.cos(yaw) * xs - math.sin(yaw) * ys; my = dy + math.sin(yaw) * xs + math.cos(yaw) * ys
    sj = ((mx - st.info.origin.position.x) / st.info.resolution).astype(int); si = ((my - st.info.origin.position.y) / st.info.resolution).astype(int)
    ok = (si >= 0) & (si < sa.shape[0]) & (sj >= 0) & (sj < sa.shape[1])
    free = np.zeros(len(ix), bool); free[ok] = sa[si[ok], sj[ok]] == 0
    on_free = int(free.sum())
    if not on_free:
        return '%d lethal cells, none on free floor' % len(ix)
    import cv2
    occ = (sa == 100).astype(np.uint8)
    dist = cv2.distanceTransform((1 - occ).astype(np.uint8), cv2.DIST_L2, 5) * st.info.resolution    # metres to the nearest mapped obstacle
    dd = dist[si[free], sj[free]]
    rx, ry = robot_xy()
    rd = np.hypot(mx[free] - rx, my[free] - ry) if rx is not None else np.array([np.nan])
    return '%d lethal cells, %d on free floor: distance to nearest mapped obstacle median %.2f m, max %.2f m; %d of them > 0.25 m from any mapped obstacle; nearest to the robot %.2f m' % (
        len(ix), on_free, np.median(dd), dd.max(), int(np.sum(dd > 0.25)), np.nanmin(rd))
wps = {str(w['name']): w for w in yaml.safe_load(open(sys.argv[1]))['waypoints']}
buf = tf2_ros.Buffer(rospy.Duration(60)); tf2_ros.TransformListener(buf)
out = open(sys.argv[2], 'w'); t_loc = [None]; t0 = [None]
def on_loc(m):
    if m.data and t_loc[0] is None:
        t_loc[0] = rospy.Time.now(); out.write('localised at bag time %.1f\n' % t_loc[0].to_sec()); out.flush()
def on_mark(m):
    d = json.loads(m.data); p = str(d['point']); st = rospy.Time.from_sec(d['stamp'])
    rospy.sleep(0.5)
    try:
        tr = buf.lookup_transform('map', 'base_link', st, rospy.Duration(1.0)); t = tr.transform.translation
        w = wps.get(p)
        e = math.hypot(t.x - w['x'], t.y - w['y']) if w else float('nan')
        out.write('point %s: robot at x=%.3f y=%.3f  waypoint x=%.3f y=%.3f  error %.3f m  %s   | local costmap: %s\n' % (p, t.x, t.y, w['x'] if w else float('nan'), w['y'] if w else float('nan'), e, 'ok' if e <= 0.25 else 'FAIL', false_obstacles()))
    except Exception as ex:
        out.write('point %s: no map pose (%s)\n' % (p, str(ex)[:80]))
    out.flush()
rospy.Subscriber('/orbslam/localized', Bool, on_loc); rospy.Subscriber('/slam_test/mark', String, on_mark)
rospy.spin()
PY
PIDS+=($!)
sleep 2
log "playing $RUN from ${START}s (full speed) ..."
rosbag play --clock -r 1 -s "$START" "$BAG" --topics /camera/infra1/image_rect_raw /camera/depth/image_rect_raw /camera/depth/camera_info /camera/infra1/metadata /camera/depth/metadata /tf /tf_static /odom /scan /slam_test/mark < /dev/null > "$OUT/play.log" 2>&1 &
PLAY=$!; PIDS+=($PLAY)
while kill -0 $PLAY 2>/dev/null; do sleep 5; done; sleep 3
echo; echo "=== result ($OUT/checker.txt)"; cat "$OUT/checker.txt" 2>/dev/null || echo "(no checker output)"
echo "move_base: $(grep -c -i 'error\|warn' "$OUT/launch.log") warnings/errors in launch.log; bridge: $(grep -c 'localised' "$OUT/launch.log") localisation messages"
