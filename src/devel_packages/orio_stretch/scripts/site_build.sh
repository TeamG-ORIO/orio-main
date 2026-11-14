#!/usr/bin/env bash
# site_build.sh - turn ONE recorded run into everything navigation needs at a new site: the saved ORB-SLAM3 map,
# its live settings, the upright map frame, the 2D floor plan and the waypoint file. Then (optional) a cold-start
# localisation check on the rest of the recording. No robot needed.
#
#   scripts/site_build.sh <run> <map_name> [--mode ir-rgbd] [--map-until S] [--no-check] [--truth FILE]
#
#   <run>        a recording from scripts/slam_record.sh with point 1 marked at the start and again when back on it
#   <map_name>   name of the saved map, e.g. hospital_corridor  -> maps/orbslam/<map_name>.osa / .live.yaml
#   --map-until  seconds into the recording where the mapping part ends (default: 6 s after the second mark of
#                point 1, i.e. just past the loop closure). The rest of the recording is the localisation check.
#
# Output: datasets/<run>/orbslam/<mode>_map/ (frame_upright.json, dense_2d_orbslam.yaml, maps, summary),
#         config/waypoints_<map_name>.yaml, and the nav command to run printed at the end.
set -uo pipefail
source "$(dirname "$0")/orio_env.sh"
RUN="${1:-}"; MAP="${2:-}"
[ -n "$RUN" ] && [ -n "$MAP" ] || { sed -n '2,16p' "$0"; exit 1; }
shift 2
MODE=ir-rgbd; UNTIL=""; CHECK=true; TRUTH=""
while [ $# -gt 0 ]; do
  case "$1" in
    --mode) MODE="$2"; shift;;
    --map-until) UNTIL="$2"; shift;;
    --no-check) CHECK=false;;
    --truth) TRUTH="$2"; shift;;
    *) die "unknown option $1";;
  esac; shift
done
[[ "$MAP" =~ ^[A-Za-z0-9_.-]+$ ]] || die "map name must be a simple word"
S="$ORIO_DIR/scripts"
RUN_DIR="$(python3 -c 'import sys; sys.path.insert(0, sys.argv[1]); import slam_bag; print(slam_bag.resolve_run(sys.argv[2])[0])' "$S" "$RUN")" || exit 1
[ -e "$MAPS_DIR/orbslam/$MAP.osa" ] && die "$MAPS_DIR/orbslam/$MAP.osa already exists - pick another name or delete it"

# ---- where does the mapping part end? (second mark of point 1 = back on the start cross) ----
read -r START MARK1_2 BAGLEN <<< "$(python3 - "$S" "$RUN" <<'PY'
import sys, json, rosbag
sys.path.insert(0, sys.argv[1]); import slam_bag as sb
_, bag_path = sb.resolve_run(sys.argv[2]); bag = rosbag.Bag(bag_path); t0 = bag.get_start_time()
marks = [(int(json.loads(m.data)['point']), t.to_sec() - t0) for _, m, t in bag.read_messages(topics=['/slam_test/mark'])]
ones = [t for p, t in marks if p == 1]
print('%.1f %s %.1f' % (sb.settings_time(bag), ('%.1f' % ones[1]) if len(ones) >= 2 else 'none', bag.get_end_time() - t0))
PY
)"
if [ -z "$UNTIL" ]; then
  [ "$MARK1_2" != none ] || die "point 1 was marked only once: cannot find the end of the mapping lap. Pass --map-until <seconds>"
  UNTIL="$(python3 -c "print(round($MARK1_2 + 6.0, 1))")"
fi
DUR="$(python3 -c "print(round($UNTIL - $START, 1))")"
log "run $RUN: settings applied at ${START}s, mapping until ${UNTIL}s (of ${BAGLEN}s), mode $MODE, map '$MAP'"

# ---- 1. ORB-SLAM3 on the mapping part, save the atlas ----
"$S/orbslam_run.sh" "$RUN" "$MODE" --tag map --rate 1 --start "$START" --duration "$DUR" --save-map "$MAP" || die "mapping run failed"
OUT="$RUN_DIR/orbslam/${MODE}_map"
[ -f "$MAPS_DIR/orbslam/$MAP.osa" ] || die "the map was not saved (see $OUT/orbslam.log)"
log "loop closures in the mapping part: $(grep -c 'Loop detected' "$OUT/orbslam.log")"

# ---- 2. frame, 2D floor plan, live settings, waypoints ----
log "scoring the taped points and writing the map frame ..."
python3 "$S/marks_report.py" "$RUN" --orbslam "${MODE}_map" ${TRUTH:+--truth "$TRUTH"} | grep -v Warning | sed -n '/revisits/,$p'
log "drawing the 2D floor plan (a few minutes) ..."
python3 "$S/orbslam_dense.py" "$RUN" "${MODE}_map" 2>&1 | grep -E 'depth frames;|rror'
python3 "$S/orbslam_settings.py" "$RUN" "$MODE" --out "$MAPS_DIR/orbslam/$MAP.live.yaml" --load-atlas "$MAP" > /dev/null || die "live settings failed"
python3 "$S/nav_points.py" "$SITE_DIR/waypoints_$MAP.yaml" --from-run "$RUN" --orbslam "${MODE}_map"
python3 "$S/orbslam_map.py" "$RUN" "${MODE}_map" > /dev/null 2>&1 || true

# ---- 3. cold-start localisation check on the rest of the recording ----
if $CHECK && python3 -c "import sys; sys.exit(0 if $BAGLEN - $UNTIL > 20 else 1)"; then
  log "cold-start check: fresh ORB-SLAM3 with the saved map on the recording from ${UNTIL}s ..."
  "$S/orbslam_run.sh" "$RUN" "$MODE" --tag cold --rate 1 --start "$UNTIL" --load-map "$MAP" | grep -E 'tracked OK|dropped|new maps'
  echo "merged into the saved map: $(grep -c 'Merge detected' "$RUN_DIR/orbslam/${MODE}_cold/orbslam.log") time(s)"
  python3 "$S/marks_report.py" "$RUN" --orbslam "${MODE}_cold" --reference "$RUN" --reference-orbslam "${MODE}_map" --same-frame 2>&1 | grep -v Warning | sed -n '/against reference/,$p'
else
  $CHECK && warn "less than 20 s of recording after the mapping part: no localisation check"
fi

cat <<MSG

Site '$MAP' is ready.
  saved map:      $MAPS_DIR/orbslam/$MAP.osa  (+ $MAP.live.yaml)
  map frame, 2D:  $OUT/frame_upright.json, dense_2d_orbslam.yaml, dense_2d.png
  waypoints:      $SITE_DIR/waypoints_$MAP.yaml   (the taped points; add more with scripts/nav_points.py)

Navigate (robot on a mapped spot, then drive a little with the teleop until LOCALISED):
  scripts/nav_orbslam_start.sh $MAP --run-dir $OUT --waypoints config/waypoints_$MAP.yaml
MSG
