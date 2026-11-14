#!/usr/bin/env bash
# rfid_nav_run.sh - AUTONOMOUS RFID run on a saved map: the robot drives the waypoints by itself while the RFID
# reader logs, and at the end every read gets the antenna's map position (reads_posed.csv). No teleop.
#
#   scripts/rfid_nav_run.sh <map> --run-dir DIR --waypoints FILE [options]
#
#   <map>          saved ORB-SLAM3 map (maps/orbslam/<map>.osa), e.g. hospital_corridor
#   --run-dir      the orbslam folder the map came from (site_build.sh prints it)
#   --waypoints    waypoint file in that map (config/waypoints_<map>.yaml)
#   --start-at N   robot parked on waypoint N facing as when N was marked (default 1): starts at once, no teleop
#   --laps K       drive the waypoint list K times (default 1)
#   (default)      the lift (arm + RFID panel) moves up and down 0.42 <-> 0.90 m (0.42 = lowest allowed: the payload touches the base at 0.39 m), one up-and-down every 11 s,
#                  the whole time the robot drives: many antenna heights per tag
#   --lift-lo M --lift-hi M --period S   change that range / speed
#   --lift-fixed M hold the lift at M metres instead of moving it
#   --label NAME   output folder datasets/<NAME> (default rfid_auto_<date>)
#   --antennas "4" reader ports;  --rfid "ARGS"  extra rfid_log.py options (e.g. "--mode 2")
#
# Output in datasets/<label>/: <label>_*.csv (raw reads), base_link.tum (live map pose, 20 Hz), lift.csv / odom.csv
# (sidecar), nav.csv + nav_path.csv (arrivals, driven path), reads_posed.csv (every read + antenna map pose),
# nav_events.jsonl (live progress, one line per goal / arrival). Pause the base while it runs:
#   echo pause > datasets/<label>/nav_control     and     echo run > datasets/<label>/nav_control
# (the lift and the reader keep going; demo_agent.py and the demo GUI use the same two files).
# Stop early from another terminal:  pkill -INT -f rfid_nav_run.sh   (everything is stopped and the reads still posed)
set -uo pipefail
source "$(dirname "$0")/orio_env.sh"
[ -n "$RFID_DIR" ] || die "the reader tools are missing: $ORIO_DIR/../orio_rfid/RFID (same repo, package orio_rfid)"
S="$ORIO_DIR/scripts"
MAP="${1:-}"; case "$MAP" in ""|-h|--help) sed -n '2,25p' "$0"; exit 1;; esac; shift
RUN_DIR_MAP=""; WP=""; START_AT=1; LAPS=1; LIFT=0.60; WAVE=true; LO=0.42; HI=0.90; PERIOD=11; LABEL="rfid_auto_$(date +%Y%m%d_%H%M%S)"; ANTS="4"; RFID_EXTRA=""
while [ $# -gt 0 ]; do
  case "$1" in
    --run-dir) RUN_DIR_MAP="$2"; shift;; --waypoints) WP="$2"; shift;; --start-at) START_AT="$2"; shift;;
    --laps) LAPS="$2"; shift;; --lift-fixed) LIFT="$2"; WAVE=false; shift;; --label) LABEL="$2"; shift;;
    --lift-lo) LO="$2"; shift;; --lift-hi) HI="$2"; shift;; --period) PERIOD="$2"; shift;;
    --antennas) ANTS="$2"; shift;; --rfid) RFID_EXTRA="$2"; shift;;
    *) die "unknown option $1";;
  esac; shift
done
[ -n "$RUN_DIR_MAP" ] && [ -n "$WP" ] || die "--run-dir and --waypoints are required (site_build.sh prints both)"
WP="$(readlink -f "$WP")"; RUN_DIR_MAP="$(readlink -f "$RUN_DIR_MAP")"
OUT="$DATASETS_DIR/$LABEL"; [ -e "$OUT" ] && die "$OUT exists - pick another --label"
grep -q '^LIFT_EFFORT=' "$SITE_DIR/payload.env" 2>/dev/null || die "home with the payload first: python3 scripts/home_payload.py"
ping -c1 -W2 192.168.0.20 >/dev/null 2>&1 || die "RFID reader 192.168.0.20 does not answer (power on, ~60 s boot, Ethernet)"
mkdir -p "$OUT"; TS="$(date +%Y%m%d_%H%M%S)"
PIDS="$(mktemp -d /tmp/rfid_nav.XXXX)"
spawn() { local f="$PIDS/$1" o="$2"; shift 2; setsid bash -c 'echo $$ > "$0"; exec "$@"' "$f" "$@" >> "$o" 2>&1 < /dev/null &
          for _ in $(seq 1 50); do [ -s "$f" ] && { cat "$f"; return 0; }; sleep 0.1; done; return 1; }
stop_pid() { [ -n "$1" ] && kill -"$2" "$1" 2>/dev/null && { for _ in $(seq 1 "${3:-20}"); do kill -0 "$1" 2>/dev/null || return 0; sleep 0.5; done; kill -9 "$1" 2>/dev/null; }; return 0; }
P_NAV=""; P_LIFT=""; P_RFID=""; P_SIDE=""; P_POSE=""; P_WP=""
stop_all() {
  stop_pid "$P_WP" INT 10; P_WP=""
  stop_pid "$P_LIFT" TERM 10; P_LIFT=""
  stop_pid "$P_RFID" TERM 20; P_RFID=""
  stop_pid "$P_POSE" TERM 10; P_POSE=""
  stop_pid "$P_SIDE" TERM 10; P_SIDE=""
  stop_pid "$P_NAV" INT 60; P_NAV=""
  # the start script's roslaunch runs in its own session and survived the INT above (2026-11-07): stop it too
  local lp; lp="$(pgrep -f 'roslaunch orio_stretch nav_orbslam.launch' | head -1)"
  if [ -n "$lp" ]; then stop_pid "$lp" INT 60; fi
}
STOPPED=false
on_int() { echo; log "stopping early (the reads recorded so far are kept and posed) ..."; STOPPED=true; stop_pid "$P_WP" INT 10; P_WP=""; }
trap on_int INT TERM
trap 'stop_all; rm -rf "$PIDS"' EXIT

# ---- 1. navigation stack, seeded at the start cross (no teleop) ----
log "starting navigation on '$MAP' (robot on waypoint $START_AT) - log: $OUT/nav_stack.log"
P_NAV=$(spawn nav "$OUT/nav_stack.log" "$S/nav_orbslam_start.sh" "$MAP" --run-dir "$RUN_DIR_MAP" --waypoints "$WP" --start-at "$START_AT" --no-drive --no-rviz --lift "$LIFT")
i=0
until grep -q 'SEEDED\|LOCALISED' "$OUT/nav_stack.log" 2>/dev/null; do
  kill -0 "$P_NAV" 2>/dev/null || { tail -5 "$OUT/nav_stack.log"; die "the navigation stack did not come up (see $OUT/nav_stack.log)"; }
  sleep 1; i=$((i+1)); [ "$i" -ge 240 ] && die "navigation not ready after 4 min (see $OUT/nav_stack.log)"
done
log "navigation ready ($(grep -o 'SEEDED\|LOCALISED' "$OUT/nav_stack.log" | head -1))"

# ---- 2. loggers: sidecar (lift/odom), live map pose, RFID ----
P_SIDE=$(spawn side "$OUT/sidecar.out" python3 "$S/pass_sidecar.py" "$OUT")
P_POSE=$(spawn pose "$OUT/pose_logger.out" python3 "$S/pose_logger.py" "$OUT/base_link.tum")
P_RFID=$(spawn rfid "$OUT/rfid_log.out" python3 -u "$RFID_DIR/rfid_log.py" scan "$LABEL" --out-dir "$OUT" --antennas $ANTS $RFID_EXTRA)
i=0
until ls "$OUT"/"$LABEL"_*.csv >/dev/null 2>&1 && [ "$(wc -l < "$(ls "$OUT"/"$LABEL"_*.csv | head -1)")" -gt 2 ]; do
  kill -0 "$P_RFID" 2>/dev/null || { tail -5 "$OUT/rfid_log.out"; die "rfid_log.py exited"; }
  sleep 1; i=$((i+1)); [ "$i" -ge 25 ] && { tail -5 "$OUT/rfid_log.out"; die "no RFID reads after 25 s (panel cable? reader settings? no tags in range?)"; }
done
log "RFID reads coming in"
if $WAVE; then
  P_LIFT=$(spawn lift "$OUT/lift_motion.out" python3 "$S/lift_motion.py" wave --lo "$LO" --hi "$HI" --period "$PERIOD")
  log "lift: $LO <-> $HI m, one up-and-down every ${PERIOD} s, while driving"
fi
{ echo "rfid_nav_run: map=$MAP run_dir=$RUN_DIR_MAP waypoints=$WP start_at=$START_AT laps=$LAPS lift=$([ "$WAVE" = true ] && echo "wave $LO-$HI m ${PERIOD}s" || echo "fixed $LIFT") antennas=$ANTS rfid_extra=${RFID_EXTRA:-none}"
  echo "date: $(date -Is)"; } > "$OUT/meta.txt"

# ---- 3. drive the waypoints, K laps ----
python3 - "$WP" "$LAPS" "$OUT/waypoints_run.yaml" <<'PY'
import sys, yaml
spec = yaml.safe_load(open(sys.argv[1])); laps = int(sys.argv[2])
base = spec['waypoints']
spec['waypoints'] = [dict(w, name='%s_lap%d' % (w['name'], k + 1) if laps > 1 else str(w['name'])) for k in range(laps) for w in base]
yaml.safe_dump(spec, open(sys.argv[3], 'w'), sort_keys=False)
PY
log "driving $LAPS lap(s) of $(grep -c 'name:' "$WP") waypoints autonomously. Stop early: pkill -INT -f rfid_nav_run.sh"
echo run > "$OUT/nav_control"
python3 "$S/nav_waypoints.py" "$OUT/waypoints_run.yaml" --out "$OUT/nav.csv" --events "$OUT/nav_events.jsonl" --control "$OUT/nav_control" &
P_WP=$!
wait "$P_WP" 2>/dev/null; P_WP=""

# ---- 4. stop, then give every read its map position ----
log "run finished - stopping lift, RFID, loggers, navigation ..."
stop_all
CSV="$(ls "$OUT"/"$LABEL"_*.csv 2>/dev/null | grep -v _posed | head -1)"
[ -n "$CSV" ] || die "no RFID log in $OUT"
log "posing the reads ..."
python3 "$RFID_DIR/rfid_pose_join.py" "$CSV" --lift-csv "$OUT/lift.csv" --traj "$OUT/base_link.tum" --out "$OUT/reads_posed.csv" 2>&1 | tail -3
python3 - "$OUT/reads_posed.csv" "$OUT/nav.csv" <<'PY'
import csv, sys, collections
rows = [r for r in csv.DictReader(open(sys.argv[1])) if r.get('epc')]
tags = collections.Counter(r.get('name') or r['epc'] for r in rows)
far = sum(1 for r in rows if r.get('pose_dt_s') and abs(float(r['pose_dt_s'])) > 0.2)
print('reads: %d, tags: %d, reads without a pose within 0.2 s: %d' % (len(rows), len(tags), far))
for n, c in tags.most_common(12): print('  %-30s %6d reads' % (n[:30], c))
try:
    nav = list(csv.DictReader(open(sys.argv[2])))
    print('waypoints reached: %d of %d' % (sum(r['ok'] == 'True' for r in nav), len(nav)))
except FileNotFoundError:
    print('no nav.csv (stopped before the first waypoint)')
PY
log "done: $OUT  (reads_posed.csv is the item-localisation input)"
