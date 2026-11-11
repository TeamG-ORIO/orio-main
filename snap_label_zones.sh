#!/usr/bin/env bash
# Collect RGB + depth snapshots of the two label zones for offline tests of the label
# perception.
#
# You place items in BOTH zones by hand and press Enter; it saves the ZED left image and
# the Fast-FoundationStereo depth of that moment, then asks for the next setup.
#   default   two overlapping items in each zone, as after a double pick. It asks for an
#             item type name, takes --setups snapshots of it, then asks for the next type.
#   --single  one item in each zone, any items: no names, it keeps going until you quit.
# No arm moves: keep both arms out of the label camera's view.
#
# Reuses a running stack if there is one. Otherwise it starts what the pictures need: the
# ROS1 master, orio_docker_container, the ZED and Fast-FoundationStereo. They are left
# running afterwards (the next run reuses them); bash stop_all.sh stops everything.
#
#   bash snap_label_zones.sh                    # -> logging/label_zones/double/<item>/
#   bash snap_label_zones.sh --single           # one item per zone -> .../single/setup_NN/
#   bash snap_label_zones.sh <outdir>           # save somewhere else
# Run it again at any time: it carries on from the snapshots already saved.
#   bash snap_label_zones.sh --setups 10        # snapshots per item type (default 10; not for --single)
#   bash snap_label_zones.sh --items a,b,c      # item type names up front, not asked (not for --single)
#
# Per snapshot, in <outdir>/double/<item>/setup_NN/ or <outdir>/single/setup_NN/: color.npy, depth.npy, K.npy,
# meta.json (the layout of a demo run's label records), zone1.png, zone2.png, preview.png.
# Env: ORIO_CONTAINER, ORIO_ROS1_CONTAINER, ORIO_PERCEPTION_IMAGE, ORIO_FFS_*.
set -uo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_TAG=snap
# shellcheck disable=SC1091
source "$REPO/src/devel_packages/orio_bringup/stack.sh"
IMAGE="${ORIO_PERCEPTION_IMAGE:-orio/perception:latest}"
SNAP_CONTAINER="orio_label_zones_snap"

# ── Args ────────────────────────────────────────────────────────────────────
OUT=""
SETUPS=10
ITEMS=""
MODE=double
while [ $# -gt 0 ]; do
    arg="$1"; shift
    case "$arg" in
        --single)   MODE=single ;;
        --double)   MODE=double ;;
        --setups)   SETUPS="${1:?--setups needs a number}"; shift ;;
        --setups=*) SETUPS="${arg#*=}" ;;
        --items)    ITEMS="${1:?--items needs a comma-separated list}"; shift ;;
        --items=*)  ITEMS="${arg#*=}" ;;
        -h|--help)  sed -n '2,25p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
        -*) echo "unknown option: $arg" >&2; exit 2 ;;
        *) OUT="$arg" ;;
    esac
done
[[ "$SETUPS" =~ ^[1-9][0-9]*$ ]] || { echo "--setups must be a whole number above 0, got '$SETUPS'" >&2; exit 2; }
OUT="${OUT:-$REPO/logging/label_zones}"
mkdir -p "$OUT" || exit 1
OUT="$(cd "$OUT" && pwd)"

cleanup() {
    docker rm -f "$SNAP_CONTAINER" >/dev/null 2>&1 || true
    kill_bg
}
install_cleanup_traps

# ── Stack: ROS1 master, orio container, ZED (no depth of its own), stereo depth ──
# The same stack as run_full_demo.sh, so a fresh master clears all of it.
STACK_SERVICES=(roscore control_pc_fi control_pc_ros planner zed ffs label_seg xtion perception rfid)
STACK_CONTAINERS=("$CONTAINER" "$DEXNET_CONTAINER" "$SUCTION_CONTAINER" "$FFS_CONTAINER" "$LABEL_SEG_CONTAINER" "$PERCEPTION_CONTAINER" orio_cameras)
stack_bringup_roscore || exit 1
main_container_bringup || exit 1
if ! zed_bringup NONE; then
    log "ZED start-up log: $STACK_DIR/zed.log"
    exit 1
fi
ffs_bringup
if ! ffs_wait; then
    log "Fast-FoundationStereo log: $STACK_DIR/ffs.log"
    exit 1
fi

# ── Capture (foreground and interactive, inside the perception container) ─────
log "saving to $OUT"
CAPTURE_ARGS=(--mode "$MODE" --setups "$SETUPS")
[ -n "$ITEMS" ] && CAPTURE_ARGS+=(--items "$ITEMS")
docker rm -f "$SNAP_CONTAINER" >/dev/null 2>&1 || true
docker run --rm -it --net=host --name "$SNAP_CONTAINER" \
    --user "$(id -u):$(id -g)" \
    --env="HOME=/tmp" \
    --env="ROS_HOME=/tmp/ros" \
    --env="PYTHONUNBUFFERED=1" \
    --env="ROS_MASTER_URI=${ROS_MASTER_URI:-http://localhost:11311}" \
    --volume="$REPO/src/devel_packages/orio_perception/test:/capture:ro" \
    --volume="$OUT:/out" \
    "$IMAGE" \
    bash -c 'mkdir -p /tmp/ros/log && source /opt/ros/noetic/setup.bash && python3 /capture/capture_label_zones.py /out "$@"' \
    _ "${CAPTURE_ARGS[@]}"
rc=$?
if [ "$rc" -eq 0 ]; then
    log "snapshots are in: $OUT"
    log "the ZED and Fast-FoundationStereo are still running. To stop everything: bash stop_all.sh"
else
    log "capture FAILED (exit $rc)."
fi
exit "$rc"
