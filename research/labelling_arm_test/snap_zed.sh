#!/usr/bin/env bash
# Save what the ZED-M sees right now, with Fast-FoundationStereo depth.
#
# Needs this test's stack up (roscore, the ZED, the FFS node): run_labelling_test.sh starts
# it and leaves it running between runs. This script only asks for one depth and saves it;
# it starts and stops nothing. The grab runs in the perception container (the host has no
# rospy), as your user, so the files are yours rather than root's.
#
#   bash research/labelling_arm_test/snap_zed.sh [outdir]   default: logging/zed/<timestamp>/
#
# Writes color.png, right.png, preview.png (colour and depth side by side), color.npy,
# depth.npy (metres, NaN where invalid), K.npy and meta.json (camera settings, FFS summary).
# Env: ORIO_PERCEPTION_IMAGE.
set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"
IMAGE="${ORIO_PERCEPTION_IMAGE:-orio/perception:latest}"
SNAP_CONTAINER="orio_zed_snap"

OUT=""
while [ $# -gt 0 ]; do
    case "$1" in
        -h|--help) sed -n '2,13p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
        -*) echo "unknown option: $1" >&2; exit 2 ;;
        *) OUT="$1" ;;
    esac
    shift
done
OUT="${OUT:-$REPO/logging/zed/$(date +%Y%m%d_%H%M%S)}"
mkdir -p "$OUT" || exit 1
OUT="$(cd "$OUT" && pwd)"

trap 'docker rm -f "$SNAP_CONTAINER" >/dev/null 2>&1; exit 130' INT TERM
docker rm -f "$SNAP_CONTAINER" >/dev/null 2>&1 || true
docker run --rm --net=host --name "$SNAP_CONTAINER" \
    --user "$(id -u):$(id -g)" \
    --env="HOME=/tmp" \
    --env="ROS_HOME=/tmp/ros" \
    --env="ROS_MASTER_URI=${ROS_MASTER_URI:-http://localhost:11311}" \
    --volume="$HERE/ffs:/capture:ro" \
    --volume="$OUT:/out" \
    "$IMAGE" \
    bash -c 'mkdir -p /tmp/ros/log && source /opt/ros/noetic/setup.bash && python3 /capture/capture_ffs.py /out'
rc=$?
if [ "$rc" -eq 0 ]; then
    echo "[snap] saved: $OUT/color.png and $OUT/preview.png"
else
    echo "[snap] capture FAILED (exit $rc)."
fi
exit "$rc"
