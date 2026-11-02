#!/usr/bin/env bash
# Tear down everything run_labelling_test.sh started: the arm script and pneumatics (if a
# run is still going), the Fast-FoundationStereo and labelling perception containers, the
# ZED launch, robot 2's stack on iam-luisa, and the shared roscore + main container.
#
#   bash research/labelling_arm_test/stop_labelling_test.sh
#   bash research/labelling_arm_test/stop_labelling_test.sh --keep-shared
#       # leave roscore, orio_docker_container and the ZED running (another launcher,
#       # e.g. run_dexnet_pnp_single.sh, may be using them)
#
# Ctrl-C on run_labelling_test.sh only stops that run; this is "everything off".
set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"
CONTAINER="${ORIO_CONTAINER:-orio_docker_container}"
FFS_CONTAINER="${ORIO_FFS_CONTAINER:-orio_ffs}"
LBL_PERC_CONTAINER="${ORIO_LBL_PERCEPTION_CONTAINER:-orio_lbl_perception}"
LUISA="${ORIO_LUISA_HOST:-snaak@iam-luisa}"
STACK_DIR="$REPO/logging/labelling_test/stack"

KEEP_SHARED=0
case "${1:-}" in
    --keep-shared) KEEP_SHARED=1 ;;
    -h|--help) sed -n '2,11p' "$0"; exit 0 ;;
    "") ;;
    *) echo "unknown option: $1" >&2; exit 2 ;;
esac

say() { echo "[stop] $*"; }

# 1. A run still going: cup off, then the arm script and pneumatics inside the container.
if docker ps -q -f "name=^${CONTAINER}\$" 2>/dev/null | grep -q .; then
    docker exec "$CONTAINER" bash -lc \
      "source /opt/ros/noetic/setup.bash 2>/dev/null; rosservice call /orio/lbl_cup/off 2>/dev/null" \
      >/dev/null 2>&1 || true
    docker exec "$CONTAINER" pkill -f 'labelling_arm_test\.py|pneumatic_control_recovery\.py' \
        >/dev/null 2>&1 && say "arm test / pneumatics stopped"
fi

# 2. This test's own containers (started with --restart unless-stopped, so rm -f).
docker rm -f "$FFS_CONTAINER" "$LBL_PERC_CONTAINER" >/dev/null 2>&1 || true
say "Fast-FoundationStereo and labelling perception containers removed"

# 3. Robot 2's stack on iam-luisa (a tmux session in a container there; it outlives ssh).
say "stopping robot 2's stack on $LUISA…"
timeout 30 ssh -o BatchMode=yes -o ConnectTimeout=8 "$LUISA" \
    "docker exec -i franka-interface bash -lc 'tmux kill-session -t franka 2>/dev/null; true'" >/dev/null 2>&1 \
    || say "WARNING: could not reach $LUISA; stop it there by hand (tmux kill-session -t franka in the franka-interface container)"

# 4. The stack's local processes (ZED launch, roscore log, log followers).
for f in "$STACK_DIR"/*.pid; do
    [ -f "$f" ] || continue
    name="$(basename "$f" .pid)"
    if [ "$KEEP_SHARED" -eq 1 ] && { [ "$name" = roscore ] || [ "$name" = zed ]; }; then continue; fi
    kill -- "-$(cat "$f")" 2>/dev/null
    rm -f "$f"
done
rm -f "$STACK_DIR"/ffs.conf "$STACK_DIR"/perception.conf "$REPO/logging/labelling_test/current.images"

# 5. The shared containers.
if [ "$KEEP_SHARED" -eq 1 ]; then
    say "done - roscore, $CONTAINER and the ZED left running (--keep-shared)"
else
    docker rm -f orio_roscore "$CONTAINER" >/dev/null 2>&1 || true
    say "done - everything is stopped"
fi
