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
LOG_TAG=stop
STACK_DIR="$REPO/logging/labelling_test/stack"
# shellcheck disable=SC1091
source "$REPO/src/devel_packages/orio_bringup/stack.sh"
LBL_PERC_CONTAINER="${ORIO_LBL_PERCEPTION_CONTAINER:-orio_lbl_perception}"

KEEP_SHARED=0
case "${1:-}" in
    --keep-shared) KEEP_SHARED=1 ;;
    -h|--help) sed -n '2,11p' "$0"; exit 0 ;;
    "") ;;
    *) echo "unknown option: $1" >&2; exit 2 ;;
esac

# 1. A run still going: cup off, then the arm script and pneumatics inside the container.
stop_run_in_container 'labelling_arm_test\.py|pneumatic_control_recovery\.py' lbl_cup \
    && log "arm test / pneumatics stopped"

# 2. This test's own containers (started with --restart unless-stopped, so rm -f).
docker rm -f "$FFS_CONTAINER" "$LBL_PERC_CONTAINER" >/dev/null 2>&1 || true
log "Fast-FoundationStereo and labelling perception containers removed"

# 3. Robot 2's stack on iam-luisa (a tmux session in a container there; it outlives ssh).
log "stopping robot 2's stack on $LUISA_PC…"
luisa_kill_session \
    || log "WARNING: could not reach $LUISA_PC; stop it there by hand (tmux kill-session -t franka in the franka-interface container)"

# 4. The stack's local processes (ZED launch, roscore log, log followers).
if [ "$KEEP_SHARED" -eq 1 ]; then
    stack_kill_pids roscore zed
else
    stack_kill_pids
fi
rm -f "$STACK_DIR"/ffs.conf "$STACK_DIR"/perception.conf "$REPO/logging/labelling_test/current.images"

# 5. The shared containers.
if [ "$KEEP_SHARED" -eq 1 ]; then
    log "done - roscore, $CONTAINER and the ZED left running (--keep-shared)"
else
    docker rm -f "$ROS1_CONTAINER" "$CONTAINER" >/dev/null 2>&1 || true
    log "done - everything is stopped"
fi
