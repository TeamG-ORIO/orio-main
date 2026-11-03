#!/usr/bin/env bash
# Tear down everything run_dexnet_pnp_single.sh OR run_full_demo.sh started (they share
# the stack): the pick loop / sequence and pneumatics (if a run is still going), the
# control PC processes on iam-doc, the label arm's stack on iam-luisa, the cameras,
# Fast-FoundationStereo, the grasp planner, perception, the RFID node, the main container
# and roscore.
#
#   bash stop_dexnet_pnp.sh
#
# Ctrl-C on either launcher only stops that run and leaves this stack up so the
# next run starts quickly; this is the clean "everything off" command.
set -uo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_TAG=stop
# shellcheck disable=SC1091
source "$REPO/src/devel_packages/orio_bringup/stack.sh"   # STACK_DIR = logging/dexnet_pnp/stack

case "${1:-}" in
    -h|--help) sed -n '2,11p' "$0"; exit 0 ;;
    "") ;;
    *) echo "unknown option: $1" >&2; exit 2 ;;
esac

# 1. A run still going: both vacuums off, then the pick loop / sequence, pneumatics and
#    the RFID node, which run in the main container.
stop_run_in_container 'dexnet_pnp\.py|full_sequence\.py|pneumatic_control_recovery\.py|db_manager\.py' pnp_cup lbl_cup \
    && log "pick loop / sequence / pneumatics / rfid stopped"

# 2. The control-PC processes ON iam-doc, gracefully (see control_pc_teardown).
control_pc_teardown

# 2b. The label arm's stack ON iam-luisa: a tmux session inside the franka-interface
#     container there. The container itself is left running.
log "stopping the label-arm stack on ${LUISA_PC#*@}…"
if luisa_kill_session; then
    log "label-arm stack stopped"
else
    log "WARNING: could not reach ${LUISA_PC#*@} (or nothing was running there)"
fi

# 3. The stack's local processes (ssh sessions, camera launches, log followers).
stack_kill_pids

# 4. The containers. The planners and Fast-FoundationStereo run with --restart
#    unless-stopped, so rm -f, not kill.
log "removing containers…"
docker rm -f "$ROS1_CONTAINER" "$CONTAINER" "$DEXNET_CONTAINER" "$SUCTION_CONTAINER" "$FFS_CONTAINER" \
    "$PERCEPTION_CONTAINER" orio_cameras >/dev/null 2>&1 || true
rm -f "$STACK_DIR"/*.conf "$REPO/logging/dexnet_pnp/current.affordance" "$REPO/logging/demo/current.affordance"
log "done - everything is stopped"
