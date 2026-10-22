#!/usr/bin/env bash
# Tear down everything run_dexnet_pnp_single.sh started: the pick loop and pneumatics (if
# a run is still going), the control PC processes on iam-doc, the cameras, the grasp
# planner, perception, the main container and roscore.
#
#   bash stop_dexnet_pnp.sh
#
# Ctrl-C on run_dexnet_pnp_single.sh only stops that run and leaves this stack up so the
# next run starts quickly; this is the clean "everything off" command.
set -uo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONTAINER="${ORIO_CONTAINER:-orio_docker_container}"
DEXNET_CONTAINER="${DEXNET_CONTAINER_NAME:-orio_dexnet}"
SUCTION_CONTAINER="${SUCTION_CONTAINER_NAME:-orio_suction}"
CTRL_PC_USER="${ORIO_CTRL_PC_USER:-student}"
CTRL_PC_HOST="${ORIO_CTRL_PC_HOST:-iam-doc}"
LOG_DIR="$REPO/logging/dexnet_pnp"
STACK_DIR="$LOG_DIR/stack"

case "${1:-}" in
    -h|--help) sed -n '2,9p' "$0"; exit 0 ;;
    "") ;;
    *) echo "unknown option: $1" >&2; exit 2 ;;
esac

say() { echo "[stop] $*"; }

# 1. A run still going: vacuum off, then stop the pick loop and pneumatics. They run in
#    the main container, so they go when it does, but the vacuum must be off first.
if docker ps -q -f "name=^${CONTAINER}\$" 2>/dev/null | grep -q .; then
    docker exec "$CONTAINER" bash -lc \
      "source /opt/ros/noetic/setup.bash 2>/dev/null; \
       rosservice call /orio/pnp_cup/off 2>/dev/null" >/dev/null 2>&1 || true
    docker exec "$CONTAINER" pkill -f 'dexnet_pnp\.py|pneumatic_control_recovery\.py' \
        >/dev/null 2>&1 && say "pick loop / pneumatics stopped"
fi

# 2. The control-PC processes ON iam-doc. Killing the local ssh sessions does NOT stop the
#    remote roslaunch/franka_interface — they would keep running there and orphan.
#
#    GRACEFULLY. franka-interface keeps its robot state in boost interprocess shared
#    memory (/dev/shm) guarded by a mutex, and holds an FCI connection to the robot.
#    SIGKILL (-9) cannot be caught, so it can leave: a LOCKED, owner-less mutex (the next
#    franka-interface then hangs forever at "Will try to acquire lock while setting
#    franka_interface status"), a half-written shared buffer (next run dies on a
#    std::bad_cast), and the arm stuck in a mode that rejects commands ("Set Joint
#    Impedance command rejected: command not possible in the current mode"). All three
#    were observed. So: SIGTERM, give it time to unwind, and only then SIGKILL stragglers,
#    then clear any shared-memory segment left behind.
say "stopping control-PC processes on $CTRL_PC_HOST…"
timeout 35 ssh -o BatchMode=yes -o ConnectTimeout=8 "$CTRL_PC_USER@$CTRL_PC_HOST" \
    'bash -s' <<'REMOTE_TEARDOWN' 2>/dev/null | sed -u 's/^/[stop]   /' \
    || say "WARNING: could not reach $CTRL_PC_HOST; stop franka-interface there by hand"
# "franka_interface" is 16 chars and the kernel truncates `comm` to 15, so
# `pkill -x franka_interface` and `ps -eo comm | grep -x franka_interface` NEVER match.
# Match the full command line.
fi_count() { pgrep -f '[f]ranka_interface --robot_ip' | wc -l; }

# 1) SIGTERM — franka-interface releases its shared-memory lock and closes the FCI
#    connection on a catchable signal.
pkill -TERM -f '[r]oslaunch franka_ros_interface' 2>/dev/null
pkill -TERM -f '[f]ranka_interface --robot_ip' 2>/dev/null

# 2) Give it a real chance to unwind (up to 10s).
for _ in $(seq 1 20); do
    [ "$(fi_count)" -eq 0 ] && break
    sleep 0.5
done

# 3) Force only what refused to exit.
if [ "$(fi_count)" -ne 0 ]; then
    echo "franka_interface did not exit on SIGTERM — forcing."
    pkill -KILL -f '[f]ranka_interface --robot_ip' 2>/dev/null
    sleep 1
fi
pkill -KILL -f '[r]oslaunch franka_ros_interface' 2>/dev/null
pkill -KILL -f '[f]ranka_ros_interface' 2>/dev/null

# 4) Remove the shared memory ONLY if nothing is still running. franka-interface never
#    removes these itself (verified: they survive even a clean SIGTERM), but deleting them
#    while a process still holds them corrupts the next start rather than helping.
if [ "$(fi_count)" -eq 0 ]; then
    rm -f /dev/shm/run_loop_* /dev/shm/current_robot_state* \
          /dev/shm/franka_interface_state_info_mutex 2>/dev/null
    echo "control PC stopped"
else
    echo "WARNING: franka_interface STILL running after SIGKILL — left /dev/shm intact."
fi
REMOTE_TEARDOWN

# 3. The stack's local processes (ssh sessions, camera launches, log followers). Each
#    leads its own process group, recorded in <name>.pid.
for f in "$STACK_DIR"/*.pid; do
    [ -f "$f" ] || continue
    kill -- "-$(cat "$f")" 2>/dev/null
    rm -f "$f"
done

# 4. The containers. The planners run with --restart unless-stopped, so rm -f, not kill.
say "removing containers…"
docker rm -f orio_roscore "$CONTAINER" "$DEXNET_CONTAINER" "$SUCTION_CONTAINER" orio_perception orio_cameras \
    >/dev/null 2>&1 || true
rm -f "$STACK_DIR"/*.conf "$LOG_DIR/current.affordance"
say "done - everything is stopped"
