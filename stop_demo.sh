#!/bin/bash
# Shut down the ORIO demo started by launch_demo.sh.
#   bash stop_demo.sh              # stop the demo, leave containers up
#   bash stop_demo.sh --containers # also stop orio, the grasp planners + the luisa container
# Robot 2 goes first: its controller is stopped over ssh, and killing the tmux
# session would take away the pane that manages it.
set -uo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SESSION="${ORIO_SESSION:-orio}"
CONTAINER="${ORIO_CONTAINER:-orio_docker_container}"
DEXNET_CONTAINER="${DEXNET_CONTAINER_NAME:-orio_dexnet}"
SUCTION_CONTAINER="${SUCTION_CONTAINER_NAME:-orio_suction}"
LUISA="${ORIO_LUISA_HOST:-snaak@iam-luisa}"

STOP_CONTAINERS=0
for arg in "$@"; do
    case "$arg" in
        --containers) STOP_CONTAINERS=1 ;;
        -h|--help) sed -n '2,4p' "$0"; exit 0 ;;
        *) echo "unknown option: $arg" >&2; exit 2 ;;
    esac
done

# 1. Robot 2's controller, in its container on the control pc. No TTY: this may
# run from a background pane, where -t would suspend us (SIGTTOU).
echo "[stop] robot 2 ($LUISA)"
if timeout 10 ssh -o BatchMode=yes -o ConnectTimeout=8 "$LUISA" true 2>/dev/null; then
    timeout 30 ssh -o BatchMode=yes "$LUISA" \
        "docker exec -i franka-interface bash -lc 'tmux kill-session -t franka 2>/dev/null; true'" \
        >/dev/null 2>&1
    echo "[stop]   franka session stopped"
else
    echo "[stop]   WARNING: cannot ssh to $LUISA; stop it there by hand" >&2
fi

# 2. Recorder + perception flush their .rrd on SIGINT; give them up to 5 s.
if tmux has-session -t "$SESSION" 2>/dev/null; then
    for p in $(tmux list-panes -s -t "$SESSION" -F '#{pane_id}:#{pane_title}' | grep -E ':(recorder|perception)$' | cut -d: -f1); do
        tmux send-keys -t "$p" C-c
    done
    for _ in $(seq 10); do
        busy=$(tmux list-panes -s -t "$SESSION" -F '#{pane_title}:#{pane_current_command}' | grep -cE '^(recorder|perception):python')
        [ "$busy" -eq 0 ] && break
        sleep 0.5
    done
    echo "[stop] rerun writers flushed"
fi

# 3. The tmux session: roscore, cameras, perception, state machine, and the ssh
# sessions holding robot 1's controller (frankapy ties it to the connection).
if tmux has-session -t "$SESSION" 2>/dev/null; then
    tmux kill-session -t "$SESSION" && echo "[stop] tmux session '$SESSION' closed"
else
    echo "[stop] no tmux session '$SESSION'"
fi

if [ "$STOP_CONTAINERS" -eq 1 ]; then
    # The planner containers run with --restart unless-stopped, so they need an explicit stop.
    for c in "$CONTAINER" "$DEXNET_CONTAINER" "$SUCTION_CONTAINER"; do
        if [ -n "$(docker ps -q -f "name=^${c}$" 2>/dev/null)" ]; then
            docker stop "$c" >/dev/null 2>&1 && echo "[stop] container $c stopped"
        fi
    done
    timeout 40 ssh -o BatchMode=yes "$LUISA" \
        "cd ~/franka-interface-docker && docker compose down" >/dev/null 2>&1 \
        && echo "[stop] franka-interface container stopped on $LUISA"
fi

# Probe the master's port directly: `rosnode list` exits 0 even with no master.
# Give it a few seconds - the socket lingers briefly after the panes are killed.
MASTER_PORT="${ORIO_MASTER_PORT:-11311}"
for _ in $(seq 10); do
    timeout 2 bash -c "</dev/tcp/127.0.0.1/$MASTER_PORT" 2>/dev/null || break
    sleep 1
done
if timeout 3 bash -c "</dev/tcp/127.0.0.1/$MASTER_PORT" 2>/dev/null; then
    echo "[stop] WARNING: something is still listening on port $MASTER_PORT" >&2
else
    echo "[stop] done - no ROS master running"
fi
