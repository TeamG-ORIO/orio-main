#!/bin/bash
# Bring up robot 2 (iam-luisa) from the workstation. Unlike frankapy's
# start_control_pc.sh, franka-interface runs in a container on the control pc:
# this starts that container if needed, then franka_interface + the ROS nodes.
# Idempotent - safe to re-run; an already-running stack is left untouched.
set -euo pipefail

# Needs `192.168.2.3  iam-luisa` in /etc/hosts, else the name resolves to an
# unrelated campus host. Override with ORIO_LUISA_HOST=snaak@192.168.2.3.
CONTROL_PC="${ORIO_LUISA_HOST:-snaak@iam-luisa}"
ROBOT_NUM="${ORIO_LUISA_ROBOT_NUM:-2}"
FI_DOCKER="~/franka-interface-docker"

# Both arms share this workstation's master, so robot 2's nodes must point back
# here instead of the container's localhost default (which would start a second,
# isolated master). ROS_IP is this workstation's robot-network address, reachable
# from iam-luisa; the hostname resolves to a Tailscale address there, so use IP.
MASTER_IP="${ORIO_MASTER_IP:-192.168.2.1}"
MASTER_URI="${ORIO_MASTER_URI:-http://$MASTER_IP:11311}"

# ROS_IP for robot 2's nodes: resolved here, not in the container (which has no
# `ip` command - detecting there yields an empty ROS_IP and nodes advertise
# "http://:PORT/", registering fine but never delivering data).
LUISA_IP="${ORIO_LUISA_IP:-$(getent hosts "${CONTROL_PC#*@}" | awk '{print $1}' | head -1)}"
if [ -z "$LUISA_IP" ]; then
    echo "[luisa] ERROR: cannot resolve ${CONTROL_PC#*@}; set ORIO_LUISA_IP" >&2
    exit 1
fi

echo "[luisa] control pc: $CONTROL_PC (robot_num=$ROBOT_NUM, master=$MASTER_URI)"

if ! timeout 10 ssh -o BatchMode=yes -o ConnectTimeout=8 "$CONTROL_PC" true 2>/dev/null; then
    echo "[luisa] ERROR: cannot ssh to $CONTROL_PC" >&2
    exit 1
fi

# run.sh starts the container only if it is down. We build the tmux session here
# rather than calling start_franka_interface.sh, because that script ends in an
# interactive `tmux attach` which would hold this ssh connection open. Window
# indices are explicit: unindexed `new-window` races and collides over ssh.
# No franka_gripper window: this arm has no Franka Hand (suction only).
# Keep the remote block free of `#` comments - ssh flattens it to one line, so a
# comment would swallow everything after it.
#
# No TTY anywhere: `ssh -tt` + run.sh's `docker exec -it` suspend this script
# (SIGTTOU, state T) when it runs from a background tmux pane, which is not the
# terminal's foreground process group. So bring the container up via run.sh's
# compose path, then exec without -t.
timeout 120 ssh -o BatchMode=yes -o ConnectTimeout=8 "$CONTROL_PC" \
    "cd $FI_DOCKER && { [ -n \"\$(docker ps -q -f name=^franka-interface\$)\" ] || docker compose up -d; } && \
     docker exec -i franka-interface bash -ic '
        set -e
        if tmux has-session -t franka 2>/dev/null; then echo ALREADY_RUNNING; exit 0; fi
        FI=~/franka-interface
        export ROS_MASTER_URI=$MASTER_URI
        export ROS_IP=$LUISA_IP
        win() { tmux new-window -t franka:\$1 -n \$2 -c \$3 \"bash -ic \\\"export ROS_MASTER_URI=$MASTER_URI ROS_IP=$LUISA_IP; \$4; exec bash\\\"\"; }
        tmux new-session -d -s franka -n shell -c ~
        if ! timeout 8 rostopic list >/dev/null 2>&1; then
            echo NO_MASTER_AT_$MASTER_URI >&2; exit 1
        fi
        win 2 franka_interface \$FI/build \"./franka_interface --robot_ip 172.16.0.2 --with_gripper 0 --log 0 --stop_on_error 0\"
        sleep 3
        win 3 ros_interface \$FI \"roslaunch franka_ros_interface franka_ros_interface.launch robot_num:=$ROBOT_NUM\"
        echo STARTED
    '" 2>&1 | tr -d '\r' | sed 's/^/[luisa] /' || true

# The stack lives in tmux inside the container, so it survives this ssh exiting.
if ! timeout 20 ssh -o BatchMode=yes -o ConnectTimeout=8 "$CONTROL_PC" \
        "docker exec franka-interface bash -lc 'tmux has-session -t franka'" 2>/dev/null; then
    echo "[luisa] ERROR: franka tmux session did not come up" >&2
    exit 1
fi

echo "[luisa] up. Attach with:"
echo "  ssh -t $CONTROL_PC 'cd $FI_DOCKER && ./run.sh tmux attach -t franka'"
echo "[luisa] stop with:"
echo "  ssh -t $CONTROL_PC 'cd $FI_DOCKER && ./run.sh tmux kill-session -t franka'"
