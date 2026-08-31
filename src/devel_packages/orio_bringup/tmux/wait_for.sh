#!/bin/bash
# Readiness gates for the ORIO bring-up (panes block on deps, not sleeps).
# Source and call the functions, or run as a CLI:
#   bash wait_for.sh {roscore | container <name> | topic <t> | service <s>}
# Docker uses --network host, so container nodes show up on the host roscore.

_ensure_ros() {
    if ! command -v rostopic >/dev/null 2>&1; then
        # shellcheck disable=SC1091
        [ -f /opt/ros/noetic/setup.bash ] && source /opt/ros/noetic/setup.bash
    fi
}

wait_for_roscore() {
    _ensure_ros
    echo "[wait] roscore…"
    until rostopic list >/dev/null 2>&1; do sleep 0.5; done
    echo "[wait] roscore is up."
}

wait_for_container() {
    local name="$1"
    echo "[wait] docker container '$name'…"
    until docker ps --format '{{.Names}}' 2>/dev/null | grep -qx "$name"; do sleep 0.5; done
    echo "[wait] container '$name' is running."
}

wait_for_topic() {
    local topic="$1"
    _ensure_ros
    echo "[wait] topic '$topic'…"
    until rostopic list 2>/dev/null | grep -qx "$topic"; do sleep 0.5; done
    echo "[wait] topic '$topic' is present."
}

wait_for_service() {
    local svc="$1"
    _ensure_ros
    echo "[wait] service '$svc'…"
    until rosservice list 2>/dev/null | grep -qx "$svc"; do sleep 0.5; done
    echo "[wait] service '$svc' is available."
}

# CLI dispatch when run directly.
if [ "${BASH_SOURCE[0]}" = "$0" ]; then
    cmd="$1"; shift || true
    case "$cmd" in
        roscore)   wait_for_roscore ;;
        container) wait_for_container "$@" ;;
        topic)     wait_for_topic "$@" ;;
        service)   wait_for_service "$@" ;;
        *) echo "usage: wait_for.sh {roscore|container <name>|topic <topic>|service <service>}" >&2; exit 2 ;;
    esac
fi
