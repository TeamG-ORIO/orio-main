#!/bin/bash
# Readiness gates for the ORIO bring-up (panes block on deps, not sleeps).
# Source and call the functions, or run as a CLI:
#   bash wait_for.sh {roscore | container <name> | topic <t> | service <s>}
# Docker uses --network host, so container nodes show up on the host roscore.

# The ROS1 master + CLI live in the roscore container on this 22.04/ROS2 host.
ROS1_CONTAINER="${ORIO_ROS1_CONTAINER:-orio_roscore}"
_ROS1_SETUP='source /opt/ros/noetic/setup.bash 2>/dev/null; source /home/ros_ws/devel/setup.bash 2>/dev/null'

_ensure_ros() {
    if ! command -v rostopic >/dev/null 2>&1; then
        # shellcheck disable=SC1091
        [ -f /opt/ros/noetic/setup.bash ] && source /opt/ros/noetic/setup.bash
    fi
}

# Run a ROS1 CLI listing. Native ROS1 host: run directly. Otherwise (this box): exec into
# the roscore container. Fails (empty/non-zero) until that container + master are up, so
# the until-loops below double as a wait for the master itself.
_ros_list() {  # $1 = rostopic|rosservice
    if command -v "$1" >/dev/null 2>&1; then
        "$1" list 2>/dev/null
    else
        docker exec "$ROS1_CONTAINER" bash -c "$_ROS1_SETUP; $1 list" 2>/dev/null
    fi
}

wait_for_roscore() {
    local deadline; deadline="$(_deadline "${1:-}")"
    _ensure_ros
    echo "[wait] roscore…"
    until _ros_list rostopic >/dev/null 2>&1; do
        if _expired "$deadline"; then
            echo "[wait] GAVE UP: roscore not up after ${1:-$ORIO_WAIT_TIMEOUT}s." >&2
            return 1
        fi
        sleep 0.5
    done
    echo "[wait] roscore is up."
}

wait_for_container() {
    local name="$1" deadline; deadline="$(_deadline "${2:-}")"
    echo "[wait] docker container '$name'…"
    until docker ps --format '{{.Names}}' 2>/dev/null | grep -qx "$name"; do
        if _expired "$deadline"; then
            echo "[wait] GAVE UP: container '$name' not running after ${2:-$ORIO_WAIT_TIMEOUT}s." >&2
            return 1
        fi
        sleep 0.5
    done
    echo "[wait] container '$name' is running."
}

# All wait_for_* accept an optional timeout in seconds ($2, or $1 where noted). With no
# timeout they loop forever (the historic tmux behaviour). With one, they return non-zero
# and print a clear "gave up" line when it elapses, so callers can surface the failure
# instead of hanging. ORIO_WAIT_TIMEOUT sets a default for all of them.
_deadline() {  # $1 = timeout seconds (empty = none); echoes an epoch deadline or nothing
    local t="${1:-${ORIO_WAIT_TIMEOUT:-}}"
    [ -n "$t" ] && echo $(( $(date +%s) + t ))
}
_expired() {  # $1 = deadline (may be empty). true if past it.
    [ -n "$1" ] && [ "$(date +%s)" -ge "$1" ]
}

wait_for_topic() {
    local topic="$1" deadline; deadline="$(_deadline "${2:-}")"
    _ensure_ros
    echo "[wait] topic '$topic'…"
    until _ros_list rostopic 2>/dev/null | grep -qx "$topic"; do
        if _expired "$deadline"; then
            echo "[wait] GAVE UP: topic '$topic' not present after ${2:-$ORIO_WAIT_TIMEOUT}s." >&2
            return 1
        fi
        sleep 0.5
    done
    echo "[wait] topic '$topic' is present."
}

# Wait until a topic is actually PUBLISHING (not merely registered). Catches the case
# where a driver advertises a topic but streams no data — e.g. a flapping camera.
# $1 = topic, $2 = timeout s (optional), $3 = min messages to see (default 1).
wait_for_topic_publishing() {
    local topic="$1" deadline min; deadline="$(_deadline "${2:-}")"; min="${3:-1}"
    _ensure_ros
    echo "[wait] topic '$topic' publishing…"
    while :; do
        # rostopic echo -n <min> returns promptly once <min> messages arrive; a short
        # timeout around it means "no data yet". Run it wherever the ROS CLI lives.
        if command -v rostopic >/dev/null 2>&1; then
            timeout 4 rostopic echo -n "$min" "$topic" >/dev/null 2>&1 && break
        else
            docker exec "$ROS1_CONTAINER" bash -c \
                "$_ROS1_SETUP; timeout 4 rostopic echo -n $min $topic" >/dev/null 2>&1 && break
        fi
        if _expired "$deadline"; then
            echo "[wait] GAVE UP: topic '$topic' not publishing after ${2:-$ORIO_WAIT_TIMEOUT}s (advertised but no data?)." >&2
            return 1
        fi
        sleep 0.5
    done
    echo "[wait] topic '$topic' is publishing."
}

wait_for_service() {
    local svc="$1" deadline; deadline="$(_deadline "${2:-}")"
    _ensure_ros
    echo "[wait] service '$svc'…"
    until _ros_list rosservice 2>/dev/null | grep -qx "$svc"; do
        if _expired "$deadline"; then
            echo "[wait] GAVE UP: service '$svc' not available after ${2:-$ORIO_WAIT_TIMEOUT}s." >&2
            return 1
        fi
        sleep 0.5
    done
    echo "[wait] service '$svc' is available."
}

# CLI dispatch when run directly.
if [ "${BASH_SOURCE[0]}" = "$0" ]; then
    cmd="$1"; shift || true
    case "$cmd" in
        roscore)          wait_for_roscore "$@" ;;
        container)        wait_for_container "$@" ;;
        topic)            wait_for_topic "$@" ;;
        topic-publishing) wait_for_topic_publishing "$@" ;;
        service)          wait_for_service "$@" ;;
        *) echo "usage: wait_for.sh {roscore [timeout] | container <name> [timeout] | topic <topic> [timeout] | topic-publishing <topic> [timeout] [min] | service <service> [timeout]}" >&2; exit 2 ;;
    esac
fi
