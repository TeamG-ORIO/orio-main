#!/usr/bin/env bash
# ROS1 master in a container. This machine is 22.04/ROS2 and has no host roscore, so the
# demo's ROS1 master runs here instead. --net host puts it at localhost:11311 for the
# other host-networked containers (orio_docker, perception, dexnet) AND for the control
# PCs, which point ROS_MASTER_URI back at this user PC. Long-running: holds the roscore
# pane. wait_for.sh execs its ROS1 CLI checks into this container (name below).
set -euo pipefail

NAME="${ORIO_ROS1_CONTAINER:-orio_roscore}"
IMAGE="${ORIO_ROS1_IMAGE:-orio_docker}"

docker rm -f "$NAME" >/dev/null 2>&1 || true
exec docker run --rm --name "$NAME" --net host \
    "$IMAGE" \
    bash -c 'source /opt/ros/noetic/setup.bash && exec roscore'
