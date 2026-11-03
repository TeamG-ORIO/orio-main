#!/bin/bash
# Start the main orio container (orio_docker_container): ROS Noetic, frankapy, the camera
# drivers and the arm scripts. The launchers start it detached (ORIO_NO_TTY=1) and
# `docker exec` into it; run it by hand for an interactive shell:
#   bash src/devel_packages/orio_bringup/docker/run_orio.sh
# Mounts are relative to the repo root, so it works from any directory.
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)"
cd "$REPO" || exit 1

xhost +local:root
docker container prune -f

# frankapy: per-machine client (git-ignored); override with FRANKAPY_DIR.
FRANKAPY_DIR="${FRANKAPY_DIR:-$REPO/src/git_packages/frankapy}"
if [ ! -d "$FRANKAPY_DIR/frankapy" ] || \
   [ ! -f "$FRANKAPY_DIR/catkin_ws/src/franka-interface-msgs/package.xml" ]; then
    echo "WARNING: frankapy (or its nested franka-interface-msgs) missing at '$FRANKAPY_DIR'."
    echo "  Clone the fork: git clone -b akshitr/widen-workspace-walls --recursive git@github.com:TeamG-ORIO/frankapy.git src/git_packages/frankapy"
fi

# Interactive by default (-it + a bash shell). A merged single-terminal launcher runs
# this backgrounded with no TTY, where -it fails ("the input device is not a TTY") and an
# interactive bash exits immediately. Set ORIO_NO_TTY=1 to run detached (-d) and keep the
# container alive with `sleep infinity`, so the launcher can `docker exec` into it.
if [ -n "${ORIO_NO_TTY:-}" ]; then
    RUN_FLAGS="-d"; RUN_CMD=(bash -lc "sleep infinity")
else
    RUN_FLAGS="-it"; RUN_CMD=(bash)
fi

docker run --privileged --rm $RUN_FLAGS \
    --name="${ORIO_CONTAINER:-orio_docker_container}" \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --env="NVIDIA_DRIVER_CAPABILITIES=all" \
    --env="ORIO_LOGGING=${ORIO_LOGGING:-1}" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="$XAUTH:$XAUTH" \
    --network host \
    -v "$REPO/src/devel_packages:/home/ros_ws/src/devel_packages" \
    -v "$REPO/data:/home/ros_ws/data" \
    -v "/etc/timezone:/etc/timezone:ro" \
    -v "/etc/localtime:/etc/localtime:ro" \
    -v "/dev:/dev" \
    -v "$REPO/src/git_packages:/home/ros_ws/src/git_packages" \
    -v "$FRANKAPY_DIR:/home/ros_ws/src/git_packages/frankapy" \
    -v "$REPO/research:/home/ros_ws/research" \
    -v "$REPO/logging:/home/ros_ws/logging" \
    -v "/usr/local/zed/resources/:/usr/local/zed/resources/" \
    --gpus all \
    orio_docker "${RUN_CMD[@]}"
