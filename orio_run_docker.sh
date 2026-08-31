#!/bin/bash
xhost +local:root
docker container prune -f

# frankapy: per-machine client (git-ignored); override with FRANKAPY_DIR.
FRANKAPY_DIR="${FRANKAPY_DIR:-$(pwd)/src/git_packages/frankapy}"
if [ ! -d "$FRANKAPY_DIR/frankapy" ] || \
   [ ! -f "$FRANKAPY_DIR/catkin_ws/src/franka-interface-msgs/package.xml" ]; then
    echo "WARNING: frankapy (or its nested franka-interface-msgs) missing at '$FRANKAPY_DIR'."
    echo "  Clone the fork: git clone -b akshitr/widen-workspace-walls --recursive git@github.com:TeamG-ORIO/frankapy.git src/git_packages/frankapy"
fi

docker run --privileged --rm -it \
    --name="orio_docker_container" \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --env="NVIDIA_DRIVER_CAPABILITIES=all" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="$XAUTH:$XAUTH" \
    --network host \
    -v "$(pwd)/src/devel_packages:/home/ros_ws/src/devel_packages" \
    -v "$(pwd)/data:/home/ros_ws/data" \
    -v "/etc/timezone:/etc/timezone:ro" \
    -v "/etc/localtime:/etc/localtime:ro" \
    -v "/dev:/dev" \
    -v "$(pwd)/src/git_packages:/home/ros_ws/src/git_packages" \
    -v "$FRANKAPY_DIR:/home/ros_ws/src/git_packages/frankapy" \
    -v "$(pwd)/research:/home/ros_ws/research" \
    -v "/usr/local/zed/resources/:/usr/local/zed/resources/" \
    --gpus all \
    orio_docker bash