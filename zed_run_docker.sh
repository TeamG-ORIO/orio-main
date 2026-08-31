# run_docker.sh
xhost +local:root 
docker container prune -f 
docker run --privileged --rm -it \
    --name="orio_cameras_container" \
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
    -v "/usr/local/zed/resources/:/usr/local/zed/resources/" \
    --gpus all \
    orio_docker bash