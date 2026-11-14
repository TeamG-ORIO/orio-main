#!/usr/bin/env bash
# in_dir.sh DIR CMD... - roslaunch launch-prefix that runs a node with DIR as its working directory
# (ORB-SLAM3 loads / saves its atlas as ./NAME.osa relative to the working directory).
cd "$1" || exit 1; shift; exec "$@"
