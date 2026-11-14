cmake_minimum_required(VERSION 3.0.2)
project(orio_stretch)
find_package(catkin REQUIRED)
catkin_package()
# Nothing to build. Scripts run straight from the source tree (scripts/orio_env.sh puts this
# package on ROS_PACKAGE_PATH), so no install rules and no workspace build are needed on the robot.
