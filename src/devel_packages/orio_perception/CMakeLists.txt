cmake_minimum_required(VERSION 3.0.2)
project(orio_perception)

# Pure-python node package (runs in a host venv). No build-time deps.
find_package(catkin REQUIRED)

catkin_package()

## Perception node (runs in a host venv; see README).
catkin_install_python(PROGRAMS
  scripts/perception_control_combined_pass_through.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
