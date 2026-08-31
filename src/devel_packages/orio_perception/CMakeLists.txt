cmake_minimum_required(VERSION 3.0.2)
project(orio_perception)

# Pure-python node package (runs in a host venv). No build-time deps.
find_package(catkin REQUIRED)

catkin_package()

## NOTE (Phase 5): the perception node was just brought under version control
## verbatim from ~/Documents/frankapy/grasp_pose. It still has grasp_pose-relative
## config paths and requires a heavy ML venv + model weights, so it is NOT yet
## launched from the install space. See README.md for the path-robustness +
## venv/weights follow-up before wiring it into rosrun/launch.
catkin_install_python(PROGRAMS
  scripts/perception_control_combined_pass_through.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
