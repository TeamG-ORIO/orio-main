cmake_minimum_required(VERSION 3.0.2)
project(orio_core)

find_package(catkin REQUIRED)

# Installs the pure-python `orio_core` package declared in setup.py so it is
# importable via devel/setup.bash (from orio_core import robot_util, ...).
catkin_python_setup()

catkin_package()

# Unit tests are pure-python and run standalone (no ROS needed):
#   cd src/devel_packages/orio_core && python3 -m pytest
# They are intentionally not wired into catkin_add_nosetests (pytest-based).
