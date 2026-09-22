cmake_minimum_required(VERSION 3.0.2)
project(orio)

# orio is a pure-python node package: it has no C++/message build step and does
# not build against custom_msgs (its scripts import custom_msgs.srv at runtime).
# So we only need catkin itself here; runtime deps live in package.xml.
find_package(catkin REQUIRED)

catkin_package()

## Node entrypoints (also run directly from source during the demo).
catkin_install_python(PROGRAMS
  state_machine.py
  pneumatic_control_recovery.py
  monitor_forces.py
  guide_mode.py
  reset_joints.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
