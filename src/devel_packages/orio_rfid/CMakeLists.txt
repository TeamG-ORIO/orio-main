cmake_minimum_required(VERSION 3.0.2)
project(orio_rfid)

# Pure-python node package; runtime deps (rospy, custom_msgs) declared in package.xml.
find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS
  db_manager.py
  inventory_dashboard.py
  bulk_read_insert.py
  RFID/shelf_scanner_node.py
  RFID/labelling_zone_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
