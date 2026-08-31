cmake_minimum_required(VERSION 3.0.2)
project(orio_bringup)

# Meta/bring-up package: docker image + run helpers (+ future launch/orchestration).
# No build step; nothing to compile.
find_package(catkin REQUIRED)

catkin_package()
