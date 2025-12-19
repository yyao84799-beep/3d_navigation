cmake_minimum_required(VERSION 3.0.2)
project(pct_planner)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(
  PROGRAMS
    scripts/global_planner.py
    scripts/local_planner.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
