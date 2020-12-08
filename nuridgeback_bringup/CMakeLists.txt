cmake_minimum_required(VERSION 3.9)
project(nuridgeback_bringup)
find_package(catkin REQUIRED)
catkin_package()

install(PROGRAMS
  scripts/nuridgeback_startup
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
