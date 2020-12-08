cmake_minimum_required(VERSION 3.9)
project(nuridgeback_robot)
find_package(catkin REQUIRED COMPONENTS)
catkin_package()

install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(DIRECTORY config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)












#
