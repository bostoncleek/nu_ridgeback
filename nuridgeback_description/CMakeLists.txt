cmake_minimum_required(VERSION 3.9)
project(nuridgeback_description)
find_package(catkin REQUIRED)
catkin_package()

install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(DIRECTORY config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
install(DIRECTORY urdf/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf)
install(DIRECTORY meshes/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes)
