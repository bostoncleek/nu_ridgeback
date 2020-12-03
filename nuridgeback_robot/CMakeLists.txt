cmake_minimum_required(VERSION 3.9)
project(nuridgeback_robot)
find_package(catkin REQUIRED COMPONENTS)
catkin_package()

install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(DIRECTORY config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
# install(DIRECTORY env-hooks/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/env-hooks)
install(DIRECTORY scripts/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts)

#configure remote env_loader script
# catkin_add_env_hooks(env_setup SHELLS sh DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)












#
