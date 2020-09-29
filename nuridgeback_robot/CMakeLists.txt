cmake_minimum_required(VERSION 3.0.2)
project(nuridgeback_robot)

add_compile_options(-Wall -Wextra -Wno-psabi)

# Compile as C++20
set(CMAKE_CXX_STANDARD 20)
# don't use gnu extensions
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
						 laser_assembler
						 roscpp
						 sensor_msgs
)

catkin_package(
 # INCLUDE_DIRS include
 # LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS
	 roscpp
	 sensor_msgs
#  DEPENDS system_lib
)

include_directories(
	# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(assemble_scans src/assemble_scans_node.cpp)

add_dependencies(assemble_scans ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(assemble_scans
 											${catkin_LIBRARIES}
)

install(TARGETS assemble_scans
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

















#
