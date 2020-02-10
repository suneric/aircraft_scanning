cmake_minimum_required(VERSION 2.8.3)
project(aircraft_scanning_teleop)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  geometry_msgs
  hector_quadrotor_interface
  hector_uav_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf2_geometry_msgs
)
include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS geometry_msgs hector_uav_msgs roscpp sensor_msgs std_msgs
  DEPENDS
)

add_executable(uav_scanning_teleop src/aircraft_scanning_teleop.cpp)
target_link_libraries(uav_scanning_teleop ${catkin_LIBRARIES})
add_dependencies(uav_scanning_teleop ${catkin_EXPORTED_TARGETS})
install(TARGETS uav_scanning_teleop
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION  ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
