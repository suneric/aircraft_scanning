cmake_minimum_required(VERSION 2.8.3)
project(aircraft_scanning_visualize)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)
find_package(PCL 1.8 REQUIRED COMPONENTS common io kdtree features surface keypoints octree recognition sample_consensus search segmentation stereo tracking visualization)
find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp
  DEPENDS PCL OpenCV
)

include_directories(
include
${catkin_INCLUDE_DIRS}
${PCL_INCLUDE_DIRS}
${OpenCV_INCLUDE_DIRS}
)

set(TARGET_H
    src/pcl_viewer.h
    src/pcl_filter.h
    src/pcl_segment.h
    )

set(TARGET_SRC
    src/pcl_viewer.cpp
    src/pcl_filter.cpp
    src/pcl_segment.cpp
    src/main.cpp
    )

add_executable(${PROJECT_NAME}_exe ${TARGET_H} ${TARGET_SRC})
set_target_properties(${PROJECT_NAME}_exe PROPERTIES OUTPUT_NAME asv3d PREFIX "")
add_dependencies(${PROJECT_NAME}_exe ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_exe
${catkin_LIBRARIES}
${PCL_LIBRARIES}
${OpenCV_LIBRARIES}
)
