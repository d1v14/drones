cmake_minimum_required(VERSION 3.0.2)
project(bresenham_mapping)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
sensor_msgs
tf2
tf2_ros
tf2_msgs
tf2_sensor_msgs
geometry_msgs 
sensor_msgs
roscpp
)
find_package(PCL 1.10 REQUIRED COMPONENTS common io visualization filters)

catkin_package(
  INCLUDE_DIRS include
  # LIBRARIES wheelPlatform 
  CATKIN_DEPENDS roscpp 

)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(bresenham_mapping src/mappingNode.cpp src/GridMapping.cpp src/Map.cpp src/VoxelVisualizer.cpp)
target_link_libraries(bresenham_mapping ${catkin_LIBRARIES} ${PCL_LIBRARIES})
