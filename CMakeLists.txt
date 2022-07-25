cmake_minimum_required(VERSION 3.0.2)
project(primitive_lidar_filter)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14 -Ofast)


find_package(catkin REQUIRED COMPONENTS
  PCL REQUIRED
  dynamic_reconfigure
  sensor_msgs
  nav_msgs
  pcl_ros
  roscpp
)


## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
  cfg/MyParams.cfg
)

catkin_package(
  INCLUDE_DIRS include
  #LIBRARIES primitive_lidar_filter
  #CATKIN_DEPENDS dynamic_reconfigure nav_msgs pcl_ros roscpp
  #DEPENDS roscpp std_msgs
)
  
  
## Specify additional locations of header files
## Your package locations should be listed before other locations
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
include_directories(
  include
  ${catkin_LIBRARIES}
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(box_node src/box_node.cpp)
#add_dependencies(box_node ${PROJECT_NAME}_gencfg)
target_link_libraries(box_node ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})