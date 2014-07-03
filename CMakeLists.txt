cmake_minimum_required(VERSION 2.8.3)
project(rosmrsmap)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rosconsole pcl_ros pcl_msgs pcl_conversions cv_bridge sensor_msgs genmsg mrsmap)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

#add_message_files(
#   FILES
# )

add_service_files(
  FILES
  StringService.srv
)

generate_messages(
   DEPENDENCIES
   std_msgs 
   sensor_msgs
   pcl_msgs
 )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES rosmrsmap
  CATKIN_DEPENDS cv_bridge opencv2 pcl_ros pcl_conversions roscpp rospy mrsmap
  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

include_directories(${PROJECT_SOURCE_DIR}/include )

add_definitions( -std=c++0x )

add_executable(snapshot_map src/training/snapshot_map.cpp )
add_dependencies(snapshot_map rosmrsmap_gencpp)
target_link_libraries(snapshot_map  ${catkin_LIBRARIES})




