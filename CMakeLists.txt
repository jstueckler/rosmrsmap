cmake_minimum_required(VERSION 2.8.3)
project(rosmrsmap)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rosconsole pcl_ros pcl_msgs pcl_conversions cv_bridge sensor_msgs genmsg mrsmap tf_conversions eigen_conversions image_geometry)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/Modules/" ${CMAKE_MODULE_PATH})


# G2O required
find_package(G2O REQUIRED)
include_directories(${G2O_INCLUDE_DIR})

find_package( Boost 1.40 COMPONENTS program_options REQUIRED )
include_directories( ${Boost_INCLUDE_DIR} )


# For building the CHOLMOD / CSPARSE solvers
find_package(Cholmod)
find_package(BLAS)
find_package(LAPACK)
find_package(CSparse)
include_directories(${CSPARSE_INCLUDE_DIR})

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

add_message_files(
  FILES
  ObjectData.msg
  ObjectTrackingData2.msg
)

add_service_files(
  FILES
  StringService.srv
  RegisterMap.srv
  ObjectTrackingRequest.srv
)

generate_messages(
   DEPENDENCIES
   std_msgs 
   sensor_msgs
   pcl_msgs
 )

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES rosmrsmap
  CATKIN_DEPENDS cv_bridge pcl_ros pcl_conversions roscpp rospy mrsmap tf_conversions eigen_conversions
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


add_executable(register_map src/perception/register_map.cpp )
add_dependencies(register_map rosmrsmap_gencpp)
target_link_libraries(register_map  ${catkin_LIBRARIES})


add_executable(collect_data src/training/collect_data.cpp )
add_dependencies(collect_data rosmrsmap_gencpp)
target_link_libraries(collect_data  ${catkin_LIBRARIES})


add_executable(train_map_from_data src/training/train_map_from_data.cpp src/utilities/utilities.cpp)
add_dependencies(train_map_from_data rosmrsmap_gencpp)
target_link_libraries(train_map_from_data ${catkin_LIBRARIES} ${G2O_CORE_LIBRARY} ${G2O_STUFF_LIBRARY} ${G2O_SOLVER_CSPARSE} ${G2O_SOLVER_CSPARSE_EXTENSION} ${G2O_TYPES_SLAM3D})


add_executable(train_map_from_data_cb src/training/train_map_from_data_cb.cpp src/utilities/utilities.cpp)
add_dependencies(train_map_from_data_cb rosmrsmap_gencpp)
target_link_libraries(train_map_from_data_cb ${catkin_LIBRARIES} ${G2O_CORE_LIBRARY} ${G2O_STUFF_LIBRARY} ${G2O_SOLVER_CSPARSE} ${G2O_SOLVER_CSPARSE_EXTENSION} ${G2O_TYPES_SLAM3D} vtkFiltering)


add_executable(object_tracking_node src/perception/object_tracking_node.cpp src/utilities/utilities.cpp)
add_dependencies(object_tracking_node rosmrsmap_gencpp)
target_link_libraries(object_tracking_node ${catkin_LIBRARIES} )


