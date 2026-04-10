cmake_minimum_required(VERSION 3.0.2)
project(loop_closure)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf
  msckf_mono
)

## Find Eigen
find_package(Eigen3 REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp geometry_msgs sensor_msgs nav_msgs tf msckf_mono
  DEPENDS EIGEN3
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

## Add executables
add_executable(loop_closure_node
  src/loop_closure_node.cpp
  src/loop_closure_main.cpp
)

## Link against catkin libraries
target_link_libraries(loop_closure_node
  ${catkin_LIBRARIES}
)

## Make sure header dependencies are handled
add_dependencies(loop_closure_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})