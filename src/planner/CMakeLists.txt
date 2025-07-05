project(planner)
cmake_minimum_required(VERSION 3.12.3)

message(PROJECT_SOURCE_DIR="${PROJECT_SOURCE_DIR}")
message(CMAKE_CURRENT_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

#! Choose the num of the trailer
# add_definitions(-DTRAILER_NUM=1)
# add_definitions(-DTRAILER_NUM=2)
# add_definitions(-DTRAILER_NUM=3)
add_definitions(-DTRAILER_NUM=4)

set(CMAKE_BUILD_TYPE "Release")
# set(CMAKE_BUILD_TYPE "Debug")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
# set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslib
  tf
  sensor_msgs
  nav_msgs
  std_msgs
  visualization_msgs
  pcl_conversions
)

find_package(ompl REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(CASADI REQUIRED)

set(BOOST_ROOT "/usr")
set(Boost_NO_SYSTEM_PATHS TRUE)
find_package(Boost REQUIRED COMPONENTS filesystem) 

add_message_files(FILES
  TrailerState.msg
  PolyTraj.msg
  ArcTrailerTraj.msg
)

generate_messages(DEPENDENCIES
  std_msgs
  nav_msgs
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES planner
 CATKIN_DEPENDS roscpp roslib std_msgs visualization_msgs message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OMPL_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${CASADI_INCLUDE_DIR}
)

add_library(planner 
  src/grid_map.cpp
  src/hybrid_astar.cpp
  src/arc_opt.cpp
  src/planner.cpp
)

target_link_libraries(planner
  ${OMPL_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_executable (planner_node src/planner_node.cpp )
add_dependencies(planner_node planner_gencpp)
target_link_libraries(planner_node
  planner
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_executable (simulator_node src/simulator.cpp )
add_dependencies(simulator_node planner_gencpp)
target_link_libraries(simulator_node
  ${catkin_LIBRARIES}
)

add_executable (mpc_node src/mpc.cpp)
add_dependencies(mpc_node planner_gencpp)
target_link_libraries(mpc_node
  ${CASADI_LIBRARIES}
  ${catkin_LIBRARIES}
)
