cmake_minimum_required(VERSION 3.0.2)
project(path_finder)

set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -fPIC")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  occ_grid
  tf2_ros
  tf2_geometry_msgs
)



find_package(Eigen3 REQUIRED)
find_package(PCL 1.10 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES path_finder
  CATKIN_DEPENDS occ_grid
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}
  src/planner_2d.cpp
  src/kdtree.c
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
