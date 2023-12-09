cmake_minimum_required(VERSION 3.0.2)

project(local_planner)

set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -fPIC")

find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  visualization_msgs
  nav_msgs
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    include
)

catkin_package()

add_executable(min_jerk src/min_jerk.cpp
)

# target_link_libraries(click_gen
#   ${catkin_LIBRARIES}
# )
target_link_libraries(min_jerk
  ${catkin_LIBRARIES}
)
