cmake_minimum_required(VERSION 2.8.3)
project(reference_path_publisher)

## Compile as C++14 (Noetic supports C++14/C++17)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  geometry_msgs
  visualization_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp nav_msgs geometry_msgs visualization_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(ref_path_publisher src/ref_path_publisher.cpp)
target_link_libraries(ref_path_publisher ${catkin_LIBRARIES})
