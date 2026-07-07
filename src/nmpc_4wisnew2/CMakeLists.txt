cmake_minimum_required(VERSION 2.8.3)
project(nmpc_4wisnew2)

## Compile as C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  geometry_msgs
  std_msgs
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
)

find_package(Eigen3 REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(IPOPT REQUIRED ipopt)

# CppAD is installed as a system dependency (e.g. /usr/include/cppad).
# Its headers are picked up from the default include path.
find_path(CPPAD_INCLUDE_DIR cppad/cppad.hpp
  PATHS /usr/include /usr/local/include
)
if(NOT CPPAD_INCLUDE_DIR)
  message(FATAL_ERROR "Could not find CppAD headers (cppad/cppad.hpp)")
endif()

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp nav_msgs geometry_msgs std_msgs tf2_ros tf2_geometry_msgs visualization_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${CPPAD_INCLUDE_DIR}
  ${IPOPT_INCLUDE_DIRS}
)

add_executable(nmpc_4wisnew2_controller_node
  src/diffmpc.cpp
  src/nmpc_4wisnew2_controller_node.cpp)

target_link_libraries(nmpc_4wisnew2_controller_node
  ${catkin_LIBRARIES}
  Eigen3::Eigen
  ${IPOPT_LIBRARIES}
  pthread
  dl
)
