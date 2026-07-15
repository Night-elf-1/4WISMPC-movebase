cmake_minimum_required(VERSION 3.0.2)
project(mpc_local_planner)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_core
  costmap_2d
  geometry_msgs
  nav_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  pluginlib
  angles
)

find_package(Eigen3 REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(IPOPT REQUIRED ipopt)

find_path(CPPAD_INCLUDE_DIR cppad/cppad.hpp
  PATHS /usr/include /usr/local/include
)
if(NOT CPPAD_INCLUDE_DIR)
  message(FATAL_ERROR "Could not find CppAD headers (cppad/cppad.hpp)")
endif()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp nav_core costmap_2d geometry_msgs nav_msgs std_msgs tf2 tf2_ros tf2_geometry_msgs pluginlib angles
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${CPPAD_INCLUDE_DIR}
  ${IPOPT_INCLUDE_DIRS}
)

# 插件库：包含 NMPC 核心与 move_base 接口
add_library(${PROJECT_NAME}
  src/mpc_local_planner.cpp
  src/diffmpc.cpp
  src/control_limits.cpp
  src/planner_utils.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  Eigen3::Eigen
  ${IPOPT_LIBRARIES}
  pthread
  dl
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_control_limits
    test/test_control_limits.cpp
    test/test_main.cpp
  )
  if(TARGET test_control_limits)
    target_link_libraries(test_control_limits ${PROJECT_NAME})
  endif()

  catkin_add_gtest(test_diffmpc_reference
    test/test_diffmpc_reference.cpp
    test/test_main.cpp
  )
  if(TARGET test_diffmpc_reference)
    target_link_libraries(test_diffmpc_reference ${PROJECT_NAME})
  endif()

  catkin_add_gtest(test_planner_utils
    test/test_planner_utils.cpp
    test/test_main.cpp
  )
  if(TARGET test_planner_utils)
    target_link_libraries(test_planner_utils ${PROJECT_NAME})
  endif()
endif()

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)

install(FILES mpc_local_planner_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY param/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/param
)
