cmake_minimum_required(VERSION 2.8.3)
project(robot_navigation)

set( ROBOT_NAVIGATION_PKG_DEPENDS
  roscpp
  tf
  pluginlib
  geometry_msgs
  dynamic_reconfigure
  robot_navigation_state
  nav_msgs
  visualization_msgs
)

find_package(catkin REQUIRED COMPONENTS ${ROBOT_NAVIGATION_PKG_DEPENDS})
find_package(Boost  COMPONENTS thread chrono REQUIRED)
generate_dynamic_reconfigure_options(
   cfg/RobotNavigation.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES robot_navigation
  CATKIN_DEPENDS ${ROBOT_NAVIGATION_PKG_DEPENDS}
  DEPENDS Boost
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_library(robot_navigation
  src/robot_navigation.cpp
  src/robot_navigation_moving.cpp
  src/robot_navigation_planning.cpp
  src/robot_navigation_controller.cpp
)
add_dependencies(robot_navigation ${PROJECT_NAME}_gencfg)
target_link_libraries(robot_navigation
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(robot_navigation_node src/robot_navigation_node.cpp)
add_dependencies(robot_navigation_node ${PROJECT_NAME}_gencfg)
target_link_libraries(robot_navigation_node robot_navigation)

install(TARGETS robot_navigation robot_navigation_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
