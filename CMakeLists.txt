cmake_minimum_required(VERSION 2.8.3)
project(pr2_motors_analyzer)

find_package(catkin REQUIRED COMPONENTS
  diagnostic_aggregator
  diagnostic_msgs
  pluginlib
  roscpp
)

catkin_package(
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(pr2_motors_analyzer src/pr2_motors_analyzer)
target_link_libraries(pr2_motors_analyzer ${catkin_LIBRARIES})

install(TARGETS pr2_motors_analyzer DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(FILES pr2_motors_analyzer_plugin.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
