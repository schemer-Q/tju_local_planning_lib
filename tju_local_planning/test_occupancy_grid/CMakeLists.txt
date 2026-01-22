cmake_minimum_required(VERSION 3.0.2)
project(test_occupancy_grid)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  std_msgs
  sensor_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}/../..
)

add_executable(occupancy_grid_test_node src/occupancy_grid_test_node.cpp)
target_link_libraries(occupancy_grid_test_node ${catkin_LIBRARIES})
