set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

cmake_minimum_required(VERSION 2.8.3)
project(s8_orientation)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs s8_msgs s8_common_node)

catkin_package(
  DEPENDS
  CATKIN_DEPENDS roscpp sensor_msgs s8_msgs s8_common_node
)

include_directories(
  # include
  ${catkin_INCLUDE_DIRS}
)

add_executable(s8_orientation_node src/orientation.cpp)
target_link_libraries(s8_orientation_node ${catkin_LIBRARIES})
