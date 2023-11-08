cmake_minimum_required(VERSION 2.8.3)
project(ros1_out)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 11)
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)


catkin_package(
  CATKIN_DEPENDS
  roscpp
  std_msgs
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)



add_library(ros1_out_lib src/ros1_out.cpp)
add_dependencies(ros1_out_lib
  ${catkin_EXPORTED_TARGETS})


add_executable(ros1_out_node
  src/ros1_out_node.cpp)

target_link_libraries(ros1_out_node
  ros1_out_lib
  ${catkin_LIBRARIES}
)

add_dependencies(ros1_out_node
  ${catkin_EXPORTED_TARGETS}
)
