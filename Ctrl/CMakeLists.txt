cmake_minimum_required(VERSION 2.8.3)
project(Ctrl)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package(

  # INCLUDE_DIRS include
  # LIBRARIES Ctrl
  # CATKIN_DEPENDS roscpp rospy
  # DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(Ctrl_node
  src/Ctrl_node.cpp)

target_link_libraries(Ctrl_node
  ${catkin_LIBRARIES}
)