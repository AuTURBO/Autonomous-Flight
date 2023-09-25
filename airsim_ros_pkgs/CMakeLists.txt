cmake_minimum_required(VERSION 3.0.2)
project(airsim_ros_pkgs)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
)

catkin_package(

  # INCLUDE_DIRS include
  # LIBRARIES airsim_ros_pkgs
  # CATKIN_DEPENDS roscpp rospy
  # DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(airsim_node src/airsim_node.cpp)
target_link_libraries(airsim_node
  ${catkin_LIBRARIES})

install(TARGETS
  airsim_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
