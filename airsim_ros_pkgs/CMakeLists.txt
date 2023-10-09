cmake_minimum_required(VERSION 3.0.2)
project(airsim_ros_pkgs)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

add_compile_options(-std=c++14)

set(MSG_DEPENDENCIES std_msgs geometry_msgs geographic_msgs)

file(GLOB MESSAGE_FILES RELATIVE ${CMAKE_CURRENT_LIST_DIR}/msg "*.msg")
file(GLOB SERVICE_FILES RELATIVE ${CMAKE_CURRENT_LIST_DIR}/srv "*.srv")

find_package(catkin REQUIRED COMPONENTS
  ${MSG_DEPENDENCIES}
  message_generation
  roscpp
  rospy
)

# Custom messages and services.
add_message_files(FILES ${MESSAGE_FILES})
add_service_files(FILES ${SERVICE_FILES})
generate_messages(DEPENDENCIES ${MSG_DEPENDENCIES})

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

install(DIRECTORY
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
