cmake_minimum_required(VERSION 3.0.2)
project(airsim_ros_pkgs)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

# Third party libraries.
add_compile_options(-std=c++17) # C++17 standard for third party libraries.
set(AIRSIM_ROOT ${PROJECT_SOURCE_DIR}/../third_party) # Couldn't change the name because it is used in other files.
add_subdirectory("${AIRSIM_ROOT}/cmake/AirLib" AirsimLib)
add_subdirectory("${AIRSIM_ROOT}/cmake/MavLinkCom" MavLinkCom)
add_subdirectory("${AIRSIM_ROOT}/cmake/rpclib_wrapper" RpcLibWrapper)

find_package(Eigen3 REQUIRED)

set(MSG_DEPENDENCIES
  geometry_msgs
  geographic_msgs
  nav_msgs
  mavros_msgs
  sensor_msgs
  std_msgs)
set(COMPONENT_DEPENDENCIES
  message_generation
  roscpp
  rospy
  tf2_ros)
find_package(catkin REQUIRED COMPONENTS
  ${MSG_DEPENDENCIES}
  ${COMPONENT_DEPENDENCIES})

# Custom messages and services.
file(GLOB MESSAGE_FILES RELATIVE ${CMAKE_CURRENT_LIST_DIR}/msg "msg/*.msg")
file(GLOB SERVICE_FILES RELATIVE ${CMAKE_CURRENT_LIST_DIR}/srv "srv/*.srv")
add_message_files(FILES ${MESSAGE_FILES})
add_service_files(FILES ${SERVICE_FILES})
generate_messages(DEPENDENCIES ${MSG_DEPENDENCIES})

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime ${MSG_DEPENDENCIES} ${COMPONENT_DEPENDENCIES})

message(STATUS "Eigen3 include dir: ${AIRSIM_ROOT}/AirLib/deps/eigen3")

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${AIRSIM_ROOT}/AirLib/include
  ${AIRSIM_ROOT}/MavLinkCom/include
  ${AIRSIM_ROOT}/MavLinkCom/common_utils
  include)

add_library(airsim_settings_parser src/airsim_settings_parser.cpp)
target_link_libraries(airsim_settings_parser
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  AirLib)

set(ARISIM_SOURCES
  src/airsim_ros_wrapper.cpp
  src/command_subscriber.cpp)
add_library(airsim_ros ${ARISIM_SOURCES})
add_dependencies(airsim_ros
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})
target_link_libraries(airsim_ros
  ${catkin_LIBRARIES}
  AirLib
  airsim_settings_parser)

add_executable(airsim_node src/airsim_node.cpp)
target_link_libraries(airsim_node
  ${catkin_LIBRARIES}
  airsim_settings_parser
  airsim_ros)

install(TARGETS
  airsim_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
