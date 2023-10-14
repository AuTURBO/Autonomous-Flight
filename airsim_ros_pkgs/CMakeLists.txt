cmake_minimum_required(VERSION 3.0.2)
project(airsim_ros_pkgs)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

# Third party libraries.
add_compile_options(-std=c++17) # C++17 standard for third party libraries.
set(AIRSIM_ROOT ${PROJECT_SOURCE_DIR}/../third_party) # Couldn't change the name because it is used in other files.
add_subdirectory("${AIRSIM_ROOT}/cmake/AirLib" AirsimLib)
include_directories(${AIRSIM_ROOT}/AirLib/include ${AIRSIM_ROOT}/AirLib/deps/eigen3)
add_subdirectory("${AIRSIM_ROOT}/cmake/MavLinkCom" MavLinkCom)
include_directories(${AIRSIM_ROOT}/MavLinkCom/include ${AIRSIM_ROOT}/MavLinkCom/common_utils)
add_subdirectory("${AIRSIM_ROOT}/cmake/rpclib_wrapper" RpcLibWrapper)

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
