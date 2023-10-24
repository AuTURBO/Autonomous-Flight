#ifndef AIRSIM_ROS_PKGS_INCLUDE_AIRSIM_ROS_PKGS_AIRSIM_ROS_WRAPPER_HPP_
#define AIRSIM_ROS_PKGS_INCLUDE_AIRSIM_ROS_PKGS_AIRSIM_ROS_WRAPPER_HPP_

#include <string>

#include "ros/ros.h"

// ROS custom msgs.
#include "airsim_ros_pkgs/Altimeter.h"
#include "airsim_ros_pkgs/CarControls.h"
#include "airsim_ros_pkgs/CarState.h"
#include "airsim_ros_pkgs/Environment.h"
#include "airsim_ros_pkgs/GPSYaw.h"
#include "airsim_ros_pkgs/GimbalAngleEulerCmd.h"
#include "airsim_ros_pkgs/GimbalAngleQuatCmd.h"
#include "airsim_ros_pkgs/VelCmd.h"
#include "airsim_ros_pkgs/VelCmdGroup.h"

// ROS custom srvs.
#include "airsim_ros_pkgs/Land.h"
#include "airsim_ros_pkgs/LandGroup.h"
#include "airsim_ros_pkgs/Reset.h"
#include "airsim_ros_pkgs/Takeoff.h"
#include "airsim_ros_pkgs/TakeoffGroup.h"

namespace airsim_ros {
namespace utils {

class AirsimRosWrapper {
 public:
  AirsimRosWrapper(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private,
                   const std::string &host_ip);
  ~AirsimRosWrapper(){};

 private:
}

}  // namespace utils
}  // namespace airsim_ros

#endif  // AIRSIM_ROS_PKGS_INCLUDE_AIRSIM_ROS_PKGS_AIRSIM_ROS_WRAPPER_HPP_
