#ifndef AIRSIM_ROS_PKGS_INCLUDE_AIRSIM_ROS_PKGS_MESSAGES_HPP_
#define AIRSIM_ROS_PKGS_INCLUDE_AIRSIM_ROS_PKGS_MESSAGES_HPP_

#include <string>

#include "common/AirSimSettings.hpp"

namespace airsim_ros {
namespace messages {

struct VelCmd {
  double x;
  double y;
  double z;
  msr::airlib::DrivetrainType drivetrain;
  msr::airlib::YawMode yaw_mode;
  std::string vehicle_name;
};

struct GimbalCmd {
  std::string vehicle_name;
  std::string camera_name;
  msr::airlib::Quaternionr target_quat;
};

}  // namespace messages
}  // namespace airsim_ros

#endif  // AIRSIM_ROS_PKGS_INCLUDE_AIRSIM_ROS_PKGS_MESSAGES_HPP_