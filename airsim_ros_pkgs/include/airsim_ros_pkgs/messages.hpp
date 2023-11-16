#ifndef AIRSIM_ROS_PKGS_INCLUDE_AIRSIM_ROS_PKGS_MESSAGES_HPP_
#define AIRSIM_ROS_PKGS_INCLUDE_AIRSIM_ROS_PKGS_MESSAGES_HPP_

#include <string>

#include "mavros_msgs/AttitudeTarget.h"
#include "tf2/LinearMath/Quaternion.h"

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

#include "common/AirSimSettings.hpp"
#include "common/CommonStructs.hpp"
#include "vehicles/multirotor/api/MultirotorCommon.hpp"

namespace airsim_ros {
namespace messages {

namespace helper_functions {

airsim_ros_pkgs::GPSYaw GetGpsMsgFromAirsimGeoPoint(
    const msr::airlib::GeoPoint& geo_point);

msr::airlib::Quaternionr GetAirlibQuatFromRos(
    const tf2::Quaternion& tf2_quat);

}  // namespace helper_functions

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

struct RpyThrustCmd {
  double body_rate_x;
  double body_rate_y;
  double body_rate_z;
  double thrust;
};

}  // namespace messages
}  // namespace airsim_ros

#endif  // AIRSIM_ROS_PKGS_INCLUDE_AIRSIM_ROS_PKGS_MESSAGES_HPP_
