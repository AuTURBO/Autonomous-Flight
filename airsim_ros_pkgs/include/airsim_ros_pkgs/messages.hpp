#ifndef AIRSIM_ROS_PKGS_INCLUDE_AIRSIM_ROS_PKGS_MESSAGES_HPP_
#define AIRSIM_ROS_PKGS_INCLUDE_AIRSIM_ROS_PKGS_MESSAGES_HPP_

#include <string>

#include "common/AirSimSettings.hpp"
#include "common/CommonStructs.hpp"

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

airsim_ros_pkgs::GPSYaw get_gps_msg_from_airsim_geo_point(
    const msr::airlib::GeoPoint& geo_point) {
  airsim_ros_pkgs::GPSYaw gps_msg;
  gps_msg.latitude = geo_point.latitude;
  gps_msg.longitude = geo_point.longitude;
  gps_msg.altitude = geo_point.altitude;
  return gps_msg;
}

}  // namespace messages
}  // namespace airsim_ros

#endif  // AIRSIM_ROS_PKGS_INCLUDE_AIRSIM_ROS_PKGS_MESSAGES_HPP_