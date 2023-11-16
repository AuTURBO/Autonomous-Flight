#ifndef AIRSIM_ROS_PKGS_INCLUDE_AIRSIM_ROS_PKGS_MESSAGES_HPP_
#define AIRSIM_ROS_PKGS_INCLUDE_AIRSIM_ROS_PKGS_MESSAGES_HPP_

#include <string>

#include "mavros_msgs/AttitudeTarget.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/Range.h"
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
#include "sensors/barometer/BarometerBase.hpp"
#include "sensors/distance/DistanceSimple.hpp"
#include "sensors/gps/GpsBase.hpp"
#include "sensors/imu/ImuBase.hpp"
#include "sensors/lidar/LidarSimple.hpp"
#include "sensors/magnetometer/MagnetometerBase.hpp"
#include "vehicles/car/api/CarApiBase.hpp"
#include "vehicles/multirotor/api/MultirotorApiBase.hpp"
#include "vehicles/multirotor/api/MultirotorCommon.hpp"

namespace airsim_ros {
namespace messages {

namespace helper_functions {

airsim_ros_pkgs::GPSYaw GetGpsMsgFromAirsimGeoPoint(
    const msr::airlib::GeoPoint &geo_point);

msr::airlib::Quaternionr GetAirlibQuatFromRos(const tf2::Quaternion &tf2_quat);

nav_msgs::Odometry GetOdomMsgFromMultirotorState(
    const msr::airlib::MultirotorState &drone_state, bool isEnu);

nav_msgs::Odometry GetOdomMsgFromTruthState(
    const msr::airlib::Kinematics::State &truth_state, bool isEnu);

sensor_msgs::PointCloud2 GetLidarMsgFromAirsim(
    const msr::airlib::LidarData &lidar_data, const std::string &vehicle_name,
    bool isEnu);

airsim_ros_pkgs::Environment GetEnvironmentMsgFromAirsim(
    const msr::airlib::Environment::State &env_data);

sensor_msgs::MagneticField GetMagMsgFromAirsim(
    const msr::airlib::MagnetometerBase::Output &mag_data);

sensor_msgs::NavSatFix GetGpsMsgFromAirsim(
    const msr::airlib::GpsBase::Output &gps_data);

sensor_msgs::Range GetRangeFromAirsim(
    const msr::airlib::DistanceSensorData &dist_data);

airsim_ros_pkgs::Altimeter GetAltimeterMsgFromAirsim(
    const msr::airlib::BarometerBase::Output &alt_data);

sensor_msgs::Imu GetImuMsgFromAirsim(
    const msr::airlib::ImuBase::Output &imu_data);

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
