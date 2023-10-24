#ifndef AIRSIM_ROS_PKGS_INCLUDE_AIRSIM_ROS_PKGS_AIRSIM_ROS_WRAPPER_HPP_
#define AIRSIM_ROS_PKGS_INCLUDE_AIRSIM_ROS_PKGS_AIRSIM_ROS_WRAPPER_HPP_

#include <memory>
#include <string>
#include <unordered_map>

#include "rpc/rpc_error.h"

// Airsim library headers
#include "common/AirSimSettings.hpp"
#include "vehicles/car/api/CarRpcLibClient.hpp"
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"

// ROS headers
#include "ros/ros.h"

#include "airsim_ros_pkgs/messages.hpp"
#include "airsim_ros_pkgs/robot_ros.hpp"

namespace airsim_ros {
namespace airsim_ros_wrapper {

struct Config {
  std::string host_ip;
  ros::NodeHandle nh;
  ros::NodeHandle nh_private;
};

class AirsimRosWrapper {
 public:
  AirsimRosWrapper() = delete;
  AirsimRosWrapper(const AirsimRosWrapper &) = delete;
  AirsimRosWrapper &operator=(const AirsimRosWrapper &) = delete;
  AirsimRosWrapper(AirsimRosWrapper &&) = delete;
  AirsimRosWrapper &operator=(AirsimRosWrapper &&) = delete;

  AirsimRosWrapper(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private,
                   const std::string &host_ip);

 private:
  enum class AirsimMode { kDrone, kCar };

  void InitializeAirsim();
  void InitializeRos();

  Config config_;

  // A flag to indicate whether the node is running in drone mode or car mode.
  AirsimMode airsim_mode_;

  // RPC clients to communicate with AirSim server.
  std::unique_ptr<msr::airlib::RpcLibClientBase> airsim_client_robot_;
  msr::airlib::RpcLibClientBase airsim_client_images_;
  msr::airlib::RpcLibClientBase airsim_client_lidar_;

  std::unordered_map<std::string, std::unique_ptr<robot_ros::VehicleROS>>
      vehicle_name_ptr_map_;

  ros::Publisher origin_geo_point_pub_;           // home geo coord of drones
  msr::airlib::GeoPoint origin_geo_point_;        // gps coord of unreal origin
  airsim_ros_pkgs::GPSYaw origin_geo_point_msg_;  // todo duplicate
};

}  // namespace airsim_ros_wrapper
}  // namespace airsim_ros

#endif  // AIRSIM_ROS_PKGS_INCLUDE_AIRSIM_ROS_PKGS_AIRSIM_ROS_WRAPPER_HPP_
