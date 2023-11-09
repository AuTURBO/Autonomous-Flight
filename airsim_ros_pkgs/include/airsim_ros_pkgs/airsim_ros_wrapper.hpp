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
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

#include "airsim_ros_pkgs/command_subscriber.hpp"
#include "airsim_ros_pkgs/messages.hpp"
#include "airsim_ros_pkgs/robot_ros.hpp"
#include "airsim_ros_pkgs/utils.hpp"

namespace airsim_ros {
namespace airsim_ros_wrapper {

namespace frame_ids {
constexpr char kAirsimFrameId[] = "world_ned";
constexpr char kAirsimOdomFrameId[] = "odom_local_ned";
constexpr char kEnuOdomFrameId[] = "odom_local_enu";
}  // namespace frame_ids

struct Config {
  bool is_vulkan;
  bool publish_clock;
  std::string host_ip;
  ros::NodeHandle nh;
  ros::NodeHandle nh_private;
};

struct TF {
  bool is_enu = false;
  std::string world_frame_id = frame_ids::kAirsimFrameId;
  std::string odom_frame_id = frame_ids::kAirsimOdomFrameId;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  tf2_ros::StaticTransformBroadcaster static_tf_pub_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_ = tf_buffer_;
};

class AirsimClient {
 public:
  AirsimClient() = delete;
  AirsimClient(const AirsimClient &) = delete;
  AirsimClient &operator=(const AirsimClient &) = delete;
  AirsimClient(AirsimClient &&) = delete;
  AirsimClient &operator=(AirsimClient &&) = delete;

  explicit AirsimClient(const std::string &host_ip) {
    try {
      airsim_client_robot_ =
          std::make_unique<msr::airlib::MultirotorRpcLibClient>(host_ip);
    } catch (rpc::rpc_error &e) {
      ROS_ERROR_STREAM("Exception raised by the API, something went wrong."
                       << std::endl
                       << e.get_error().as<std::string>() << std::endl);
    }
  }

  void ConfirmConnection() const { airsim_client_robot_->confirmConnection(); }

  void EnableApiControl(const std::string &vehicle_name) const {
    airsim_client_robot_->enableApiControl(true, vehicle_name);
    airsim_client_robot_->armDisarm(true, vehicle_name);
  }

  airsim_ros_pkgs::GPSYaw GetHomeGeoPoint() const {
    const auto origin_geo_point = airsim_client_robot_->getHomeGeoPoint("");
    return utils::GetGpsMsgFromAirsimGeoPoint(origin_geo_point);
  }

 private:
  std::unique_ptr<msr::airlib::RpcLibClientBase> airsim_client_robot_;
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

  void CreateRosPubsFromSettingsJson();

  Config config_;
  TF tf_;

  // A flag to indicate whether the node is running in drone mode or car mode.
  AirsimMode airsim_mode_;

  // RPC clients to communicate with AirSim server.
  AirsimClient airsim_client_robot_;
  msr::airlib::RpcLibClientBase airsim_client_images_;
  msr::airlib::RpcLibClientBase airsim_client_lidar_;

  std::unordered_map<std::string, std::unique_ptr<robot_ros::VehicleROS>>
      vehicle_name_ptr_map_;

  ros::Timer airsim_control_update_timer_;

  ros::Publisher origin_geo_point_pub_;           // home geo coord of drones
  airsim_ros_pkgs::GPSYaw origin_geo_point_msg_;  // todo duplicate

  command_subscriber::CommandSubscriber command_subscriber_;
};

}  // namespace airsim_ros_wrapper
}  // namespace airsim_ros

#endif  // AIRSIM_ROS_PKGS_INCLUDE_AIRSIM_ROS_PKGS_AIRSIM_ROS_WRAPPER_HPP_
