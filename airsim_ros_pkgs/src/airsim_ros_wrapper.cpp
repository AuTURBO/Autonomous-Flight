#include "airsim_ros_pkgs/airsim_ros_wrapper.hpp"

#include <memory>
#include <string>
#include <unordered_map>

#include "common/AirSimSettings.hpp"
#include "vehicles/car/api/CarRpcLibClient.hpp"
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"

#include "ros/ros.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

#include "airsim_ros_pkgs/messages.hpp"

namespace airsim_ros {
namespace airsim_ros_wrapper {

AirsimRosWrapper::AirsimRosWrapper(const ros::NodeHandle &nh,
                                   const ros::NodeHandle &nh_private,
                                   const std::string &host_ip)
    : config_{false, true, host_ip, nh, nh_private},
      tf_{},
      airsim_client_robot_(host_ip),
      command_subscriber_(nh, nh_private),
      airsim_mode_{AirsimMode::kDrone} {
  airsim_mode_ = AirsimMode::kDrone;
  ROS_INFO("Setting ROS wrapper to DRONE mode");

  InitializeRos();

  ROS_INFO("Initialized AirsimRosWrapper");
}

void AirsimRosWrapper::InitializeAirsim() {
  try {
    airsim_client_robot_.ConfirmConnection();

    airsim_client_images_.confirmConnection();
    airsim_client_lidar_.confirmConnection();

    for (const auto &vehicle_name_ptr_pair : vehicle_name_ptr_map_) {
      airsim_client_robot_.EnableApiControl(vehicle_name_ptr_pair.first);
    }

    origin_geo_point_msg_ = airsim_client_robot_.GetHomeGeoPoint();
  } catch (rpc::rpc_error &e) {
    std::string msg = e.get_error().as<std::string>();
    ROS_ERROR_STREAM("Exception raised by the API, something went wrong."
                     << std::endl
                     << msg << std::endl);
  }
}

void AirsimRosWrapper::InitializeRos() {
  // ros params
  double update_airsim_control_every_n_sec;
  config_.nh_private.getParam("is_vulkan", config_.is_vulkan);
  config_.nh_private.getParam("update_airsim_control_every_n_sec",
                              update_airsim_control_every_n_sec);
  config_.nh_private.getParam("publish_clock", config_.publish_clock);
  config_.nh_private.param("world_frame_id", tf_.world_frame_id,
                           tf_.world_frame_id);
  tf_.odom_frame_id = tf_.world_frame_id == frame_ids::kAirsimFrameId
                          ? frame_ids::kAirsimOdomFrameId
                          : frame_ids::kEnuOdomFrameId;
  config_.nh_private.param("odom_frame_id", tf_.odom_frame_id,
                           tf_.odom_frame_id);
  tf_.is_enu = !(tf_.odom_frame_id == frame_ids::kAirsimOdomFrameId);
  config_.nh_private.param("coordinate_system_enu", tf_.is_enu, tf_.is_enu);
}

void AirsimRosWrapper::CreateRosPubsFromSettingsJson() {
}

}  // namespace airsim_ros_wrapper
}  // namespace airsim_ros
