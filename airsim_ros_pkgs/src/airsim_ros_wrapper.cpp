#include "airsim_ros_pkgs/airsim_ros_wrapper.hpp"

#include <memory>
#include <string>
#include <unordered_map>

#include "common/AirSimSettings.hpp"
#include "vehicles/car/api/CarRpcLibClient.hpp"
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"

#include "ros/ros.h"

#include "airsim_ros_pkgs/messages.hpp"

namespace airsim_ros {
namespace airsim_ros_wrapper {

AirsimRosWrapper::AirsimRosWrapper(const ros::NodeHandle &nh,
                                   const ros::NodeHandle &nh_private,
                                   const std::string &host_ip)
    : config_{host_ip, nh, nh_private} {
  if (msr::airlib::AirSimSettings::singleton().simmode_name == "Car") {
    airsim_mode_ = AirsimMode::kCar;
    ROS_INFO("Setting ROS wrapper to CAR mode");
  } else {
    airsim_mode_ = AirsimMode::kDrone;
    ROS_INFO("Setting ROS wrapper to DRONE mode");
  }
  // TODO: check other modes

  InitializeRos();

  ROS_INFO("Initialized AirsimRosWrapper");
}

void AirsimRosWrapper::InitializeAirsim() {
  try {
    if (airsim_mode_ == AirsimMode::kDrone) {
      airsim_client_robot_ =
          std::make_unique<msr::airlib::MultirotorRpcLibClient>(
              config_.host_ip);
    } else if (airsim_mode_ == AirsimMode::kCar) {
      airsim_client_robot_ =
          std::make_unique<msr::airlib::CarRpcLibClient>(config_.host_ip);
    }

    airsim_client_robot_->confirmConnection();
    airsim_client_images_.confirmConnection();
    airsim_client_lidar_.confirmConnection();

    for (const auto &vehicle_name_ptr_pair : vehicle_name_ptr_map_) {
      airsim_client_robot_->enableApiControl(
          true, vehicle_name_ptr_pair.first);  // todo expose as rosservice?
      airsim_client_robot_->armDisarm(
          true, vehicle_name_ptr_pair.first);  // todo exposes as rosservice?
    }

    origin_geo_point_ = airsim_client_robot_->getHomeGeoPoint("");
    // todo there's only one global origin geopoint for environment. but airsim
    // API accept a parameter vehicle_name? inside carsimpawnapi.cpp, there's a
    // geopoint being assigned in the constructor. by?
    origin_geo_point_msg_ =
        messages::get_gps_msg_from_airsim_geo_point(origin_geo_point_);
  } catch (rpc::rpc_error &e) {
    std::string msg = e.get_error().as<std::string>();
    ROS_ERROR_STREAM("Exception raised by the API, something went wrong."
                     << std::endl
                     << msg << std::endl);
  }
}

void AirsimRosWrapper::InitializeRos() {}

}  // namespace airsim_ros_wrapper
}  // namespace airsim_ros