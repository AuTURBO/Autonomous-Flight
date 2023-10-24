#include "airsim_ros_pkgs/airsim_ros_wrapper.hpp"

#include <string>

#include "ros/ros.h"

namespace airsim_ros {
namespace airsim_ros_wrapper {

AirsimRosWrapper::AirsimRosWrapper(const ros::NodeHandle &nh,
                                   const ros::NodeHandle &nh_private,
                                   const std::string &host_ip) {}

}  // namespace airsim_ros_wrapper
}  // namespace airsim_ros