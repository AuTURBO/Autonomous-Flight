#include "ros/ros.h"

#include "airsim_ros_pkgs/airsim_ros_wrapper.hpp"

namespace {  // unnamed namespace
using airsim_ros::airsim_ros_wrapper::AirsimRosWrapper;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "airsim_node");
  ros::NodeHandle nh;  // not be used
  ros::NodeHandle nh_private("~");

  ROS_INFO("Running airsim_node...");

  std::string host_ip = "localhost";
  nh_private.getParam("host_ip", host_ip);
  AirsimRosWrapper airsim_ros_wrapper(nh, nh_private, host_ip);

  ros::spin();

  return 0;
}
