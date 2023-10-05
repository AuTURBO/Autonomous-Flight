#include "ros/ros.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "airsim_node");
  ros::NodeHandle nh;  // not be used
  ros::NodeHandle nh_private("~");

  ROS_INFO("Running airsim_node...");

  ros::spin();

  return 0;
}
