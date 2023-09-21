#include "ros/ros.h"

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "Ctrl");
  ros::NodeHandle nh("~");

  while (ros::ok()) {
    // Do nothing.
  }

  return 0;
}