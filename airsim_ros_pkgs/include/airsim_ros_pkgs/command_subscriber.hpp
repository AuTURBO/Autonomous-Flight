#ifndef AIRSIM_ROS_PKGS_INCLUDE_AIRSIM_ROS_PKGS_COMMAND_SUBSCRIBER_HPP_
#define AIRSIM_ROS_PKGS_INCLUDE_AIRSIM_ROS_PKGS_COMMAND_SUBSCRIBER_HPP_

#include "ros/ros.h"

#include "airsim_ros_pkgs/messages.hpp"
#include "airsim_ros_pkgs/utils.hpp"

namespace airsim_ros {
namespace command_subscriber {

class CommandSubscriber {
 public:
  CommandSubscriber() = delete;
  CommandSubscriber(const CommandSubscriber &) = delete;
  CommandSubscriber &operator=(const CommandSubscriber &) = delete;
  CommandSubscriber(CommandSubscriber &&) = delete;
  CommandSubscriber &operator=(CommandSubscriber &&) = delete;

  CommandSubscriber(const ros::NodeHandle &nh,
                    const ros::NodeHandle &nh_private);

 private:
  void GimbalAngleQuatCommandCallback(
      const airsim_ros_pkgs::GimbalAngleQuatCmdPtr &gimbal_angle_quat_cmd_msg);
  void GimbalAngleEulerCommandCallback(
      const airsim_ros_pkgs::GimbalAngleEulerCmdPtr
          &gimbal_angle_euler_cmd_msg);
  void RpyThrustCommandCallback(
      const mavros_msgs::AttitudeTargetPtr &rpy_thrust_cmd_msg);

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Subscriber gimbal_angle_quat_cmd_sub_;
  ros::Subscriber gimbal_angle_euler_cmd_sub_;
  ros::Subscriber rpy_thrust_sub_;

  bool has_gimbal_cmd_;
  messages::GimbalCmd gimbal_cmd_;
  messages::RpyThrustCmd rpy_thrust_cmd_;
};

}  // namespace command_subscriber
}  // namespace airsim_ros

#endif  // AIRSIM_ROS_PKGS_INCLUDE_AIRSIM_ROS_PKGS_COMMAND_SUBSCRIBER_HPP_
