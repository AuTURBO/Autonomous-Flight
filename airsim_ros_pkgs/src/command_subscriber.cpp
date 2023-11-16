#include "airsim_ros_pkgs/command_subscriber.hpp"

#include "ros/ros.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "airsim_ros_pkgs/utils.hpp"

namespace airsim_ros {
namespace command_subscriber {

CommandSubscriber::CommandSubscriber(const ros::NodeHandle &nh,
                                     const ros::NodeHandle &nh_private)
    : nh_(nh), nh_private_(nh_private) {
  gimbal_angle_quat_cmd_sub_ = nh_private_.subscribe(
      "gimbal_angle_quat_cmd", 50,
      &CommandSubscriber::GimbalAngleQuatCommandCallback, this);
  gimbal_angle_euler_cmd_sub_ = nh_private_.subscribe(
      "gimbal_angle_euler_cmd", 50,
      &CommandSubscriber::GimbalAngleEulerCommandCallback, this);
  ros::Subscriber rpy_thrust_sub_ =
      nh_private_.subscribe("/setpoint_raw/attitude", 50,
                            &CommandSubscriber::RpyThrustCommandCallback, this,
                            ros::TransportHints().tcpNoDelay());
}

void CommandSubscriber::GimbalAngleQuatCommandCallback(
    const airsim_ros_pkgs::GimbalAngleQuatCmdPtr &gimbal_angle_quat_cmd_msg) {
  tf2::Quaternion quaternion_command;
  try {
    tf2::convert(gimbal_angle_quat_cmd_msg->orientation, quaternion_command);
    quaternion_command.normalize();
    gimbal_cmd_.target_quat =
        messages::helper_functions::GetAirlibQuatFromRos(quaternion_command);
    gimbal_cmd_.camera_name = gimbal_angle_quat_cmd_msg->camera_name;
    gimbal_cmd_.vehicle_name = gimbal_angle_quat_cmd_msg->vehicle_name;
    has_gimbal_cmd_ = true;
  } catch (tf2::TransformException &ex) {
    ROS_WARN("%s", ex.what());
  }
}

void CommandSubscriber::GimbalAngleEulerCommandCallback(
    const airsim_ros_pkgs::GimbalAngleEulerCmdPtr &gimbal_angle_euler_cmd_msg) {
  try {
    tf2::Quaternion quaternion_command;
    quaternion_command.setRPY(utils::deg2rad(gimbal_angle_euler_cmd_msg->roll),
                              utils::deg2rad(gimbal_angle_euler_cmd_msg->pitch),
                              utils::deg2rad(gimbal_angle_euler_cmd_msg->yaw));
    quaternion_command.normalize();
    gimbal_cmd_.target_quat =
        messages::helper_functions::GetAirlibQuatFromRos(quaternion_command);
    gimbal_cmd_.camera_name = gimbal_angle_euler_cmd_msg->camera_name;
    gimbal_cmd_.vehicle_name = gimbal_angle_euler_cmd_msg->vehicle_name;
    has_gimbal_cmd_ = true;
  } catch (tf2::TransformException &ex) {
    ROS_WARN("%s", ex.what());
  }
}

void CommandSubscriber::RpyThrustCommandCallback(
    const mavros_msgs::AttitudeTargetPtr &rpy_thrust_cmd_msg) {
  rpy_thrust_cmd_.body_rate_x = rpy_thrust_cmd_msg->body_rate.x;
  rpy_thrust_cmd_.body_rate_y = rpy_thrust_cmd_msg->body_rate.y;
  rpy_thrust_cmd_.body_rate_z = rpy_thrust_cmd_msg->body_rate.z;
  rpy_thrust_cmd_.thrust = rpy_thrust_cmd_msg->thrust;
}

}  // namespace command_subscriber
}  // namespace airsim_ros
