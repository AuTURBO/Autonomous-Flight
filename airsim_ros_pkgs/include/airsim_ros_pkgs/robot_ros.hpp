#ifndef AIRSIM_ROS_PKGS_INCLUDE_AIRSIM_ROS_PKGS_ROBOT_ROS_HPP_
#define AIRSIM_ROS_PKGS_INCLUDE_AIRSIM_ROS_PKGS_ROBOT_ROS_HPP_

#include <string>
#include <vector>

// Airsim library headers
#include "common/AirSimSettings.hpp"

// ROS headers
#include "geometry_msgs/TransformStamped.h"
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"

#include "airsim_ros_pkgs/messages.hpp"

namespace airsim_ros {
namespace robot_ros {

struct SensorPublisher {
  SensorBase::SensorType sensor_type;
  std::string sensor_name;
  ros::Publisher publisher;
};

class VehicleROS {
 public:
  virtual ~VehicleROS() {}
  std::string vehicle_name;

  /// All things ROS
  ros::Publisher odom_local_pub;
  ros::Publisher global_gps_pub;
  ros::Publisher env_pub;
  airsim_ros_pkgs::Environment env_msg;
  std::vector<SensorPublisher> sensor_pubs;
  // handle lidar seperately for max performance as data is collected on its
  // own thread/callback
  std::vector<SensorPublisher> lidar_pubs;

  nav_msgs::Odometry curr_odom;
  sensor_msgs::NavSatFix gps_sensor_msg;

  std::vector<geometry_msgs::TransformStamped> static_tf_msg_vec;

  ros::Time stamp;

  std::string odom_frame_id;
};

class CarROS : public VehicleROS {
 public:
  msr::airlib::CarApiBase::CarState curr_car_state;

  ros::Subscriber car_cmd_sub;
  ros::Publisher car_state_pub;
  airsim_ros_pkgs::CarState car_state_msg;

  bool has_car_cmd;
  msr::airlib::CarApiBase::CarControls car_cmd;
};

class MultiRotorROS : public VehicleROS {
 public:
  /// State
  msr::airlib::MultirotorState curr_drone_state;
  msr::airlib::Kinematics::State truth_state;

  ros::Subscriber vel_cmd_body_frame_sub;
  ros::Subscriber vel_cmd_world_frame_sub;

  ros::ServiceServer takeoff_srvr;
  ros::ServiceServer land_srvr;

  bool has_vel_cmd;
  messages::VelCmd vel_cmd;
};

}  // namespace robot_ros
}  // namespace airsim_ros

#endif  // AIRSIM_ROS_PKGS_INCLUDE_AIRSIM_ROS_PKGS_ROBOT_ROS_HPP_
