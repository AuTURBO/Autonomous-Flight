#include "airsim_ros_pkgs/messages.hpp"

#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/Range.h"

#include "common/CommonStructs.hpp"

namespace {

ros::Time ChronoTimeToRosTime(
    const std::chrono::system_clock::time_point &stamp) {
  auto dur = std::chrono::duration<double>(stamp.time_since_epoch());
  ros::Time cur_time;
  cur_time.fromSec(dur.count());
  return cur_time;
}

ros::Time AirsimTimeToRosTime(const msr::airlib::TTimePoint &stamp) {
  std::chrono::nanoseconds dur(stamp);
  std::chrono::time_point<std::chrono::system_clock> tp(dur);
  ros::Time cur_time = ChronoTimeToRosTime(tp);
  return cur_time;
}

}  // namespace

namespace airsim_ros {
namespace messages {
namespace helper_functions {

airsim_ros_pkgs::GPSYaw GetGpsMsgFromAirsimGeoPoint(
    const msr::airlib::GeoPoint &geo_point) {
  airsim_ros_pkgs::GPSYaw gps_msg;
  gps_msg.latitude = geo_point.latitude;
  gps_msg.longitude = geo_point.longitude;
  gps_msg.altitude = geo_point.altitude;
  return gps_msg;
}

msr::airlib::Quaternionr GetAirlibQuatFromRos(const tf2::Quaternion &tf2_quat) {
  return msr::airlib::Quaternionr(tf2_quat.w(), tf2_quat.x(), tf2_quat.y(),
                                  tf2_quat.z());
}

nav_msgs::Odometry GetOdomMsgFromMultirotorState(
    const msr::airlib::MultirotorState &drone_state, bool isEnu) {
  nav_msgs::Odometry odom_msg;

  odom_msg.pose.pose.position.x = drone_state.getPosition().x();
  odom_msg.pose.pose.position.y = drone_state.getPosition().y();
  odom_msg.pose.pose.position.z = drone_state.getPosition().z();
  odom_msg.pose.pose.orientation.x = drone_state.getOrientation().x();
  odom_msg.pose.pose.orientation.y = drone_state.getOrientation().y();
  odom_msg.pose.pose.orientation.z = drone_state.getOrientation().z();
  odom_msg.pose.pose.orientation.w = drone_state.getOrientation().w();

  odom_msg.twist.twist.linear.x =
      drone_state.kinematics_estimated.twist.linear.x();
  odom_msg.twist.twist.linear.y =
      drone_state.kinematics_estimated.twist.linear.y();
  odom_msg.twist.twist.linear.z =
      drone_state.kinematics_estimated.twist.linear.z();
  odom_msg.twist.twist.angular.x =
      drone_state.kinematics_estimated.twist.angular.x();
  odom_msg.twist.twist.angular.y =
      drone_state.kinematics_estimated.twist.angular.y();
  odom_msg.twist.twist.angular.z =
      drone_state.kinematics_estimated.twist.angular.z();

  if (isEnu) {
    std::swap(odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y);
    odom_msg.pose.pose.position.z = -odom_msg.pose.pose.position.z;

    std::swap(odom_msg.pose.pose.orientation.x,
              odom_msg.pose.pose.orientation.y);
    odom_msg.pose.pose.orientation.z = -odom_msg.pose.pose.orientation.z;
    double roll, pitch, yaw;
    Eigen::Quaterniond attitude_quater;
    attitude_quater.w() = odom_msg.pose.pose.orientation.w;
    attitude_quater.x() = odom_msg.pose.pose.orientation.x;
    attitude_quater.y() = odom_msg.pose.pose.orientation.y;
    attitude_quater.z() = odom_msg.pose.pose.orientation.z;
    Eigen::Matrix3d rota = attitude_quater.toRotationMatrix();
    Eigen::Matrix3d nedtoenu;
    nedtoenu << 0, 1, 0, 1, 0, 0, 0, 0, -1;
    rota = nedtoenu * rota;
    Eigen::Quaterniond enuq(rota);

    odom_msg.pose.pose.orientation.w = enuq.w();
    odom_msg.pose.pose.orientation.x = enuq.x();
    odom_msg.pose.pose.orientation.y = enuq.y();
    odom_msg.pose.pose.orientation.z = enuq.z();

    std::swap(odom_msg.twist.twist.linear.x, odom_msg.twist.twist.linear.y);
    odom_msg.twist.twist.linear.z = -odom_msg.twist.twist.linear.z;
    std::swap(odom_msg.twist.twist.angular.x, odom_msg.twist.twist.angular.y);
    odom_msg.twist.twist.angular.z = -odom_msg.twist.twist.angular.z;
  }

  return odom_msg;
}

nav_msgs::Odometry GetOdomMsgFromTruthState(
    const msr::airlib::Kinematics::State &truth_state, bool isEnu) {
  nav_msgs::Odometry odom_msg;
  odom_msg.pose.pose.position.x = truth_state.pose.position.x();
  odom_msg.pose.pose.position.y = truth_state.pose.position.y();
  odom_msg.pose.pose.position.z = truth_state.pose.position.z();
  odom_msg.pose.pose.orientation.x = truth_state.pose.orientation.x();
  odom_msg.pose.pose.orientation.y = truth_state.pose.orientation.y();
  odom_msg.pose.pose.orientation.z = truth_state.pose.orientation.z();
  odom_msg.pose.pose.orientation.w = truth_state.pose.orientation.w();
  odom_msg.twist.twist.linear.x = truth_state.twist.linear.x();
  odom_msg.twist.twist.linear.y = truth_state.twist.linear.y();
  odom_msg.twist.twist.linear.z = truth_state.twist.linear.z();
  odom_msg.twist.twist.angular.x = truth_state.twist.angular.x();
  odom_msg.twist.twist.angular.y = truth_state.twist.angular.y();
  odom_msg.twist.twist.angular.z = truth_state.twist.angular.z();

  if (isEnu) {
    std::swap(odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y);
    odom_msg.pose.pose.position.z = -odom_msg.pose.pose.position.z;
    double roll, pitch, yaw;
    Eigen::Quaterniond attitude_quater;
    attitude_quater.w() = odom_msg.pose.pose.orientation.w;
    attitude_quater.x() = odom_msg.pose.pose.orientation.x;
    attitude_quater.y() = odom_msg.pose.pose.orientation.y;
    attitude_quater.z() = odom_msg.pose.pose.orientation.z;  // NED
    Eigen::Matrix3d rota = attitude_quater.toRotationMatrix();
    Eigen::Matrix3d nedtoenu;
    nedtoenu << 0, 1, 0, 1, 0, 0, 0, 0, -1;  // enu
    Eigen::Matrix3d body1tobody2;
    body1tobody2 << 1, 0, 0, 0, -1, 0, 0, 0, -1;
    rota = nedtoenu * rota;
    rota = rota * body1tobody2;
    Eigen::Quaterniond enuq(rota);
    odom_msg.pose.pose.orientation.w = enuq.w();
    odom_msg.pose.pose.orientation.x = enuq.x();
    odom_msg.pose.pose.orientation.y = enuq.y();
    odom_msg.pose.pose.orientation.z = enuq.z();

    std::swap(odom_msg.twist.twist.linear.x, odom_msg.twist.twist.linear.y);
    odom_msg.twist.twist.linear.z = -odom_msg.twist.twist.linear.z;
    odom_msg.twist.twist.angular.y = -odom_msg.twist.twist.angular.y;
    odom_msg.twist.twist.angular.z = -odom_msg.twist.twist.angular.z;
  }

  return odom_msg;
}

sensor_msgs::PointCloud2 GetLidarMsgFromAirsim(
    const msr::airlib::LidarData &lidar_data, const std::string &vehicle_name,
    bool isEnu) {
  sensor_msgs::PointCloud2 lidar_msg;
  lidar_msg.header.stamp = ros::Time::now();
  lidar_msg.header.frame_id = vehicle_name;

  if (lidar_data.point_cloud.size() > 3) {
    lidar_msg.height = 1;
    lidar_msg.width = lidar_data.point_cloud.size() / 3;

    lidar_msg.fields.resize(3);
    lidar_msg.fields[0].name = "x";
    lidar_msg.fields[1].name = "y";
    lidar_msg.fields[2].name = "z";

    int offset = 0;

    for (size_t d = 0; d < lidar_msg.fields.size(); ++d, offset += 4) {
      lidar_msg.fields[d].offset = offset;
      lidar_msg.fields[d].datatype = sensor_msgs::PointField::FLOAT32;
      lidar_msg.fields[d].count = 1;
    }

    lidar_msg.is_bigendian = false;
    lidar_msg.point_step = offset;  // 4 * num fields
    lidar_msg.row_step = lidar_msg.point_step * lidar_msg.width;

    lidar_msg.is_dense = true;  // todo
    std::vector<float> data_std = lidar_data.point_cloud;

    const unsigned char *bytes =
        reinterpret_cast<const unsigned char *>(data_std.data());
    std::vector<unsigned char> lidar_msg_data(
        bytes, bytes + sizeof(float) * data_std.size());
    lidar_msg.data = std::move(lidar_msg_data);
  } else {
    // msg = []
  }

  if (isEnu) {
    // transform lidar_msg to ENU not using ROS library.
    sensor_msgs::PointCloud2 lidar_msg_enu;
    lidar_msg_enu.header.stamp = ros::Time::now();
    lidar_msg_enu.header.frame_id = vehicle_name;
    lidar_msg_enu.height = lidar_msg.height;
    lidar_msg_enu.width = lidar_msg.width;
    lidar_msg_enu.fields.resize(3);
    lidar_msg_enu.fields[0].name = "x";
    lidar_msg_enu.fields[1].name = "y";
    lidar_msg_enu.fields[2].name = "z";
    int offset = 0;
    for (size_t d = 0; d < lidar_msg_enu.fields.size(); ++d, offset += 4) {
      lidar_msg_enu.fields[d].offset = offset;
      lidar_msg_enu.fields[d].datatype = sensor_msgs::PointField::FLOAT32;
      lidar_msg_enu.fields[d].count = 1;
    }
    lidar_msg_enu.is_bigendian = false;
    lidar_msg_enu.point_step = offset;  // 4 * num fields
    lidar_msg_enu.row_step = lidar_msg_enu.point_step * lidar_msg_enu.width;
    lidar_msg_enu.is_dense = true;  // todo
    std::vector<float> data_std = lidar_data.point_cloud;
    for (size_t i = 0; i < data_std.size(); i += 3) {
      std::swap(data_std[i], data_std[i + 1]);
      data_std[i + 2] = -data_std[i + 2];
    }
    const unsigned char *bytes =
        reinterpret_cast<const unsigned char *>(data_std.data());
    std::vector<unsigned char> lidar_msg_data(
        bytes, bytes + sizeof(float) * data_std.size());
    lidar_msg_enu.data = std::move(lidar_msg_data);
    lidar_msg = lidar_msg_enu;
  }

  return lidar_msg;
}

airsim_ros_pkgs::Environment GetEnvironmentMsgFromAirsim(
    const msr::airlib::Environment::State &env_data) {
  airsim_ros_pkgs::Environment env_msg;
  env_msg.position.x = env_data.position.x();
  env_msg.position.y = env_data.position.y();
  env_msg.position.z = env_data.position.z();
  env_msg.geo_point.latitude = env_data.geo_point.latitude;
  env_msg.geo_point.longitude = env_data.geo_point.longitude;
  env_msg.geo_point.altitude = env_data.geo_point.altitude;
  env_msg.gravity.x = env_data.gravity.x();
  env_msg.gravity.y = env_data.gravity.y();
  env_msg.gravity.z = env_data.gravity.z();
  env_msg.air_pressure = env_data.air_pressure;
  env_msg.temperature = env_data.temperature;
  env_msg.air_density = env_data.temperature;

  return env_msg;
}

sensor_msgs::MagneticField GetMagMsgFromAirsim(
    const msr::airlib::MagnetometerBase::Output &mag_data) {
  sensor_msgs::MagneticField mag_msg;
  mag_msg.magnetic_field.x = mag_data.magnetic_field_body.x();
  mag_msg.magnetic_field.y = mag_data.magnetic_field_body.y();
  mag_msg.magnetic_field.z = mag_data.magnetic_field_body.z();
  std::copy(std::begin(mag_data.magnetic_field_covariance),
            std::end(mag_data.magnetic_field_covariance),
            std::begin(mag_msg.magnetic_field_covariance));
  mag_msg.header.stamp = AirsimTimeToRosTime(mag_data.time_stamp);

  return mag_msg;
}

sensor_msgs::NavSatFix GetGpsMsgFromAirsim(
    const msr::airlib::GpsBase::Output &gps_data) {
  sensor_msgs::NavSatFix gps_msg;
  gps_msg.header.stamp = AirsimTimeToRosTime(gps_data.time_stamp);
  gps_msg.latitude = gps_data.gnss.geo_point.latitude;
  gps_msg.longitude = gps_data.gnss.geo_point.longitude;
  gps_msg.altitude = gps_data.gnss.geo_point.altitude;
  gps_msg.status.service = sensor_msgs::NavSatStatus::SERVICE_GLONASS;
  gps_msg.status.status = gps_data.gnss.fix_type;

  return gps_msg;
}

sensor_msgs::Range GetRangeFromAirsim(
    const msr::airlib::DistanceSensorData &dist_data) {
  sensor_msgs::Range dist_msg;
  dist_msg.header.stamp = AirsimTimeToRosTime(dist_data.time_stamp);
  dist_msg.range = dist_data.distance;
  dist_msg.min_range = dist_data.min_distance;
  dist_msg.max_range = dist_data.min_distance;

  return dist_msg;
}

airsim_ros_pkgs::Altimeter GetAltimeterMsgFromAirsim(
    const msr::airlib::BarometerBase::Output &alt_data);

sensor_msgs::Imu GetImuMsgFromAirsim(
    const msr::airlib::ImuBase::Output &imu_data);

}  // namespace helper_functions
}  // namespace messages
}  // namespace airsim_ros
