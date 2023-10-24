#ifndef AIRSIM_ROS_PKGS_INCLUDE_AIRSIM_ROS_PKGS_UTILS_HPP_
#define AIRSIM_ROS_PKGS_INCLUDE_AIRSIM_ROS_PKGS_UTILS_HPP_

namespace airsim_ros {
namespace utils {

// TODO: use Eigen instead of this.
struct SimpleMatrix {
  int rows;
  int cols;
  double *data;

  SimpleMatrix(int rows, int cols, double *data)
      : rows(rows), cols(cols), data(data) {}
};

airsim_ros_pkgs::GPSYaw get_gps_msg_from_airsim_geo_point(
    const msr::airlib::GeoPoint &geo_point);
}  // namespace utils
}  // namespace airsim_ros

#endif  // AIRSIM_ROS_PKGS_INCLUDE_AIRSIM_ROS_PKGS_UTILS_HPP_
