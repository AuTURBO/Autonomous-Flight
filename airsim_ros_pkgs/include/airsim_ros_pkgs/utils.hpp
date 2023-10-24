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

}  // namespace utils
}  // namespace airsim_ros

#endif  // AIRSIM_ROS_PKGS_INCLUDE_AIRSIM_ROS_PKGS_UTILS_HPP_