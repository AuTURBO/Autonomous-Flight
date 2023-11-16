#include "messages.hpp"

namespace airsim_ros {
namespace messages {
namespace helper_functions {

airsim_ros_pkgs::GPSYaw GetGpsMsgFromAirsimGeoPoint(
    const msr::airlib::GeoPoint& geo_point) {
  airsim_ros_pkgs::GPSYaw gps_msg;
  gps_msg.latitude = geo_point.latitude;
  gps_msg.longitude = geo_point.longitude;
  gps_msg.altitude = geo_point.altitude;
  return gps_msg;
}

msr::airlib::Quaternionr GetAirlibQuatFromRos(const tf2::Quaternion& tf2_quat) {
  return msr::airlib::Quaternionr(tf2_quat.w(), tf2_quat.x(), tf2_quat.y(),
                                  tf2_quat.z());
}

}  // namespace helper_functions
}  // namespace messages
}  // namespace airsim_ros
