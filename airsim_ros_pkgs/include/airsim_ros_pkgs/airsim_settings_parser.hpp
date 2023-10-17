#ifndef AIRSIM_ROS_PKGS_INCLUDE_AIRSIM_ROS_PKGS_AIRSIM_SETTINGS_PARSER_HPP_
#define AIRSIM_ROS_PKGS_INCLUDE_AIRSIM_ROS_PKGS_AIRSIM_SETTINGS_PARSER_HPP_

#include <iostream>

// Airsim library headers
#include "common/AirSimSettings.hpp"
#include "common/common_utils/FileSystem.hpp"
#include "common/common_utils/StrictMode.hpp"
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"

STRICT_MODE_OFF
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif  // !RPCLIB_MSGPACK
#include "rpc/rpc_error.h"
STRICT_MODE_ON

namespace airsim_ros {
namespace airsim_settings_parser {

class AirSimSettingsParser {
 public:
  using AirSimSettings = msr::airlib::AirSimSettings;
  using VehicleSetting = msr::airlib::AirSimSettings::VehicleSetting;

  AirSimSettingsParser();

  bool success() const;

 private:
  bool initializeSettings();
  std::string getSimMode() const;
  bool readSettingsTextFromFile(const std::string& settingsFilepath,
                                std::string& settingsText);
  bool getSettingsText(std::string& settingsText);

  bool success_;
  std::string settingsText_;
};

}  // namespace airsim_settings_parser
}  // namespace airsim_ros

#endif  // AIRSIM_ROS_PKGS_INCLUDE_AIRSIM_ROS_PKGS_AIRSIM_SETTINGS_PARSER_HPP_
