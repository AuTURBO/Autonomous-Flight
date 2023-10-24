#include "airsim_ros_pkgs/airsim_settings_parser.hpp"

#include <iostream>
#include <string>

namespace airsim_ros {
namespace airsim_settings_parser {

AirSimSettingsParser::AirSimSettingsParser() {
  success_ = initializeSettings();
}

// mimics void ASimHUD::initializeSettings()
bool AirSimSettingsParser::initializeSettings() {
  if (!getSettingsText(settingsText_)) {
    std::cerr << "failed to get settings text" << std::endl;
    return false;
  }

  AirSimSettings::initializeSettings(settingsText_);

  // not sure where settings_json initialized in
  // AirSimSettings::initializeSettings() is actually used
  Settings& settings_json = Settings::loadJSonString(settingsText_);
  std::cout << "simmode_name: " << settings_json.getString("SimMode", "")
            << "\n";

  AirSimSettings::singleton().load(
      std::bind(&AirSimSettingsParser::getSimMode, this));

  return true;
}

bool AirSimSettingsParser::success() const { return success_; }

std::string AirSimSettingsParser::getSimMode() const {
  return Settings::loadJSonString(settingsText_).getString("SimMode", "");
}

bool AirSimSettingsParser::readSettingsTextFromFile(
    const std::string& settingsFilepath, std::string& settingsText) {
  auto ifs = std::ifstream(settingsFilepath);
  if (ifs.good()) {
    std::stringstream buffer;
    buffer << ifs.rdbuf();
    // todo airsim's simhud.cpp does error checking here
    settingsText = buffer.str();  // todo convert to utf8 as done in simhud.cpp?
    return true;
  }
  return false;
}

bool AirSimSettingsParser::getSettingsText(std::string& settingsText) {
  bool success = readSettingsTextFromFile(
      msr::airlib::Settings::Settings::getUserDirectoryFullPath(
          "settings.json"),
      settingsText);
  return success;
}

}  // namespace airsim_settings_parser
}  // namespace airsim_ros
