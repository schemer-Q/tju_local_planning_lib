/**
 * @file lidar_ground_detection_manager.cpp
 * @author Fan Dongsheng (fandongsheng@trunk.tech)
 * @brief
 * @version 0.1
 * @date 2024-11-04
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "trunk_perception/app/lidar_ground_detection/lidar_ground_detection_manager.h"
#include "trunk_perception/common/error/code.hpp"
#include "trunk_perception/common/macros.h"
#include "trunk_perception/tools/log/t_log.h"
#include "trunk_perception/tools/system/utils.hpp"

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_BEGIN

using namespace TRUNK_PERCEPTION_LIB_COMMON_NAMESPACE;

std::shared_ptr<LidarGroundDetectionBase> LidarGroundDetectionManager::Create(const YAML::Node& config) {
  if (!config["Type"].IsDefined() || !config["Config"].IsDefined()) {
    TFATAL << "LidarGroundDetectionManager::Create: config is invalid";
    return nullptr;
  }

  std::shared_ptr<LidarGroundDetectionBase> ground_detection = nullptr;

  if (config["Type"].as<std::string>() == "BASE") {
    ground_detection = std::make_shared<LidarGroundDetectionBase>();
  } else {
    TFATAL << "LidarGroundDetectionManager::Create: ground detection type not supported: "
           << config["Type"].as<std::string>();
    return nullptr;
  }

  if (ground_detection) {
    auto ret = ground_detection->Init(config["Config"]);
    if (ret != ErrorCode::SUCCESS) {
      TFATAL << "LidarGroundDetectionManager::Create: init ground detection failed, error code: " << ret;
      return nullptr;
    }
  }

  return ground_detection;
}

std::shared_ptr<LidarGroundDetectionBase> LidarGroundDetectionManager::Create(const std::string& file) {
  if (!is_file_exist(file)) {
    TFATAL << "LidarGroundDetectionManager::Create: file not exist: " << file;
    return nullptr;
  }

  YAML::Node config = YAML::LoadFile(file);
  return Create(config);
}

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_END
