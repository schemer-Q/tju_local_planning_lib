/**
 * @file fod_vision_manager.cpp
 * @author Fan Dongsheng (fandongsheng@trunk.tech)
 * @brief
 * @version 0.1
 * @date 2024-11-25
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "trunk_perception/app/object_detection_front_vision/fod_vision_manager.h"
#include "trunk_perception/app/object_detection_front_vision/fod_vision_sdk.h"
#include "trunk_perception/common/error/code.hpp"
#include "trunk_perception/tools/log/t_log.h"
#include "trunk_perception/tools/system/utils.hpp"

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_BEGIN

using namespace TRUNK_PERCEPTION_LIB_COMMON_NAMESPACE;

std::shared_ptr<FodVisionBase> FodVisionManager::Create(const std::string& file) {
  if (!is_file_exist(file)) {
    TFATAL << "FodVisionManager::Create: file not exist: " << file;
    return nullptr;
  }

  YAML::Node config = YAML::LoadFile(file);
  return Create(config);
}

std::shared_ptr<FodVisionBase> FodVisionManager::Create(const YAML::Node& config) {
  if (!config["Type"].IsDefined() || !config["Config"].IsDefined()) {
    TFATAL << "FodVisionManager::Create: config is invalid";
    return nullptr;
  }

  std::shared_ptr<FodVisionBase> detector = nullptr;

  if (config["Type"].as<std::string>() == "FodVisionSDK") {
    detector = std::make_shared<FodVisionSDK>();
  } else {
    TFATAL << "FodVisionManager::Create: detector type not supported: " << config["Type"].as<std::string>();
    return nullptr;
  }

  if (detector) {
    auto ret = detector->Init(config["Config"]);
    if (ret != ErrorCode::SUCCESS) {
      TFATAL << "FodVisionManager::Create: init detector failed, error code: " << ret;
      return nullptr;
    }
  }

  return detector;
}

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_END