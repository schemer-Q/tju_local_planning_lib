/**
 * @file fod_vision_post_process_manager.cpp
 * @author Fan Dongsheng (fandongsheng@trunk.tech)
 * @brief 
 * @version 0.1
 * @date 2024-12-05
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "trunk_perception/app/object_detection_front_vision_post_process/fod_vision_post_process_manager.h"
#include "trunk_perception/app/object_detection_front_vision_post_process/fod_vision_post_process_sany.h"
#include "trunk_perception/common/error/code.hpp"
#include "trunk_perception/tools/log/t_log.h"
#include "trunk_perception/tools/system/utils.hpp"

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_BEGIN

using namespace TRUNK_PERCEPTION_LIB_COMMON_NAMESPACE;

std::shared_ptr<FodVisionPostProcessBase> FodVisionPostProcessManager::Create(const std::string& file) {
  if (!is_file_exist(file)) {
    TFATAL << "FodVisionPostProcessManager::Create: file not exist: " << file;
    return nullptr;
  }

  YAML::Node config = YAML::LoadFile(file);
  return Create(config);
}

std::shared_ptr<FodVisionPostProcessBase> FodVisionPostProcessManager::Create(const YAML::Node& config) {
  if (!config["Type"].IsDefined() || !config["Config"].IsDefined()) {
    TFATAL << "FodVisionPostProcessManager::Create: config is invalid";
    return nullptr;
  }

  std::shared_ptr<FodVisionPostProcessBase> post_process = nullptr;

  if (config["Type"].as<std::string>() == "FodVisionPostProcessSany") {
    post_process = std::make_shared<FodVisionPostProcessSany>();
  } else {
    TFATAL << "FodVisionPostProcessManager::Create: post_process type not supported: "
           << config["Type"].as<std::string>();
    return nullptr;
  }

  if (post_process) {
    auto ret = post_process->Init(config["Config"]);
    if (ret != ErrorCode::SUCCESS) {
      TFATAL << "FodVisionPostProcessManager::Create: init post_process failed, error code: " << ret;
      return nullptr;
    }
  }

  return post_process;
}

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_END