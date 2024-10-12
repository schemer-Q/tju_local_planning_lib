/**
 * @file od_lidar_post_process_manager.cpp
 * @author Fan Dongsheng
 * @brief
 * @version 0.1
 * @date 2024-09-26
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "trunk_perception/app/object_detection_lidar_post_process/od_lidar_post_process_manager.h"
#include "trunk_perception/app/object_detection_lidar_post_process/od_lidar_post_process_sany.h"
#include "trunk_perception/common/error/code.hpp"
#include "trunk_perception/tools/log/t_log.h"
#include "trunk_perception/tools/system/utils.hpp"

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_BEGIN

using namespace TRUNK_PERCEPTION_LIB_COMMON_NAMESPACE;

std::shared_ptr<OdLidarPostProcessBase> OdLidarPostProcessManager::Create(const std::string& file) {
  if (!is_file_exist(file)) {
    TFATAL << "OdLidarPostProcessManager::Create: file not exist: " << file;
    return nullptr;
  }

  YAML::Node config = YAML::LoadFile(file);
  return Create(config);
}

std::shared_ptr<OdLidarPostProcessBase> OdLidarPostProcessManager::Create(const YAML::Node& config) {
  if (!config["Type"].IsDefined() || !config["Config"].IsDefined()) {
    TFATAL << "OdLidarPostProcessManager::Create: config is invalid";
    return nullptr;
  }

  std::shared_ptr<OdLidarPostProcessBase> post_process = nullptr;

  if (config["Type"].as<std::string>() == "OdLidarPostProcessSany") {
    post_process = std::make_shared<OdLidarPostProcessSany>();
  } else {
    TFATAL << "OdLidarPostProcessManager::Create: post_process type not supported: "
           << config["Type"].as<std::string>();
    return nullptr;
  }

  if (post_process) {
    auto ret = post_process->Init(config["Config"]);
    if (ret != ErrorCode::SUCCESS) {
      TFATAL << "OdLidarPostProcessManager::Create: init post_process failed, error code: " << ret;
      return nullptr;
    }
  }

  return post_process;
}

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_END