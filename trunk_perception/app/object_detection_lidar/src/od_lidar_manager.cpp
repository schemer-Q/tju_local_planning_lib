#include "trunk_perception/app/object_detection_lidar/od_lidar_manager.h"
#include "trunk_perception/tools/system/utils.hpp"
#include "trunk_perception/tools/log/t_log.h"
#include "trunk_perception/common/error/code.hpp"
#include "trunk_perception/app/object_detection_lidar/od_lidar_lidarnetsdk.h"

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_BEGIN

using namespace TRUNK_PERCEPTION_LIB_COMMON_NAMESPACE;

std::shared_ptr<OdLidarBase> OdLidarManager::Create(const std::string& file) {
  if (!is_file_exist(file)) {
    TFATAL << "OdLidarManager::Create: file not exist: " << file;
    return nullptr;
  }

  YAML::Node config = YAML::LoadFile(file);
  return Create(config);
}

std::shared_ptr<OdLidarBase> OdLidarManager::Create(const YAML::Node& config) {
  if (!config["Type"].IsDefined() || !config["Config"].IsDefined()) {
    TFATAL << "OdLidarManager::Create: config is invalid";
    return nullptr;
  }

  std::shared_ptr<OdLidarBase> detector = nullptr;

  if (config["Type"].as<std::string>() == "OdLidarLidarNetSdk") {
    detector = std::make_shared<OdLidarLidarNetSdk>();
  } else {
    TFATAL << "OdLidarManager::Create: detector type not supported: " << config["Type"].as<std::string>();
    return nullptr;
  }

  if (detector) {
    auto ret = detector->Init(config["Config"]);
    if (ret != ErrorCode::SUCCESS) {
      TFATAL << "OdLidarManager::Create: init detector failed, error code: " << ret;
      return nullptr;
    }
  }

  return detector;
}

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_END