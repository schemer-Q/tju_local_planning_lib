#include "trunk_perception/app/lane_detection/ld_manager.h"
#include "trunk_perception/app/lane_detection/ld_SDKBevLaneConeDet.h"
#include "trunk_perception/common/error/code.hpp"
#include "trunk_perception/common/macros.h"
#include "trunk_perception/tools/log/t_log.h"
#include "trunk_perception/tools/system/utils.hpp"

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_BEGIN

using namespace TRUNK_PERCEPTION_LIB_COMMON_NAMESPACE;

std::shared_ptr<LaneDetectorBase> LaneDetectorManager::Create(const YAML::Node& config) {
  if (!config["Type"].IsDefined() || !config["Config"].IsDefined()) {
    TFATAL << "LaneDetectorManager::Create: config Type or Config not defined";
    return nullptr;
  }

  std::shared_ptr<LaneDetectorBase> detector = nullptr;
  std::string type = config["Type"].as<std::string>();
  if (type == "BEVLANECONE") {
    detector = std::make_shared<LaneDetectorSDKBevLaneConeDet>();
    auto ret = detector->Init(config["Config"]);
    if (ret != ErrorCode::SUCCESS) {
      TERROR << "LaneDetectorManager::Create: detector init failed: " << ret;
      return nullptr;
    }
  }

  return detector;
}

std::shared_ptr<LaneDetectorBase> LaneDetectorManager::Create(const std::string& file) {
  if (!is_file_exist(file)) {
    TFATAL << "LaneDetectorManager::Create: file not exist: " << file;
    return nullptr;
  }

  YAML::Node config = YAML::LoadFile(file);
  return Create(config);
}

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_END
