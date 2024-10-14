
#include "trunk_perception/app/lane_detection_post/ld_post_manager.h"
#include "trunk_perception/app/lane_detection_post/bev_lane_post_impl.h"
#include "trunk_perception/common/error/code.hpp"
#include "trunk_perception/common/macros.h"
#include "trunk_perception/tools/log/t_log.h"
#include "trunk_perception/tools/system/utils.hpp"

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_BEGIN

using namespace TRUNK_PERCEPTION_LIB_COMMON_NAMESPACE;

std::shared_ptr<LdPostBase> LdPostManager::Create(const YAML::Node& config) {
  if (!config["Type"].IsDefined() || !config["Config"].IsDefined()) {
    TFATAL << "LdPostManager::Create: config is invalid";
    return nullptr;
  }

  std::shared_ptr<LdPostBase> processor = nullptr;

  if (config["Type"].as<std::string>() == "BEVLanePost") {
    processor = std::make_shared<BevLanePostImpl>();
  } else {
    TFATAL << "LdPostManager::Create: processor type not supported: " << config["Type"].as<std::string>();
    return nullptr;
  }

  if (processor) {
    auto ret = processor->Init(config["Config"]);
    if (ret != ErrorCode::SUCCESS) {
      TFATAL << "LdPostManager::Create: init processor failed, error code: " << ret;
      return nullptr;
    }
  }

  return processor;
}

std::shared_ptr<LdPostBase> LdPostManager::Create(const std::string& file) {
  if (!is_file_exist(file)) {
    TFATAL << "LdPostManager::Create: file not exist: " << file;
    return nullptr;
  }

  YAML::Node config = YAML::LoadFile(file);
  return Create(config);
}

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_END
