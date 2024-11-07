#include "trunk_perception/app/target_fusion/target_fusion_manager.h"
#include "trunk_perception/app/target_fusion/target_fusion_impl/target_fusion_a.h"
#include "trunk_perception/common/error/code.hpp"
#include "trunk_perception/common/macros.h"
#include "trunk_perception/tools/log/t_log.h"
#include "trunk_perception/tools/system/utils.hpp"

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_BEGIN

using namespace TRUNK_PERCEPTION_LIB_COMMON_NAMESPACE;

std::shared_ptr<TargetFusionBase> TargetFusionManager::Create(const YAML::Node& config) {
  if (!config["Type"].IsDefined() || !config["Config"].IsDefined()) {
    TFATAL << "TargetFusionManager::Create: config is invalid";
    return nullptr;
  }

  std::shared_ptr<TargetFusionBase> target_fusion = nullptr;

  if (config["Type"].as<std::string>() == "A") {
    target_fusion = std::make_shared<TargetFusionA>();
  } else {  
    TFATAL << "TargetFusionManager::Create: target fusion type not supported: " << config["Type"].as<std::string>();
    return nullptr;
  }

  if (target_fusion) {
    auto ret = target_fusion->Init(config["Config"]);
    if (ret != ErrorCode::SUCCESS) {
      TFATAL << "TargetFusionManager::Create: init target fusion failed, error code: " << ret;
      return nullptr;
    }
  }

  return target_fusion;
}

std::shared_ptr<TargetFusionBase> TargetFusionManager::Create(const std::string& config_file) {
  if (!is_file_exist(config_file)) {
    TFATAL << "TargetFusionManager::Create: file not exist: " << config_file;
    return nullptr;
  }

  YAML::Node config = YAML::LoadFile(config_file);
  return Create(config);
}

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_END