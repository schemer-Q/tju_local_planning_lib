#include "trunk_perception/app/object_detection/od_manager.h"
#include "trunk_perception/common/error/code.hpp"
#include "trunk_perception/tools/log/t_log.h"
#include "trunk_perception/tools/system/utils.hpp"
#include "trunk_perception/app/object_detection/od_sparse4d_impl.h"

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_BEGIN

using namespace TRUNK_PERCEPTION_LIB_COMMON_NAMESPACE;

std::shared_ptr<OdBase> OdManager::Create(const YAML::Node& config) {
  if (!config["Type"].IsDefined() || !config["Config"].IsDefined()) {
    TFATAL << "OdManager::Create: config is invalid";
    return nullptr;
  }

  std::shared_ptr<OdBase> detector = nullptr;

  if (config["Type"].as<std::string>() == "ODSparse4D") {
    detector = std::make_shared<OdSparse4DImpl>();
  } else {
    TFATAL << "OdManager::Create: detector type not supported: " << config["Type"].as<std::string>();
    return nullptr;
  }

  if (detector) {
    auto ret = detector->Init(config["Config"]);
    if (ret != ErrorCode::SUCCESS) {
      TFATAL << "OdManager::Create: init detector failed, error code: " << ret;
      return nullptr;
    }
  }

  return detector;
}

std::shared_ptr<OdBase> OdManager::Create(const std::string& file) {
  if (!is_file_exist(file)) {
    TFATAL << "OdManager::Create: file not exist: " << file;
    return nullptr;
  }

  YAML::Node config = YAML::LoadFile(file);
  return Create(config);
}

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_END