#include "trunk_perception/app/camera_preprocess/preprocess_manager.h"
#include "trunk_perception/common/error/code.hpp"
#include "trunk_perception/common/macros.h"
#include "trunk_perception/tools/log/t_log.h"
#include "trunk_perception/tools/system/utils.hpp"

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_BEGIN

using namespace TRUNK_PERCEPTION_LIB_COMMON_NAMESPACE;

std::shared_ptr<CameraPreprocessBase> CameraPreprocessManager::Create(const YAML::Node& config) {
  if (!config["Type"].IsDefined() || !config["Config"].IsDefined()) {
    TFATAL << "PreprocessManager::Create: config is invalid";
    return nullptr;
  }

  std::shared_ptr<CameraPreprocessBase> processor = nullptr;

  if (config["Type"].as<std::string>() == "BASE") {
    processor = std::make_shared<CameraPreprocessBase>();
  } else {
    TFATAL << "PreprocessManager::Create: processor type not supported: " << config["Type"].as<std::string>();
    return nullptr;
  }

  if (processor) {
    auto ret = processor->Init(config["Config"]);
    if (ret != ErrorCode::SUCCESS) {
      TFATAL << "PreprocessManager::Create: init processor failed, error code: " << ret;
      return nullptr;
    }
  }

  return processor;
}

std::shared_ptr<CameraPreprocessBase> CameraPreprocessManager::Create(const std::string& file) {
  if (!is_file_exist(file)) {
    TFATAL << "PreprocessManager::Create: file not exist: " << file;
    return nullptr;
  }

  YAML::Node config = YAML::LoadFile(file);
  return Create(config);
}

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_END