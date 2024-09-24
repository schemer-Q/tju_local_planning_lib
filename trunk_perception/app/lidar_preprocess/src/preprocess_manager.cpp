#include "trunk_perception/app/lidar_preprocess/preprocess_manager.h"

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_BEGIN

std::shared_ptr<PreprocessBase> PreprocessManager::Create(const std::string& type) {
  if (type == "BASE") {
    return std::make_shared<PreprocessBase>();
  }
  return nullptr;
}

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_END
