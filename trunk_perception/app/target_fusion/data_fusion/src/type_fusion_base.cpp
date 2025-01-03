#include "trunk_perception/app/target_fusion/data_fusion/type_fusion_base.h"
#include "trunk_perception/app/target_fusion/data_fusion/type_fusion_sliding_window.h"
#include "trunk_perception/tools/log/t_log.h"

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_BEGIN

std::shared_ptr<TypeFusionBase> TypeFusionFactory::Create(const TypeFusionConfig::ConstPtr& config) {
  std::shared_ptr<TypeFusionBase> type_fusion = nullptr;

  if (!config) {
    TFATAL << "TypeFusionFactory::Create: config is nullptr";              // @zzg 2024-12-26
    return nullptr;
  }

  if (config->config_type == "SlidingWindow") {
    type_fusion = std::make_shared<TypeFusionSlidingWindow>(config);
  } else {
    TFATAL << "TypeFusionFactory::Create: type is not supported: ";        // @zzg 2024-12-26
    return nullptr;
  }

  return type_fusion;
}

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_END