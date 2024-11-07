#include "trunk_perception/app/target_fusion/data_fusion/yaw_fusion_base.h"
#include "trunk_perception/app/target_fusion/data_fusion/yaw_fusion_kf_impl.h"
#include "trunk_perception/common/error/code.hpp"
#include "trunk_perception/common/macros.h"
#include "trunk_perception/tools/log/t_log.h"

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_BEGIN

using namespace TRUNK_PERCEPTION_LIB_COMMON_NAMESPACE;

std::shared_ptr<YawFusionBase> YawFusionFactory::Create(const YawFusionConfig::ConstPtr& config) {
  if (!config) {
    TFATAL << "YawFusionFactory::Create: config is nullptr";
    return nullptr;
  }

  if (config->type == "KF") {
    return std::make_shared<YawFusionKFImpl>(config);
  } else {
    TFATAL << "YawFusionFactory::Create: invalid yaw fusion type: " << config->type;
    return nullptr;
  }
}

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_END
