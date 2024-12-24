#include "trunk_perception/app/target_fusion/data_fusion/existence_fusion_1l1r.h"
#include "trunk_perception/app/target_fusion/data_fusion/existence_fusion_1l1r1v.h"
#include "trunk_perception/app/target_fusion/data_fusion/existence_fusion_base.h"
#include "trunk_perception/tools/log/t_log.h"

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_BEGIN

ExistenceFusionBase::Ptr ExistenceFusionFactory::Create(const ExistenceFusionConfig::ConstPtr& config) {
  if (config == nullptr) {
    TFATAL << "ExistenceFusionFactory::Create: config is nullptr";
    return nullptr;
  }

  if (config->type == "1L1R") {
    return std::make_shared<ExistenceFusion1L1R>(config);
  } else if (config->type == "1L1R1V") {
		return std::make_shared<ExistenceFusion1L1R1V>(config);
	} else {
    TFATAL << "ExistenceFusionFactory::Create: unknown existence fusion type: " << config->type;
  }

  return nullptr;
}

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_END