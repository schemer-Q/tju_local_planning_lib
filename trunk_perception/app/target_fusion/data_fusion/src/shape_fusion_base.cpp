#include "trunk_perception/app/target_fusion/data_fusion/shape_fusion_base.h"
#include "trunk_perception/app/target_fusion/data_fusion/shape_fusion_lidar_only_impl.h"
#include "trunk_perception/tools/log/t_log.h"

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_BEGIN

std::shared_ptr<ShapeFusionBase> ShapeFusionFactory::Create(const ShapeFusionConfig::ConstPtr& config) {
  std::shared_ptr<ShapeFusionBase> shape_fusion = nullptr;

  if (!config) {
    TFATAL << "ShapeFusionFactory::Create: config is nullptr";
    return nullptr;
  }

  if (config->type == "LidarOnly") {
    shape_fusion = std::make_shared<ShapeFusionLidarOnlyImpl>(config);
  } else {
    TFATAL << "ShapeFusionFactory::Create: type is not supported: " << config->type;
    return nullptr;
  }

  return shape_fusion;
}

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_END
