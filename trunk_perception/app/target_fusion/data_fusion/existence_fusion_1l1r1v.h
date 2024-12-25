/**
 * @file existence_fusion_1l1r1v.h
 * @author zzg 2024-12-18
 * @brief 1L1R1V 融合目标存在性更新策略 初版调试
 * @version 0.1
 * @date 2024-12-18
 * 
 * @copyright Copyright (c) 2024
*/

#pragma once

#include "trunk_perception/app/target_fusion/data_fusion/existence_fusion_base.h"
#include "trunk_perception/common/macros.h"

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_BEGIN

using namespace TRUNK_PERCEPTION_LIB_COMMON_NAMESPACE;

struct ExistenceFusion1L1R1VConfig : ExistenceFusionConfig {
  typedef std::shared_ptr<ExistenceFusion1L1R1VConfig> Ptr;
  typedef std::shared_ptr<const ExistenceFusion1L1R1VConfig> ConstPtr;
};

/**
 * @brief 1L1R1V 1L1R1V 融合目标存在性更新策略
 * @note 设计文档：https://trunk.feishu.cn/docx/X2paddDDFoIcx7x299wc0cN1ntb
 * 
 */
class ExistenceFusion1L1R1V : public ExistenceFusionBase {
 public:
  ExistenceFusion1L1R1V(const ExistenceFusionConfig::ConstPtr& config);
  virtual ~ExistenceFusion1L1R1V() = default;

  virtual float Compute(const FusedObject::Ptr& fused_object_ptr) override;
};

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_END
