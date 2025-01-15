/**
 * @file existence_fusion_1l1r1v.h
 * @author zzg 2025-01-07
 * @brief 1L5R1V 融合目标存在性更新策略 初版调试
 * @version 0.1
 * @date 2025-01-07
 *
 * @copyright Copyright (c) 2024
 */

#pragma once

#include "trunk_perception/app/target_fusion/data_fusion/existence_fusion_base.h"
#include "trunk_perception/common/macros.h"

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_BEGIN

using namespace TRUNK_PERCEPTION_LIB_COMMON_NAMESPACE;

struct ExistenceFusion1L5R1VConfig : ExistenceFusionConfig {
  typedef std::shared_ptr<ExistenceFusion1L5R1VConfig> Ptr;
  typedef std::shared_ptr<const ExistenceFusion1L5R1VConfig> ConstPtr;
};

/**
 * @brief 1L5R1V 1L5R1V 融合目标存在性更新策略
 * @note 设计文档：https://trunk.feishu.cn/docx/DGh0dH91jo3UGLxSWdpc9jKxncd，加入环视后，使用1L5R2V
 *
 */
class ExistenceFusion1L5R1V : public ExistenceFusionBase {
 public:
  ExistenceFusion1L5R1V(const ExistenceFusionConfig::ConstPtr& config);
  virtual ~ExistenceFusion1L5R1V() = default;

  virtual float Compute(const FusedObject::Ptr& fused_object_ptr) override;
  bool IsObjectHasNFrameRadar(const FusedObject::Ptr& fused_object_ptr, int num);
};

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_END
