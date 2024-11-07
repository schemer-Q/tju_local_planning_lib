/**
 * @file existence_fusion_1l1r.h
 * @author WangXu (xuwang_note@hotmail.com)
 * @brief 1L1R存在性融合
 * @version 0.1
 * @date 2024-11-06
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include "trunk_perception/app/target_fusion/data_fusion/existence_fusion_base.h"
#include "trunk_perception/common/macros.h"

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_BEGIN

using namespace TRUNK_PERCEPTION_LIB_COMMON_NAMESPACE;

struct ExistenceFusion1L1RConfig : ExistenceFusionConfig {
  typedef std::shared_ptr<ExistenceFusion1L1RConfig> Ptr;
  typedef std::shared_ptr<const ExistenceFusion1L1RConfig> ConstPtr;
};

/**
 * @brief 1L1R存在性融合
 * @note 设计文档：
 * http://192.168.3.220:8090/pages/viewpage.action?pageId=98996880#id-%E5%90%8E%E8%9E%8D%E5%90%88%E5%BC%80%E5%8F%91%E8%AE%B0%E5%BD%95-1L1R
 */
class ExistenceFusion1L1R : public ExistenceFusionBase {
 public:
  ExistenceFusion1L1R(const ExistenceFusionConfig::ConstPtr& config);
  virtual ~ExistenceFusion1L1R() = default;

  virtual float Compute(const FusedObject::Ptr& fused_object_ptr) override;
};

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_END