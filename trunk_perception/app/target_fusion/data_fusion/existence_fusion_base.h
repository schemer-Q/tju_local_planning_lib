/**
 * @file existence_fusion.h
 * @author WangXu (xuwang_note@hotmail.com)
 * @brief 存在性融合
 * @version 0.1
 * @date 2024-11-06
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include <memory>
#include <string>

#include "trunk_perception/common/macros.h"
#include "trunk_perception/common/types/fused_object.h"

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_BEGIN

using namespace TRUNK_PERCEPTION_LIB_COMMON_NAMESPACE;

struct ExistenceFusionConfig {
  std::string type = "1L1R";

  virtual ~ExistenceFusionConfig() = default;

  typedef std::shared_ptr<ExistenceFusionConfig> Ptr;
  typedef std::shared_ptr<const ExistenceFusionConfig> ConstPtr;
};

/**
 * @brief 存在性融合基类
 */
class ExistenceFusionBase {
 public:
  ExistenceFusionBase(const ExistenceFusionConfig::ConstPtr& config) : type_(config->type) {}
  virtual ~ExistenceFusionBase() = default;

  virtual float Compute(const FusedObject::Ptr& fused_object_ptr) = 0;

 protected:
  std::string type_;

 public:
  typedef std::shared_ptr<ExistenceFusionBase> Ptr;
  typedef std::shared_ptr<const ExistenceFusionBase> ConstPtr;
};

class ExistenceFusionFactory {
 public:
  static ExistenceFusionBase::Ptr Create(const ExistenceFusionConfig::ConstPtr& config);
};

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_END