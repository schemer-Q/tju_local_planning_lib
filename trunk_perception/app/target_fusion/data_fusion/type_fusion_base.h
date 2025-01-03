/**
 * @file type_fusion_base.h
 * @author zzg
 * @brief 类型融合  初版调试
 * @version 0.1
 * @date 2024-12-23
 * 
 * @copyright Copyright (c) 2024
 * 
*/

#pragma once

#include <memory>
#include <cstdint>

#include "trunk_perception/common/error/code.hpp"
#include "trunk_perception/common/macros.h"
#include "trunk_perception/common/types/fused_object.h"

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_BEGIN

using namespace TRUNK_PERCEPTION_LIB_COMMON_NAMESPACE;

struct TypeFusionConfig {
  std::string config_type;

  virtual ~TypeFusionConfig() = default;

  typedef std::shared_ptr<TypeFusionConfig> Ptr;
  typedef std::shared_ptr<const TypeFusionConfig> ConstPtr;
};

/**
 * @brief 对目标类型进行滤波
 * 
*/
class TypeFusionBase {
 public:
  TypeFusionBase(const TypeFusionConfig::ConstPtr& config) : config_type_(config->config_type) {}
  virtual ~TypeFusionBase() = default;

  virtual std::uint32_t Init(const ObjectType& type) = 0;

  virtual std::uint32_t Update(const SensorMeasureFrame::ConstPtr& sensor_measure_frame) = 0;

  ObjectType GetFusedType() const { return object_type_; }
 protected:
  std::string config_type_;
  ObjectType object_type_;
};

class TypeFusionFactory {
 public:
  static std::shared_ptr<TypeFusionBase> Create(const TypeFusionConfig::ConstPtr& config);
};

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_END