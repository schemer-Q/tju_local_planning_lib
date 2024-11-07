/**
 * @file shape_fusion.h
 * @author WangXu (xuwang_note@hotmail.com)
 * @brief 形状融合
 * @version 0.1
 * @date 2024-11-04
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

struct ShapeFusionConfig {
  std::string type;

  virtual ~ShapeFusionConfig() = default;

  typedef std::shared_ptr<ShapeFusionConfig> Ptr;
  typedef std::shared_ptr<const ShapeFusionConfig> ConstPtr;
};

/**
 * @brief 基于滑动窗口的形状融合
 *
 */
class ShapeFusionBase {
 public:
  ShapeFusionBase(const ShapeFusionConfig::ConstPtr& config) : type_(config->type) {}
  virtual ~ShapeFusionBase() =default;

  virtual std::uint32_t Init(const Eigen::Vector3f& size) = 0;

  virtual std::uint32_t Update(const SensorMeasureFrame::ConstPtr& sensor_measure_frame) = 0;

  Eigen::Vector3f GetFusedSize() const { return size_; }

 protected:
  std::string type_;
  Eigen::Vector3f size_;
};

class ShapeFusionFactory {
 public:
  static std::shared_ptr<ShapeFusionBase> Create(const ShapeFusionConfig::ConstPtr& config);
};

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_END
