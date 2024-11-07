/**
 * @file yaw_fusion.h
 * @author WangXu (xuwang_note@hotmail.com)
 * @brief
 * @version 0.1
 * @date 2024-11-05
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include <memory>

#include "trunk_perception/common/macros.h"

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_BEGIN

struct YawFusionConfig {
  std::string type;

  virtual ~YawFusionConfig() = default;

  typedef std::shared_ptr<YawFusionConfig> Ptr;
  typedef std::shared_ptr<const YawFusionConfig> ConstPtr;
};

/**
 * @brief 航向角融合
 *
 */
class YawFusionBase {
 public:
  YawFusionBase(const YawFusionConfig::ConstPtr& config) : type_(config->type) {}
  virtual ~YawFusionBase() = default;

  virtual std::uint32_t Init(const float& yaw) = 0;

  virtual void Predict(const double& dt) = 0;

  virtual std::uint32_t Update(const float& yaw) = 0;

  virtual float GetYaw() const { return yaw_; }

 protected:
  std::string type_;

  float yaw_ = 0.0f;
};

class YawFusionFactory {
 public:
  static std::shared_ptr<YawFusionBase> Create(const YawFusionConfig::ConstPtr& config);
};

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_END
