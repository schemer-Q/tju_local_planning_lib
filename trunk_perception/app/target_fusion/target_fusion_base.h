/**
 * @file target_fusion_base.h
 * @author WangXu (xuwang_note@hotmail.com)
 * @brief 后融合任务基类
 * @version 0.1
 * @date 2024-10-22
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include "trunk_perception/app/base/app_base.h"
#include "trunk_perception/common/macros.h"

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_BEGIN

class TargetFusionBase : public AppBase {
 public:
  TargetFusionBase() = default;
  ~TargetFusionBase() override = default;

  std::uint32_t Init(const YAML::Node& config) override = 0;
  std::uint32_t Run(const double& ts) override = 0;
  std::any GetData(const std::string& key) override = 0;
};

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_END
