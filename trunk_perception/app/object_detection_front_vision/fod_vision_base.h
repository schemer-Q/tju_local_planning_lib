/**
 * @file fod_vision_base.h
 * @author Fan Dongsheng (fandongsheng@trunk.tech)
 * @brief 前向视觉检测应用
 * @version 0.1
 * @date 2024-11-25
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include "trunk_perception/app/base/app_base.h"

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_BEGIN

class FodVisionBase : public AppBase {
 public:
  FodVisionBase() = default;
  ~FodVisionBase() override = default;

  std::uint32_t Init(const YAML::Node& config) override = 0;
  std::uint32_t Run(const double& ts = 0.0) override = 0;
  std::any GetData(const std::string& key) override = 0;
};

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_END