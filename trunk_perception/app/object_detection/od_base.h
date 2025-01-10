/**
 * @file od_base.h
 * @author WangXu (xuwang_note@hotmail.com)
 * @brief 物体检测app基类
 * @version 0.1
 * @date 2024-12-05
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include "trunk_perception/app/base/app_base.h"

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_BEGIN

/**
 * @brief 物体检测app基类,纯虚类
 */
class OdBase : public AppBase {
 public:
  OdBase() = default;
  ~OdBase() override = default;

  std::uint32_t Init(const YAML::Node& config) override = 0;
  std::uint32_t Run(const double& ts = 0.0) override = 0;
  std::any GetData(const std::string& key) override = 0;
};

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_END
