/**
 * @file od_lidar_base.h
 * @author WangXu (xuwang_note@hotmail.com)
 * @brief 激光物体检测模型应用
 * @version 0.1
 * @date 2024-09-14
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include "trunk_perception/app/base/app_base.h"

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_BEGIN

/**
 * @brief 未实现，后续可以将基类实现为基于 DetectionNetSDK 的物体检测模型
 */
class OdLidarBase : public AppBase {
 public:
  OdLidarBase() = default;
  ~OdLidarBase() override = default;

  std::uint32_t Init(const YAML::Node& config) override = 0;
  std::uint32_t Run(const double& ts = 0.0) override = 0;
  std::any GetData(const std::string& key) override = 0;
};

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_END