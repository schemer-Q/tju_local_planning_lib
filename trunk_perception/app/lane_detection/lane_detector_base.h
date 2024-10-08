/**
 * @file lane_detector_base.h
 * @author WangXu (xuwang_note@hotmail.com)
 * @brief 车道线检测器基类
 * @version 0.1
 * @date 2024-09-25
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include "trunk_perception/app/base/app_base.h"
#include "trunk_perception/common/macros.h"

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_BEGIN

/**
 * @brief 车道线检测器基类
 * @details 基类不可实例化，所有车道线检测器都继承自该类
 */
class LaneDetectorBase : public AppBase {
 public:
  LaneDetectorBase() = default;
  ~LaneDetectorBase() = default;

  std::uint32_t Init(const YAML::Node& config) override = 0;
  std::uint32_t Run(const double& ts = 0.0) override = 0;
  std::any GetData(const std::string& key) override = 0;
};

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_END
